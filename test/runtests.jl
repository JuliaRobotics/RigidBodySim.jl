using RigidBodySim
using RigidBodyDynamics
using DrakeVisualizer
using RigidBodyTreeInspector
using OrdinaryDiffEq
using JSON
using LCMCore
using Base.Test

function send_control_message(lcm::LCM, contents::Associative)
    utime = round(Int, time() * 1e-3)
    format = "rigid_body_sim_json"
    version_major = 1
    version_minor = 1
    data = JSON.json(contents)
    msg = DrakeVisualizer.Comms.CommsT(utime, format, version_major, version_minor, data)
    publish(lcm, RigidBodySim.LCM_CONTROL_CHANNEL, msg)
end

@testset "PeriodicCallback" begin
    tmin = 0.1
    tmax = 5.2
    for tmax_problem in [tmax; Inf]
        # Test with both a finite and an infinite tspan for the ODEProblem.
        #
        # Having support for infinite tspans is one of the main reasons for implementing PeriodicCallback
        # using add_tstop!, instead of just passing in a linspace as the tstops solve argument.
        # (the other being that the length of the internal tstops collection would otherwise become
        # linear in the length of the integration time interval.
        #
        # Testing a finite tspan is necessary because a naive implementation could add tstops after
        # tmax and thus integrate for too long (or even indefinitely).

        # Dynamics: two independent single integrators:
        du = [0; 0]
        u0 = [0.; 0.]
        dynamics = (t, u) -> eltype(u).(du)
        prob = ODEProblem(dynamics, u0, (tmin, tmax_problem))

        # Callbacks periodically increase the input to the integrators:
        Δt1 = 0.5
        increase_du_1 = integrator -> du[1] += 1
        cb1 = PeriodicCallback(increase_du_1, Δt1)

        Δt2 = 1.
        increase_du_2 = integrator -> du[2] += 1
        cb2 = PeriodicCallback(increase_du_2, Δt2)

        # Terminate at tmax (regardless of whether the tspan of the ODE problem is infinite).
        terminator = DiscreteCallback((t, u, integrator) -> t == tmax, terminate!)

        # Solve.
        sol = solve(prob, Tsit5(); callback = CallbackSet(terminator, cb1, cb2), tstops = [tmax])

        # Make sure we haven't integrated past tmax:
        @test sol.t[end] == tmax

        # Make sure that the components of du have been incremented the appropriate number of times.
        Δts = [Δt1, Δt2]
        expected_num_calls = map(Δts) do Δt
            floor(Int, (tmax - tmin) / Δt) + 1
        end
        @test du == expected_num_calls

        # Make sure that the final state matches manual integration of the piecewise-linear function
        foreach(Δts, sol.u[end], du) do Δt, u_i, du_i
            @test u_i ≈ Δt * sum(1 : du_i - 1) + rem(tmax - tmin, Δt) * du_i atol = 1e-5
        end
    end
end

@testset "compare to simulate" begin
    srand(1)

    urdf = Pkg.dir("RigidBodySim", "test", "urdf", "Acrobot.urdf")
    mechanism = parse_urdf(Float64, urdf)

    state = MechanismState(mechanism)
    rand!(state)
    x0 = state_vector(state) # TODO: Vector constructor

    final_time = 5.
    problem = ODEProblem(state, (0., final_time))
    sol = solve(problem, Vern7(), abs_tol = 1e-10, dt = 0.05)

    set!(state, x0)
    ts, qs, vs = RigidBodyDynamics.simulate(state, final_time)

    @test [qs[end]; vs[end]] ≈ sol[end] atol = 1e-2
end

@testset "visualizer callbacks" begin
    visualizer_process = new_visualizer_window(); sleep(1)
    try
        mechanism = rand_tree_mechanism(Float64, [Revolute{Float64} for i = 1 : 30]...)
        state = MechanismState(mechanism)

        vis = Visualizer(mechanism; show_inertias = true)
        settransform!(vis, state)
        vis_callbacks = CallbackSet(vis, state)

        tfinal = 100.
        dt = 1e-4
        problem = ODEProblem(state, (0., tfinal))

        # Simulate for 3 seconds (wall time) and then send a termination command
        @async (sleep(3.); send_control_message(LCM(), Dict("terminate" => nothing)))
        sol = solve(problem, RK4(), adaptive = false, dt = dt, callback = vis_callbacks)
        @test sol.t[end] > 2 * dt
        @test sol.t[end] < tfinal
        println("last(sol.t) after early termination 1: $(last(sol.t))")

        # Rinse and repeat with the same ODEProblem (make sure that we don't terminate straight away)
        @async (sleep(3.); send_control_message(LCM(), Dict("terminate" => nothing)))
        sol = solve(problem, RK4(), adaptive = false, dt = dt, callback = vis_callbacks)
        @test sol.t[end] > 2 * dt
        @test sol.t[end] < tfinal
        println("last(sol.t) after early termination 2: $(last(sol.t))")
    finally
        kill(visualizer_process)
    end
end

@testset "renormalization callback" begin
    mechanism = rand_tree_mechanism(Float64, QuaternionFloating{Float64})
    floatingjoint = first(joints(mechanism))
    state = MechanismState(mechanism)

    rand!(configuration(state))
    @test !RigidBodyDynamics.is_configuration_normalized(floatingjoint, configuration(state, floatingjoint))

    problem = ODEProblem(state, (0., 1e-3))
    sol = solve(problem, Vern7(), dt = 1e-4, callback = configuration_renormalizer(state))

    set!(state, sol[end])
    @test RigidBodyDynamics.is_configuration_normalized(floatingjoint, configuration(state, floatingjoint))
end

@testset "ODESolution animation" begin
    visualizer_process = new_visualizer_window(); sleep(1)
    try
        mechanism = rand_tree_mechanism(Float64, [Revolute{Float64} for i = 1 : 30]...)
        state = MechanismState(mechanism)

        final_time = 5.
        problem = ODEProblem(state, (0., final_time))
        sol = solve(problem, Vern7(), abs_tol = 1e-10, dt = 0.05)

        # regular playback
        realtime_rate = 2.
        vis = Visualizer(mechanism; show_inertias = true)
        animate(vis, state, sol, realtime_rate = 1000.)
        elapsed = @elapsed animate(vis, state, sol, realtime_rate = realtime_rate, max_fps = 60.)
        @test elapsed ≈ final_time / realtime_rate atol = 0.1

        # premature termination
        termination_time = 1.5
        @async (sleep(termination_time); send_control_message(LCM(), Dict("terminate" => nothing)))
        elapsed = @elapsed animate(vis, state, sol, realtime_rate = realtime_rate)
        @test elapsed ≈ termination_time atol = 0.1
    finally
        kill(visualizer_process)
    end
end

@testset "PeriodicController" begin
    urdf = Pkg.dir("RigidBodySim", "test", "urdf", "Acrobot.urdf")
    mechanism = parse_urdf(Float64, urdf)
    state = MechanismState(mechanism)
    controltimes = Float64[]
    Δt = 0.25
    controller = PeriodicController(state, Δt, function (τ, t, state)
        push!(controltimes, t)
        τ[1] = sin(t)
        τ[2] = cos(t)
    end)
    final_time = 25.3
    problem = ODEProblem(state, (0., final_time), controller)

    # don't forget to pass in callback:
    @test_throws RigidBodySim.PeriodicControlFailure solve(problem, Vern7(), abs_tol = 1e-10, dt = 0.05)

    # ensure that controller gets called at appropriate times:
    initialize = (c, t, u, integrator) -> empty!(controltimes)
    controller_callback = PeriodicCallback(controller, initialize = initialize)
    sol = solve(problem, Vern7(), abs_tol = 1e-10, dt = 0.05, callback = controller_callback)
    @test controltimes == collect(0. : Δt : final_time - rem(final_time, Δt))

    # ensure that we can solve the same problem again without errors
    sol = solve(problem, Vern7(), abs_tol = 1e-10, dt = 0.05, callback = controller_callback)
end

# notebooks
@testset "example notebooks" begin
    using NBInclude
    notebookdir = Pkg.dir("RigidBodySim", "notebooks")
    for file in readdir(notebookdir)
        name, ext = splitext(file)
        if lowercase(ext) == ".ipynb"
            @testset "$name" begin
                println("Testing $name.")
                nbinclude(joinpath(notebookdir, file), regex = r"^((?!\#NBSKIP).)*$"s)
            end
        end
    end
end
