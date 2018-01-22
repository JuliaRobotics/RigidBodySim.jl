module RigidBodySimTest

using RigidBodySim
using RigidBodyDynamics
using RigidBodyTreeInspector
using OrdinaryDiffEq
using DiffEqCallbacks

using DrakeVisualizer
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
        send_control_message(LCM(), Dict("terminate" => nothing))
        sleep(0.1)
        @async (sleep(3.); send_control_message(LCM(), Dict("terminate" => nothing)))
        sol = solve(problem, RK4(), adaptive = false, dt = dt, callback = vis_callbacks)
        @test sol.t[end] > 2 * dt
        @test sol.t[end] < tfinal
        println("last(sol.t) after early termination 2: $(last(sol.t))")

        # Pause and unpause a short simulation, make sure that the simulation takes longer than without pausing
        problem = ODEProblem(state, (0., 1.))
        normaltime = @elapsed solve(problem, RK4(), adaptive = false, dt = dt, callback = vis_callbacks)
        @show normaltime
        pausetime = normaltime / 3
        unpausetime = pausetime + normaltime * 3
        @async (sleep(pausetime); send_control_message(LCM(), Dict("pause" => nothing)))
        @async (sleep(unpausetime); send_control_message(LCM(), Dict("pause" => nothing)))
        timewithpause = @elapsed solve(problem, RK4(), adaptive = false, dt = dt, callback = vis_callbacks)
        @test timewithpause > normaltime * 2

        # Simulate for 3 seconds wall time, then pause, and then terminate a second later to make sure terminating works while paused
        problem = ODEProblem(state, (0., tfinal))
        @async (sleep(3.); send_control_message(LCM(), Dict("pause" => nothing)))
        @async (sleep(4.); send_control_message(LCM(), Dict("terminate" => nothing)))
        sol = solve(problem, RK4(), adaptive = false, dt = dt, callback = vis_callbacks)
        @test sol.t[end] > 2 * dt
        @test sol.t[end] < tfinal
        println("last(sol.t) after early termination 3: $(last(sol.t))")
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

        # pause and unpause
        pause_time = 1.0
        unpause_time = 3.5
        @async (sleep(pause_time); send_control_message(LCM(), Dict("pause" => nothing)))
        @async (sleep(unpause_time); send_control_message(LCM(), Dict("pause" => nothing)))
        elapsed = @elapsed animate(vis, state, sol, realtime_rate = realtime_rate)
        @show elapsed
        @test elapsed ≈ final_time / realtime_rate + (unpause_time - pause_time) atol = 0.2 # higher atol because of pause poll int

        # pause and terminate
        @async (sleep(pause_time); send_control_message(LCM(), Dict("pause" => nothing)))
        @async (sleep(termination_time); send_control_message(LCM(), Dict("terminate" => nothing)))
        elapsed = @elapsed animate(vis, state, sol, realtime_rate = realtime_rate)
        @test elapsed ≈ termination_time atol = 0.2 # higher atol because of pause poll int
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

end
