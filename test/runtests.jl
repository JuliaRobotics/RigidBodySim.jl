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
    version_major = 0
    version_minor = 1
    data = JSON.json(contents)
    msg = DrakeVisualizer.Comms.CommsT(utime, format, version_major, version_minor, data)
    publish(lcm, RigidBodySim.LCM_CONTROL_CHANNEL, msg)
end

visualizer_process = new_visualizer_window(); sleep(1)

try
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
        mechanism = rand_tree_mechanism(Float64, [Revolute{Float64} for i = 1 : 30]...)
        state = MechanismState(mechanism)

        vis = Visualizer(mechanism; show_inertias = true)
        settransform!(vis, state)
        vis_callbacks = CallbackSet(vis, state)

        tfinal = 100.
        dt = 1e-4
        problem = ODEProblem(state, (0., tfinal))

        # Simulate for 3 seconds (wall time) and then send a termination command
        @async (sleep(3.); send_control_message(LCM(), Dict("simulate" => false)))
        sol = solve(problem, RK4(), adaptive = false, dt = dt, callback = vis_callbacks)
        @test sol.t[end] > 2 * dt
        @test sol.t[end] < tfinal
        println("last(sol.t) after early termination 1: $(last(sol.t))")

        # Rinse and repeat with the same ODEProblem (make sure that we don't terminate straight away)
        @async (sleep(3.); send_control_message(LCM(), Dict("simulate" => false)))
        sol = solve(problem, RK4(), adaptive = false, dt = dt, callback = vis_callbacks)
        @test sol.t[end] > 2 * dt
        @test sol.t[end] < tfinal
        println("last(sol.t) after early termination 2: $(last(sol.t))")
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
        mechanism = rand_tree_mechanism(Float64, [Revolute{Float64} for i = 1 : 30]...)
        state = MechanismState(mechanism)

        final_time = 5.
        problem = ODEProblem(state, (0., final_time))
        sol = solve(problem, Vern7(), abs_tol = 1e-10, dt = 0.05)

        realtime_rate = 2.
        vis = Visualizer(mechanism; show_inertias = true)
        animate(vis, state, sol, realtime_rate = 1000.)
        elapsed = @elapsed animate(vis, state, sol, realtime_rate = realtime_rate, max_fps = 60.)
        @test elapsed ≈ final_time / realtime_rate atol = 0.1
    end
finally
    kill(visualizer_process)
end
