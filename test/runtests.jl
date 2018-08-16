module RigidBodySimTest

using Compat
using Compat.Test
using Compat.Random
using Compat.LinearAlgebra

using RigidBodySim

using RigidBodyDynamics

using DiffEqCallbacks: DiscreteCallback
using DiffEqBase: add_tstop!
using OrdinaryDiffEq: Rodas4P

using MechanismGeometries
using MeshCatMechanisms
using InteractBase: observe
using Blink: Window

if VERSION < v"0.7-"
    const seed! = srand
else
    import Random: seed!
end

function dynamics_allocations(dynamics::Dynamics, state::MechanismState) # introduce function barrier
    x = Vector(state)
    ẋ = similar(x)
    p = nothing
    t = 3.
    dynamics(ẋ, x, p, t)
    allocs = @allocated dynamics(ẋ, x, p, t)
end

@testset "Dynamics" begin
    seed!(134)
    mechanism = rand_tree_mechanism(Float64, [Revolute{Float64} for i = 1 : 30]...)
    dynamics = Dynamics(mechanism)
    state = MechanismState(mechanism)
    rand!(state)
    @test dynamics_allocations(dynamics, state) <= 80
end

@testset "compare to simulate" begin
    seed!(1)
    urdf = joinpath(@__DIR__, "urdf", "Acrobot.urdf")
    mechanism = parse_urdf(Float64, urdf)
    state = MechanismState(mechanism)
    rand!(state)
    x0 = Vector(state)
    final_time = 5.
    problem = ODEProblem(Dynamics(mechanism), state, (0., final_time))
    sol = solve(problem, Vern7(), abs_tol = 1e-10, dt = 0.05)
    Compat.copyto!(state, x0)
    ts, qs, vs = RigidBodyDynamics.simulate(state, final_time)
    @test [qs[end]; vs[end]] ≈ sol[end] atol = 1e-2
end

@testset "renormalization callback" begin
    mechanism = rand_tree_mechanism(Float64, QuaternionFloating{Float64})
    floatingjoint = first(joints(mechanism))
    state = MechanismState(mechanism)

    rand!(configuration(state))
    @test !RigidBodyDynamics.is_configuration_normalized(floatingjoint, configuration(state, floatingjoint))

    problem = ODEProblem(Dynamics(mechanism), state, (0., 1e-3))
    sol = solve(problem, Vern7(), dt = 1e-4, callback = configuration_renormalizer(state))

    Compat.copyto!(state, sol[end])
    @test RigidBodyDynamics.is_configuration_normalized(floatingjoint, configuration(state, floatingjoint))
end

@testset "RealtimeRateLimiter" begin
    du = [0; 0]
    u0 = [0.; 0.]
    dynamics = (u, p, t) -> eltype(u).(du)
    tmin = 10.1
    tmax = 12.2
    for max_rate in [2.0, 0.5]
        prob = ODEProblem(dynamics, u0, (tmin, tmax))
        rate_limiter = RealtimeRateLimiter(max_rate = max_rate)
        sol = solve(prob, Tsit5(); callback = rate_limiter)
        soltime = @elapsed solve(prob, Tsit5(); callback = rate_limiter)
        expected = (tmax - tmin) / max_rate
        @show soltime
        @show expected
        @test soltime ≈ expected atol = 0.5
    end
end

@testset "PeriodicController" begin
    urdf = joinpath(@__DIR__, "urdf", "Acrobot.urdf")
    mechanism = parse_urdf(Float64, urdf)
    state = MechanismState(mechanism)
    controltimes = Float64[]
    initialize = (c, u, t, integrator) -> empty!(controltimes)
    τ = similar(velocity(state))
    Δt = 0.25

    make_controller = function ()
        PeriodicController(τ, Δt, function (τ, t, state)
            push!(controltimes, t)
            τ[1] = sin(t)
            τ[2] = cos(t)
        end; initialize = initialize)
    end

    controller = make_controller()
    final_time = 25.3
    problem = ODEProblem(Dynamics(mechanism, controller), state, (0., final_time))

    # ensure that controller gets called at appropriate times:
    sol = solve(problem, Tsit5(), abs_tol = 1e-10, dt = 0.05)
    @test controltimes == collect(0. : Δt : final_time - rem(final_time, Δt))

    # ensure that we can solve the same problem again without errors and with a different integrator
    empty!(controltimes)
    sol = solve(problem, Vern7(), abs_tol = 1e-10, dt = 0.05)
    @test controltimes == collect(0. : Δt : final_time - rem(final_time, Δt))

    # issue #60
    empty!(controltimes)
    problem60 = ODEProblem(Dynamics(mechanism, (τ, t, state) -> controller(τ, t, state)), state, (0., final_time))
    @test_throws RigidBodySim.Control.PeriodicControlFailure solve(problem60, Vern7(), abs_tol = 1e-10, dt = 0.05)
    controller = controller = make_controller()
    problem60 = ODEProblem(Dynamics(mechanism, (τ, t, state) -> controller(τ, t, state)), state, (0., final_time))
    @test_throws RigidBodySim.Control.PeriodicControlFailure solve(problem60, Vern7(), abs_tol = 1e-10, dt = 0.05)
    problem60_fixed = ODEProblem(Dynamics(mechanism, (τ, t, state) -> controller(τ, t, state)), state, (0., final_time),
        callback = PeriodicCallback(controller))
    sol60 = solve(problem60_fixed, Vern7(), abs_tol = 1e-10, dt = 0.05)
    @test controltimes == collect(0. : Δt : final_time - rem(final_time, Δt))
    @test sol60.t == sol.t
    @test sol60.u == sol.u
end

@testset "visualizer callbacks" begin
    mechanism = rand_tree_mechanism(Float64, [Revolute{Float64} for i = 1 : 30]...)
    state = MechanismState(mechanism)
    gui = GUI(mechanism, MeshCatMechanisms.Skeleton(randomize_colors = true, inertias = true))
    vis = gui.visualizer
    controls = gui.controls
    open(vis, Window())
    open(controls, Window())
    open(gui)
    dt = 1e-4

    set_configuration!(vis, configuration(state))
    @test Vector(vis.state) == Vector(state)

    tfinal = 0.5
    problem = ODEProblem(Dynamics(mechanism), state, (0., tfinal), callback=CallbackSet(gui))

    # Simulate without interaction
    sol = solve(problem, RK4(), adaptive = false, dt = dt)
    @test sol.t[end] == tfinal
    @test Vector(vis.state) == sol.u[end]

    # Test simulation controls
    tfinal = 100.0
    problem = ODEProblem(Dynamics(mechanism), state, (0., tfinal), callback=CallbackSet(gui))

    # Simulate for 3 seconds (wall time) and then send a termination command
    @async (sleep(3.); observe(controls.terminate)[] += 1)
    sol = solve(problem, RK4(), adaptive = false, dt = dt)
    @test sol.t[end] > 2 * dt
    @test sol.t[end] < tfinal
    println("last(sol.t) after early termination 1: $(last(sol.t))")

    # Rinse and repeat with the same ODEProblem (make sure that we don't terminate straight away)
    observe(controls.terminate)[] += 1
    sleep(0.1)
    @async (sleep(3.); observe(controls.terminate)[] += 1)
    sol = solve(problem, RK4(), adaptive = false, dt = dt)
    @test sol.t[end] > 2 * dt
    @test sol.t[end] < tfinal
    println("last(sol.t) after early termination 2: $(last(sol.t))")

    # Pause and unpause a short simulation, make sure that the simulation takes longer than without pausing
    problem = ODEProblem(Dynamics(mechanism), state, (0., 1.), callback=CallbackSet(gui))
    tpause = 0.5
    pausecondition = Condition()
    havepaused = Ref(false)
    condition = (u, t, integrator) -> !havepaused[] && t >= tpause
    action = function (integrator)
        observe(controls.pause)[] += 1
        notify(pausecondition)
        havepaused[] = true
    end
    initialize = (c, t, u, integrator) -> (havepaused[] = false; add_tstop!(integrator, tpause))
    pauser = DiscreteCallback(condition, action, save_positions=(false, false), initialize = initialize)
    integrator = init(problem, RK4(), adaptive = false, dt = dt, callback=pauser)
    @async begin
        wait(pausecondition)
        yield() # wait for message to reach command handler callback
        integrator_time = integrator.t
        @test integrator_time < tpause + 0.1 # it takes a while for the pause message to reach command handler callback
        sleep(2.)
        @test integrator.t == integrator_time # make sure simulation remains paused
        havepaused[] = true
        observe(controls.pause)[] += 1 # unpause
    end
    solve!(integrator)
    @test havepaused[]

    # Simulate for 3 seconds wall time, then pause, and then terminate a second later to make sure terminating works while paused
    tfinal = 100.0
    problem = ODEProblem(Dynamics(mechanism), state, (0., tfinal), callback=CallbackSet(gui))
    @async (sleep(3.); observe(controls.pause)[] += 1)
    @async (sleep(4.); observe(controls.terminate)[] += 1)
    sol = solve(problem, RK4(), adaptive = false, dt = dt)
    @test sol.t[end] > 2 * dt
    @test sol.t[end] < tfinal
    println("last(sol.t) after early termination 3: $(last(sol.t))")
end

@testset "ODESolution animation" begin
    mechanism = rand_tree_mechanism(Float64, [Revolute{Float64} for i = 1 : 30]...)
    state = MechanismState(mechanism)
    vis = MechanismVisualizer(mechanism, MeshCatMechanisms.Skeleton(randomize_colors = true, inertias = true))
    open(vis, Window())
    wait(vis)

    final_time = 5.
    problem = ODEProblem(Dynamics(mechanism), state, (0., final_time))
    sol = solve(problem, Vern7(), abs_tol = 1e-10, dt = 0.05)

    # regular playback
    realtime_rate = 2.
    setanimation!(vis, sol, realtime_rate = realtime_rate, max_fps = 60.)
    sleep(final_time / realtime_rate)

    # broken:
    #=
    setanimation!(vis, sol, realtime_rate = 1000.)
    realtime_rate = 2.
    elapsed = @elapsed setanimation!(vis, sol, realtime_rate = realtime_rate, max_fps = 60.)
    @test elapsed ≈ final_time / realtime_rate atol = 0.1

    # premature termination
    termination_time = 1.5
    @async (sleep(termination_time); send_terminate_message())
    elapsed = @elapsed animate(vis, state, sol, realtime_rate = realtime_rate)
    @test elapsed ≈ termination_time atol = 0.1

    # pause and unpause
    pause_time = 1.0
    unpause_time = 3.5
    @async (sleep(pause_time); send_pause_message())
    @async (sleep(unpause_time); send_pause_message())
    elapsed = @elapsed animate(vis, state, sol, realtime_rate = realtime_rate)
    @show elapsed
    @test elapsed ≈ final_time / realtime_rate + (unpause_time - pause_time) atol = 0.2 # higher atol because of pause poll int

    # pause and terminate
    @async (sleep(pause_time); send_pause_message())
    @async (sleep(termination_time); send_terminate_message())
    elapsed = @elapsed animate(vis, state, sol, realtime_rate = realtime_rate)
    @show elapsed
    @test elapsed ≈ termination_time atol = 0.2 # higher atol because of pause poll int
    =#
end

@testset "Stiff integrator" begin
    urdf = joinpath(@__DIR__, "urdf", "Acrobot.urdf")
    mechanism = parse_urdf(Float64, urdf)
    state = MechanismState(mechanism)
    seed!(1)
    rand!(state)
    x0 = Vector(state)
    final_time = 1.
    problem = ODEProblem(Dynamics(mechanism), state, (0., final_time))

    sol_nonstiff = solve(problem, Vern7(), abs_tol = 1e-10, dt = 0.05)
    sol_stiff = solve(problem, Rodas4P(), abs_tol = 1e-10, dt = 0.05)
    @test last(sol_nonstiff) ≈ last(sol_stiff) atol = 1e-2
end

# notebooks
include("test_notebooks.jl")

end
