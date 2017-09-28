using RigidBodyDynamicsDiffEqIntegration
using RigidBodyDynamics
using RigidBodyTreeInspector
using OrdinaryDiffEq
using Base.Test

@testset "compare to simulate" begin
    srand(1)

    urdf = Pkg.dir("RigidBodyDynamicsDiffEqIntegration", "test", "urdf", "Acrobot.urdf")
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

@testset "visualizer callback" begin
    srand(1)
    urdf = Pkg.dir("RigidBodyDynamicsDiffEqIntegration", "test", "urdf", "Acrobot.urdf")
    mechanism = parse_urdf(Float64, urdf)

    state = MechanismState(mechanism)
    rand!(state)

    # DrakeVisualizer.new_window(); sleep(3)
    vis = Visualizer()
    setgeometry!(vis, mechanism, parse_urdf(urdf, mechanism))
    settransform!(vis, state)

    problem = ODEProblem(state, (0., 100.))
    vis_callback = VisualizerCallback(state, vis)

    sol = solve(problem, Vern7(), abs_tol = 1e-10, dt = 0.05, callback = vis_callback)
end
