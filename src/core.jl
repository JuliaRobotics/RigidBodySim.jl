function _create_ode_problem(state::MechanismState{X, M, C}, tspan, control!, callback) where {X, M, C}
    # TODO: running controller at a reduced rate
    # TODO: ability to affect external wrenches

    result = DynamicsResult{C}(state.mechanism)
    τ = similar(velocity(state))
    closed_loop_dynamics! = let state = state, result = result, τ = τ # https://github.com/JuliaLang/julia/issues/15276
        function (ẋ, x, p, t)
            # TODO: unpack function in RigidBodyDynamics:
            nq = num_positions(state)
            nv = num_velocities(state)
            ns = num_additional_states(state)

            q̇ = view(ẋ, 1 : nq)
            v̇ = view(ẋ, nq + 1 : nq + nv)
            ṡ = view(ẋ, nq + nv + 1 : nq + nv + ns)

            set!(state, x)
            configuration_derivative!(q̇, state)
            control!(τ, t, state)
            dynamics!(result, state, τ)
            v̇[:] = result.v̇
            ṡ[:] = result.ṡ

            ẋ
        end
    end
    x = state_vector(state) # TODO: Vector constructor
    ODEProblem(closed_loop_dynamics!, x, tspan; callback = callback)
end

"""
Create a `DiffEqBase.ODEProblem` representing the closed-loop dynamics of a
`RigidBodyDynamics.Mechanism`.

The initial state is given by the `state` argument (a [`RigidBodyDynamics.MechanismState`](http://tkoolen.github.io/RigidBodyDynamics.jl/release-0.4/mechanismstate.html#RigidBodyDynamics.MechanismState)).
The `state` argument will be modified during the simulation, as it is used to evaluate the dynamics.

The `control!` argument is a callable with the signature `control!(τ, t, state)`, where `τ` is the
torque vector to be set in the body of `control!`, `t` is the current time, and `state` is a `MechanismState` object.
By default, `control!` is [`zero_control!`](@ref).

The `callback` keyword argument can be used to pass in additional
[DifferentialEquations.jl callbacks](http://docs.juliadiffeq.org/release-4.0/features/callback_functions.html#Using-Callbacks-1).

# Examples

The following is a ten second simulation of the passive dynamics of an Acrobot (double pendulum) with a
`Vern7` integrator (see [DifferentialEquations.jl documentation](http://docs.juliadiffeq.org/release-4.0/solvers/ode_solve.html#Non-Stiff-Problems-1)).

```jldoctest
julia> using RigidBodySim, RigidBodyDynamics, OrdinaryDiffEq

julia> mechanism = parse_urdf(Float64, Pkg.dir("RigidBodySim", "test", "urdf", "Acrobot.urdf"))
Spanning tree:
Vertex: world (root)
  Vertex: base_link, Edge: base_link_to_world
    Vertex: upper_link, Edge: shoulder
      Vertex: lower_link, Edge: elbow
No non-tree joints.

julia> state = MechanismState(mechanism);

julia> set_configuration!(state, [0.1; 0.2]);

julia> problem = ODEProblem(state, (0., 10.))
DiffEqBase.ODEProblem with uType Array{Float64,1} and tType Float64. In-place: true
timespan: (0.0, 10.0)
u0: [0.1, 0.2, 0.0, 0.0]

julia> solution = solve(problem, Vern7());
```
"""
function DiffEqBase.ODEProblem(state::MechanismState, tspan, control! = zero_control!; callback = nothing)
    _create_ode_problem(state, tspan, control!, callback)
end
