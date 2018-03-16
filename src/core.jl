module Core

export
    configuration_renormalizer,
    zero_control!,
    RealtimeRateLimiter

using DocStringExtensions
@template (FUNCTIONS, METHODS, MACROS) =
    """
    $(SIGNATURES)
    $(DOCSTRING)
    """

@template (TYPES,) =
    """
    $(TYPEDEF)
    $(DOCSTRING)
    """

import DiffEqBase:
    ODEProblem, DiscreteCallback, u_modified!
import DiffEqCallbacks:
    PeriodicCallback
import RigidBodyDynamics:
    MechanismState, DynamicsResult,
    velocity, configuration,
    num_positions, num_velocities, num_additional_states,
    set!, set_configuration!, normalize_configuration!,
    configuration_derivative!, dynamics!, state_vector


"""
A 'zero' controller, i.e. one that sets all control torques to zero at all times.
"""
zero_control!(τ::AbstractVector, t, state) = τ[:] = 0

"""
Create a `DiffEqBase.ODEProblem` representing the closed-loop dynamics of a
`RigidBodyDynamics.Mechanism`.

The initial state is given by the `state` argument (a [`RigidBodyDynamics.MechanismState`](http://JuliaRobotics.github.io/RigidBodyDynamics.jl/release-0.4/mechanismstate.html#RigidBodyDynamics.MechanismState)).
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
function ODEProblem(state::MechanismState, tspan, control! = zero_control!; callback = nothing)
    create_ode_problem(state, tspan, control!, callback)
end

function create_ode_problem(state::MechanismState{X, M, C}, tspan, control!, callback) where {X, M, C}
    # TODO: running controller at a reduced rate
    # TODO: ability to affect external wrenches

    result = DynamicsResult{C}(state.mechanism)
    τ = similar(velocity(state))
    q̇ = similar(configuration(state))
    closed_loop_dynamics! = let state = state, result = result, τ = τ, q̇ = q̇ # https://github.com/JuliaLang/julia/issues/15276
        function (ẋ, x, p, t)
            # TODO: unpack function in RigidBodyDynamics:
            nq = num_positions(state)
            nv = num_velocities(state)
            ns = num_additional_states(state)

            set!(state, x)
            configuration_derivative!(q̇, state)
            control!(τ, t, state)
            dynamics!(result, state, τ)

            copy!(ẋ, 1, q̇, 1, nq)
            copy!(ẋ, nq + 1, result.v̇, 1, nv)
            copy!(ẋ, nq + nv + 1, result.ṡ, 1, ns)

            ẋ
        end
    end
    x = state_vector(state) # TODO: Vector constructor
    ODEProblem(closed_loop_dynamics!, x, tspan; callback = callback)
end

"""
`configuration_renormalizer` can be used to create a callback that projects the configuration
of a mechanism's state onto the configuration manifold. This may be necessary for mechanism's
with e.g. quaternion-parameterized orientations as part of their joint configuration vectors,
as numerical integration can cause the configuration to drift away from the unit norm constraints.

The callback is implemented as a [`DiffEqCallbacks.DiscreteCallback`](http://docs.juliadiffeq.org/release-4.0/features/callback_functions.html#DiscreteCallback-1)
By default, it is called at every integrator time step.
"""
function configuration_renormalizer(state::MechanismState, condition = (u, t, integrator) -> true)
    renormalize = let state = state # https://github.com/JuliaLang/julia/issues/15276
        function (integrator)
            q = view(integrator.u, 1 : num_positions(state)) # TODO: allocates
            set_configuration!(state, q)
            normalize_configuration!(state)
            copy!(q, configuration(state))
            u_modified!(integrator, true)
        end
    end
    DiscreteCallback(condition, renormalize; save_positions = (false, false))
end

# TODO: move to DiffEqCallbacks:
mutable struct RealtimeRateLimiterState{T}
    simtime0::T
    walltime0::Float64
    reset::Bool
    RealtimeRateLimiterState{T}() where {T} = new{T}(zero(T), 0.0, true)
end

"""
    RealtimeRateLimiter(; max_rate = 1., poll_interval = 1 / 30; save_positions = (false, false))

A `DiscreteCallback` that limit the rate of integration so that integration time `t`
increases at a rate no higher than `max_rate` compared to wall time.

A `RealtimeRateLimiter` can be used, for example, if you want to simulate a physical system
including its timing characteristics. Specific use cases may include realtime animation
and user interaction during the simulation.

The `poll_interval` keyword argument can be used to control how often the integration is
stopped to check whether to sleep (and for how long). Specifically, this operation happens
every `poll_interval / max_rate` *in terms of integration time*, which corresponds to approximately
every `poll_interval` seconds *wall time* if `max_rate` is actually achieved.
"""
function RealtimeRateLimiter(; max_rate = 1., poll_interval = 1 / 60, save_positions = (false, false))
    T = promote_type(typeof(max_rate), typeof(poll_interval))
    state = RealtimeRateLimiterState{T}()
    limit_rate = let state = state, max_rate = max_rate, poll_interval = poll_interval # https://github.com/JuliaLang/julia/issues/15276
        function (integrator)
            simtime = integrator.t
            if state.reset
                state.simtime0 = simtime
                state.walltime0 = time()
                state.reset = false
            else
                Δsimtime = simtime - state.simtime0
                minΔwalltime = Δsimtime / max_rate
                Δwalltime = time() - state.walltime0
                sleeptime = Δsimtime / max_rate - Δwalltime
                if sleeptime > 0
                    sleep(sleeptime)
                end
                # Note: more accurate results can be achieved by never doing the following reset,
                # (since there's no accumulated drift), but if there's a slowdown in
                # integration rate at some point, the maximum rate would not be enforced after
                # the slowdown
                state.simtime0 = simtime
                state.walltime0 = time()
            end
            u_modified!(integrator, false)
        end
    end
    initialize = let state = state
        (c, u, t, integrator) -> (state.reset = true)
    end
    PeriodicCallback(limit_rate, poll_interval / max_rate; initialize = initialize, save_positions = save_positions)
end

end # module
