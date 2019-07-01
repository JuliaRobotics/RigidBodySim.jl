module Core

export
    Dynamics,
    controlcallback,
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

import DiffEqBase
import RigidBodyDynamics

using DiffEqBase:
    ODEProblem, DiscreteCallback, u_modified!, CallbackSet
using DiffEqCallbacks:
    PeriodicCallback
using RigidBodyDynamics:
    Mechanism,
    MechanismState, DynamicsResult,
    StateCache, DynamicsResultCache, SegmentedVectorCache,
    JointID,
    velocity, configuration,
    num_positions, num_velocities, num_additional_states,
    set_configuration!, normalize_configuration!,
    configuration_derivative!, dynamics!,
    ranges

using RigidBodyDynamics.Contact:
    ContactModel,
    SoftContactState,
    SoftContactResult,
    contact_dynamics!

"""
A 'zero' controller, i.e. one that sets all control torques to zero at all times.
"""
zero_control!(τ::AbstractVector, t, state) = τ .= 0

struct Dynamics{M, JointCollection, C, P, CS, CR}
    statecache::StateCache{M, JointCollection}
    τcache::SegmentedVectorCache{JointID, Base.OneTo{JointID}}
    resultcache::DynamicsResultCache{M}
    contact_state::CS
    contact_result::CR
    control!::C
    setparams!::P
end

"""
Create a `Dynamics` object, representing either the passive or closed-loop dynamics of a `RigidBodyDynamics.Mechanism`.

The `control!` argument is a callable with the signature `control!(τ, t, state)`, where `τ` is the
torque vector to be set in the body of `control!`, `t` is the current time, and `state` is a `MechanismState` object.
By default, `control!` is [`zero_control!`](@ref) (resulting in the passive dynamics).

The `setparams!` keyword argument is a callable with the signature `setparams!(state, p)` where `state` is a
`MechanismState` and `p` is a vector of parameters, as used in OrdinaryDiffEq.jl.
"""
function Dynamics(mechanism::Mechanism, control! = zero_control!; setparams! = (state, p) -> nothing, contact_model::Union{ContactModel, Nothing} = nothing)
    vranges = let state = MechanismState(mechanism) # just to get velocity ranges; TODO: consider adding method that creates this from a Mechanism
        ranges(velocity(state))
    end
    if contact_model === nothing
        contact_state = nothing
        contact_result = nothing
    else
        contact_state = SoftContactState{Float64}(contact_model)
        contact_result = SoftContactResult{Float64}(mechanism, contact_model)
    end
    Dynamics(StateCache(mechanism), SegmentedVectorCache(vranges), DynamicsResultCache(mechanism), contact_state, contact_result, control!, setparams!)
end

Base.@pure cachevartype(::Type{<:MechanismState{<:Any, <:Any, C}}) where {C} = C

function (dynamics::Dynamics)(ẋ::AbstractVector, x::AbstractVector{X}, p, t::T) where {X, T}
    has_contact = dynamics.contact_state !== nothing
    state = dynamics.statecache[X]
    nq = num_positions(state)
    nv = num_velocities(state)
    C = cachevartype(typeof(state))
    Tau = promote_type(T, C)
    τ = dynamics.τcache[Tau]
    result = dynamics.resultcache[Tau]
    if has_contact
        copyto!(state, view(x, 1 : nq + nv))
        copyto!(dynamics.contact_state.x, view(x, nq + nv + 1 : length(x)))
    else
        copyto!(state, x)
    end
    dynamics.setparams!(state, p)
    dynamics.control!(τ, t, state)
    if dynamics.contact_state !== nothing
        contact_dynamics!(dynamics.contact_result, dynamics.contact_state, state)
        dynamics!(result, state, τ, dynamics.contact_result.wrenches)
        copyto!(view(ẋ, 1 : nq + nv), result)
        copyto!(view(ẋ, nq + nv + 1 : length(ẋ)), dynamics.contact_result.ẋ)
    else
        dynamics!(result, state, τ)
        copyto!(ẋ, result)
    end
    ẋ
end

vectorize(x::AbstractVector) = x
vectorize(x::MechanismState) = Vector(x)
vectorize(x::Tuple{<:MechanismState, <:SoftContactState}) = [Vector(x[1]); x[2].x]

"""
Can be used to create a callback associated with a given controller.
"""
controlcallback(control!::Any) = nothing

"""
Create a `DiffEqBase.ODEProblem` associated with the dynamics of a `RigidBodyDynamics.Mechanism`.

The initial state `x0` can be either a [`RigidBodyDynamics.MechanismState`]
(http://JuliaRobotics.github.io/RigidBodyDynamics.jl/stable/mechanismstate.html#RigidBodyDynamics.MechanismState)),
or an `AbstractVector` containing the initial state represented as `[q; v; s]`, where `q` is the configuration vector,
`v` is the velocity vector, and `s` is the vector of additional states.

The `callback` keyword argument can be used to pass in additional [DifferentialEquations.jl callbacks]
(http://docs.juliadiffeq.org/stable/features/callback_functions.html#Using-Callbacks-1).
"""
function ODEProblem(dynamics::Dynamics, x0, tspan, p = nothing;
        callback = nothing, kwargs...)
    callbacks = CallbackSet(controlcallback(dynamics.control!), callback)
    ODEProblem{true}(dynamics, Vector(vectorize(x0)), tspan, p; callback = callbacks, kwargs...)
end

"""
`configuration_renormalizer` can be used to create a callback that projects the configuration
of a mechanism's state onto the configuration manifold. This may be necessary for mechanism's
with e.g. quaternion-parameterized orientations as part of their joint configuration vectors,
as numerical integration can cause the configuration to drift away from the unit norm constraints.

The callback is implemented as a [`DiffEqCallbacks.DiscreteCallback`]
(http://docs.juliadiffeq.org/stable/features/callback_functions.html#DiscreteCallback-1)
By default, it is called at every integrator time step.
"""
function configuration_renormalizer(state::MechanismState, condition = (u, t, integrator) -> true)
    renormalize = let state = state # https://github.com/JuliaLang/julia/issues/15276
        function (integrator)
            q = view(integrator.u, 1 : num_positions(state)) # TODO: allocates
            set_configuration!(state, q)
            normalize_configuration!(state)
            copyto!(q, configuration(state))
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

A `DiscreteCallback` that limits the rate of integration so that integration time `t`
increases at a rate no higher than `max_rate` compared to wall time.

A `RealtimeRateLimiter` can be used, for example, if you want to simulate a physical system
including its timing characteristics. Specific use cases may include realtime animation
and user interaction during the simulation.

The `poll_interval` keyword argument can be used to control how often the integration is
stopped to check whether to sleep (and for how long). Specifically, this operation happens
every `poll_interval / max_rate` *in terms of integration time*, which corresponds to approximately
every `poll_interval` seconds *wall time* if `max_rate` is actually achieved.
"""
function RealtimeRateLimiter(; max_rate = 1., poll_interval = 1 / 60, save_positions = (false, false), reset_interval = 1.0)
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
                sleeptime = minΔwalltime - Δwalltime
                if sleeptime > 1e-3 # minimum time for `sleep` function
                    sleep(sleeptime)
                end
                if Δwalltime > reset_interval
                    # Reset every once in a while. If we don't do this, a slowdown in simulation
                    # rate at some point will cause the maximum rate to not be enforced until the
                    # simulation catches up. Don't do this too often to get more accurate results
                    # (drift doesn't accumulate as quickly).
                    state.simtime0 = simtime
                    state.walltime0 = time()
                end
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
