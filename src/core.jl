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

using Compat
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

import DiffEqDiffTools
import ForwardDiff

"""
A 'zero' controller, i.e. one that sets all control torques to zero at all times.
"""
zero_control!(τ::AbstractVector, t, state) = τ .= 0

struct Dynamics{M, JointCollection, C, P}
    statecache::StateCache{M, JointCollection}
    τcache::SegmentedVectorCache{JointID, Base.OneTo{JointID}}
    resultcache::DynamicsResultCache{M}
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
function Dynamics(mechanism::Mechanism, control! = zero_control!; setparams! = (state, p) -> nothing)
    vranges = let state = MechanismState(mechanism) # just to get velocity ranges; TODO: consider adding method that creates this from a Mechanism
        ranges(velocity(state))
    end
    Dynamics(StateCache(mechanism), SegmentedVectorCache(vranges), DynamicsResultCache(mechanism), control!, setparams!)
end

Base.@pure cachevartype(::Type{<:MechanismState{<:Any, <:Any, C}}) where {C} = C

function (dynamics::Dynamics)(ẋ::AbstractVector, x::AbstractVector{X}, p, t::T) where {X, T}
    state = dynamics.statecache[X]
    C = cachevartype(typeof(state))
    Tau = promote_type(T, C)
    τ = dynamics.τcache[Tau]
    result = dynamics.resultcache[Tau]
    Compat.copyto!(state, x)
    dynamics.setparams!(state, p)
    dynamics.control!(τ, t, state)
    dynamics!(result, state, τ)
    Compat.copyto!(ẋ, result)
    ẋ
end

# Disable ForwardDiff tag mechanism to work around https://github.com/JuliaDiff/ForwardDiff.jl/issues/267
# The Tag type becomes too complicated due to the TypeSortedCollection type parameter, resulting in inference issues
ForwardDiff.Tag(::DiffEqDiffTools.TimeGradientWrapper{<:Dynamics}, ::Type{V}) where {V} = nothing
ForwardDiff.Tag(::DiffEqDiffTools.UJacobianWrapper{<:Dynamics}, ::Type{V}) where {V} = nothing
ForwardDiff.Tag(::DiffEqDiffTools.TimeDerivativeWrapper{<:Dynamics}, ::Type{V}) where {V} = nothing
ForwardDiff.Tag(::DiffEqDiffTools.UDerivativeWrapper{<:Dynamics}, ::Type{V}) where {V} = nothing
ForwardDiff.Tag(::DiffEqDiffTools.ParamJacobianWrapper{<:Dynamics}, ::Type{V}) where {V} = nothing

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
function ODEProblem(dynamics::Dynamics, x0::Union{AbstractVector, MechanismState}, tspan, p = nothing;
        callback = nothing, kwargs...)
    callbacks = CallbackSet(controlcallback(dynamics.control!), callback)
    ODEProblem{true}(dynamics, Vector(x0), tspan, p; callback = callbacks, kwargs...)
end

Base.@deprecate(ODEProblem(state::MechanismState, tspan, control! = zero_control!; callback = nothing),
    ODEProblem(Dynamics(state.mechanism, control!), state, tspan; callback = callback)
)

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
            Compat.copyto!(q, configuration(state))
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
