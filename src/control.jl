module Control

export
    PeriodicController,
    SumController

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

using DiffEqBase: ODEProblem, CallbackSet, u_modified!
using DiffEqCallbacks: PeriodicCallback
using RigidBodyDynamics: MechanismState

import DiffEqBase
import DiffEqCallbacks

import RigidBodySim.Core: controlcallback

"""
A `PeriodicController` can be used to simulate a digital controller that runs at a
fixed rate (in terms of simulation time). It does so by performing a zero-order
hold on a provided control function.

`PeriodicController`s can be constructed using

```julia
PeriodicController(τ, Δt, control!; initialize = DiffEqBase.INITIALIZE_DEFAULT, save_positions = (false, false))
```

where `control!` is a controller satisfying the standard RigidBodySim controller signature
(`control!(τ, Δt, state)`), `Δt` is the simulation time interval between calls to the
`control!` function, and `τ` is used to call `control!`.
The `initialize` and `save_positions` keyword arguments are documented in
the [`DiscreteCallback`](http://docs.juliadiffeq.org/release-4.0/features/callback_functions.html#DiscreteCallback-1)
section of the DifferentialEquations documentation.

`PeriodicController`s are callable objects, and themselves fit the standard
RigidBodySim controller signature.

A `DiffEqCallbacks.PeriodicCallback` can be created from a `PeriodicController`,
and is used to stop ODE integration exactly every `Δt` seconds, so that the
`control!` function can be called. Typically, users will not have to explicitly create
this `PeriodicCallback`, as it is automatically created and
added to the `ODEProblem` when the `PeriodicController` is passed into the
RigidBodySim-provided `DiffEqBase.ODEProblem` constructor overload.

# Examples

In the following example, a `PeriodicController` is used to simulate
a digital PD controller running at a fixed rate of 200 Hz.

```jldoctest; output = false
julia> using RigidBodySim, RigidBodyDynamics, OrdinaryDiffEq

julia> mechanism = parse_urdf(Float64, joinpath(dirname(pathof(RigidBodySim)), "..", "test", "urdf", "Acrobot.urdf"));

julia> state = MechanismState(mechanism);

julia> set_configuration!(state, [0.1; 0.2]);

julia> controlcalls = Ref(0);

julia> pdcontrol!(τ, t, state) = (controlcalls[] += 1; τ .= -20 .* velocity(state) .- 100 .* configuration(state));

julia> τ = zero(velocity(state)); Δt = 1 / 200
0.005

julia> problem = ODEProblem(Dynamics(mechanism, PeriodicController(τ, Δt, pdcontrol!)), state, (0., 5.));

julia> sol = solve(problem, Tsit5());

julia> @assert all(x -> isapprox(x, 0, atol = 1e-4), sol.u[end]) # ensure state converges to zero

julia> controlcalls[]
1001
```
"""
struct PeriodicController{Tau<:AbstractVector, T<:Number, C, I}
    τ::Tau
    Δt::T
    control!::C
    initialize::I
    save_positions::Tuple{Bool, Bool}
    docontrol::Base.RefValue{Bool}
    start_time::Base.RefValue{T} # only used for checking that PeriodicCallback is correctly set up
    index::Base.RefValue{Int}
    function PeriodicController(τ::Tau, Δt::T, control!::C;
            initialize::I = DiffEqBase.INITIALIZE_DEFAULT,
            save_positions = (false, false)) where {Tau<:AbstractVector, T<:Number, C, I}
        new{Tau, T, C, I}(τ, Δt, control!, initialize, save_positions, Ref(true), Ref(T(NaN)), Ref(-1))
    end
end

function DiffEqCallbacks.PeriodicCallback(controller::PeriodicController)
    periodic_initialize = let controller = controller
        function (c, u, t, integrator)
            controller.docontrol[] = true
            controller.start_time[] = t
            controller.initialize(c, u, t, integrator)
        end
    end
    f = let controller = controller
        function (integrator)
            controller.docontrol[] = true
            controller.index[] += 1
            u_modified!(integrator, true) # see https://github.com/JuliaRobotics/RigidBodySim.jl/pull/126
        end
    end
    PeriodicCallback(f, controller.Δt; initialize = periodic_initialize, save_positions = controller.save_positions)
end

controlcallback(controller::PeriodicController) = PeriodicCallback(controller)

struct PeriodicControlFailure <: Exception
    Δt
    t
    last_control_time
 end

function Base.showerror(io::IO, e::PeriodicControlFailure)
    print(io,
        """
        Output of PeriodicController with Δt = $(e.Δt) was last updated at $(e.last_control_time), but current time is $(e.t).
        Please ensure that an associated `PeriodicCallback` was passed into the `ODEProblem` constructor.
        This is done automatically if the `ODEProblem` was created using a `Dynamics` object with the `PeriodicController` as the `control!` field,
        but if a `PeriodicController` is called from some other control function, the callback needs to be created manually using
        `PeriodicCallback(periodiccontroller)` and passed into the `ODEProblem` constructor as the `callback` keyword argument
        (or use `CallbackSet` to combine the `PeriodicCallback` with any other callbacks you may have).
        """)
end

function (controller::PeriodicController)(τ::AbstractVector, t, state)
    if controller.docontrol[]
        controller.control!(controller.τ, t, state)
        controller.docontrol[] = false
    end
    copyto!(τ, controller.τ)
    last_control_time = controller.start_time[] + controller.index[] * controller.Δt
    next_control_time = controller.start_time[] + (controller.index[]+1) * controller.Δt
    if t > next_control_time || t < last_control_time
        throw(PeriodicControlFailure(controller.Δt, t, last_control_time))
    end
    τ
end

"""
A `SumController` can be used to combine multiple controllers, summing the control
torques that each of these controllers produces.

# Examples

```jldoctest; output = false
julia> using RigidBodySim, RigidBodyDynamics

julia> mechanism = parse_urdf(Float64, joinpath(dirname(pathof(RigidBodySim)), "..", "test", "urdf", "Acrobot.urdf"));

julia> state = MechanismState(mechanism);

julia> c1 = (τ, t, state) -> τ .= t;

julia> c2 = (τ, t, state) -> τ .= 2 * t;

julia> sumcontroller = SumController(similar(velocity(state)), (c1, c2))

julia> τ = similar(velocity(state))

julia> controller(τ, 1.0, state);

julia> @assert all(τ .== 3.0);
```
"""
struct SumController{Tau<:AbstractVector, C<:Tuple}
    τbuffer::Tau
    controllers::C
end

@inline _add_controller_contributions(τ, τbuffer, t, state, controllers::Tuple{}) = nothing
@inline function _add_controller_contributions(τ, τbuffer, t, state, controllers::Tuple)
    controllers[1](τbuffer, t, state)
    τ .+= τbuffer
    _add_controller_contributions(τ, τbuffer, t, state, Base.tail(controllers))
    nothing
end

function (controller::SumController)(τ::AbstractVector, t::Number, state::MechanismState)
    τ .= 0
    _add_controller_contributions(τ, controller.τbuffer, t, state, controller.controllers)
    τ
end

controlcallback(controller::SumController) = CallbackSet(map(controlcallback, controller.controllers)...)

end # module
