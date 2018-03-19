module Control

export
    PeriodicController

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

import RigidBodySim.Core: controlcallback
import DiffEqBase
import DiffEqBase: ODEProblem, CallbackSet, u_modified!
import DiffEqCallbacks: PeriodicCallback
import RigidBodyDynamics: MechanismState

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

```jldoctest
julia> using RigidBodySim, RigidBodyDynamics, OrdinaryDiffEq

julia> mechanism = parse_urdf(Float64, Pkg.dir("RigidBodySim", "test", "urdf", "Acrobot.urdf"));

julia> state = MechanismState(mechanism);

julia> set_configuration!(state, [0.1; 0.2]);

julia> controlcalls = Ref(0);

julia> pdcontrol!(τ, t, state) = (controlcalls[] += 1; τ .= -20 .* velocity(state) .- 100 .* configuration(state));

julia> τ = zeros(velocity(state)); Δt = 1 / 200
0.005

julia> problem = ODEProblem(Dynamics(mechanism, PeriodicController(τ, Δt, pdcontrol!)), state, (0., 5.));

julia> sol = solve(problem, Tsit5());

julia> sol.u[end]
4-element Array{Float64,1}:
 -3.25923e-5
 -1.67942e-5
  8.16715e-7
  1.55292e-8

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

    function PeriodicController(τ::Tau, Δt::T, control!::C;
            initialize::I = DiffEqBase.INITIALIZE_DEFAULT,
            save_positions = (false, false)) where {Tau<:AbstractVector, T<:Number, C, I}
        new{Tau, T, C, I}(τ, Δt, control!, initialize, save_positions, Ref(true))
    end
end

function PeriodicCallback(controller::PeriodicController)
    periodic_initialize = let controller = controller
        function (c, u, t, integrator)
            controller.docontrol[] = true
            controller.initialize(c, u, t, integrator)
        end
    end
    f = let controller = controller
        function (integrator)
            controller.docontrol[] = true
            u_modified!(integrator, false)
        end
    end
    PeriodicCallback(f, controller.Δt; initialize = periodic_initialize, save_positions = controller.save_positions)
end

controlcallback(controller::PeriodicController) = PeriodicCallback(controller)

function (controller::PeriodicController)(τ::AbstractVector, t, state)
    if controller.docontrol[]
        controller.control!(controller.τ, t, state)
        controller.docontrol[] = false
    end
    copy!(τ, controller.τ)
end

end # module
