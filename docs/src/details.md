# Details

## ODE problem creation

```@docs
Dynamics
```

```@docs
ODEProblem(::RigidBodySim.Dynamics, ::Union{Base.AbstractVector, RigidBodyDynamics.MechanismState}, tspan)
```

## [Control](@id control)

```@docs
zero_control!
controlcallback
PeriodicController
```

## Visualization

RigidBodySim uses [MeshCatMechanisms.jl](https://github.com/JuliaRobotics/MeshCatMechanisms.jl) for 3D visualization.

```@docs
GUI
SimulationControls
DiffEqBase.CallbackSet(::MeshCatMechanisms.MechanismVisualizer)
DiffEqBase.CallbackSet(::RigidBodySim.GUI)
MeshCatMechanisms.setanimation!(::MeshCatMechanisms.MechanismVisualizer, ::DiffEqBase.ODESolution)
```

## Utilities

```@docs
configuration_renormalizer
RealtimeRateLimiter(; kwargs...)
```
