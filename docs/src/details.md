# Details

## Index

```@index
Pages   = ["details.md"]
Order   = [:type, :function, :macro]
```

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

## Visualization interface

```@docs
DiffEqBase.CallbackSet(::DrakeVisualizer.Visualizer, ::RigidBodyDynamics.MechanismState)
window
animate
visualize
```

## Visualizers

TODO
```@docs
```

## Utilities

```@docs
configuration_renormalizer
```
