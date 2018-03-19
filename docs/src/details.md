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

## Visualization

```@docs
DiffEqBase.CallbackSet(::DrakeVisualizer.Visualizer, ::RigidBodyDynamics.MechanismState)
any_open_visualizer_windows
new_visualizer_window
animate(::DrakeVisualizer.Visualizer, ::RigidBodyDynamics.MechanismState, ::DiffEqBase.ODESolution)
```

## Utilities

```@docs
configuration_renormalizer
```
