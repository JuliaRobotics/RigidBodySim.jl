# Details

## Index

```@index
Pages   = ["details.md"]
Order   = [:type, :function, :macro]
```

## ODE problem creation

```@docs
ODEProblem
```

## [Control](@id control)

```@docs
zero_control!
PeriodicController
```

## Visualization

```@docs
DiffEqBase.CallbackSet(::DrakeVisualizer.Visualizer, ::RigidBodyDynamics.MechanismState)
any_open_visualizer_windows
new_visualizer_window
RigidBodyTreeInspector.animate(::DrakeVisualizer.Visualizer, ::RigidBodyDynamics.MechanismState, ::DiffEqBase.ODESolution)
```

## Utilities

```@docs
configuration_renormalizer
```
