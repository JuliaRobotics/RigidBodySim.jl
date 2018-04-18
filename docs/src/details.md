# Details

## Index


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

### User interface

```@docs
DiffEqBase.CallbackSet(::DrakeVisualizer.Visualizer, ::RigidBodyDynamics.MechanismState)
RigidBodySim.Visualization.visualize
RigidBodySim.Visualization.window
RigidBodySim.Visualization.animate
```

### Visualizers

#### RigidBodyTreeInspector

RigidBodySim provides an interface for the [RigidBodyTreeInspector.jl](https://github.com/rdeits/RigidBodyTreeInspector.jl)
visualizer, built on top of [Director](https://github.com/RobotLocomotion/director).

The RigidBodyTreeInspector visualizer handle is a `DrakeVisualizer.Visualizer`. The easiest way to create a `Visualizer` for a robot is
from a URDF, which can be parsed by RigidBodyTreeInspector's `parse_urdf` function.

```@docs
window(vis::DrakeVisualizer.Visualizer)
```

#### MeshCatMechanisms

RigidBodySim has support for the [MeshCatMechanisms.jl](https://github.com/JuliaRobotics/MeshCatMechanisms.jl) `MechanismVisualizer`. Support is currently non-interactive, i.e., the simulation cannot be paused or terminated through the visualizer.

### [Visualizer interface](@id vis_interface)

RigidBodySim visualizers must implement the following interface:

* [`RigidBodySim.Visualization.VisualizerInterface.visualize`](@ref)
* [`RigidBodySim.Visualization.VisualizerInterface.window`](@ref)

In addition, visualizers should also implement the following:

```@docs
RigidBodySim.Visualization.VisualizerInterface.SimulationCommands
RigidBodySim.Visualization.VisualizerInterface.isinteractive
```

## Utilities

```@docs
configuration_renormalizer
```
