# RigidBodySim

RigidBodySim provides Julia tools for simulation and visualization of systems of interconnected rigid bodies.

RigidBodySim is mainly built on top of the following packages:

* [RigidBodyDynamics](https://github.com/tkoolen/RigidBodyDynamics.jl) for construction of rigid body dynamics mechanisms and evaluation of their dynamics.
* [DifferentialEquations](https://github.com/JuliaDiffEq/DifferentialEquations.jl) packages, for numerical integration of the differential equations.
* [RigidBodyTreeInspector](https://github.com/rdeits/RigidBodyTreeInspector.jl) for visualization.

RigidBodySim does not attempt to abstract away its dependence on these packages, as doing so would necessarily expose only a subset of their functionality, and would require users familiar with these packages to learn yet another API. Instead, RigidBodySim simply plugs into existing functionality, providing convenience methods and extensions. Only this additional functionality is documented here, and we refer to the documentation for these packages for further information:

* RigidBodyDynamics: [![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://tkoolen.github.io/RigidBodyDynamics.jl/stable)
* DifferentialEquations: [![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](http://docs.juliadiffeq.org/stable/)

## Functionality

RigidBodySim currently provides the following key features:

* Convenient creation of [`DiffEqBase.ODEProblem`s](http://docs.juliadiffeq.org/release-4.0/types/ode_types.html) given a [`RigidBodyDynamics.MechanismState`](http://tkoolen.github.io/RigidBodyDynamics.jl/release-0.4/mechanismstate.html#RigidBodyDynamics.MechanismState) and, optionally, a controller.
* Integration with [RigidBodyTreeInspector](https://github.com/rdeits/RigidBodyTreeInspector.jl) for visualization, both during simulation and after. The visualizer window can also control (currently, pause or terminate) the simulation.
* Easy simulation of a digital controller running at a fixed rate (see [PeriodicController](@ref control)).

## Performance

Performance is fairly good. For example, we have used RigidBodySim to perform a 10-second simulation of the humanoid robot [Atlas (v5)](https://en.wikipedia.org/wiki/Atlas_(robot)) standing on flat ground with a controller running at 100 Hz in 13 seconds with the Tsitouras 5/4 variable-step integrator on a 3GHz machine.

## Installation

### Installing Julia

Download links and more detailed instructions are available on the [Julia](http://julialang.org/) website. The latest release of RigidBodySim.jl requires version 0.6 of Julia (the latest stable version).

!!! warning

    Do **not** use `apt-get` or `brew` to install Julia, as the versions provided by these package managers tend to be out of date.

### Installing RigidBodySim

To install the latest tagged release of RigidBodySim, simply run

```julia
Pkg.add("RigidBodySim")
```

To check out the master branch and work on the bleeding edge (generally, not recommended), additionally run

```julia
Pkg.checkout("RigidBodySim")
```

### First steps

To load the package, use the command:

```julia
using RigidBodySim
```

It is recommended to follow the [quick start guide](@ref quickstart) to get up to speed.

## Contents

```@contents
Pages = [
  "quickstart.md",
  "details.md"
]
Depth = 2
```

## Citing this library

```bibtex
@misc{rigidbodysimjl,
 author = "Twan Koolen and contributors",
 title = "RigidBodySim.jl",
 year = 2016,
 url = "https://github.com/tkoolen/RigidBodySim.jl"
}
```
