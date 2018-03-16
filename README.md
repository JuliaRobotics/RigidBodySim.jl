# RigidBodySim

[![Build Status](https://travis-ci.org/JuliaRobotics/RigidBodySim.jl.svg?branch=master)](https://travis-ci.org/JuliaRobotics/RigidBodySim.jl)
[![codecov.io](http://codecov.io/github/JuliaRobotics/RigidBodySim.jl/coverage.svg?branch=master)](http://codecov.io/github/JuliaRobotics/RigidBodySim.jl?branch=master)
[![Docs (latest)](https://img.shields.io/badge/docs-latest-blue.svg)](https://JuliaRobotics.github.io/RigidBodySim.jl/latest)
[![Docs (stable)](https://img.shields.io/badge/docs-stable-blue.svg)](https://JuliaRobotics.github.io/RigidBodySim.jl/stable)

RigidBodySim provides Julia tools for simulation and visualization of systems of interconnected rigid bodies (both passive and controlled), built on top of [RigidBodyDynamics](https://github.com/JuliaRobotics/RigidBodyDynamics.jl), [DifferentialEquations](https://github.com/JuliaDiffEq/DifferentialEquations.jl), and [RigidBodyTreeInspector](https://github.com/rdeits/RigidBodyTreeInspector.jl).

See the [latest documentation](https://JuliaRobotics.github.io/RigidBodySim.jl/latest) and the [quick start guide](https://nbviewer.jupyter.org/github/JuliaRobotics/RigidBodySim.jl/blob/master/notebooks/Quick%20start%20guide.ipynb) for more information and examples.

# Demo video

The video below shows Atlas walking using the MIT Robot Locomotion Group controller, simulated in realtime using RigidBodySim.jl.

[![Watch the demo video](https://user-images.githubusercontent.com/2585880/37498721-3c25bb8a-2896-11e8-8c14-91c97e46d0d0.png)](https://player.vimeo.com/video/260344845)

Off-screen, commands are being given to the robot using a separate user interface. The controller is unfortunately hard to set up, so a hands-on version of this demo is currently not available.
