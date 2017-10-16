__precompile__()

module RigidBodySim

# Control
export
    PeriodicController,
    zero_control!

# Util
export
    configuration_renormalizer

# Visualization
export
    any_open_visualizer_windows,
    new_visualizer_window

# Temporary (use DiffEqCallbacks version once PR is accepted):
export PeriodicCallback

using Reexport

@reexport using RigidBodyDynamics
@reexport using OrdinaryDiffEq
@reexport using RigidBodyTreeInspector

using LoopThrottle
using JSON
using LCMCore
using DrakeVisualizer

using RigidBodyDynamics: configuration_derivative! # TODO: export from RigidBodyDynamics

include("periodic.jl")
include("control.jl")
include("core.jl")
include("visualization.jl")

end # module
