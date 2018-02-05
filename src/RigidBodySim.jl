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

using RigidBodyDynamics
using OrdinaryDiffEq
using DiffEqCallbacks
using RigidBodyTreeInspector

using LoopThrottle
using JSON
using LCMCore
using DrakeVisualizer

using DataStructures: top
using RigidBodyDynamics: configuration_derivative! # TODO: export from RigidBodyDynamics

include("lcmtypes/utime_t.jl")
include("lcmtypes/comms_t.jl")
include("control.jl")
include("core.jl")
include("visualization.jl")

end # module
