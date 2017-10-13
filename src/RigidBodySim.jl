__precompile__()

module RigidBodySim

export
    any_open_visualizer_windows,
    new_visualizer_window,
    configuration_renormalizer

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

include("core.jl")
include("visualization.jl")

end # module
