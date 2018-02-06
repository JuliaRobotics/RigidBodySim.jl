__precompile__()

module RigidBodySim

using Compat
using Reexport

@reexport using RigidBodyDynamics
@reexport using RigidBodyTreeInspector

using OrdinaryDiffEq
using DiffEqCallbacks
using LoopThrottle
using JSON
using LCMCore
using DrakeVisualizer

using DataStructures: top
using RigidBodyDynamics: configuration_derivative! # TODO: export from RigidBodyDynamics

using DocStringExtensions
@template (FUNCTIONS, METHODS, MACROS) =
    """
    $(SIGNATURES)
    $(DOCSTRING)
    """

@template (TYPES,) =
    """
    $(TYPEDEF)
    $(DOCSTRING)
    """

# Select DifferentialEquations exports
export
    ODEProblem, # from DiffEqBase
    init, # from DiffEqBase
    solve!, # from DiffEqBase
    solve, # from DiffEqBase
    Tsit5, # from OrdinaryDiffEq
    Vern7, # from OrdinaryDiffEq
    RK4, # from OrdinaryDiffEq
    CallbackSet # from DiffEqCallbacks

# Control
export
    PeriodicController,
    zero_control!

# Visualization
export
    any_open_visualizer_windows,
    new_visualizer_window

# Util
export
    configuration_renormalizer

include("control.jl")
include("core.jl")

include("lcmtypes/utime_t.jl")
include("lcmtypes/comms_t.jl")
include("visualization.jl")

include("util.jl")

end # module
