__precompile__()

module RigidBodySim

using Reexport

import DiffEqBase: ODEProblem, init, solve!, solve, CallbackSet
import DiffEqCallbacks: PeriodicCallback
import OrdinaryDiffEq: Tsit5, Vern7, RK4

include("simcore.jl")
include("control.jl")
include("visualization.jl")

@reexport using JSExpr # FIXME: needed because of https://github.com/JuliaGizmos/JSExpr.jl/issues/13

@reexport using .SimCore
@reexport using .Control
@reexport using .Visualization

# Select DifferentialEquations exports
export
    ODEProblem, # from DiffEqBase
    init, # from DiffEqBase
    solve!, # from DiffEqBase
    solve, # from DiffEqBase
    Tsit5, # from OrdinaryDiffEq
    Vern7, # from OrdinaryDiffEq
    RK4, # from OrdinaryDiffEq
    CallbackSet, # from DiffEqBase
    PeriodicCallback # from DiffEqCallbacks

# # Core
# export
#     Dynamics,
#     controlcallback,
#     configuration_renormalizer,
#     zero_control!,
#     RealtimeRateLimiter

# # Control
# export
#     PeriodicController

# # Visualization
# export
#     animate,
#     window,
#     visualize,
#     SimulationControls,
#     GUI

end # module
