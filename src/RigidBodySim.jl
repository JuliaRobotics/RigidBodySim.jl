__precompile__()

module RigidBodySim

using Compat

include("util.jl")
include("core.jl")
include("control.jl")
include("visualization.jl")

@reexport using JSExpr # FIXME: needed because of https://github.com/JuliaGizmos/JSExpr.jl/issues/13

@reexport using .Core
@reexport using .Control
@reexport using .Visualization

# Select DifferentialEquations exports
import DiffEqBase: ODEProblem, init, solve!, solve, CallbackSet
export ODEProblem, init, solve!, solve, CallbackSet

import DiffEqCallbacks: PeriodicCallback
export PeriodicCallback

import OrdinaryDiffEq: Tsit5, Vern7, RK4
export Tsit5, Vern7, RK4

end # module
