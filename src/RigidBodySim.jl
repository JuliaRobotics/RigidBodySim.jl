__precompile__()

module RigidBodySim

include("util.jl")
include("core.jl")
include("control.jl")
include("visualization.jl")

@reexport using .Core
@reexport using .Control
@reexport using .Visualization

# Select DifferentialEquations exports
using DiffEqBase: ODEProblem, init, solve!, solve, CallbackSet
export ODEProblem, init, solve!, solve, CallbackSet

using DiffEqCallbacks: PeriodicCallback
export PeriodicCallback

using OrdinaryDiffEq: Tsit5, Vern7, RK4
export Tsit5, Vern7, RK4

end # module
