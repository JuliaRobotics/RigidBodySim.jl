module Visualization

# user-side functionality
export
    animate,
    window,
    visualize,
    SimulationControls

"""
Contains the interface that should be implemented by specific viewer types.
"""
module VisualizerInterface

struct SimulationCommands
    terminate::Bool
    pause::Bool
end

function SimulationCommands(vis)
    error("""
SimulationCommands() is discontinued. You can construct a set of controls,
open them in a new window, and create a callback from them by doing:

```
using Blink: Window

controls = SimulationControls()
open(controls, Window())
callback = CallbackSet(controls)
```
""")
end


function visualize(args...)
    error("""
visualize() is discontinued. You can render a state in a MeshCatMechanisms
visualizer with:

```
set_configuration!(vis, configuration(state))
```
""")
end

function window(args...; kw...)
    error("""
window() is discontinued. You can open a MeshCatMechanisms visualizer
in a new standalone window with:

```
using Blink: Window
open(vis, Window())
```
""")
end

function isinteractive(args...)
    error("""
isinteractive() is discontinued. Interactive controls are now separate
from the visualizer. See SimulationControls.
""")
end

end # module

using MeshCatMechanisms
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

import DiffEqBase: DiscreteCallback, ODESolution, CallbackSet, u_modified!, terminate!
import DataStructures
import RigidBodyDynamics: MechanismState, normalize_configuration!, configuration
using Observables: Observable
using InteractBase: Widget, button, observe
using WebIO: Node, Scope
using Blink: Window, body!

struct SimulationStatus
    terminate::Observable{Bool}
    pause::Observable{Bool}
end

function initialize!(status::SimulationStatus)
    status.terminate[] = false
    status.pause[] = false
end

const DEFAULT_PAUSE_POLLINT = 0.05

function CommandHandler(status::SimulationStatus; pause_pollint::Float64 = DEFAULT_PAUSE_POLLINT)
    condition = (u, t, integrator) -> true
    action = let status = status
        function (integrator)
            yield()
            while status.pause[] && !status.terminate[]
                sleep(pause_pollint)
            end
            status.terminate[] && (status.terminate[] = false; terminate!(integrator))
            u_modified!(integrator, false)
        end
    end
    initialize = (c, t, u, integrator) -> initialize!(status)
    DiscreteCallback(condition, action, initialize = initialize, save_positions=(false, false))
end

function TransformPublisher(vis::MechanismVisualizer; max_fps = 60.)
    last_update_time = Ref(-Inf)
    condition = let last_update_time = last_update_time, min_Δt = 1 / max_fps
        function (u, t, integrator)
            last_time_step = length(integrator.opts.tstops) == 1 && t == DataStructures.top(integrator.opts.tstops)
            last_time_step || time() - last_update_time[] >= min_Δt
        end
    end
    action = let vis = vis, last_update_time = last_update_time
        function (integrator)
            last_update_time[] = time()
            copy!(vis, integrator.u)
            u_modified!(integrator, false)
        end
    end
    DiscreteCallback(condition, action; save_positions = (false, false))
end

struct SimulationControls
    terminate::Widget{:button}
    pause::Widget{:button}
end

SimulationControls() = SimulationControls(button("Terminate"), button("Pause"))

function render_default(controls::SimulationControls)
    Node(:div,
        Node(:style, """
            .rigidbodysim-controls button {height: 100vh; width: 100%}
            .rigidbodysim-controls {
                display: flex;
                flex-direction: row;
            }
            .rigidbodysim-controls > div {
                flex-grow: 1;
                height: 100%
            }
        """),
        controls.pause,
        controls.terminate,
        attributes=Dict(:class => "rigidbodysim-controls")
    )
end

function Base.open(controls::SimulationControls, window::Window)
    size(window, 200, 50)
    body!(window, render_default(controls))
end

function SimulationStatus(controls::SimulationControls)
    terminate = map!(!iszero, Observable{Bool}(false), observe(controls.terminate))
    pause = map!(isodd, Observable{Bool}(false), observe(controls.pause))
    SimulationStatus(terminate, pause)
end

CallbackSet(controls::SimulationControls) = CommandHandler(SimulationStatus(controls))

@deprecate CallbackSet(vis, state::MechanismState; max_fps = 60.) CallbackSet(vis; max_fps = max_fps)

"""
Create the DifferentialEquations.jl callbacks needed for publishing to a
visualizer during simulation.

`max_fps` is the maximum number of frames per second (in terms of wall time) to draw. Default: `60.0`.
"""
CallbackSet(vis::MechanismVisualizer; max_fps = 60.) = CallbackSet(TransformPublisher(vis; max_fps = max_fps))

"""
Play back a visualization of a RigidBodySim.jl simulation.

Positional arguments:

* `vis` is a `MeshCatMechanisms.MechanismVisualizer`
* `sol` is a `DiffEqBase.ODESolution` obtained from a RigidBodySim.jl simulation.

`setanimation` accepts the following keyword arguments:

* `max_fps`: the maximum number of frames per second to draw. Default: `60.0`.
* `realtime_rate`: can be used to slow down or speed up playback compared to wall time. A `realtime_rate` of `2`
  will result in playback that is sped up 2x. Default: `1.0`.

# Examples

Visualizing the result of a simulation of the passive dynamics of an Acrobot (double pendulum) at half speed:

```jldoctest
using RigidBodySim, RigidBodyDynamics, MeshCatMechanisms, Blink
urdf = Pkg.dir("RigidBodySim", "test", "urdf", "Acrobot.urdf")
mechanism = parse_urdf(Float64, urdf)
state = MechanismState(mechanism)
set_configuration!(state, [0.1; 0.2])
problem = ODEProblem(Dynamics(mechanism), state, (0., 2.))
sol = solve(problem, Vern7())
vis = MechanismVisualizer(mechanism, URDFVisuals(urdf))
open(vis, Window())
setanimation!(vis, sol; realtime_rate = 0.5);

# output

```
"""
function MeshCatMechanisms.setanimation!(vis::MechanismVisualizer,
        sol::ODESolution;
        max_fps::Number = 60., realtime_rate::Number = 1., pause_pollint = nothing)
    if pause_pollint !== nothing
        warn("pause_pollint is no longer used. You can control the animation directly from the visualizer.")
    end
    @assert max_fps > 0
    @assert 0 < realtime_rate < Inf
    t0, tf = first(sol.t), last(sol.t)
    ts = linspace(t0, tf, (tf - t0) * max_fps)
    qs = let state = vis.state, sol = sol
        map(ts) do t
            x = sol(t)
            copy!(state, x)
            normalize_configuration!(state)
            copy(configuration(state))
        end
    end
    # MeshCat animations currently don't support a realtime_rate option
    # (although it can be adjusted in the GUI), so we instead just scale
    # the times
    setanimation!(vis, ts / realtime_rate, qs)
end

end # module
