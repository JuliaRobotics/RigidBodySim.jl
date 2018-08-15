module Visualization

# user-side functionality
export
    SimulationControls,
    GUI,
    setanimation!

# deprecated:
export
    animate,
    window,
    visualize

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

using Compat
using DiffEqBase: DiscreteCallback, ODESolution, CallbackSet, u_modified!, terminate!
using RigidBodyDynamics: Mechanism, MechanismState, normalize_configuration!, configuration
using MeshCatMechanisms: setanimation!
using Observables: Observable
using InteractBase: Widget, button, observe
import WebIO
using WebIO: render
if isdefined(WebIO, :node) # TODO: remove once a new WebIO tag is in
    using WebIO: node
else
    using WebIO: Node
    const node = Node
end
using Blink: Window, body!, title
using CSSUtil: vbox
using Printf: @sprintf

using DataStructures: top

struct SimulationStatus
    terminate::Observable{Bool}
    pause::Observable{Bool}
    time::Observable{Float64}
end

function initialize!(status::SimulationStatus)
    status.terminate[] = false
    status.pause[] = false
    status.time[] = 0.0
end

const TIME_DISPLAY_INTERVAL = 1e-2
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
            if abs(status.time[] - integrator.t) >= TIME_DISPLAY_INTERVAL
                status.time[] = integrator.t
            end
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
            last_time_step = length(integrator.opts.tstops) == 1 && t == top(integrator.opts.tstops)
            last_time_step || time() - last_update_time[] >= min_Δt
        end
    end
    action = let vis = vis, last_update_time = last_update_time
        function (integrator)
            last_update_time[] = time()
            Compat.copyto!(vis, integrator.u)
            u_modified!(integrator, false)
        end
    end
    DiscreteCallback(condition, action; save_positions = (false, false))
end

struct SimulationControls
    terminate::Widget{:button}
    pause::Widget{:button}
    time::Observable{HTML{String}}
end

"""
Create a new `SimulationControls` object, which may be used to pause and terminate the simulation.

The controls can be displayed in a standalone window using `open(controls, Blink.Window())`.
"""
SimulationControls() = SimulationControls(button(""), button(""), Observable(HTML("0.0")))

# Icons taken from the freely available set at https://icons8.com/color-icons/
function render_default(controls::SimulationControls)
    node(:div,
        node(:style, """
            .rigidbodysim-controls button {
                height: 2.5em;
                width: 2.5em;
                margin: 0.1em;
                padding: 0.2em 0.2em;
                image-rendering: pixelated;
                user-select: none;
            }
            .rigidbodysim-controls button:active {
                filter: brightness(50%);
            }
            .rigidbodysim-controls-pause button {
                background: url(data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAADAAAAAwCAYAAABXAvmHAAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAAJcEhZcwAADsMAAA7DAcdvqGQAAADUSURBVGhD7Y6xCQMBEMN+lSydRTNAgkGdVR8ELFAlF37GGGOMMcb4T17vz9ckF7aN5HvsTCQXto3ke+xMJBe2jeR77EwkF7aN5HvsTCQXto3ke+xMJBe2jeR77EwkF7aN5HvsTCQXto3ke+xMJBe2jeR77EwkF7aN5HvsTCQXto3ke+xMJBe2jeR77EwkF7aN5HvsTCQXto3ke+xMJBe2jeR77EwkF7aN5HvsTCQXto3ke+xMJBe2jeR77EwkF7aN5HvsTCQXto3kMcYYY4wx/orn+QEf136Pgv8IWQAAAABJRU5ErkJggg==) no-repeat;
                background-size: cover;
            }
            .rigidbodysim-controls-terminate button {
                background: url(data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAADAAAAAwCAYAAABXAvmHAAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAAJcEhZcwAADsMAAA7DAcdvqGQAAACQSURBVGhD7c6xCcBAAMPAXyz7j5U0Kt05GB50oNb4SJIkSfd5n/P+GbM76UQTszvpRBOzO+lEE7M76UQTszvpRBOzO+lEE7M76UQTszvpRBOzO+lEE7M76UQTszvpRBOzO+lEE7M76UQTszvpRBOzO+lEE7M76UQTszvpRBOzO+lEE7M76UQTs5IkSdI1zvkACnV8XnyMZlcAAAAASUVORK5CYII=) no-repeat;
                background-size: cover;
            }
            .rigidbodysim-controls > div {flex-grow: 0; ; }
        """),
        node(:div,
            node(:div,
                node(:div,
                    node(:div, "Time:"),
                    render(controls.time),
                    style = Dict(:fontSize => "10pt", :fontFamily => "sans-serif",
                                :userSelect => "none", :cursor => "default",
                                :height => "2.5em", :width => "8em", :marginLeft => "0.5em", :marginRight => "0.5em", :lineHeight => "2.5em",
                                :display => "flex", :justifyContent => "space-between")
                ),
                node(:div, controls.pause, attributes = Dict(:class => "rigidbodysim-controls-pause")),
                node(:div, controls.terminate, attributes = Dict(:class => "rigidbodysim-controls-terminate")),
                attributes = Dict(:class => "rigidbodysim-controls"),
                style = Dict(:display => "flex", :flexWrap => "wrap")
            )
        ),
        style = Dict(:overflow => "hidden")
    )
end

function Base.open(controls::SimulationControls, window::Window)
    size(window, 300, 55)
    title(window, "RigidBodySim controls")
    body!(window, render_default(controls))
    nothing
end

function SimulationStatus(controls::SimulationControls)
    terminate = map!(!iszero, Observable(false), observe(controls.terminate))
    pause = map!(isodd, Observable(false), observe(controls.pause))
    time = Observable(0.0)
    status = SimulationStatus(terminate, pause, time)
    map!(t -> HTML(@sprintf("%4.2f", t)), controls.time, status.time)
    status
end

CallbackSet(controls::SimulationControls) = CommandHandler(SimulationStatus(controls))

struct GUI
    visualizer::MechanismVisualizer
    controls::SimulationControls
end

"""
Create a new RigidBodySim graphical user interface from a `MeshCatMechanisms.MechanismVisualizer`.

Use `open(gui)` to open the GUI in a standalone window.
"""
GUI(visualizer::MechanismVisualizer) = GUI(visualizer, SimulationControls())

"""
Create a new RigidBodySim graphical user interface for the given Mechanism.
All arguments are passed on to the `MeshCatMechanisms.MechanismVisualizer` constructor.

Use `open(gui)` to open the GUI in a standalone window.
"""
GUI(mechanism::Mechanism, args...) = GUI(MechanismVisualizer(mechanism, args...))

function Base.open(gui::GUI, window::Window)
    title(window, "RigidBodySim")
    # TODO: vbox(render_default(gui.controls), iframe(gui.visualizer.visualizer.core))
    body!(window, vbox(render_default(gui.controls), gui.visualizer.visualizer.core))
    wait(gui)
    nothing
end

Base.open(gui::GUI) = open(gui, Window())
Base.wait(gui::GUI) = wait(gui.visualizer)

"""
Create the DifferentialEquations.jl callbacks associated with the [GUI](@ref).

`max_fps` is the maximum number of frames per second (in terms of wall time) to draw. Default: `60.0`.
"""
CallbackSet(gui::GUI; max_fps=60) = CallbackSet(CallbackSet(gui.controls), CallbackSet(gui.visualizer; max_fps = max_fps))

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

```jldoctest; output = false
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
function MeshCatMechanisms.setanimation!(vis::MechanismVisualizer, sol::ODESolution;
        max_fps::Number = 60., realtime_rate::Number = 1., pause_pollint = nothing)
    if pause_pollint !== nothing
        warn("pause_pollint is no longer used. You can control the animation directly from the visualizer.")
    end
    @assert max_fps > 0
    @assert 0 < realtime_rate < Inf
    t0, tf = first(sol.t), last(sol.t)
    ts = Compat.range(t0, stop=tf, step=round(Int, (tf - t0) * max_fps + 1))
    qs = let state = vis.state, sol = sol
        map(ts) do t
            x = sol(t)
            Compat.copyto!(state, x)
            normalize_configuration!(state)
            copy(configuration(state))
        end
    end
    # MeshCat animations currently don't support a realtime_rate option
    # (although it can be adjusted in the GUI), so we instead just scale
    # the times
    setanimation!(vis, ts / realtime_rate, qs)
end

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

end # module
