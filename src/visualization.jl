module Visualization

# user-side functionality
export
    SimulationControls,
    GUI,
    setanimation!

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


using MeshCatMechanisms

import MeshCat
import WebIO

using Printf: @sprintf
using DiffEqBase: DiscreteCallback, ODESolution, CallbackSet, u_modified!, terminate!
using RigidBodyDynamics: Mechanism, MechanismState, normalize_configuration!, configuration
using RigidBodyDynamics: num_positions, num_velocities
using MeshCat: Animation, atframe, setanimation!
using MeshCatMechanisms: setanimation!
using Observables: Observable
using InteractBase: Widget, button, observe
using WebIO: render, node, Node
using Blink: Window, body!, title

using DataStructures: top

mechanism_visualizer(vis::MechanismVisualizer) = vis

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

function TransformPublisher(vis; max_fps = 60.)
    last_update_time = Ref(-Inf)
    condition = let last_update_time = last_update_time, min_Δt = 1 / max_fps
        function (u, t, integrator)
            last_time_step = length(integrator.opts.tstops) == 1 && t == top(integrator.opts.tstops)
            last_time_step || time() - last_update_time[] >= min_Δt
        end
    end
    mvis = mechanism_visualizer(vis)
    num_states = num_positions(mvis.state) + num_velocities(mvis.state)
    action = let vis = vis, last_update_time = last_update_time, num_states = num_states
        function (integrator)
            last_update_time[] = time()
            copyto!(vis, view(integrator.u, 1 : num_states))
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
function WebIO.render(controls::SimulationControls)
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
            style = Dict(:display => "flex", :flexWrap => "wrap", :minHeight => "2.5em", :overflow => "hidden")
        )
    )
end

function Base.open(controls::SimulationControls, window::Window)
    size(window, 300, 70)
    title(window, "RigidBodySim controls")
    body!(window, render(controls))
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

struct GUI{V}
    visualizer::V
    controls::SimulationControls
    usernode::Union{Node, Nothing}
end

"""
Create a new RigidBodySim graphical user interface given a visualizer (e.g. a `MeshCatMechanisms.MechanismVisualizer`).

The visualizer must support:

* `Base.copyto!(visualizer, state::Union{MechanismState, AbstractVector})`
* `Base.wait(visualizer)`
* `MeshCatMechanisms.visualizer(visualizer)`, which should return a `MeshCat.Visualizer`.

Use `open(gui)` to open the GUI in a standalone window.
"""
GUI(visualizer; usernode=nothing) = GUI(visualizer, SimulationControls(), usernode)

"""
Create a new RigidBodySim graphical user interface for the given Mechanism.
All arguments are passed on to the `MeshCatMechanisms.MechanismVisualizer` constructor.

Use `open(gui)` to open the GUI in a standalone window.
"""
GUI(mechanism::Mechanism, args...; usernode=nothing) = GUI(MechanismVisualizer(mechanism, args...); usernode=usernode)

function Base.open(gui::GUI, window::Window)
    title(window, "RigidBodySim")
    body = node(:div,
        WebIO.render(gui.controls),
        WebIO.iframe(MeshCatMechanisms.visualizer(mechanism_visualizer(gui.visualizer)).core, minHeight="0"),
        WebIO.render(gui.usernode),
        style = Dict(:display => "flex", :flexDirection => "column", :height => "100vh")
    )
    body!(window, body)
    wait(gui)
    nothing
end

Base.open(gui::GUI) = (window = Window(); open(gui, window); window)
Base.wait(gui::GUI) = wait(gui.visualizer)

"""
Create the DifferentialEquations.jl callbacks associated with the [`GUI`](@ref).

`max_fps` is the maximum number of frames per second (in terms of wall time) to draw. Default: `60.0`.
"""
CallbackSet(gui::GUI; max_fps=60) = CallbackSet(CallbackSet(gui.controls), TransformPublisher(gui.visualizer; max_fps = max_fps))

"""
Create the DifferentialEquations.jl callbacks needed for publishing to a
visualizer during simulation.

`max_fps` is the maximum number of frames per second (in terms of wall time) to draw. Default: `60.0`.
"""
CallbackSet(vis::MechanismVisualizer; max_fps = 60.) = CallbackSet(TransformPublisher(vis; max_fps = max_fps))


"""
Create a `MeshCat.Animation` from an `ODESolution` obtained by `solve!`ing an
`ODEProblem` created using this package.

* `vis` is a `MeshCatMechanisms.MechanismVisualizer`
* `sol` is a `DiffEqBase.ODESolution` obtained from a RigidBodySim.jl simulation.

Keyword arguments:

* `fps`: the frame rate to be used for keyframes.
* `realtime_rate`: can be used to slow down or speed up playback compared to wall time. A `realtime_rate` of `2`
  will result in playback that is sped up 2x. Default: `1`.
"""
function MeshCat.Animation(mvis::MechanismVisualizer, sol::ODESolution; fps::Number=30, realtime_rate::Number=1)
    t0, tf = first(sol.t), last(sol.t)
    animation = Animation(fps)
    # MeshCat animations don't support a realtime_rate option
    # (although it can be adjusted in the GUI), so we instead just
    # do some fps scaling.
    num_frames = floor(Int, (tf - t0) * fps / realtime_rate)
    for frame in 0 : num_frames
        time = t0 + frame * realtime_rate / fps
        u = sol(time)
        q = view(u, 1 : num_positions(mvis.state)) # TODO: make nicer
        atframe(animation, frame) do
            set_configuration!(mvis, q)
        end
    end
    return animation
end

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
urdf = joinpath(dirname(pathof(RigidBodySim)), "..", "test", "urdf", "Acrobot.urdf")
mechanism = parse_urdf(Float64, urdf)
state = MechanismState(mechanism)
set_configuration!(state, [0.1; 0.2])
problem = ODEProblem(Dynamics(mechanism), state, (0., 2.))
sol = solve(problem, Vern7())
vis = MechanismVisualizer(mechanism, URDFVisuals(urdf))
# open(vis, Window()) # uncomment to open the visualizer window
setanimation!(vis, sol; realtime_rate = 0.5);

# output

```
"""
function MeshCat.setanimation!(mvis::MechanismVisualizer, sol::ODESolution;
        max_fps::Number = 60., realtime_rate::Number = 1., pause_pollint = nothing)
    # TODO: deprecate.
    if pause_pollint !== nothing
        warn("pause_pollint is no longer used. You can control the animation directly from the visualizer.")
    end
    @assert max_fps > 0
    @assert 0 < realtime_rate < Inf
    q0 = view(sol[1], 1 : num_positions(mvis.state)) # TODO: make nicer
    animation = MeshCat.Animation(mvis, sol; fps=max_fps, realtime_rate=realtime_rate)
    MeshCat.setanimation!(MeshCatMechanisms.visualizer(mvis), animation)
    set_configuration!(mvis.state, q0)
    nothing
end

end # module
