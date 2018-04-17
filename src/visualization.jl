module Visualization

# user-side functionality
export
    animate,
    window,
    visualize #, CallbackSet

"""
Contains the interface that should be implemented by specific viewer types.
"""
module VisualizerInterface

using DocStringExtensions

"""
$(TYPEDEF)

Stores visualizer-independent commands used to control the simulation.

A module providing a specific visualizer instance, say `MyVisualizer`, should
provide a `SimulationCommands` constructor method with the signature

```julia
SimulationCommands(vis::MyVisualizer)
```

which returns a `SimulationCommands` object and performs any visualizer-dependent
setup.
"""
mutable struct SimulationCommands
    terminate::Bool
    pause::Bool
    SimulationCommands() = (ret = new(); initialize!(ret); ret)
end

function initialize!(commands::SimulationCommands)
    commands.terminate = false
    commands.pause = false
end

"""
    visualize(vis, t::Number, state::MechanismState)

Visualize a `Mechanism` at the given time and in the given state using
visualizer `vis`.
"""
function visualize end

"""
    window(vis; kwargs...)

Create a new window for visualizer `vis`. Visualizer-specific keyword
arguments (`kwargs`) may be passed in.
"""
function window end

end # module

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

import RigidBodySim.Visualization.VisualizerInterface: window, visualize, SimulationCommands
import DiffEqBase: DiscreteCallback, ODESolution, CallbackSet, u_modified!, terminate!
import DataStructures
import RigidBodyDynamics: MechanismState, set!, normalize_configuration!
import LoopThrottle: @throttle

const DEFAULT_PAUSE_POLLINT = 0.05

function CommandHandler(commands::SimulationCommands; pause_pollint::Float64 = DEFAULT_PAUSE_POLLINT)
    condition = (u, t, integrator) -> true
    action = let commands = commands
        function (integrator)
            yield()
            while commands.pause && !commands.terminate
                sleep(pause_pollint)
            end
            commands.terminate && (commands.terminate = false; terminate!(integrator))
            u_modified!(integrator, false)
        end
    end
    initialize = (c, t, u, integrator) -> VisualizerInterface.initialize!(commands)
    DiscreteCallback(condition, action, initialize = initialize, save_positions=(false, false))
end

function TransformPublisher(state::MechanismState, vis; max_fps = 60.)
    last_update_time = Ref(-Inf)
    condition = let last_update_time = last_update_time, min_Δt = 1 / max_fps
        function (u, t, integrator)
            last_time_step = length(integrator.opts.tstops) == 1 && t == DataStructures.top(integrator.opts.tstops)
            last_time_step || time() - last_update_time[] >= min_Δt
        end
    end
    action = let state = state, vis = vis, last_update_time = last_update_time
        function (integrator)
            last_update_time[] = time()
            copy!(state, integrator.u)
            visualize(vis, integrator.t, state)
            u_modified!(integrator, false)
        end
    end
    DiscreteCallback(condition, action; save_positions = (false, false))
end

"""
Create the DifferentialEquations.jl callbacks needed for publishing to and receiving commands from a
visualizer during simulation.

`max_fps` is the maximum number of frames per second (in terms of wall time) to draw. Default: `60.0`.
"""
function CallbackSet(vis, state::MechanismState; max_fps = 60.)
    commands = SimulationCommands(vis)
    CallbackSet(TransformPublisher(state, vis; max_fps = max_fps), CommandHandler(commands))
end

"""
Play back a visualization of a `DiffEqBase.ODESolution` obtained from a RigidBodySim.jl simulation.

`vis` visualizer satisfying the RigidBodySim visualizer interface. `state` is a [`RigidBodyDynamics.MechanismState`](http://JuliaRobotics.github.io/RigidBodyDynamics.jl/release-0.4/mechanismstate.html#RigidBodyDynamics.MechanismState),
representing the state of the mechanism that was simulated, and will be modified during the visualization.

`animate` accepts the following keyword arguments:

* `max_fps`: the maximum number of frames per second to draw. Default: `60.0`.
* `realtime_rate`: can be used to slow down or speed up playback compared to wall time. A `realtime_rate` of `2`
  will result in playback that is sped up 2x. Default: `1.0`.
* `pause_pollint`: how often to poll for commands coming from the director window when playback is paused. Default: $DEFAULT_PAUSE_POLLINT.

# Examples

Visualizing the result of a simulation of the passive dynamics of an Acrobot (double pendulum) at half speed:

```jldoctest
julia> using RigidBodySim, RigidBodyDynamics, OrdinaryDiffEq, RigidBodyTreeInspector

julia> urdf = Pkg.dir("RigidBodySim", "test", "urdf", "Acrobot.urdf");

julia> mechanism = parse_urdf(Float64, urdf);

julia> state = MechanismState(mechanism);

julia> set_configuration!(state, [0.1; 0.2]);

julia> problem = ODEProblem(Dynamics(mechanism), state, (0., 2.));

julia> sol = solve(problem, Vern7());

julia> any_open_visualizer_windows() || (new_visualizer_window(); sleep(1));

julia> vis = Visualizer(mechanism, parse_urdf(urdf, mechanism));

julia> RigidBodySim.animate(vis, state, sol; realtime_rate = 0.5);
```
"""
function animate(vis, state::MechanismState, sol::ODESolution;
        max_fps::Number = 60., realtime_rate::Number = 1., pause_pollint = DEFAULT_PAUSE_POLLINT)
    @assert max_fps > 0
    @assert 0 < realtime_rate < Inf

    commands = SimulationCommands(vis)
    t0, tf = first(sol.t), last(sol.t)
    framenum = 0
    walltime0 = time()
    @throttle framenum while true
        t = min(tf, t0 + (time() - walltime0) * realtime_rate)
        while commands.pause && !commands.terminate
            sleep(pause_pollint)
            t0 = t
            walltime0 = time()
        end
        commands.terminate && (commands.terminate = false; break)
        x = sol(t)
        copy!(state, x)
        normalize_configuration!(state)
        visualize(vis, t, state)
        framenum += 1
        t == tf && break
        yield()
    end max_rate = max_fps
end

# Visualizer interfaces
include("visualizers/rigid_body_tree_inspector.jl")

end # module
