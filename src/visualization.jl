module Visualization

export
    any_open_visualizer_windows,
    new_visualizer_window,
    animate

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

import JSON
import LCMCore: LCM, subscribe, publish
import RigidBodySim.LCMTypes: CommsT, UTimeT
import DiffEqBase: DiscreteCallback, ODESolution, CallbackSet, u_modified!, terminate!
import DataStructures
import DrakeVisualizer
import DrakeVisualizer: Visualizer, settransform!
import RigidBodyDynamics: MechanismState, set!, normalize_configuration!
import LoopThrottle: @throttle

const DRAKE_VISUALIZER_SCRIPT = joinpath(@__DIR__, "rigid_body_sim_visualizer_script.py")
const LCM_CONTROL_CHANNEL = "RIGID_BODY_SIM_CONTROL"
const LCM_TIME_CHANNEL = "RIGID_BODY_SIM_TIME"
const DEFAULT_PAUSE_POLLINT = 0.05

mutable struct SimulationCommands
    terminate::Bool
    pause::Bool
    SimulationCommands() = (ret = new(); initialize!(ret); ret)
end

function initialize!(commands::SimulationCommands)
    commands.terminate = false
    commands.pause = false
end

function SimulationCommands(lcm::LCM)
    commands = SimulationCommands()
    subscribe(lcm, LCM_CONTROL_CHANNEL, (channel, msg) -> handle_control_msg(commands, msg), CommsT)
    commands
end

function handle_control_msg(commands::SimulationCommands, msg::CommsT)
    @assert msg.format == "rigid_body_sim_json"
    @assert msg.format_version_major == 1
    @assert msg.format_version_minor == 1
    for (command, arg) in JSON.parse(IOBuffer(msg.data))
        if command == "terminate"
            commands.terminate = true
        elseif command == "pause"
            commands.pause = !commands.pause
        else
            throw(ArgumentError("Command $command not recognized."))
        end
    end
end

function command_handler(commands::SimulationCommands; pause_pollint::Float64 = DEFAULT_PAUSE_POLLINT)
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
    initialize = (c, t, u, integrator) -> initialize!(commands)
    DiscreteCallback(condition, action, initialize = initialize, save_positions=(false, false))
end

function visualize(vis::Visualizer, t::Number, state::MechanismState)
    lcm = vis.core.lcm
    settransform!(vis, state)
    time_msg = UTimeT()
    time_msg.utime = floor(Int64, t * 1e3)
    publish(lcm, LCM_TIME_CHANNEL, time_msg)
    nothing
end

function transform_publisher(state::MechanismState, vis::Visualizer, lcm::LCM; max_fps = 60.)
    time_msg = UTimeT()
    last_update_time = Ref(-Inf)
    condition = let last_update_time = last_update_time, min_Δt = 1 / max_fps
        function (u, t, integrator)
            last_time_step = length(integrator.opts.tstops) == 1 && t == DataStructures.top(integrator.opts.tstops)
            last_time_step || time() - last_update_time[] >= min_Δt
        end
    end
    action = let state = state, vis = vis, last_update_time = last_update_time, time_msg = time_msg, lcm = lcm
        function (integrator)
            last_update_time[] = time()
            set!(state, integrator.u)
            visualize(vis, integrator.t, state)
            u_modified!(integrator, false)
        end
    end
    DiscreteCallback(condition, action; save_positions = (false, false))
end

"""
Create the DifferentialEquations.jl callbacks needed for publishing to and receiving commands from a
[director](https://github.com/RobotLocomotion/director) visualizer instance during simulation.

`max_fps` is the maximum number of frames per second (in terms of wall time) to draw. Default: `60.0`.
"""
function CallbackSet(vis::Visualizer, state::MechanismState; max_fps = 60.)
    commands = SimulationCommands(vis.core.lcm)
    CallbackSet(transform_publisher(state, vis, vis.core.lcm; max_fps = max_fps), command_handler(commands))
end

"""
Return whether any [director](https://github.com/RobotLocomotion/director) visualizer windows are open.
"""
any_open_visualizer_windows() = DrakeVisualizer.any_open_windows()

"""
Open a new [director](https://github.com/RobotLocomotion/director) visualizer window.

The director instance will be started with a script that handles communication between RigidBodySim.jl
and the director instance.
"""
new_visualizer_window() = DrakeVisualizer.new_window(script = DRAKE_VISUALIZER_SCRIPT)

"""
Play back a visualization of a `DiffEqBase.ODESolution` obtained from a RigidBodySim.jl simulation.

`vis` is a `DrakeVisualizer.Visualizer`. The easiest way to create a `Visualizer` for a robot is
from a URDF, which can be parsed by [RigidBodyTreeInspector.jl](https://github.com/rdeits/RigidBodyTreeInspector.jl)'s `parse_urdf` function.

`state` is a [`RigidBodyDynamics.MechanismState`](http://tkoolen.github.io/RigidBodyDynamics.jl/release-0.4/mechanismstate.html#RigidBodyDynamics.MechanismState),
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

julia> problem = ODEProblem(state, (0., 2.));

julia> sol = solve(problem, Vern7());

julia> any_open_visualizer_windows() || (new_visualizer_window(); sleep(1));

julia> vis = Visualizer(mechanism, parse_urdf(urdf, mechanism));

julia> animate(vis, state, sol; realtime_rate = 0.5);
```
"""
function animate(vis::Visualizer, state::MechanismState, sol::ODESolution;
        max_fps::Number = 60., realtime_rate::Number = 1., pause_pollint = DEFAULT_PAUSE_POLLINT)
    @assert max_fps > 0
    @assert 0 < realtime_rate < Inf

    commands = SimulationCommands(vis.core.lcm)
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
        set!(state, x)
        normalize_configuration!(state)
        visualize(vis, t, state)
        framenum += 1
        t == tf && break
        yield()
    end max_rate = max_fps
end

end # module
