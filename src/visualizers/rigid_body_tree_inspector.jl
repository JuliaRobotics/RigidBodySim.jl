module RigidBodyTreeInspectorInterface

import RigidBodySim.Visualization.VisualizerInterface
import DrakeVisualizer
import JSON
import RigidBodyDynamics: MechanismState
import LCMCore: LCM, subscribe, publish
import RigidBodySim.LCMTypes: CommsT, UTimeT

const DRAKE_VISUALIZER_SCRIPT = joinpath(@__DIR__, "rigid_body_sim_visualizer_script.py")
const LCM_CONTROL_CHANNEL = "RIGID_BODY_SIM_CONTROL"
const LCM_TIME_CHANNEL = "RIGID_BODY_SIM_TIME"

function VisualizerInterface.SimulationCommands(lcm::LCM)
    commands = VisualizerInterface.SimulationCommands()
    subscribe(lcm, LCM_CONTROL_CHANNEL, (channel, msg) -> handle_control_msg(commands, msg), CommsT)
    commands
end

VisualizerInterface.SimulationCommands(vis::DrakeVisualizer.Visualizer) = VisualizerInterface.SimulationCommands(vis.core.lcm)

function handle_control_msg(commands::VisualizerInterface.SimulationCommands, msg::CommsT)
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

function VisualizerInterface.visualize(vis::DrakeVisualizer.Visualizer, t::Number, state::MechanismState)
    lcm = vis.core.lcm
    DrakeVisualizer.settransform!(vis, state)
    time_msg = UTimeT()
    time_msg.utime = floor(Int64, t * 1e3)
    publish(lcm, LCM_TIME_CHANNEL, time_msg)
    nothing
end

"""
Open a new [director](https://github.com/RobotLocomotion/director) visualizer window.

The director instance will be started with a script that handles communication between RigidBodySim.jl
and the director instance.

Supports the following keyword arguments:

* `reuse`: skip opening a new visualizer window if one is already open.
"""
function VisualizerInterface.window(vis::DrakeVisualizer.Visualizer; reuse = true)
    if !reuse || !DrakeVisualizer.any_open_windows()
        DrakeVisualizer.new_window(script = DRAKE_VISUALIZER_SCRIPT)
        sleep(0.5)
    end
    nothing
end

end # module
