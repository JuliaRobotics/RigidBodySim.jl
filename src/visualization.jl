const DRAKE_VISUALIZER_SCRIPT = joinpath(@__DIR__, "rigid_body_sim_visualizer_script.py")
const LCM_CONTROL_CHANNEL = "RIGID_BODY_SIM_CONTROL"

mutable struct SimulationCommands
    terminate::Bool
    SimulationCommands() = new(false)
end

function SimulationCommands(lcm::LCM)
    commands = SimulationCommands()
    subscribe(lcm, LCM_CONTROL_CHANNEL, (channel, msg) -> handle_control_msg(commands, msg), DrakeVisualizer.Comms.CommsT)
    commands
end

function handle_control_msg(commands::SimulationCommands, msg::DrakeVisualizer.Comms.CommsT)
    @assert msg.format == "rigid_body_sim_json"
    @assert msg.format_version_major == 1
    @assert msg.format_version_minor == 1
    for (command, arg) in JSON.parse(IOBuffer(msg.data))
        if command == "terminate"
            commands.terminate = true
        else
            throw(ArgumentError("Command $command not recognized."))
        end
    end
end

function terminator(commands::SimulationCommands)
    condition = (t, u, integrator) -> (yield(); commands.terminate)
    action = integrator -> (terminate!(integrator); commands.terminate = false)
    DiscreteCallback(condition, action)
end

function transform_publisher(state::MechanismState, vis::Visualizer; max_fps = 60.)
    last_update_time = Ref(-Inf)
    condition = let last_update_time = last_update_time, min_Δt = 1 / max_fps
        (t, u, integrator) -> time() - last_update_time[] >= min_Δt
    end
    visualize = let state = state, vis = vis, last_update_time = last_update_time
        function (integrator)
            set!(state, integrator.u)
            settransform!(vis, state)
            last_update_time[] = time()
            u_modified!(integrator, false)
        end
    end
    DiscreteCallback(condition, visualize; save_positions = (false, false))
end

function DiffEqBase.CallbackSet(vis::Visualizer, state::MechanismState; max_fps = 60.)
    commands = SimulationCommands(vis.core.lcm)
    CallbackSet(terminator(commands), transform_publisher(state, vis; max_fps = max_fps))
end

any_open_visualizer_windows() = DrakeVisualizer.any_open_windows()
new_visualizer_window() = DrakeVisualizer.new_window(script = DRAKE_VISUALIZER_SCRIPT)

function RigidBodyTreeInspector.animate(vis::Visualizer, state::MechanismState, sol::ODESolution;
        max_fps::Number = 60., realtime_rate::Number = 1.)
    @assert max_fps > 0
    @assert 0 < realtime_rate < Inf

    commands = SimulationCommands(vis.core.lcm)
    t0, tf = first(sol.t), last(sol.t)
    framenum = 0
    walltime0 = time()
    @throttle framenum while (t = t0 + (time() - walltime0) * realtime_rate) <= tf
        if commands.terminate
            commands.terminate = false
            break
        end
        x = sol(min(t, tf))
        set!(state, x)
        normalize_configuration!(state)
        settransform!(vis, state)
        framenum += 1
        yield()
    end max_rate = max_fps
end
