const DRAKE_VISUALIZER_SCRIPT = joinpath(@__DIR__, "rigid_body_sim_visualizer_script.py")
const LCM_CONTROL_CHANNEL = "RIGID_BODY_SIM_CONTROL"
const LCM_TIME_CHANNEL = "RIGID_BODY_SIM_TIME"

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
    initialize = (c, t, u, integrator) -> commands.terminate = false
    DiscreteCallback(condition, action, initialize = initialize)
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
        function (t, u, integrator)
            last_time_step = length(integrator.opts.tstops) == 1 && t == top(integrator.opts.tstops)
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

function DiffEqBase.CallbackSet(vis::Visualizer, state::MechanismState; max_fps = 60.)
    commands = SimulationCommands(vis.core.lcm)
    CallbackSet(terminator(commands), transform_publisher(state, vis, vis.core.lcm; max_fps = max_fps))
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
    @throttle framenum while true
        if commands.terminate
            commands.terminate = false
            break
        end
        t = min(tf, t0 + (time() - walltime0) * realtime_rate)
        x = sol(t)
        set!(state, x)
        normalize_configuration!(state)
        visualize(vis, t, state)
        framenum += 1
        t == tf && break
        yield()
    end max_rate = max_fps
end
