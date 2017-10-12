__precompile__()

module RigidBodySim

export
    any_open_visualizer_windows,
    new_visualizer_window,
    configuration_renormalizer

using Reexport

@reexport using RigidBodyDynamics
@reexport using OrdinaryDiffEq
@reexport using RigidBodyTreeInspector

using LoopThrottle
using JSON
using LCMCore
using DrakeVisualizer

using RigidBodyDynamics: configuration_derivative! # TODO: export from RigidBodyDynamics
using DrakeVisualizer.Comms: CommsT

const DRAKE_VISUALIZER_SCRIPT = joinpath(@__DIR__, "drake_visualizer", "rigid_body_sim_visualizer_script.py")
const LCM_CONTROL_CHANNEL = "RIGID_BODY_SIM_CONTROL"

function zero_control!(τ::AbstractVector, t, state::MechanismState)
    τ[:] = 0
end

function DiffEqBase.ODEProblem(state::MechanismState{X, M, C}, tspan, control! = zero_control!) where {X, M, C}
    # TODO: running controller at a reduced rate
    # TODO: ability to affect external wrenches

    result = DynamicsResult{C}(state.mechanism)
    τ = similar(velocity(state))
    closed_loop_dynamics! = let state = state, result = result, τ = τ # https://github.com/JuliaLang/julia/issues/15276
        function (t, x, ẋ)
            # TODO: unpack function in RigidBodyDynamics:
            nq = num_positions(state)
            nv = num_velocities(state)
            ns = num_additional_states(state)

            q̇ = view(ẋ, 1 : nq)
            v̇ = view(ẋ, nq + 1 : nq + nv)
            ṡ = view(ẋ, nq + nv + 1 : nq + nv + ns)

            set!(state, x)
            configuration_derivative!(q̇, state)
            control!(τ, t, state)
            dynamics!(result, state, τ)
            v̇[:] = result.v̇
            ṡ[:] = result.ṡ

            ẋ
        end
    end
    x = state_vector(state) # TODO: Vector constructor
    ODEProblem(closed_loop_dynamics!, x, tspan)
end

function configuration_renormalizer(state::MechanismState, condition = (t, u, integrator) -> true)
    renormalize = let state = state # https://github.com/JuliaLang/julia/issues/15276
        function (integrator)
            q = view(integrator.u, 1 : num_positions(state))
            set_configuration!(state, q)
            normalize_configuration!(state)
            q[:] = configuration(state)
            u_modified!(integrator, true)
        end
    end
    DiscreteCallback(condition, renormalize; save_positions = (false, false))
end

mutable struct SimulationCommands
    terminate::Bool
    SimulationCommands() = new(false)
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
    time_per_frame = realtime_rate / max_fps
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

end # module
