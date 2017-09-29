__precompile__()

module RigidBodyDynamicsDiffEqIntegration

# Callbacks
export
    VisualizerCallback,
    ConfigurationRenormalizationCallback

using RigidBodyDynamics
using RigidBodyTreeInspector
using DiffEqBase

using RigidBodyDynamics: configuration_derivative! # TODO: export from RigidBodyDynamics

function zero_control!(τ::AbstractVector, t, state::MechanismState)
    τ[:] = 0
end

function DiffEqBase.ODEProblem(state::MechanismState{X, M, C}, tspan, control! = zero_control!) where {X, M, C}
    result = DynamicsResult{C}(state.mechanism)
    τ = similar(velocity(state))
    x = state_vector(state) # TODO: Vector constructor

    # TODO: renormalization of configuration vector
    # TODO: running controller at a reduced rate
    # TODO: ability to affect external wrenches

    nq = num_positions(state)
    nv = num_velocities(state)
    ns = num_additional_states(state)

    closed_loop_dynamics! = function (t, x, ẋ)
        # TODO: unpack function in RigidBodyDynamics:
        q̇ = view(ẋ, 1 : nq)
        v̇ = view(ẋ, nq + 1 : nq + nv)
        ṡ = view(ẋ, nq + nv + 1 : nq + nv + ns)

        set!(state, x)
        control!(τ, t, state)
        dynamics!(result, state, τ)

        configuration_derivative!(q̇, state)
        v̇ .= result.v̇
        ṡ .= result.ṡ

        ẋ
    end

    ODEProblem(closed_loop_dynamics!, x, tspan)
end

function VisualizerCallback(state::MechanismState, vis::Visualizer; max_fps = 60.)
    min_Δt = 1 / max_fps
    last_update_time = Ref(-Inf)
    condition = (t, u, integrator) -> time() - last_update_time[] >= min_Δt
    visualize = function (integrator)
        set!(state, integrator.u)
        settransform!(vis, state)
        last_update_time[] = time()
        u_modified!(integrator, false)
    end
    DiscreteCallback(condition, visualize; save_positions = (false, false))
end

function ConfigurationRenormalizationCallback(state::MechanismState)
    let state = state # https://github.com/JuliaLang/julia/issues/15276
        renormalize = function (integrator)
            q = view(integrator.u, 1 : num_positions(state))
            set_configuration!(state, q)
            normalize_configuration!(state)
            q[:] = configuration(state)
            u_modified!(integrator, true)
        end
        DiscreteCallback((t, u, integrator) -> true, renormalize; save_positions = (false, false))
    end
end

end # module
