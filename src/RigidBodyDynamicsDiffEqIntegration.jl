module RigidBodyDynamicsDiffEqIntegration

using RigidBodyDynamics
import DiffEqBase

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

    DiffEqBase.ODEProblem(closed_loop_dynamics!, x, tspan)
end

end # module
