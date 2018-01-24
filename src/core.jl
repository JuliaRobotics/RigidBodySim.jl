function _create_ode_problem(state::MechanismState{X, M, C}, tspan, control!, callback) where {X, M, C}
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
    ODEProblem(closed_loop_dynamics!, x, tspan; callback = callback)
end

function DiffEqBase.ODEProblem(state::MechanismState, tspan, control! = zero_control!; callback = nothing)
    _create_ode_problem(state, tspan, control!, callback)
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
