"""
`configuration_renormalizer` can be used to create a callback that projects the configuration
of a mechanism's state onto the configuration manifold. This may be necessary for mechanism's
with e.g. quaternion-parameterized orientations as part of their joint configuration vectors,
as numerical integration can cause the configuration to drift away from the unit norm constraints.

The callback is implemented as a [`DiffEqCallbacks.DiscreteCallback`](http://docs.juliadiffeq.org/release-4.0/features/callback_functions.html#DiscreteCallback-1)
By default, it is called at every integrator time step.
"""
function configuration_renormalizer(state::MechanismState, condition = (u, t, integrator) -> true)
    renormalize = let state = state # https://github.com/JuliaLang/julia/issues/15276
        function (integrator)
            q = view(integrator.u, 1 : num_positions(state)) # TODO: allocates
            set_configuration!(state, q)
            normalize_configuration!(state)
            copy!(q, configuration(state))
            u_modified!(integrator, true)
        end
    end
    DiscreteCallback(condition, renormalize; save_positions = (false, false))
end
