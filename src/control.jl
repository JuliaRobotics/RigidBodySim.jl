zero_control!(τ::AbstractVector, t, state) = τ[:] = 0

struct PeriodicController{Tau, T<:Number, C, I}
    τ::Vector{Tau}
    Δt::T
    control::C
    initialize::I
    save_positions::Tuple{Bool, Bool}
    docontrol::Base.RefValue{Bool}

    function PeriodicController(τ::Vector{Tau}, Δt::T, control::C;
            initialize::I = DiffEqBase.INITIALIZE_DEFAULT,
            save_positions = (false, false)) where {Tau, T<:Number, C, I}
        new{Tau, T, C, I}(τ, Δt, control, initialize, save_positions, Ref(true))
    end
end

function PeriodicController(state::MechanismState, Δt::Number, control; kwargs...)
    PeriodicController(zeros(velocity(state)), Δt, control; kwargs...)
end

function DiffEqCallbacks.PeriodicCallback(controller::PeriodicController)
    periodic_initialize = let controller = controller
        function (c, u, t, integrator)
            controller.docontrol[] = true
            controller.initialize(c, u, t, integrator)
        end
    end
    f = let controller = controller
        function (integrator)
            controller.docontrol[] = true
            u_modified!(integrator, false)
        end
    end
    PeriodicCallback(f, controller.Δt; initialize = periodic_initialize, save_positions = controller.save_positions)
end

function (controller::PeriodicController)(τ::AbstractVector, t, state)
    if controller.docontrol[]
        controller.control(controller.τ, t, state)
        controller.docontrol[] = false
    end
    τ[:] = controller.τ
end

function DiffEqBase.ODEProblem(state::MechanismState, tspan, controller::PeriodicController; callback = nothing)
    _create_ode_problem(state, tspan, controller, CallbackSet(PeriodicCallback(controller), callback))
end
