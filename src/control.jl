zero_control!(τ::AbstractVector, t, state) = τ[:] = 0

struct PeriodicController{Tau, T<:Number, C}
    τ::Vector{Tau}
    Δt::T
    control!::C
    docontrol::Base.RefValue{Bool}

    PeriodicController(τ::Vector{Tau}, Δt::T, control!::C) where {Tau, T<:Number, C} = new{Tau, T, C}(τ, Δt, control!, Ref(true))
    PeriodicController{Tau}(τlength::Integer, Δt::Number, control!) where {Tau} = PeriodicController(zeros(Tau, τlength), Δt, control!)
end

PeriodicController(state::MechanismState{X}, Δt::Number, control!) where {X} = PeriodicController{X}(num_velocities(state), Δt, control!)

function DiffEqCallbacks.PeriodicCallback(controller::PeriodicController; initialize = DiffEqBase.INITIALIZE_DEFAULT, save_positions = (false, false))
    periodic_initialize = let controller = controller, initialize = initialize
        function (c, t, u, integrator)
            controller.docontrol[] = true
            initialize(c, t, u, integrator)
        end
    end
    f = let controller = controller
            function (integrator)
                controller.docontrol[] = true
                u_modified!(integrator, false)
            end
    end
    PeriodicCallback(f, controller.Δt; initialize = periodic_initialize, save_positions = save_positions)
end

function (controller::PeriodicController)(τ::AbstractVector, t, state)
    if controller.docontrol[]
        controller.control!(controller.τ, t, state)
        controller.docontrol[] = false
    end
    τ[:] = controller.τ
end

function DiffEqBase.ODEProblem(state::MechanismState, tspan, controller::PeriodicController;
        callback = nothing,
        controller_initialize = DiffEqBase.INITIALIZE_DEFAULT,
        controller_save_positions = (false, false))
    controlcallback = PeriodicCallback(controller; initialize = controller_initialize, save_positions = controller_save_positions)
    _create_ode_problem(state, tspan, controller, CallbackSet(controlcallback, callback))
end
