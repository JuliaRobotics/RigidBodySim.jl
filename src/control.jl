zero_control!(τ::AbstractVector, t, state) = τ[:] = 0

struct PeriodicController{Tau, T<:Number, C}
    τ::Vector{Tau}
    Δt::T
    control!::C
    docontrol::Base.RefValue{Bool}
    last_control_time::Base.RefValue{T} # only used for checking that PeriodicCallback is correctly set up

    PeriodicController(τ::Vector{Tau}, Δt::T, control!::C) where {Tau, T<:Number, C} = new{Tau, T, C}(τ, Δt, control!, Ref(true), Ref(typemin(T)))
    PeriodicController{Tau}(τlength::Integer, Δt::Number, control!) where {Tau} = PeriodicController(zeros(Tau, τlength), Δt, control!)
end

PeriodicController(state::MechanismState{X}, Δt::Number, control!) where {X} = PeriodicController{X}(num_velocities(state), Δt, control!)

function DiffEqCallbacks.PeriodicCallback(controller::PeriodicController; initialize = DiffEqBase.INITIALIZE_DEFAULT, kwargs...)
    periodic_initialize = function (c, t, u, integrator)
        T = typeof(controller.Δt)
        controller.last_control_time[] = ifelse(integrator.tdir > 0, typemin(T), typemax(T))
        initialize(c, t, u, integrator)
    end
    PeriodicCallback(integrator -> controller.docontrol[] = true, controller.Δt; initialize = periodic_initialize, kwargs...)
end

struct PeriodicControlFailure <: Exception
    Δt
    time_since_last_control
end

function Base.showerror(io::IO, e::PeriodicControlFailure)
    print(io, """
        output of PeriodicController with Δt = $(e.Δt) has not been updated for Δt = $(e.time_since_last_control).
        Please ensure that an associated PeriodicCallback was created and passed into OrdinaryDiffEq.solve or OrdinaryDiffEq.init.""")
end

function (controller::PeriodicController)(τ::AbstractVector, t, state)
    if controller.docontrol[]
        controller.control!(controller.τ, t, state)
        controller.docontrol[] = false
        controller.last_control_time[] = t
    end
    tprev = controller.last_control_time[]
    # TODO: https://github.com/JuliaDiffEq/OrdinaryDiffEq.jl/issues/211:
    # (tprev <= t <= tprev + controller.Δt) || throw(PeriodicControlFailure(controller.Δt, t - tprev))
    (t <= tprev + controller.Δt) || throw(PeriodicControlFailure(controller.Δt, t - tprev))
    τ[:] = controller.τ
end
