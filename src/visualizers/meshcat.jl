module MeshCatInterface

module Modes
@enum VisualizerMode BrowserWindow IJuliaCell
end

using .Modes
using DocStringExtensions
import RigidBodySim.Visualization.VisualizerInterface
import RigidBodyDynamics: MechanismState, configuration
import MeshCatMechanisms: MechanismVisualizer, set_configuration!

function VisualizerInterface.SimulationCommands(vis::MechanismVisualizer)
    # TODO
    VisualizerInterface.SimulationCommands()
end

function VisualizerInterface.visualize(vis::MechanismVisualizer, t::Number, state::MechanismState)
    set_configuration!(vis, configuration(state))
end

function VisualizerInterface.window(vis::MechanismVisualizer; mode::Modes.VisualizerMode = Modes.BrowserWindow)
    if mode == Modes.BrowserWindow
        open(vis)
        wait(vis)
        return nothing
    elseif mode == Modes.IJuliaCell
        return MeshCat.IJuliaCell(vis)
    else
        error()
    end
end

VisualizerInterface.isinteractive(::MechanismVisualizer) = false

end # module
