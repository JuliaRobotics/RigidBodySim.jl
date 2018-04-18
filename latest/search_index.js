var documenterSearchIndex = {"docs": [

{
    "location": "index.html#",
    "page": "Home",
    "title": "Home",
    "category": "page",
    "text": ""
},

{
    "location": "index.html#RigidBodySim-1",
    "page": "Home",
    "title": "RigidBodySim",
    "category": "section",
    "text": "RigidBodySim provides Julia tools for simulation and visualization of systems of interconnected rigid bodies.RigidBodySim is mainly built on top of the following packages:RigidBodyDynamics for construction of rigid body dynamics mechanisms and evaluation of their dynamics.\nDifferentialEquations packages, for numerical integration of the differential equations.\nRigidBodyTreeInspector for visualization.RigidBodySim does not attempt to abstract away its dependence on these packages, as doing so would necessarily expose only a subset of their functionality, and would require users familiar with these packages to learn yet another API. Instead, RigidBodySim simply plugs into existing functionality, providing convenience methods and extensions. Only this additional functionality is documented here, and we refer to the documentation for these packages for further information:RigidBodyDynamics: (Image: Stable)\nDifferentialEquations: (Image: Stable)"
},

{
    "location": "index.html#Functionality-1",
    "page": "Home",
    "title": "Functionality",
    "category": "section",
    "text": "RigidBodySim currently provides the following key features:Convenient creation of DiffEqBase.ODEProblems given a RigidBodyDynamics.MechanismState and, optionally, a controller.\nIntegration with RigidBodyTreeInspector for visualization, both during simulation and after. The visualizer window can also control (currently, pause or terminate) the simulation.\nEasy simulation of a digital controller running at a fixed rate (see PeriodicController)."
},

{
    "location": "index.html#Performance-1",
    "page": "Home",
    "title": "Performance",
    "category": "section",
    "text": "Performance is fairly good. For example, we have used RigidBodySim to perform a 10-second simulation of the humanoid robot Atlas (v5) standing on flat ground with a controller running at 100 Hz in 13 seconds with the Tsitouras 5/4 variable-step integrator on a 3GHz machine."
},

{
    "location": "index.html#Installation-1",
    "page": "Home",
    "title": "Installation",
    "category": "section",
    "text": ""
},

{
    "location": "index.html#Installing-Julia-1",
    "page": "Home",
    "title": "Installing Julia",
    "category": "section",
    "text": "Download links and more detailed instructions are available on the Julia website. The latest release of RigidBodySim.jl requires version 0.6 of Julia (the latest stable version).warning: Warning\nDo not use apt-get or brew to install Julia, as the versions provided by these package managers tend to be out of date."
},

{
    "location": "index.html#Installing-RigidBodySim-1",
    "page": "Home",
    "title": "Installing RigidBodySim",
    "category": "section",
    "text": "To install the latest tagged release of RigidBodySim, simply runPkg.add(\"RigidBodySim\")To check out the master branch and work on the bleeding edge (generally, not recommended), additionally runPkg.checkout(\"RigidBodySim\")"
},

{
    "location": "index.html#First-steps-1",
    "page": "Home",
    "title": "First steps",
    "category": "section",
    "text": "To load the package, use the command:using RigidBodySimIt is recommended to follow the quick start guide to get up to speed."
},

{
    "location": "index.html#Contents-1",
    "page": "Home",
    "title": "Contents",
    "category": "section",
    "text": "Pages = [\n  \"quickstart.md\",\n  \"details.md\"\n]\nDepth = 2"
},

{
    "location": "index.html#Citing-this-library-1",
    "page": "Home",
    "title": "Citing this library",
    "category": "section",
    "text": "@misc{rigidbodysimjl,\n author = \"Twan Koolen and contributors\",\n title = \"RigidBodySim.jl\",\n year = 2016,\n url = \"https://github.com/JuliaRobotics/RigidBodySim.jl\"\n}"
},

{
    "location": "quickstart.html#",
    "page": "Quick start guide",
    "title": "Quick start guide",
    "category": "page",
    "text": ""
},

{
    "location": "quickstart.html#quickstart-1",
    "page": "Quick start guide",
    "title": "Quick start guide",
    "category": "section",
    "text": "To get started, see this Jupyter notebook. To run it locally, you\'ll need the IJulia package."
},

{
    "location": "details.html#",
    "page": "Details",
    "title": "Details",
    "category": "page",
    "text": ""
},

{
    "location": "details.html#Details-1",
    "page": "Details",
    "title": "Details",
    "category": "section",
    "text": ""
},

{
    "location": "details.html#Index-1",
    "page": "Details",
    "title": "Index",
    "category": "section",
    "text": ""
},

{
    "location": "details.html#RigidBodySim.Core.Dynamics",
    "page": "Details",
    "title": "RigidBodySim.Core.Dynamics",
    "category": "type",
    "text": "Dynamics(mechanism)\nDynamics(mechanism, control!; setparams!)\n\n\nCreate a Dynamics object, representing either the passive or closed-loop dynamics of a RigidBodyDynamics.Mechanism.\n\nThe control! argument is a callable with the signature control!(τ, t, state), where τ is the torque vector to be set in the body of control!, t is the current time, and state is a MechanismState object. By default, control! is zero_control! (resulting in the passive dynamics).\n\nThe setparams! keyword argument is a callable with the signature setparams!(state, p) where state is a MechanismState and p is a vector of parameters, as used in OrdinaryDiffEq.jl.\n\n\n\n"
},

{
    "location": "details.html#DiffEqBase.ODEProblem-Tuple{RigidBodySim.Core.Dynamics,Union{AbstractArray{T,1} where T, RigidBodyDynamics.MechanismState},Any}",
    "page": "Details",
    "title": "DiffEqBase.ODEProblem",
    "category": "method",
    "text": "ODEProblem(dynamics, x0, tspan)\nODEProblem(dynamics, x0, tspan, p; callback, kwargs...)\n\n\nCreate a DiffEqBase.ODEProblem associated with the dynamics of a RigidBodyDynamics.Mechanism.\n\nThe initial state x0 can be either a RigidBodyDynamics.MechanismState), or an AbstractVector containing the initial state represented as [q; v; s], where q is the configuration vector, v is the velocity vector, and s is the vector of additional states.\n\nThe callback keyword argument can be used to pass in additional DifferentialEquations.jl callbacks.\n\n\n\n"
},

{
    "location": "details.html#ODE-problem-creation-1",
    "page": "Details",
    "title": "ODE problem creation",
    "category": "section",
    "text": "DynamicsODEProblem(::RigidBodySim.Dynamics, ::Union{Base.AbstractVector, RigidBodyDynamics.MechanismState}, tspan)"
},

{
    "location": "details.html#RigidBodySim.Core.zero_control!",
    "page": "Details",
    "title": "RigidBodySim.Core.zero_control!",
    "category": "function",
    "text": "zero_control!(τ, t, state)\n\n\nA \'zero\' controller, i.e. one that sets all control torques to zero at all times.\n\n\n\n"
},

{
    "location": "details.html#RigidBodySim.Core.controlcallback",
    "page": "Details",
    "title": "RigidBodySim.Core.controlcallback",
    "category": "function",
    "text": "controlcallback(control!)\n\n\nCan be used to create a callback associated with a given controller.\n\n\n\n"
},

{
    "location": "details.html#RigidBodySim.Control.PeriodicController",
    "page": "Details",
    "title": "RigidBodySim.Control.PeriodicController",
    "category": "type",
    "text": "struct PeriodicController{Tau<:(AbstractArray{T,1} where T), T<:Number, C, I}\n\nA PeriodicController can be used to simulate a digital controller that runs at a fixed rate (in terms of simulation time). It does so by performing a zero-order hold on a provided control function.\n\nPeriodicControllers can be constructed using\n\nPeriodicController(τ, Δt, control!; initialize = DiffEqBase.INITIALIZE_DEFAULT, save_positions = (false, false))\n\nwhere control! is a controller satisfying the standard RigidBodySim controller signature (control!(τ, Δt, state)), Δt is the simulation time interval between calls to the control! function, and τ is used to call control!. The initialize and save_positions keyword arguments are documented in the DiscreteCallback section of the DifferentialEquations documentation.\n\nPeriodicControllers are callable objects, and themselves fit the standard RigidBodySim controller signature.\n\nA DiffEqCallbacks.PeriodicCallback can be created from a PeriodicController, and is used to stop ODE integration exactly every Δt seconds, so that the control! function can be called. Typically, users will not have to explicitly create this PeriodicCallback, as it is automatically created and added to the ODEProblem when the PeriodicController is passed into the RigidBodySim-provided DiffEqBase.ODEProblem constructor overload.\n\nExamples\n\nIn the following example, a PeriodicController is used to simulate a digital PD controller running at a fixed rate of 200 Hz.\n\njulia> using RigidBodySim, RigidBodyDynamics, OrdinaryDiffEq\n\njulia> mechanism = parse_urdf(Float64, Pkg.dir(\"RigidBodySim\", \"test\", \"urdf\", \"Acrobot.urdf\"));\n\njulia> state = MechanismState(mechanism);\n\njulia> set_configuration!(state, [0.1; 0.2]);\n\njulia> controlcalls = Ref(0);\n\njulia> pdcontrol!(τ, t, state) = (controlcalls[] += 1; τ .= -20 .* velocity(state) .- 100 .* configuration(state));\n\njulia> τ = zeros(velocity(state)); Δt = 1 / 200\n0.005\n\njulia> problem = ODEProblem(Dynamics(mechanism, PeriodicController(τ, Δt, pdcontrol!)), state, (0., 5.));\n\njulia> sol = solve(problem, Tsit5());\n\njulia> sol.u[end]\n4-element Array{Float64,1}:\n -3.25923e-5\n -1.67942e-5\n  8.16715e-7\n  1.55292e-8\n\njulia> @assert all(x -> isapprox(x, 0, atol = 1e-4), sol.u[end]) # ensure state converges to zero\n\njulia> controlcalls[]\n1001\n\n\n\n"
},

{
    "location": "details.html#control-1",
    "page": "Details",
    "title": "Control",
    "category": "section",
    "text": "zero_control!\ncontrolcallback\nPeriodicController"
},

{
    "location": "details.html#Visualization-1",
    "page": "Details",
    "title": "Visualization",
    "category": "section",
    "text": ""
},

{
    "location": "details.html#DiffEqBase.CallbackSet-Tuple{DrakeVisualizer.Visualizer,RigidBodyDynamics.MechanismState}",
    "page": "Details",
    "title": "DiffEqBase.CallbackSet",
    "category": "method",
    "text": "CallbackSet(vis, state; max_fps)\n\n\nCreate the DifferentialEquations.jl callbacks needed for publishing to and receiving commands from a visualizer during simulation.\n\nmax_fps is the maximum number of frames per second (in terms of wall time) to draw. Default: 60.0.\n\n\n\n"
},

{
    "location": "details.html#RigidBodySim.Visualization.VisualizerInterface.visualize",
    "page": "Details",
    "title": "RigidBodySim.Visualization.VisualizerInterface.visualize",
    "category": "function",
    "text": "visualize(vis, t::Number, state::MechanismState)\n\nVisualize a Mechanism at the given time and in the given state using visualizer vis.\n\n\n\n"
},

{
    "location": "details.html#RigidBodySim.Visualization.VisualizerInterface.window",
    "page": "Details",
    "title": "RigidBodySim.Visualization.VisualizerInterface.window",
    "category": "function",
    "text": "window(vis; kwargs...)\n\nCreate a new window for visualizer vis. Visualizer-specific keyword arguments (kwargs) may be passed in.\n\n\n\n"
},

{
    "location": "details.html#RigidBodySim.Visualization.animate",
    "page": "Details",
    "title": "RigidBodySim.Visualization.animate",
    "category": "function",
    "text": "animate(vis, state, sol; max_fps, realtime_rate, pause_pollint)\n\n\nPlay back a visualization of a RigidBodySim.jl simulation.\n\nPositional arguments:\n\nvis is a visualizer satisfying the RigidBodySim visualizer interface.\nstate is a RigidBodyDynamics.MechanismState,\n\nrepresenting the state of the mechanism that was simulated, and will be modified during the visualization.\n\nsol is a DiffEqBase.ODESolution obtained from a RigidBodySim.jl simulation.\n\nanimate accepts the following keyword arguments:\n\nmax_fps: the maximum number of frames per second to draw. Default: 60.0.\nrealtime_rate: can be used to slow down or speed up playback compared to wall time. A realtime_rate of 2 will result in playback that is sped up 2x. Default: 1.0.\npause_pollint: how often to poll for commands coming from the director window when playback is paused. Default: 0.05.\n\nExamples\n\nVisualizing the result of a simulation of the passive dynamics of an Acrobot (double pendulum) at half speed:\n\nusing RigidBodySim, RigidBodyDynamics, MechanismGeometries\nimport RigidBodyTreeInspector: Visualizer\nurdf = Pkg.dir(\"RigidBodySim\", \"test\", \"urdf\", \"Acrobot.urdf\")\nmechanism = parse_urdf(Float64, urdf)\nstate = MechanismState(mechanism)\nset_configuration!(state, [0.1; 0.2])\nproblem = ODEProblem(Dynamics(mechanism), state, (0., 2.))\nsol = solve(problem, Vern7())\nvis = Visualizer(mechanism, visual_elements(mechanism, URDFVisuals(urdf)))\nwindow(vis)\nanimate(vis, state, sol; realtime_rate = 0.5)\n\n# output\n\n\n\n\n"
},

{
    "location": "details.html#User-interface-1",
    "page": "Details",
    "title": "User interface",
    "category": "section",
    "text": "DiffEqBase.CallbackSet(::DrakeVisualizer.Visualizer, ::RigidBodyDynamics.MechanismState)\nRigidBodySim.Visualization.visualize\nRigidBodySim.Visualization.window\nRigidBodySim.Visualization.animate"
},

{
    "location": "details.html#Visualizers-1",
    "page": "Details",
    "title": "Visualizers",
    "category": "section",
    "text": ""
},

{
    "location": "details.html#RigidBodySim.Visualization.VisualizerInterface.window-Tuple{DrakeVisualizer.Visualizer}",
    "page": "Details",
    "title": "RigidBodySim.Visualization.VisualizerInterface.window",
    "category": "method",
    "text": "window(vis; reuse)\n\n\nOpen a new director visualizer window.\n\nThe director instance will be started with a script that handles communication between RigidBodySim.jl and the director instance.\n\nSupports the following keyword arguments:\n\nreuse: skip opening a new visualizer window if one is already open.\n\n\n\n"
},

{
    "location": "details.html#RigidBodyTreeInspector-1",
    "page": "Details",
    "title": "RigidBodyTreeInspector",
    "category": "section",
    "text": "RigidBodySim provides an interface for the RigidBodyTreeInspector.jl visualizer, built on top of Director.The RigidBodyTreeInspector visualizer handle is a DrakeVisualizer.Visualizer. The easiest way to create a Visualizer for a robot is from a URDF, which can be parsed by RigidBodyTreeInspector\'s parse_urdf function.window(vis::DrakeVisualizer.Visualizer)"
},

{
    "location": "details.html#MeshCatMechanisms-1",
    "page": "Details",
    "title": "MeshCatMechanisms",
    "category": "section",
    "text": "RigidBodySim has support for the MeshCatMechanisms.jl MechanismVisualizer. Support is currently non-interactive, i.e., the simulation cannot be paused or terminated through the visualizer."
},

{
    "location": "details.html#RigidBodySim.Visualization.VisualizerInterface.SimulationCommands",
    "page": "Details",
    "title": "RigidBodySim.Visualization.VisualizerInterface.SimulationCommands",
    "category": "type",
    "text": "type SimulationCommands\n\nStores visualizer-independent commands used to control the simulation.\n\nA module providing a specific visualizer instance, say MyVisualizer, should provide a SimulationCommands constructor method with the signature\n\nSimulationCommands(vis::MyVisualizer)\n\nwhich returns a SimulationCommands object (to be constructed using SimulationCommands() in addition to performing any visualizer-dependent setup.\n\n\n\n"
},

{
    "location": "details.html#RigidBodySim.Visualization.VisualizerInterface.isinteractive",
    "page": "Details",
    "title": "RigidBodySim.Visualization.VisualizerInterface.isinteractive",
    "category": "function",
    "text": "isinteractive(vis)\n\nReturn whether or not visualizer vis supports user interaction (e.g. sending pause and terminate commands).\n\n\n\n"
},

{
    "location": "details.html#vis_interface-1",
    "page": "Details",
    "title": "Visualizer interface",
    "category": "section",
    "text": "RigidBodySim visualizers must implement the following interface:RigidBodySim.Visualization.VisualizerInterface.visualize\nRigidBodySim.Visualization.VisualizerInterface.windowIn addition, visualizers should also implement the following:RigidBodySim.Visualization.VisualizerInterface.SimulationCommands\nRigidBodySim.Visualization.VisualizerInterface.isinteractive"
},

{
    "location": "details.html#RigidBodySim.Core.configuration_renormalizer",
    "page": "Details",
    "title": "RigidBodySim.Core.configuration_renormalizer",
    "category": "function",
    "text": "configuration_renormalizer(state)\nconfiguration_renormalizer(state, condition)\n\n\nconfiguration_renormalizer can be used to create a callback that projects the configuration of a mechanism\'s state onto the configuration manifold. This may be necessary for mechanism\'s with e.g. quaternion-parameterized orientations as part of their joint configuration vectors, as numerical integration can cause the configuration to drift away from the unit norm constraints.\n\nThe callback is implemented as a DiffEqCallbacks.DiscreteCallback By default, it is called at every integrator time step.\n\n\n\n"
},

{
    "location": "details.html#Utilities-1",
    "page": "Details",
    "title": "Utilities",
    "category": "section",
    "text": "configuration_renormalizer"
},

]}
