# Workaround for JuliaLang/julia/pull/28625
if Base.HOME_PROJECT[] !== nothing
    Base.HOME_PROJECT[] = abspath(Base.HOME_PROJECT[])
end

using RigidBodySim, Documenter, RigidBodyDynamics
import DiffEqBase, MeshCatMechanisms

makedocs(
    doctest = true,
    modules = [RigidBodySim],
    checkdocs = :none, # because of the reexported functionality
    linkcheck = true,
    format = :html,
    root = @__DIR__,
    sitename ="RigidBodySim.jl",
    authors = "Twan Koolen and contributors.",
    pages = [
        "Home" => "index.md",
        "Quick start guide" => "quickstart.md",
        "Details" => "details.md"
    ],
    html_prettyurls = parse(Bool, get(ENV, "CI", "false")),
)

deploydocs(
    repo = "github.com/JuliaRobotics/RigidBodySim.jl.git"
)
