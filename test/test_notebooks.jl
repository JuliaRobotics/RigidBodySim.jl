@testset "example notebooks" begin
    notebookdir = joinpath(@__DIR__, "..", "notebooks")
    excludes = String[]
    push!(excludes, joinpath(notebookdir, "Uncertainty propagation using Measurements.jl.ipynb"))
    printinterval = 60 # seconds
    printcallback = timer -> Compat.@info "Running notebook tests."
    timertask = if VERSION < v"0.7-"
        Timer(printcallback, printinterval, printinterval)
    else
        Timer(printcallback, printinterval, interval=printinterval)
    end
    for file in readdir(notebookdir)
        path = joinpath(notebookdir, file)
        path in excludes && continue
        name, ext = splitext(file)
        lowercase(ext) == ".ipynb" || continue

        @eval module $(gensym()) # Each notebook is run in its own module.
        using Compat
        using Compat.Test
        using NBInclude
        @testset "$($name)" begin
            @nbinclude($path; regex = r"^((?!\#NBSKIP).)*$"s) # Use #NBSKIP in a cell to skip it during tests.
        end
        end # module
    end
    close(timertask)
end
