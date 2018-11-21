# Modified from Reexport.jl, with one difference: it doesn't reexport the module names themselves
#
# Copyright (c) 2014: Simon Kornblith.
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
# associated documentation files (the "Software"), to deal in the Software without restriction,
# including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies or substantial
# portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
# LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
# WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
macro reexport(ex)
    isa(ex, Expr) && (ex.head == :module ||
        ex.head == :using ||
        ex.head == :importall ||
        (ex.head == :toplevel &&
        all(e->isa(e, Expr) && (e.head == :using || e.head == :importall), ex.args))) ||
    error("@reexport: syntax error")

    if ex.head == :module
    modules = Any[ex.args[2]]
    ex = Expr(:toplevel, ex, :(using .$(ex.args[2])))
    elseif (ex.head == :using || ex.head == :importall) && all(e->isa(e, Symbol), ex.args)
    modules = Any[ex.args[end]]
    else
    modules = Any[e.args[end] for e in ex.args]
    end

    Expr(:toplevel,
        ex,
        [:(eval(Expr(:export, setdiff(names($mod), [nameof($mod)])...))) for mod in modules]...
    ) |> esc
end
