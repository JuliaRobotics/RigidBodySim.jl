# TODO: move somewhere else
using LCMCore
using StaticArrays

mutable struct UTimeT <: LCMType
    utime::Int64
end

LCMCore.fingerprint(::Type{UTimeT}) = SVector(0x4d, 0x0d, 0x41, 0xc1, 0xf1, 0x05, 0xb1, 0x2f)
LCMCore.size_fields(::Type{UTimeT}) = ()
Base.resize!(::UTimeT) = nothing
LCMCore.check_valid(::UTimeT) = nothing
