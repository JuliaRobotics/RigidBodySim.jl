using LCMCore
using StaticArrays

mutable struct CommsT <: LCMType
    utime::Int64
    format::String
    format_version_major::Int32
    format_version_minor::Int32
    num_bytes::Int32
    data::Vector{UInt8}
end

function CommsT(utime::Integer, format::String, format_version_major::Integer, format_version_minor::Integer, data::Vector{UInt8})
    CommsT(utime, format, format_version_major, format_version_minor, length(data), data)
end

LCMCore.fingerprint(::Type{CommsT}) = SVector(0xd3, 0x68, 0xe0, 0x3f, 0x33, 0xc5, 0x68, 0xbe)
LCMCore.size_fields(::Type{CommsT}) = (:num_bytes,)
LCMCore.check_valid(x::CommsT) = @assert length(x.data) == x.num_bytes
Base.resize!(x::CommsT) = resize!(x.data, x.num_bytes)
