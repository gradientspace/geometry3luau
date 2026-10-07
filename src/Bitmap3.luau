--!native
-- Port of geometry3Sharp spatial/Bitmap3.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- 3D binary voxel grid. Implements the IBinaryVoxelGrid interface (GridBounds(), Get(Vector3i), NonZeros()).
--
-- Luau port notes:
--   * Bits is a BitArray (0-based Get/Set). Its backing array `Bits.bits` is 1-based: linear index
--     i = x + dimx*(y + dimy*z) is stored at Bits.bits[i + 1]. Hot code may use it directly.
--   * Constructor: Bitmap3.new(dims: Vector3i)
--   * Indexers: this[int i] -> GetLinear(i)/SetLinear(i, v); this[Vector3i] -> Get(idx)/Set(idx, v)
--   * Properties as methods: Dimensions(), GridBounds()
--   * SafeSet is the same as Set (no threads in Luau).
--   * Indices() / NonZeros() return iterators yielding new Vector3i.

local Vector3i = require(script.Parent.Vector3i)
local BitArray = require(script.Parent.BitArray)

local Bitmap3 = {}
Bitmap3.__index = Bitmap3
Bitmap3.ClassName = "Bitmap3"

function Bitmap3.new(dims)
	local size = dims.x * dims.y * dims.z
	return setmetatable({
		Bits = BitArray.new(size),
		dimensions = dims,
		row_size = dims.x,
		slab_size = dims.x * dims.y,
	}, Bitmap3)
end

function Bitmap3:Dimensions()
	return self.dimensions
end

-- bounds are lower-inclusive, upper-exclusive
function Bitmap3:GridBounds()
	local AxisAlignedBox3i = require(script.Parent.AxisAlignedBox3i)
	return AxisAlignedBox3i.FromMinMax(Vector3i.Zero, self.dimensions)
end

-- this[int i]
function Bitmap3:GetLinear(i: number): boolean
	return self.Bits.bits[i + 1]
end
function Bitmap3:SetLinear(i: number, value: boolean)
	self.Bits.bits[i + 1] = value
end

-- this[Vector3i idx]
function Bitmap3:Set(idx, val: boolean)
	self.Bits.bits[idx.z * self.slab_size + idx.y * self.row_size + idx.x + 1] = val
end
Bitmap3.SafeSet = Bitmap3.Set

function Bitmap3:Get(idx): boolean
	return self.Bits.bits[idx.z * self.slab_size + idx.y * self.row_size + idx.x + 1]
end

function Bitmap3:ToIndex(i: number)
	local slab_size, row_size = self.slab_size, self.row_size
	local c = i // slab_size
	i -= c * slab_size
	local b = i // row_size
	i -= b * row_size
	return Vector3i.new(i, b, c)
end

function Bitmap3:ToLinear(idx): number
	return idx.z * self.slab_size + idx.y * self.row_size + idx.x
end

function Bitmap3:Indices()
	local d = self.dimensions
	local nx, ny, nz = d.x, d.y, d.z
	local x, y, z = -1, 0, 0
	return function()
		x += 1
		if x >= nx then
			x = 0
			y += 1
			if y >= ny then
				y = 0
				z += 1
			end
		end
		if z >= nz or nx <= 0 or ny <= 0 then
			return nil
		end
		return Vector3i.new(x, y, z)
	end
end

function Bitmap3:NonZeros()
	local bits, count = self.Bits.bits, self.Bits.Count
	local i = -1
	return function()
		i += 1
		while i < count do
			if bits[i + 1] then
				return self:ToIndex(i)
			end
			i += 1
		end
		return nil
	end
end

-- count 6-nbrs of each voxel, discard if count <= minNbrs
function Bitmap3:Filter(nMinNbrs: number)
	local bounds = self:GridBounds()
	bounds.Max = bounds.Max - Vector3i.One
	local gIndices = require(script.Parent.IndexUtil).gIndices
	local offsets = gIndices.GridOffsets6
	local bits = self.Bits.bits
	for i = 0, self.Bits.Count - 1 do
		if bits[i + 1] == false then
			continue
		end
		local idx = self:ToIndex(i)
		local nc = 0
		local k = 0
		while k < 6 and nc <= nMinNbrs do
			local o = offsets[k + 1]
			local nx, ny, nz = idx.x + o.x, idx.y + o.y, idx.z + o.z
			k += 1
			if not bounds:ContainsXYZ(nx, ny, nz) then
				continue
			end
			if bits[nz * self.slab_size + ny * self.row_size + nx + 1] then
				nc += 1
			end
		end
		if nc <= nMinNbrs then
			bits[i + 1] = false
		end
	end
end

-- IGridElement interface, creates a new object and potentially copies this one
function Bitmap3:CreateNewGridElement(bCopy: boolean)
	local copy = Bitmap3.new(self.dimensions)
	if bCopy then
		error("Bitmap3.CreateNewGridElement: copy not implemented")
	end
	return copy
end

return Bitmap3
