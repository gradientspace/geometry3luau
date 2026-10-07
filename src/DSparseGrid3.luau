-- Port of geometry3Sharp spatial/DSparseGrid3.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Dynamic sparse 3D grid. Idea is that we have grid of some type of object
-- and we don't want to pre-allocate full grid of them. So we allocate on-demand.
-- This can be used to implement multi-grid schemes, eg for example the GridElement
-- type could be Bitmap3 of a fixed dimension.
--
-- Luau port notes:
--   * IGridElement3 is duck-typed: the exemplar must implement elem:CreateNewGridElement(bCopy) -> new element.
--   * The C# Dictionary<Vector3i, ElemType> is keyed by a scalar computed from the index (an exact integer
--     for small indices, else the string "x,y,z"), since Luau tables compare keys by identity.
--   * GetXYZ(x, y, z, allocateIfMissing?) is an allocation-free variant of Get(Vector3i, ...).
--   * Properties -> methods: grid:Count(), grid:Density(), grid:BoundsInclusive(), grid:Dimensions().
--   * AllocatedIndices() iterates Vector3i keys; Allocated() iterates (Vector3i, element) pairs.
--     Iteration order is unspecified (as for a C# Dictionary).
--   * The IGrid3/IFixedGrid3 interfaces are duck-typed (BoundsInclusive / Dimensions methods).

local Vector3i = require(script.Parent.Vector3i)
local AxisAlignedBox3i = require(script.Parent.AxisAlignedBox3i)

local DSparseGrid3 = {}
DSparseGrid3.__index = DSparseGrid3
DSparseGrid3.ClassName = "DSparseGrid3"

-- cell key: an exact integer for |x|,|y|,|z| < 2^16 (fast path), otherwise the string "x,y,z"
local KOFF, KMUL = 65536, 131072
local function keyXYZ(x: number, y: number, z: number): any
	if x > -KOFF and x < KOFF and y > -KOFF and y < KOFF and z > -KOFF and z < KOFF then
		return (x + KOFF) + KMUL * ((y + KOFF) + KMUL * (z + KOFF))
	end
	return x .. "," .. y .. "," .. z
end
local function key(index): any
	return keyXYZ(index.x, index.y, index.z)
end
DSparseGrid3.Key = key
DSparseGrid3.KeyXYZ = keyXYZ

-- Must provide a sample instance of the element type that we can Duplicate()
-- to make additional copies. Should be no data in here
function DSparseGrid3.new(toDuplicate)
	local self = setmetatable({}, DSparseGrid3)
	self.exemplar = toDuplicate
	self.elements = {} -- key -> element
	self.indices = {} -- key -> Vector3i
	self.count = 0
	self.bounds = AxisAlignedBox3i.Empty()
	return self
end

function DSparseGrid3:Has(index): boolean
	return self.elements[key(index)] ~= nil
end

function DSparseGrid3:Get(index, allocateIfMissing: boolean?)
	local k = key(index)
	local result = self.elements[k]
	if result ~= nil then
		return result
	end
	if allocateIfMissing ~= false then
		return self:allocate(index, k)
	end
	return nil
end

-- Get(new Vector3i(x, y, z), allocateIfMissing) without allocating the index
function DSparseGrid3:GetXYZ(x: number, y: number, z: number, allocateIfMissing: boolean?)
	local k = keyXYZ(x, y, z)
	local result = self.elements[k]
	if result ~= nil then
		return result
	end
	if allocateIfMissing ~= false then
		return self:allocate(Vector3i.new(x, y, z), k)
	end
	return nil
end

function DSparseGrid3:Free(index): boolean
	local k = key(index)
	if self.elements[k] ~= nil then
		self.elements[k] = nil
		self.indices[k] = nil
		self.count -= 1
		return true
	end
	return false
end

function DSparseGrid3:FreeAll()
	table.clear(self.elements)
	table.clear(self.indices)
	self.count = 0
end

function DSparseGrid3:Count(): number
	return self.count
end

function DSparseGrid3:Density(): number
	return self.count / self.bounds:Volume()
end

-- returns integer-aabb where indices range from [min,max] (inclusive)
function DSparseGrid3:BoundsInclusive()
	return self.bounds:Clone()
end

function DSparseGrid3:Dimensions()
	return self.bounds:Diagonal() + Vector3i.One
end

function DSparseGrid3:AllocatedIndices(): () -> any
	local k = nil
	local indices = self.indices
	return function()
		k = next(indices, k)
		if k == nil then
			return nil
		end
		return indices[k]
	end
end

function DSparseGrid3:Allocated(): () -> (any, any)
	local k = nil
	local indices, elements = self.indices, self.elements
	return function()
		k = next(indices, k)
		if k == nil then
			return nil, nil
		end
		return indices[k], elements[k]
	end
end

function DSparseGrid3:allocate(index, k: any?)
	local kk = k or key(index)
	local new_elem = self.exemplar:CreateNewGridElement(false)
	self.elements[kk] = new_elem
	self.indices[kk] = Vector3i.new(index.x, index.y, index.z)
	self.count += 1
	self.bounds:Contain(index)
	return new_elem
end

return DSparseGrid3
