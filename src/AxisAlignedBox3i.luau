--!native
-- Port of geometry3Sharp math/AxisAlignedBox3i.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- AxisAlignedBox3i is a MUTABLE struct in C#. Mutating methods (Expand, Contain, ...) replace the
-- Min/Max Vector3i fields (which are themselves immutable). Use :Clone() where C# would copy the struct.
-- Integer division (Extents, Center) truncates toward zero like C#.
--
-- Constructors:
--   AxisAlignedBox3i.new(xmin, ymin, zmin, xmax, ymax, zmax)
--   AxisAlignedBox3i.Empty()                         C# AxisAlignedBox3i.Empty / new AxisAlignedBox3i(false)
--   AxisAlignedBox3i.Zero(), .UnitPositive(), .Infinite()
--   AxisAlignedBox3i.FromCubeSize(n)                 new AxisAlignedBox3i(int fCubeSize)
--   AxisAlignedBox3i.FromSize(w, h, d)               new AxisAlignedBox3i(int fWidth, fHeight, fDepth)
--   AxisAlignedBox3i.FromMinMax(vMin, vMax)          new AxisAlignedBox3i(Vector3i vMin, Vector3i vMax) (sorts components)
--   AxisAlignedBox3i.FromCenterHalfSizes(c, hw, hh, hd)
--   AxisAlignedBox3i.FromCenterHalfSize(c, h)
--   AxisAlignedBox3i.FromPoint(c)                    new AxisAlignedBox3i(Vector3i vCenter)
-- Overloads:
--   Contain(v) accepts a Vector3i, a Vector3d (truncated), an AxisAlignedBox3i or an AxisAlignedBox3d (truncated)
--   MoveMin(Vector3i) or MoveMin(x, y, z)
--   IndicesInclusive() / IndicesExclusive() return iterators yielding new Vector3i values (x fastest)

local MathUtil = require(script.Parent.MathUtil)
local Vector3i = require(script.Parent.Vector3i)

local V3i = Vector3i.new
local max, min, sqrt = math.max, math.min, math.sqrt
local IntDiv, TruncInt = MathUtil.IntDiv, MathUtil.TruncInt

local AxisAlignedBox3i = {}
AxisAlignedBox3i.__index = AxisAlignedBox3i
AxisAlignedBox3i.ClassName = "AxisAlignedBox3i"

export type AxisAlignedBox3i = typeof(setmetatable({} :: { Min: Vector3i.Vector3i, Max: Vector3i.Vector3i }, AxisAlignedBox3i))

local function make(vmin, vmax): AxisAlignedBox3i
	return setmetatable({ Min = vmin, Max = vmax }, AxisAlignedBox3i) :: any
end

function AxisAlignedBox3i.new(xmin: number, ymin: number, zmin: number, xmax: number, ymax: number, zmax: number): AxisAlignedBox3i
	return make(V3i(xmin, ymin, zmin), V3i(xmax, ymax, zmax))
end

function AxisAlignedBox3i.Empty(): AxisAlignedBox3i
	return make(V3i(MathUtil.MaxInt, MathUtil.MaxInt, MathUtil.MaxInt), V3i(MathUtil.MinInt, MathUtil.MinInt, MathUtil.MinInt))
end
function AxisAlignedBox3i.Zero(): AxisAlignedBox3i
	return make(V3i(0, 0, 0), V3i(0, 0, 0))
end
function AxisAlignedBox3i.UnitPositive(): AxisAlignedBox3i
	return make(V3i(0, 0, 0), V3i(1, 1, 1))
end
function AxisAlignedBox3i.Infinite(): AxisAlignedBox3i
	return make(V3i(MathUtil.MinInt, MathUtil.MinInt, MathUtil.MinInt), V3i(MathUtil.MaxInt, MathUtil.MaxInt, MathUtil.MaxInt))
end

function AxisAlignedBox3i.FromCubeSize(fCubeSize: number): AxisAlignedBox3i
	return make(V3i(0, 0, 0), V3i(fCubeSize, fCubeSize, fCubeSize))
end
function AxisAlignedBox3i.FromSize(fWidth: number, fHeight: number, fDepth: number): AxisAlignedBox3i
	return make(V3i(0, 0, 0), V3i(fWidth, fHeight, fDepth))
end
function AxisAlignedBox3i.FromMinMax(vMin: Vector3i.Vector3i, vMax: Vector3i.Vector3i): AxisAlignedBox3i
	return make(
		V3i(min(vMin.x, vMax.x), min(vMin.y, vMax.y), min(vMin.z, vMax.z)),
		V3i(max(vMin.x, vMax.x), max(vMin.y, vMax.y), max(vMin.z, vMax.z))
	)
end
function AxisAlignedBox3i.FromCenterHalfSizes(
	vCenter: Vector3i.Vector3i,
	fHalfWidth: number,
	fHalfHeight: number,
	fHalfDepth: number
): AxisAlignedBox3i
	return make(
		V3i(vCenter.x - fHalfWidth, vCenter.y - fHalfHeight, vCenter.z - fHalfDepth),
		V3i(vCenter.x + fHalfWidth, vCenter.y + fHalfHeight, vCenter.z + fHalfDepth)
	)
end
function AxisAlignedBox3i.FromCenterHalfSize(vCenter: Vector3i.Vector3i, fHalfSize: number): AxisAlignedBox3i
	return AxisAlignedBox3i.FromCenterHalfSizes(vCenter, fHalfSize, fHalfSize, fHalfSize)
end
function AxisAlignedBox3i.FromPoint(vCenter: Vector3i.Vector3i): AxisAlignedBox3i
	return make(vCenter, vCenter)
end

function AxisAlignedBox3i.Clone(self: AxisAlignedBox3i): AxisAlignedBox3i
	return make(self.Min, self.Max)
end

function AxisAlignedBox3i.Width(self: AxisAlignedBox3i): number
	return max(self.Max.x - self.Min.x, 0)
end
function AxisAlignedBox3i.Height(self: AxisAlignedBox3i): number
	return max(self.Max.y - self.Min.y, 0)
end
function AxisAlignedBox3i.Depth(self: AxisAlignedBox3i): number
	return max(self.Max.z - self.Min.z, 0)
end
function AxisAlignedBox3i.Volume(self: AxisAlignedBox3i): number
	return self:Width() * self:Height() * self:Depth()
end
function AxisAlignedBox3i.DiagonalLength(self: AxisAlignedBox3i): number
	local dx, dy, dz = self.Max.x - self.Min.x, self.Max.y - self.Min.y, self.Max.z - self.Min.z
	return TruncInt(sqrt(dx * dx + dy * dy + dz * dz))
end
function AxisAlignedBox3i.MaxDim(self: AxisAlignedBox3i): number
	return max(self:Width(), max(self:Height(), self:Depth()))
end
function AxisAlignedBox3i.Diagonal(self: AxisAlignedBox3i): Vector3i.Vector3i
	return V3i(self.Max.x - self.Min.x, self.Max.y - self.Min.y, self.Max.z - self.Min.z)
end
function AxisAlignedBox3i.Extents(self: AxisAlignedBox3i): Vector3i.Vector3i
	return V3i(IntDiv(self.Max.x - self.Min.x, 2), IntDiv(self.Max.y - self.Min.y, 2), IntDiv(self.Max.z - self.Min.z, 2))
end
function AxisAlignedBox3i.Center(self: AxisAlignedBox3i): Vector3i.Vector3i
	return V3i(IntDiv(self.Min.x + self.Max.x, 2), IntDiv(self.Min.y + self.Max.y, 2), IntDiv(self.Min.z + self.Max.z, 2))
end

AxisAlignedBox3i.__eq = function(a: AxisAlignedBox3i, b: AxisAlignedBox3i): boolean
	return a.Min == b.Min and a.Max == b.Max
end
function AxisAlignedBox3i.Equals(self: AxisAlignedBox3i, other: AxisAlignedBox3i): boolean
	return self.Min == other.Min and self.Max == other.Max
end
function AxisAlignedBox3i.CompareTo(self: AxisAlignedBox3i, other: AxisAlignedBox3i): number
	local c = self.Min:CompareTo(other.Min)
	if c == 0 then
		return self.Max:CompareTo(other.Max)
	end
	return c
end

-- value is subtracted from min and added to max
function AxisAlignedBox3i.Expand(self: AxisAlignedBox3i, nRadius: number)
	local Min, Max = self.Min, self.Max
	self.Min = V3i(Min.x - nRadius, Min.y - nRadius, Min.z - nRadius)
	self.Max = V3i(Max.x + nRadius, Max.y + nRadius, Max.z + nRadius)
end
-- value is added to min and subtracted from max
function AxisAlignedBox3i.Contract(self: AxisAlignedBox3i, nRadius: number)
	local Min, Max = self.Min, self.Max
	self.Min = V3i(Min.x + nRadius, Min.y + nRadius, Min.z + nRadius)
	self.Max = V3i(Max.x - nRadius, Max.y - nRadius, Max.z - nRadius)
end

function AxisAlignedBox3i.Scale(self: AxisAlignedBox3i, sx: number, sy: number, sz: number)
	local c = self:Center()
	local e = self:Extents()
	local ex, ey, ez = e.x * sx, e.y * sy, e.z * sz
	self.Min = V3i(c.x - ex, c.y - ey, c.z - ez)
	self.Max = V3i(c.x + ex, c.y + ey, c.z + ez)
end

-- Contain(Vector3i), Contain(AxisAlignedBox3i), Contain(Vector3d), Contain(AxisAlignedBox3d)
function AxisAlignedBox3i.Contain(self: AxisAlignedBox3i, v: any)
	local Min, Max = self.Min, self.Max
	if type(v.Min) == "table" then
		local bmin, bmax = v.Min, v.Max
		local isInt = getmetatable(bmin) == Vector3i
		local nx = isInt and bmin.x or TruncInt(bmin.x)
		local ny = isInt and bmin.y or TruncInt(bmin.y)
		local nz = isInt and bmin.z or TruncInt(bmin.z)
		local xx = isInt and bmax.x or TruncInt(bmax.x)
		local xy = isInt and bmax.y or TruncInt(bmax.y)
		local xz = isInt and bmax.z or TruncInt(bmax.z)
		self.Min = V3i(min(Min.x, nx), min(Min.y, ny), min(Min.z, nz))
		self.Max = V3i(max(Max.x, xx), max(Max.y, xy), max(Max.z, xz))
	else
		local x, y, z = v.x, v.y, v.z
		if getmetatable(v) ~= Vector3i then
			x, y, z = TruncInt(x), TruncInt(y), TruncInt(z)
		end
		self.Min = V3i(min(Min.x, x), min(Min.y, y), min(Min.z, z))
		self.Max = V3i(max(Max.x, x), max(Max.y, y), max(Max.z, z))
	end
end

function AxisAlignedBox3i.Intersect(self: AxisAlignedBox3i, box: AxisAlignedBox3i): AxisAlignedBox3i
	local Min, Max = self.Min, self.Max
	local intersect = AxisAlignedBox3i.new(
		max(Min.x, box.Min.x),
		max(Min.y, box.Min.y),
		max(Min.z, box.Min.z),
		min(Max.x, box.Max.x),
		min(Max.y, box.Max.y),
		min(Max.z, box.Max.z)
	)
	if intersect:Height() <= 0 or intersect:Width() <= 0 or intersect:Depth() <= 0 then
		return AxisAlignedBox3i.Empty()
	end
	return intersect
end

function AxisAlignedBox3i.Contains(self: AxisAlignedBox3i, v: Vector3i.Vector3i): boolean
	local Min, Max = self.Min, self.Max
	return (Min.x <= v.x) and (Min.y <= v.y) and (Min.z <= v.z) and (Max.x >= v.x) and (Max.y >= v.y) and (Max.z >= v.z)
end

-- Luau extension: Contains for scalar coordinates (allocation-free)
function AxisAlignedBox3i.ContainsXYZ(self: AxisAlignedBox3i, x: number, y: number, z: number): boolean
	local Min, Max = self.Min, self.Max
	return (Min.x <= x) and (Min.y <= y) and (Min.z <= z) and (Max.x >= x) and (Max.y >= y) and (Max.z >= z)
end

function AxisAlignedBox3i.Intersects(self: AxisAlignedBox3i, box: AxisAlignedBox3i): boolean
	local Min, Max = self.Min, self.Max
	return not (
		(box.Max.x <= Min.x)
		or (box.Min.x >= Max.x)
		or (box.Max.y <= Min.y)
		or (box.Min.y >= Max.y)
		or (box.Max.z <= Min.z)
		or (box.Min.z >= Max.z)
	)
end

function AxisAlignedBox3i.DistanceSquared(self: AxisAlignedBox3i, v: Vector3i.Vector3i): number
	local Min, Max = self.Min, self.Max
	local dx = (v.x < Min.x) and (Min.x - v.x) or ((v.x > Max.x) and (v.x - Max.x) or 0)
	local dy = (v.y < Min.y) and (Min.y - v.y) or ((v.y > Max.y) and (v.y - Max.y) or 0)
	local dz = (v.z < Min.z) and (Min.z - v.z) or ((v.z > Max.z) and (v.z - Max.z) or 0)
	return dx * dx + dy * dy + dz * dz
end
function AxisAlignedBox3i.Distance(self: AxisAlignedBox3i, v: Vector3i.Vector3i): number
	return TruncInt(sqrt(self:DistanceSquared(v)))
end

function AxisAlignedBox3i.NearestPoint(self: AxisAlignedBox3i, v: Vector3i.Vector3i): Vector3i.Vector3i
	local Min, Max = self.Min, self.Max
	local x = (v.x < Min.x) and Min.x or ((v.x > Max.x) and Max.x or v.x)
	local y = (v.y < Min.y) and Min.y or ((v.y > Max.y) and Max.y or v.y)
	local z = (v.z < Min.z) and Min.z or ((v.z > Max.z) and Max.z or v.z)
	return V3i(x, y, z)
end

-- Clamp v to grid bounds [min, max]
function AxisAlignedBox3i.ClampInclusive(self: AxisAlignedBox3i, v: Vector3i.Vector3i): Vector3i.Vector3i
	local Min, Max = self.Min, self.Max
	return V3i(MathUtil.Clamp(v.x, Min.x, Max.x), MathUtil.Clamp(v.y, Min.y, Max.y), MathUtil.Clamp(v.z, Min.z, Max.z))
end
-- clamp v to grid bounds [min,max)
function AxisAlignedBox3i.ClampExclusive(self: AxisAlignedBox3i, v: Vector3i.Vector3i): Vector3i.Vector3i
	local Min, Max = self.Min, self.Max
	return V3i(MathUtil.Clamp(v.x, Min.x, Max.x - 1), MathUtil.Clamp(v.y, Min.y, Max.y - 1), MathUtil.Clamp(v.z, Min.z, Max.z - 1))
end

-- relative translation
function AxisAlignedBox3i.Translate(self: AxisAlignedBox3i, vTranslate: Vector3i.Vector3i)
	self.Min = self.Min + vTranslate
	self.Max = self.Max + vTranslate
end

-- MoveMin(Vector3i vNewMin) or MoveMin(x, y, z)
function AxisAlignedBox3i.MoveMin(self: AxisAlignedBox3i, a: any, b: number?, c: number?)
	local nx, ny, nz
	if type(a) == "number" then
		nx, ny, nz = a, b :: number, c :: number
	else
		nx, ny, nz = a.x, a.y, a.z
	end
	local Min, Max = self.Min, self.Max
	self.Max = V3i(nx + (Max.x - Min.x), ny + (Max.y - Min.y), nz + (Max.z - Min.z))
	self.Min = V3i(nx, ny, nz)
end

local function indices(x0, y0, z0, x1, y1, z1): () -> Vector3i.Vector3i?
	local x, y, z = x0 - 1, y0, z0
	return function()
		x += 1
		if x > x1 then
			x = x0
			y += 1
			if y > y1 then
				y = y0
				z += 1
			end
		end
		if z > z1 or x0 > x1 or y0 > y1 then
			return nil
		end
		return V3i(x, y, z)
	end
end

function AxisAlignedBox3i.IndicesInclusive(self: AxisAlignedBox3i): () -> Vector3i.Vector3i?
	local Min, Max = self.Min, self.Max
	return indices(Min.x, Min.y, Min.z, Max.x, Max.y, Max.z)
end
function AxisAlignedBox3i.IndicesExclusive(self: AxisAlignedBox3i): () -> Vector3i.Vector3i?
	local Min, Max = self.Min, self.Max
	return indices(Min.x, Min.y, Min.z, Max.x - 1, Max.y - 1, Max.z - 1)
end

AxisAlignedBox3i.__tostring = function(b: AxisAlignedBox3i): string
	return string.format("x[%d,%d] y[%d,%d] z[%d,%d]", b.Min.x, b.Max.x, b.Min.y, b.Max.y, b.Min.z, b.Max.z)
end
AxisAlignedBox3i.ToString = AxisAlignedBox3i.__tostring

return AxisAlignedBox3i
