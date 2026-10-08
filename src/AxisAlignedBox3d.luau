--!native
-- Port of geometry3Sharp math/AxisAlignedBox3d.cs (AxisAlignedBox3d; AxisAlignedBox3f is not ported, see PORTING.md)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- AxisAlignedBox3d is a MUTABLE struct in C#. Mutating methods (Expand, Contain, ...) replace the
-- Min/Max vectors (which are themselves immutable). Use :Clone() where C# would copy the struct.
--
-- Constructors:
--   AxisAlignedBox3d.new(xmin, ymin, zmin, xmax, ymax, zmax)
--   AxisAlignedBox3d.Empty()                       C# AxisAlignedBox3d.Empty / new AxisAlignedBox3d(false)
--   AxisAlignedBox3d.Zero(), .UnitPositive(), .Infinite()
--   AxisAlignedBox3d.FromCubeSize(f)              new AxisAlignedBox3d(double fCubeSize)
--   AxisAlignedBox3d.FromSize(w, h, d)            new AxisAlignedBox3d(double fWidth, fHeight, fDepth)
--   AxisAlignedBox3d.FromMinMax(vMin, vMax)       new AxisAlignedBox3d(Vector3d vMin, Vector3d vMax) (sorts components)
--   AxisAlignedBox3d.FromCenterHalfSizes(c, hw, hh, hd)
--   AxisAlignedBox3d.FromCenterHalfSize(c, h)
--   AxisAlignedBox3d.FromPoint(c)                 new AxisAlignedBox3d(Vector3d vCenter)

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)

local V3 = Vector3d.new
local max, min, sqrt, abs = math.max, math.min, math.sqrt, math.abs

local AxisAlignedBox3d = {}
AxisAlignedBox3d.__index = AxisAlignedBox3d
AxisAlignedBox3d.ClassName = "AxisAlignedBox3d"

export type AxisAlignedBox3d = typeof(setmetatable({} :: { Min: Vector3d.Vector3d, Max: Vector3d.Vector3d }, AxisAlignedBox3d))

local function make(vmin, vmax): AxisAlignedBox3d
	return setmetatable({ Min = vmin, Max = vmax }, AxisAlignedBox3d) :: any
end

function AxisAlignedBox3d.new(xmin: number, ymin: number, zmin: number, xmax: number, ymax: number, zmax: number): AxisAlignedBox3d
	return make(V3(xmin, ymin, zmin), V3(xmax, ymax, zmax))
end

-- new AxisAlignedBox3d(false): "empty" box with inverted min/max
function AxisAlignedBox3d.Empty(): AxisAlignedBox3d
	return make(
		V3(MathUtil.MaxDouble, MathUtil.MaxDouble, MathUtil.MaxDouble),
		V3(MathUtil.MinDouble, MathUtil.MinDouble, MathUtil.MinDouble)
	)
end

function AxisAlignedBox3d.Zero(): AxisAlignedBox3d
	return make(V3(0, 0, 0), V3(0, 0, 0))
end

function AxisAlignedBox3d.UnitPositive(): AxisAlignedBox3d
	return make(V3(0, 0, 0), V3(1, 1, 1))
end

function AxisAlignedBox3d.Infinite(): AxisAlignedBox3d
	return make(
		V3(MathUtil.MinDouble, MathUtil.MinDouble, MathUtil.MinDouble),
		V3(MathUtil.MaxDouble, MathUtil.MaxDouble, MathUtil.MaxDouble)
	)
end

function AxisAlignedBox3d.FromCubeSize(fCubeSize: number): AxisAlignedBox3d
	return make(V3(0, 0, 0), V3(fCubeSize, fCubeSize, fCubeSize))
end

function AxisAlignedBox3d.FromSize(fWidth: number, fHeight: number, fDepth: number): AxisAlignedBox3d
	return make(V3(0, 0, 0), V3(fWidth, fHeight, fDepth))
end

function AxisAlignedBox3d.FromMinMax(vMin: Vector3d.Vector3d, vMax: Vector3d.Vector3d): AxisAlignedBox3d
	return make(
		V3(min(vMin.x, vMax.x), min(vMin.y, vMax.y), min(vMin.z, vMax.z)),
		V3(max(vMin.x, vMax.x), max(vMin.y, vMax.y), max(vMin.z, vMax.z))
	)
end

function AxisAlignedBox3d.FromCenterHalfSizes(
	vCenter: Vector3d.Vector3d,
	fHalfWidth: number,
	fHalfHeight: number,
	fHalfDepth: number
): AxisAlignedBox3d
	return make(
		V3(vCenter.x - fHalfWidth, vCenter.y - fHalfHeight, vCenter.z - fHalfDepth),
		V3(vCenter.x + fHalfWidth, vCenter.y + fHalfHeight, vCenter.z + fHalfDepth)
	)
end

function AxisAlignedBox3d.FromCenterHalfSize(vCenter: Vector3d.Vector3d, fHalfSize: number): AxisAlignedBox3d
	return AxisAlignedBox3d.FromCenterHalfSizes(vCenter, fHalfSize, fHalfSize, fHalfSize)
end

function AxisAlignedBox3d.FromPoint(vCenter: Vector3d.Vector3d): AxisAlignedBox3d
	return make(vCenter, vCenter)
end

function AxisAlignedBox3d.Clone(self: AxisAlignedBox3d): AxisAlignedBox3d
	return make(self.Min, self.Max)
end

function AxisAlignedBox3d.Width(self: AxisAlignedBox3d): number
	return max(self.Max.x - self.Min.x, 0)
end
function AxisAlignedBox3d.Height(self: AxisAlignedBox3d): number
	return max(self.Max.y - self.Min.y, 0)
end
function AxisAlignedBox3d.Depth(self: AxisAlignedBox3d): number
	return max(self.Max.z - self.Min.z, 0)
end
function AxisAlignedBox3d.Volume(self: AxisAlignedBox3d): number
	return self:Width() * self:Height() * self:Depth()
end
function AxisAlignedBox3d.DiagonalLength(self: AxisAlignedBox3d): number
	local dx, dy, dz = self.Max.x - self.Min.x, self.Max.y - self.Min.y, self.Max.z - self.Min.z
	return sqrt(dx * dx + dy * dy + dz * dz)
end
function AxisAlignedBox3d.MaxDim(self: AxisAlignedBox3d): number
	return max(self:Width(), max(self:Height(), self:Depth()))
end
function AxisAlignedBox3d.Diagonal(self: AxisAlignedBox3d): Vector3d.Vector3d
	return V3(self.Max.x - self.Min.x, self.Max.y - self.Min.y, self.Max.z - self.Min.z)
end
function AxisAlignedBox3d.Extents(self: AxisAlignedBox3d): Vector3d.Vector3d
	return V3((self.Max.x - self.Min.x) * 0.5, (self.Max.y - self.Min.y) * 0.5, (self.Max.z - self.Min.z) * 0.5)
end
function AxisAlignedBox3d.Center(self: AxisAlignedBox3d): Vector3d.Vector3d
	return V3(0.5 * (self.Min.x + self.Max.x), 0.5 * (self.Min.y + self.Max.y), 0.5 * (self.Min.z + self.Max.z))
end

AxisAlignedBox3d.__eq = function(a: AxisAlignedBox3d, b: AxisAlignedBox3d): boolean
	return a.Min == b.Min and a.Max == b.Max
end

function AxisAlignedBox3d.Equals(self: AxisAlignedBox3d, other: AxisAlignedBox3d): boolean
	return self.Min == other.Min and self.Max == other.Max
end

function AxisAlignedBox3d.CompareTo(self: AxisAlignedBox3d, other: AxisAlignedBox3d): number
	local c = self.Min:CompareTo(other.Min)
	if c == 0 then
		return self.Max:CompareTo(other.Max)
	end
	return c
end

-- See Box3d.Corner for details on which corner is which
function AxisAlignedBox3d.Corner(self: AxisAlignedBox3d, i: number): Vector3d.Vector3d
	local b1 = bit32.band(i, 1) ~= 0
	local b2 = bit32.band(i, 2) ~= 0
	local x = (b1 ~= b2) and self.Max.x or self.Min.x
	local y = ((i // 2) % 2 == 0) and self.Min.y or self.Max.y
	local z = (i < 4) and self.Min.z or self.Max.z
	return V3(x, y, z)
end

-- Returns point on face/edge/corner. For each coord value neg==min, 0==center, pos==max
function AxisAlignedBox3d.Point(self: AxisAlignedBox3d, xi: number, yi: number, zi: number): Vector3d.Vector3d
	local Min, Max = self.Min, self.Max
	local x = (xi < 0) and Min.x or ((xi == 0) and (0.5 * (Min.x + Max.x)) or Max.x)
	local y = (yi < 0) and Min.y or ((yi == 0) and (0.5 * (Min.y + Max.y)) or Max.y)
	local z = (zi < 0) and Min.z or ((zi == 0) and (0.5 * (Min.z + Max.z)) or Max.z)
	return V3(x, y, z)
end

-- TODO
--public void Rotate(Quaterniond rotation)
--public void Scale(Vector3d vScale)

function AxisAlignedBox3d.Expand(self: AxisAlignedBox3d, fRadius: number)
	local Min, Max = self.Min, self.Max
	self.Min = V3(Min.x - fRadius, Min.y - fRadius, Min.z - fRadius)
	self.Max = V3(Max.x + fRadius, Max.y + fRadius, Max.z + fRadius)
end

function AxisAlignedBox3d.Expanded(self: AxisAlignedBox3d, fRadius: number): AxisAlignedBox3d
	local Min, Max = self.Min, self.Max
	return AxisAlignedBox3d.new(Min.x - fRadius, Min.y - fRadius, Min.z - fRadius, Max.x + fRadius, Max.y + fRadius, Max.z + fRadius)
end

function AxisAlignedBox3d.Contract(self: AxisAlignedBox3d, fRadius: number)
	local w = 2 * fRadius
	local mnx, mny, mnz = self.Min.x, self.Min.y, self.Min.z
	local mxx, mxy, mxz = self.Max.x, self.Max.y, self.Max.z
	if w > mxx - mnx then
		mnx = 0.5 * (mnx + mxx)
		mxx = mnx
	else
		mnx += fRadius
		mxx -= fRadius
	end
	if w > mxy - mny then
		mny = 0.5 * (mny + mxy)
		mxy = mny
	else
		mny += fRadius
		mxy -= fRadius
	end
	if w > mxz - mnz then
		mnz = 0.5 * (mnz + mxz)
		mxz = mnz
	else
		mnz += fRadius
		mxz -= fRadius
	end
	self.Min = V3(mnx, mny, mnz)
	self.Max = V3(mxx, mxy, mxz)
end

function AxisAlignedBox3d.Contracted(self: AxisAlignedBox3d, fRadius: number): AxisAlignedBox3d
	local Min, Max = self.Min, self.Max
	local mnx, mny, mnz = Min.x + fRadius, Min.y + fRadius, Min.z + fRadius
	local mxx, mxy, mxz = Max.x - fRadius, Max.y - fRadius, Max.z - fRadius
	if mnx > mxx then
		mnx = 0.5 * (Min.x + Max.x)
		mxx = mnx
	end
	if mny > mxy then
		mny = 0.5 * (Min.y + Max.y)
		mxy = mny
	end
	if mnz > mxz then
		mnz = 0.5 * (Min.z + Max.z)
		mxz = mnz
	end
	return AxisAlignedBox3d.new(mnx, mny, mnz, mxx, mxy, mxz)
end

function AxisAlignedBox3d.Scale(self: AxisAlignedBox3d, sx: number, sy: number, sz: number)
	local c = self:Center()
	local e = self:Extents()
	local ex, ey, ez = e.x * sx, e.y * sy, e.z * sz
	self.Min = V3(c.x - ex, c.y - ey, c.z - ez)
	self.Max = V3(c.x + ex, c.y + ey, c.z + ez)
end

-- Contain(Vector3d v) or Contain(AxisAlignedBox3d box)
function AxisAlignedBox3d.Contain(self: AxisAlignedBox3d, v: any)
	local Min, Max = self.Min, self.Max
	if type(v.Min) == "table" then
		local bmin, bmax = v.Min, v.Max
		self.Min = V3(min(Min.x, bmin.x), min(Min.y, bmin.y), min(Min.z, bmin.z))
		self.Max = V3(max(Max.x, bmax.x), max(Max.y, bmax.y), max(Max.z, bmax.z))
	else
		self.Min = V3(min(Min.x, v.x), min(Min.y, v.y), min(Min.z, v.z))
		self.Max = V3(max(Max.x, v.x), max(Max.y, v.y), max(Max.z, v.z))
	end
end

-- Contain a point given as scalar coordinates (avoids allocating a Vector3d in hot loops)
function AxisAlignedBox3d.ContainXYZ(self: AxisAlignedBox3d, x: number, y: number, z: number)
	local Min, Max = self.Min, self.Max
	if x < Min.x or y < Min.y or z < Min.z then
		self.Min = V3(min(Min.x, x), min(Min.y, y), min(Min.z, z))
	end
	if x > Max.x or y > Max.y or z > Max.z then
		self.Max = V3(max(Max.x, x), max(Max.y, y), max(Max.z, z))
	end
end

function AxisAlignedBox3d.Intersect(self: AxisAlignedBox3d, box: AxisAlignedBox3d): AxisAlignedBox3d
	local Min, Max = self.Min, self.Max
	local intersect = AxisAlignedBox3d.new(
		max(Min.x, box.Min.x),
		max(Min.y, box.Min.y),
		max(Min.z, box.Min.z),
		min(Max.x, box.Max.x),
		min(Max.y, box.Max.y),
		min(Max.z, box.Max.z)
	)
	if intersect:Height() <= 0 or intersect:Width() <= 0 or intersect:Depth() <= 0 then
		return AxisAlignedBox3d.Empty()
	end
	return intersect
end

-- Contains(Vector3d v) or Contains(AxisAlignedBox3d box2)
function AxisAlignedBox3d.Contains(self: AxisAlignedBox3d, v: any): boolean
	if type(v.Min) == "table" then
		return self:Contains(v.Min) and self:Contains(v.Max)
	end
	local Min, Max = self.Min, self.Max
	return (Min.x <= v.x) and (Min.y <= v.y) and (Min.z <= v.z) and (Max.x >= v.x) and (Max.y >= v.y) and (Max.z >= v.z)
end

function AxisAlignedBox3d.Intersects(self: AxisAlignedBox3d, box: AxisAlignedBox3d): boolean
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

-- DistanceSquared(Vector3d v) or DistanceSquared(AxisAlignedBox3d box2)
function AxisAlignedBox3d.DistanceSquared(self: AxisAlignedBox3d, v: any): number
	local Min, Max = self.Min, self.Max
	if type(v.Min) == "table" then
		-- compute lensqr( max(0, abs(center1-center2) - (extent1+extent2)) )
		local box2 = v
		local delta_x = abs((box2.Min.x + box2.Max.x) - (Min.x + Max.x)) - ((Max.x - Min.x) + (box2.Max.x - box2.Min.x))
		if delta_x < 0 then
			delta_x = 0
		end
		local delta_y = abs((box2.Min.y + box2.Max.y) - (Min.y + Max.y)) - ((Max.y - Min.y) + (box2.Max.y - box2.Min.y))
		if delta_y < 0 then
			delta_y = 0
		end
		local delta_z = abs((box2.Min.z + box2.Max.z) - (Min.z + Max.z)) - ((Max.z - Min.z) + (box2.Max.z - box2.Min.z))
		if delta_z < 0 then
			delta_z = 0
		end
		return 0.25 * (delta_x * delta_x + delta_y * delta_y + delta_z * delta_z)
	end
	local dx = (v.x < Min.x) and (Min.x - v.x) or ((v.x > Max.x) and (v.x - Max.x) or 0)
	local dy = (v.y < Min.y) and (Min.y - v.y) or ((v.y > Max.y) and (v.y - Max.y) or 0)
	local dz = (v.z < Min.z) and (Min.z - v.z) or ((v.z > Max.z) and (v.z - Max.z) or 0)
	return dx * dx + dy * dy + dz * dz
end

function AxisAlignedBox3d.Distance(self: AxisAlignedBox3d, v: Vector3d.Vector3d): number
	return sqrt(self:DistanceSquared(v))
end

function AxisAlignedBox3d.SignedDistance(self: AxisAlignedBox3d, v: Vector3d.Vector3d): number
	if not self:Contains(v) then
		return self:Distance(v)
	end
	local Min, Max = self.Min, self.Max
	local dx = min(abs(v.x - Min.x), abs(v.x - Max.x))
	local dy = min(abs(v.y - Min.y), abs(v.y - Max.y))
	local dz = min(abs(v.z - Min.z), abs(v.z - Max.z))
	return -MathUtil.Min(dx, dy, dz)
end

function AxisAlignedBox3d.Translate(self: AxisAlignedBox3d, vTranslate: Vector3d.Vector3d)
	self.Min = self.Min + vTranslate
	self.Max = self.Max + vTranslate
end

-- MoveMin(Vector3d vNewMin) or MoveMin(x, y, z)
function AxisAlignedBox3d.MoveMin(self: AxisAlignedBox3d, a: any, b: number?, c: number?)
	local nx, ny, nz
	if type(a) == "number" then
		nx, ny, nz = a, b :: number, c :: number
	else
		nx, ny, nz = a.x, a.y, a.z
	end
	local Min, Max = self.Min, self.Max
	self.Max = V3(nx + (Max.x - Min.x), ny + (Max.y - Min.y), nz + (Max.z - Min.z))
	self.Min = V3(nx, ny, nz)
end

AxisAlignedBox3d.__tostring = function(b: AxisAlignedBox3d): string
	return string.format("x[%.8f,%.8f] y[%.8f,%.8f] z[%.8f,%.8f]", b.Min.x, b.Max.x, b.Min.y, b.Max.y, b.Min.z, b.Max.z)
end

return AxisAlignedBox3d
