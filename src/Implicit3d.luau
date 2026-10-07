--!native
-- Port of geometry3Sharp implicit/Implicit3d.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- This module returns a table holding every type in Implicit3d.cs:
--   Implicit3d.ImplicitSphere3d, ImplicitHalfSpace3d, ImplicitAxisAlignedBox3d, ImplicitBox3d, ImplicitLine3d,
--   ImplicitOffset3d, ImplicitShell3d, ImplicitUnion3d, ImplicitIntersection3d, ImplicitDifference3d,
--   ImplicitNaryUnion3d, ImplicitNaryIntersection3d, ImplicitNaryDifference3d,
--   ImplicitSmoothUnion3d, ImplicitSmoothIntersection3d, ImplicitSmoothDifference3d, ImplicitBlend3d,
--   DistanceFieldToSkeletalField, SkeletalBlend3d, SkeletalRicciBlend3d, SkeletalRicciNaryBlend3d
--
-- Interfaces (duck-typed):
--   ImplicitFunction3d:          f:Value(pt: Vector3d) -> number           (C# Value(ref Vector3d pt))
--   BoundedImplicitFunction3d:   ImplicitFunction3d + f:Bounds() -> AxisAlignedBox3d (must return a fresh box)
--   Optional Luau fast path:     f:ValueXYZ(x, y, z) -> number, same result as Value(Vector3d.new(x, y, z)).
--     Every class here implements it, and MarchingCubes / the operators below use it when present
--     (it avoids allocating a Vector3d per evaluation). Implicits that only define Value() still work.
--
-- Construction: C# object initializers become an optional field table:
--   Implicit3d.ImplicitSphere3d.new({ Origin = Vector3d.new(0, 0, 0), Radius = 2 })
--   Implicit3d.ImplicitUnion3d.new({ A = sphere1, B = sphere2 })
-- Lists (Children, BSet) are 1-based Luau arrays.
-- C# properties with setters become methods: ImplicitBox3d:Box()/SetBox(box),
--   ImplicitBlend3d:WeightA()/SetWeightA(v), WeightB()/SetWeightB(v), Blend()/SetBlend(v)
--   (the field table passed to .new may still contain Box / WeightA / WeightB / Blend; the setters are applied).

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local AxisAlignedBox3d = require(script.Parent.AxisAlignedBox3d)

local V3 = Vector3d.new
local sqrt, abs, min, max = math.sqrt, math.abs, math.min, math.max

local Implicit3d = {}

-- evaluate an implicit at (x,y,z), using the ValueXYZ fast path if available
local function evalXYZ(f: any, x: number, y: number, z: number): number
	local fxyz = f.ValueXYZ
	if fxyz ~= nil then
		return fxyz(f, x, y, z)
	end
	return f:Value(V3(x, y, z))
end
Implicit3d.EvalXYZ = evalXYZ

local function valueFromXYZ(self: any, pt: any): number
	return self:ValueXYZ(pt.x, pt.y, pt.z)
end

local function defineClass(name: string, defaults: { [string]: any }, setters: { [string]: string }?)
	local class = {}
	class.__index = class
	class.ClassName = name
	class.Value = valueFromXYZ
	function class.new(init: { [string]: any }?)
		local self = setmetatable({}, class)
		for k, v in defaults do
			self[k] = v
		end
		if init then
			for k, v in init do
				local setter = setters and setters[k]
				if setter then
					self[setter](self, v)
				else
					self[k] = v
				end
			end
		end
		return self
	end
	Implicit3d[name] = class
	return class
end

-- box.SignedDistance(Vector3d) inlined on scalars
local function boxSignedDistance(box: any, x: number, y: number, z: number): number
	local Min, Max = box.Min, box.Max
	local mnx, mny, mnz, mxx, mxy, mxz = Min.x, Min.y, Min.z, Max.x, Max.y, Max.z
	if mnx <= x and mny <= y and mnz <= z and mxx >= x and mxy >= y and mxz >= z then
		local dx = min(abs(x - mnx), abs(x - mxx))
		local dy = min(abs(y - mny), abs(y - mxy))
		local dz = min(abs(z - mnz), abs(z - mxz))
		return -min(dx, min(dy, dz))
	end
	local ddx = (x < mnx) and (mnx - x) or ((x > mxx) and (x - mxx) or 0)
	local ddy = (y < mny) and (mny - y) or ((y > mxy) and (y - mxy) or 0)
	local ddz = (z < mnz) and (mnz - z) or ((z > mxz) and (z - mxz) or 0)
	return sqrt(ddx * ddx + ddy * ddy + ddz * ddz)
end

----------------------------------------------------------------------------------------------------
-- Implicit sphere, where zero isocontour is at Radius
----------------------------------------------------------------------------------------------------
local ImplicitSphere3d = defineClass("ImplicitSphere3d", { Origin = Vector3d.Zero, Radius = 0 })
function ImplicitSphere3d:ValueXYZ(x: number, y: number, z: number): number
	local o = self.Origin
	local dx, dy, dz = x - o.x, y - o.y, z - o.z
	return sqrt(dx * dx + dy * dy + dz * dz) - self.Radius
end
function ImplicitSphere3d:Bounds()
	return AxisAlignedBox3d.FromCenterHalfSize(self.Origin, self.Radius)
end

----------------------------------------------------------------------------------------------------
-- Implicit half-space. "Inside" is opposite of Normal direction.
----------------------------------------------------------------------------------------------------
local ImplicitHalfSpace3d = defineClass("ImplicitHalfSpace3d", { Origin = Vector3d.Zero, Normal = Vector3d.Zero })
function ImplicitHalfSpace3d:ValueXYZ(x: number, y: number, z: number): number
	local o, n = self.Origin, self.Normal
	return (x - o.x) * n.x + (y - o.y) * n.y + (z - o.z) * n.z
end
function ImplicitHalfSpace3d:Bounds()
	return AxisAlignedBox3d.FromCenterHalfSize(self.Origin, MathUtil.Epsilon)
end

----------------------------------------------------------------------------------------------------
-- Implicit axis-aligned box
----------------------------------------------------------------------------------------------------
local ImplicitAxisAlignedBox3d = defineClass("ImplicitAxisAlignedBox3d", {})
function ImplicitAxisAlignedBox3d:ValueXYZ(x: number, y: number, z: number): number
	return boxSignedDistance(self.AABox, x, y, z)
end
function ImplicitAxisAlignedBox3d:Bounds()
	return self.AABox:Clone()
end

----------------------------------------------------------------------------------------------------
-- Implicit oriented box (Box is a Box3d with fields Center, AxisX, AxisY, AxisZ, Extent and method ToAABB())
----------------------------------------------------------------------------------------------------
local ImplicitBox3d = defineClass("ImplicitBox3d", {}, { Box = "SetBox" })
function ImplicitBox3d:Box()
	return self.box
end
function ImplicitBox3d:SetBox(value)
	self.box = value
	local e = value.Extent
	self.local_aabb = AxisAlignedBox3d.new(-e.x, -e.y, -e.z, e.x, e.y, e.z)
	self.bounds_aabb = value:ToAABB()
end
function ImplicitBox3d:ValueXYZ(x: number, y: number, z: number): number
	local box = self.box
	local c = box.Center
	local px, py, pz = x - c.x, y - c.y, z - c.z
	local ax, ay, az = box.AxisX, box.AxisY, box.AxisZ
	local dx = px * ax.x + py * ax.y + pz * ax.z
	local dy = px * ay.x + py * ay.y + pz * ay.z
	local dz = px * az.x + py * az.y + pz * az.z
	return boxSignedDistance(self.local_aabb, dx, dy, dz)
end
function ImplicitBox3d:Bounds()
	return self.bounds_aabb:Clone()
end

----------------------------------------------------------------------------------------------------
-- Implicit tube around line segment
----------------------------------------------------------------------------------------------------
local ImplicitLine3d = defineClass("ImplicitLine3d", { Radius = 0 })
function ImplicitLine3d:ValueXYZ(x: number, y: number, z: number): number
	-- sqrt(Segment.DistanceSquared(pt)) - Radius, inlined
	local seg = self.Segment
	local c, dir, ext = seg.Center, seg.Direction, seg.Extent
	local t = (x - c.x) * dir.x + (y - c.y) * dir.y + (z - c.z) * dir.z
	if t >= ext then
		t = ext
	elseif t <= -ext then
		t = -ext
	end
	local qx, qy, qz = c.x + t * dir.x - x, c.y + t * dir.y - y, c.z + t * dir.z - z
	return sqrt(qx * qx + qy * qy + qz * qz) - self.Radius
end
function ImplicitLine3d:Value(pt): number
	return sqrt(self.Segment:DistanceSquared(pt)) - self.Radius
end
function ImplicitLine3d:Bounds()
	local o = self.Radius * Vector3d.One
	local p0, p1 = self.Segment:P0(), self.Segment:P1()
	local box = AxisAlignedBox3d.FromMinMax(p0 - o, p0 + o)
	box:Contain(p1 - o)
	box:Contain(p1 + o)
	return box
end

----------------------------------------------------------------------------------------------------
-- Offset the zero-isocontour of an implicit function.
-- Assumes that negative is inside, if not, reverse offset.
----------------------------------------------------------------------------------------------------
local ImplicitOffset3d = defineClass("ImplicitOffset3d", { Offset = 0 })
function ImplicitOffset3d:ValueXYZ(x: number, y: number, z: number): number
	return evalXYZ(self.A, x, y, z) - self.Offset
end
function ImplicitOffset3d:Bounds()
	local box = self.A:Bounds()
	box:Expand(self.Offset)
	return box
end

----------------------------------------------------------------------------------------------------
-- remaps values so that values within given interval are negative, and values outside this
-- interval are positive. (Inside is an Interval1d with fields a, b)
----------------------------------------------------------------------------------------------------
local ImplicitShell3d = defineClass("ImplicitShell3d", {})
function ImplicitShell3d:ValueXYZ(x: number, y: number, z: number): number
	local f = evalXYZ(self.A, x, y, z)
	local inside = self.Inside
	if f < inside.a then
		f = inside.a - f
	elseif f > inside.b then
		f = f - inside.b
	else
		f = -min(abs(f - inside.a), abs(f - inside.b))
	end
	return f
end
function ImplicitShell3d:Bounds()
	local box = self.A:Bounds()
	box:Expand(max(0, self.Inside.b))
	return box
end

----------------------------------------------------------------------------------------------------
-- Boolean Union of two implicit functions, A OR B.
----------------------------------------------------------------------------------------------------
local ImplicitUnion3d = defineClass("ImplicitUnion3d", {})
function ImplicitUnion3d:ValueXYZ(x: number, y: number, z: number): number
	return min(evalXYZ(self.A, x, y, z), evalXYZ(self.B, x, y, z))
end
function ImplicitUnion3d:Bounds()
	local box = self.A:Bounds()
	box:Contain(self.B:Bounds())
	return box
end

----------------------------------------------------------------------------------------------------
-- Boolean Intersection of two implicit functions, A AND B
----------------------------------------------------------------------------------------------------
local ImplicitIntersection3d = defineClass("ImplicitIntersection3d", {})
function ImplicitIntersection3d:ValueXYZ(x: number, y: number, z: number): number
	return max(evalXYZ(self.A, x, y, z), evalXYZ(self.B, x, y, z))
end
function ImplicitIntersection3d:Bounds()
	-- [TODO] intersect boxes
	local box = self.A:Bounds()
	box:Contain(self.B:Bounds())
	return box
end

----------------------------------------------------------------------------------------------------
-- Boolean Difference/Subtraction of two implicit functions A-B = A AND (NOT B)
----------------------------------------------------------------------------------------------------
local ImplicitDifference3d = defineClass("ImplicitDifference3d", {})
function ImplicitDifference3d:ValueXYZ(x: number, y: number, z: number): number
	return max(evalXYZ(self.A, x, y, z), -evalXYZ(self.B, x, y, z))
end
function ImplicitDifference3d:Bounds()
	-- [TODO] can actually subtract B.Bounds() here...
	return self.A:Bounds()
end

----------------------------------------------------------------------------------------------------
-- Boolean Union of N implicit functions
----------------------------------------------------------------------------------------------------
local ImplicitNaryUnion3d = defineClass("ImplicitNaryUnion3d", {})
function ImplicitNaryUnion3d:ValueXYZ(x: number, y: number, z: number): number
	local children = self.Children
	local f = evalXYZ(children[1], x, y, z)
	for k = 2, #children do
		f = min(f, evalXYZ(children[k], x, y, z))
	end
	return f
end
function ImplicitNaryUnion3d:Bounds()
	local children = self.Children
	local box = children[1]:Bounds()
	for k = 2, #children do
		box:Contain(children[k]:Bounds())
	end
	return box
end

----------------------------------------------------------------------------------------------------
-- Boolean Intersection of N implicit functions
----------------------------------------------------------------------------------------------------
local ImplicitNaryIntersection3d = defineClass("ImplicitNaryIntersection3d", {})
function ImplicitNaryIntersection3d:ValueXYZ(x: number, y: number, z: number): number
	local children = self.Children
	local f = evalXYZ(children[1], x, y, z)
	for k = 2, #children do
		f = max(f, evalXYZ(children[k], x, y, z))
	end
	return f
end
function ImplicitNaryIntersection3d:Bounds()
	local children = self.Children
	local box = children[1]:Bounds()
	for k = 2, #children do
		box = box:Intersect(children[k]:Bounds())
	end
	return box
end

----------------------------------------------------------------------------------------------------
-- Boolean Difference of N implicit functions, A - Union(B1..BN)
----------------------------------------------------------------------------------------------------
local ImplicitNaryDifference3d = defineClass("ImplicitNaryDifference3d", {})
function ImplicitNaryDifference3d:ValueXYZ(x: number, y: number, z: number): number
	local fA = evalXYZ(self.A, x, y, z)
	local bset = self.BSet
	local N = #bset
	if N == 0 then
		return fA
	end
	local fB = evalXYZ(bset[1], x, y, z)
	for k = 2, N do
		fB = min(fB, evalXYZ(bset[k], x, y, z))
	end
	return max(fA, -fB)
end
function ImplicitNaryDifference3d:Bounds()
	-- [TODO] could actually subtract other bounds here...
	return self.A:Bounds()
end

----------------------------------------------------------------------------------------------------
-- Continuous R-Function Boolean Union / Intersection / Difference
----------------------------------------------------------------------------------------------------
local mul = 1.0 / 1.5

local function containAB(self)
	local box = self.A:Bounds()
	box:Contain(self.B:Bounds())
	return box
end

local ImplicitSmoothUnion3d = defineClass("ImplicitSmoothUnion3d", {})
function ImplicitSmoothUnion3d:ValueXYZ(x: number, y: number, z: number): number
	local fA = evalXYZ(self.A, x, y, z)
	local fB = evalXYZ(self.B, x, y, z)
	return mul * (fA + fB - sqrt(fA * fA + fB * fB - fA * fB))
end
ImplicitSmoothUnion3d.Bounds = containAB

local ImplicitSmoothIntersection3d = defineClass("ImplicitSmoothIntersection3d", {})
function ImplicitSmoothIntersection3d:ValueXYZ(x: number, y: number, z: number): number
	local fA = evalXYZ(self.A, x, y, z)
	local fB = evalXYZ(self.B, x, y, z)
	return mul * (fA + fB + sqrt(fA * fA + fB * fB - fA * fB))
end
ImplicitSmoothIntersection3d.Bounds = containAB

local ImplicitSmoothDifference3d = defineClass("ImplicitSmoothDifference3d", {})
function ImplicitSmoothDifference3d:ValueXYZ(x: number, y: number, z: number): number
	local fA = evalXYZ(self.A, x, y, z)
	local fB = -evalXYZ(self.B, x, y, z)
	return mul * (fA + fB + sqrt(fA * fA + fB * fB - fA * fB))
end
ImplicitSmoothDifference3d.Bounds = containAB

----------------------------------------------------------------------------------------------------
-- Blend of two implicit surfaces. Uses Pasko blend from http://www.hyperfun.org/F-rep.pdf
----------------------------------------------------------------------------------------------------
local ImplicitBlend3d = defineClass(
	"ImplicitBlend3d",
	{ weightA = 0.01, weightB = 0.01, blend = 2.0, ExpandBounds = 0.25 },
	{ WeightA = "SetWeightA", WeightB = "SetWeightB", Blend = "SetBlend" }
)
-- Weight on implicit A
function ImplicitBlend3d:WeightA(): number
	return self.weightA
end
function ImplicitBlend3d:SetWeightA(value: number)
	self.weightA = MathUtil.Clamp(value, 0.00001, 100000)
end
-- Weight on implicit B
function ImplicitBlend3d:WeightB(): number
	return self.weightB
end
function ImplicitBlend3d:SetWeightB(value: number)
	self.weightB = MathUtil.Clamp(value, 0.00001, 100000)
end
-- Blending power
function ImplicitBlend3d:Blend(): number
	return self.blend
end
function ImplicitBlend3d:SetBlend(value: number)
	self.blend = MathUtil.Clamp(value, 0.0, 100000)
end
function ImplicitBlend3d:ValueXYZ(x: number, y: number, z: number): number
	local fA = evalXYZ(self.A, x, y, z)
	local fB = evalXYZ(self.B, x, y, z)
	local sqr_sum = fA * fA + fB * fB
	if sqr_sum > 1e12 then
		return min(fA, fB)
	end
	local wa, wb = fA / self.weightA, fB / self.weightB
	local b = self.blend / (1.0 + wa * wa + wb * wb)
	return 0.666666 * (fA + fB - sqrt(sqr_sum - fA * fB)) - b
end
function ImplicitBlend3d:Bounds()
	local box = self.A:Bounds()
	box:Contain(self.B:Bounds())
	box:Expand(self.ExpandBounds * box:MaxDim())
	return box
end

----------------------------------------------------------------------------------------------------
-- Skeletal implicit ops
----------------------------------------------------------------------------------------------------

-- Converts the interval [-falloff,falloff] to [0,1], then applies Wyvill falloff (1-t^2)^3.
local DistanceFieldToSkeletalField = defineClass("DistanceFieldToSkeletalField", { FalloffDistance = 0 })
DistanceFieldToSkeletalField.ZeroIsocontour = 0.421875
function DistanceFieldToSkeletalField:Bounds()
	local bounds = self.DistanceField:Bounds()
	bounds:Expand(self.FalloffDistance)
	return bounds
end
function DistanceFieldToSkeletalField:ValueXYZ(x: number, y: number, z: number): number
	local d = evalXYZ(self.DistanceField, x, y, z)
	local falloff = self.FalloffDistance
	if d > falloff then
		return 0
	elseif d < -falloff then
		return 1.0
	end
	local a = (d + falloff) / (2 * falloff)
	local t = 1 - (a * a)
	return t * t * t
end

-- sum-blend
local SkeletalBlend3d = defineClass("SkeletalBlend3d", {})
function SkeletalBlend3d:ValueXYZ(x: number, y: number, z: number): number
	return evalXYZ(self.A, x, y, z) + evalXYZ(self.B, x, y, z)
end
function SkeletalBlend3d:Bounds()
	local box = self.A:Bounds()
	box:Contain(self.B:Bounds())
	box:Expand(0.25 * box:MaxDim())
	return box
end

-- Ricci blend
local SkeletalRicciBlend3d = defineClass("SkeletalRicciBlend3d", { BlendPower = 2.0 })
function SkeletalRicciBlend3d:ValueXYZ(x: number, y: number, z: number): number
	local a = evalXYZ(self.A, x, y, z)
	local b = evalXYZ(self.B, x, y, z)
	local p = self.BlendPower
	if p == 1.0 then
		return a + b
	elseif p == 2.0 then
		return sqrt(a * a + b * b)
	end
	return math.pow(math.pow(a, p) + math.pow(b, p), 1.0 / p)
end
SkeletalRicciBlend3d.Bounds = SkeletalBlend3d.Bounds

-- N-ary Ricci blend
local SkeletalRicciNaryBlend3d = defineClass("SkeletalRicciNaryBlend3d", { BlendPower = 2.0, FieldShift = 0 })
function SkeletalRicciNaryBlend3d:ValueXYZ(x: number, y: number, z: number): number
	local children = self.Children
	local N = #children
	local f = 0
	local p = self.BlendPower
	if p == 1.0 then
		for k = 1, N do
			f += evalXYZ(children[k], x, y, z)
		end
	elseif p == 2.0 then
		for k = 1, N do
			local v = evalXYZ(children[k], x, y, z)
			f += v * v
		end
		f = sqrt(f)
	else
		for k = 1, N do
			local v = evalXYZ(children[k], x, y, z)
			f += math.pow(v, p)
		end
		f = math.pow(f, 1.0 / p)
	end
	return f + self.FieldShift
end
function SkeletalRicciNaryBlend3d:Bounds()
	local children = self.Children
	local box = children[1]:Bounds()
	for k = 2, #children do
		box:Contain(children[k]:Bounds())
	end
	box:Expand(0.25 * box:MaxDim())
	return box
end

return Implicit3d
