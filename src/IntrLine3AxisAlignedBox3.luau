--!native
-- Port of geometry3Sharp intersection/IntrLine3AxisAlignedBox3.cs (adapted from IntrLine3Box3)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau API notes:
--   IntrLine3AxisAlignedBox3.new(line, box)   (Line3d, AxisAlignedBox3d; stored by reference)
--   q:Find(), q:Compute(), q:Test(), q:IsSimpleIntersection()
--   fields: Quantity, Result, Type, LineParam0, LineParam1, Point0, Point1
--   C# properties Line/Box -> q:Line(), q:SetLine(l), q:Box(), q:SetBox(b)
--   static DoClipping(t0, t1, origin, direction, box, solid, quantity, point0, point1, intrType)
--       -> bool, t0, t1, quantity, point0, point1, intrType          (C# ref parameters become returns)
--   static Clip(denom, numer, t0, t1) -> bool, t0, t1
--   static DoClippingXYZ(t0, t1, ox,oy,oz, dx,dy,dz, minx,miny,minz, maxx,maxy,maxz)
--       -> bool notAllClipped, t0, t1    (allocation-free core of DoClipping, box given by min/max)

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local IntersectionResult = require(script.Parent.IntersectionResult)
local IntersectionType = require(script.Parent.IntersectionType)

local ZeroTolerance = MathUtil.ZeroTolerance
local abs = math.abs

local IntrLine3AxisAlignedBox3 = {}
IntrLine3AxisAlignedBox3.__index = IntrLine3AxisAlignedBox3
IntrLine3AxisAlignedBox3.ClassName = "IntrLine3AxisAlignedBox3"

function IntrLine3AxisAlignedBox3.new(l, b)
	return setmetatable({
		line = l,
		box = b,
		Quantity = 0,
		Result = IntersectionResult.NotComputed,
		Type = IntersectionType.Empty,
		LineParam0 = 0,
		LineParam1 = 0,
		Point0 = Vector3d.Zero,
		Point1 = Vector3d.Zero,
	}, IntrLine3AxisAlignedBox3)
end

function IntrLine3AxisAlignedBox3:Line()
	return self.line
end
function IntrLine3AxisAlignedBox3:SetLine(value)
	self.line = value
	self.Result = IntersectionResult.NotComputed
end
function IntrLine3AxisAlignedBox3:Box()
	return self.box
end
function IntrLine3AxisAlignedBox3:SetBox(value)
	self.box = value
	self.Result = IntersectionResult.NotComputed
end

function IntrLine3AxisAlignedBox3:IsSimpleIntersection(): boolean
	return self.Result == IntersectionResult.Intersects and self.Type == IntersectionType.Point
end

function IntrLine3AxisAlignedBox3:Compute()
	self:Find()
	return self
end

function IntrLine3AxisAlignedBox3:Find(): boolean
	if self.Result ~= IntersectionResult.NotComputed then
		return self.Result == IntersectionResult.Intersects
	end

	-- [RMS] if either line direction is not a normalized vector,
	--   results are garbage, so fail query
	if self.line.Direction:IsNormalized() == false then
		self.Type = IntersectionType.Empty
		self.Result = IntersectionResult.InvalidQuery
		return false
	end

	local _
	_, self.LineParam0, self.LineParam1, self.Quantity, self.Point0, self.Point1, self.Type = IntrLine3AxisAlignedBox3.DoClipping(
		-MathUtil.MaxDouble,
		MathUtil.MaxDouble,
		self.line.Origin,
		self.line.Direction,
		self.box,
		true,
		self.Quantity,
		self.Point0,
		self.Point1,
		self.Type
	)

	self.Result = (self.Type ~= IntersectionType.Empty) and IntersectionResult.Intersects or IntersectionResult.NoIntersection
	return self.Result == IntersectionResult.Intersects
end

-- [RMS TODO: lots of useless dot products below!! left over from obox conversion]
function IntrLine3AxisAlignedBox3:Test(): boolean
	local line, box = self.line, self.box
	local diff = line.Origin - box:Center()
	local WxD = line.Direction:Cross(diff)
	local extent = box:Extents()

	local AWdU1 = abs(line.Direction:Dot(Vector3d.AxisY))
	local AWdU2 = abs(line.Direction:Dot(Vector3d.AxisZ))
	local AWxDdU0 = abs(WxD:Dot(Vector3d.AxisX))
	local RHS = extent.y * AWdU2 + extent.z * AWdU1
	if AWxDdU0 > RHS then
		return false
	end

	local AWdU0 = abs(line.Direction:Dot(Vector3d.AxisX))
	local AWxDdU1 = abs(WxD:Dot(Vector3d.AxisY))
	RHS = extent.x * AWdU2 + extent.z * AWdU0
	if AWxDdU1 > RHS then
		return false
	end

	local AWxDdU2 = abs(WxD:Dot(Vector3d.AxisZ))
	RHS = extent.x * AWdU1 + extent.y * AWdU0
	if AWxDdU2 > RHS then
		return false
	end

	return true
end

-- Return value is 'true' if line segment intersects the current test
-- plane.  Otherwise 'false' is returned in which case the line segment
-- is entirely clipped.
local function Clip(denom: number, numer: number, t0: number, t1: number): (boolean, number, number)
	if denom > 0 then
		if numer - denom * t1 > ZeroTolerance then
			return false, t0, t1
		end
		if numer > denom * t0 then
			t0 = numer / denom
		end
		return true, t0, t1
	elseif denom < 0 then
		if numer - denom * t0 > ZeroTolerance then
			return false, t0, t1
		end
		if numer > denom * t1 then
			t1 = numer / denom
		end
		return true, t0, t1
	else
		return numer <= 0, t0, t1
	end
end
IntrLine3AxisAlignedBox3.Clip = Clip

-- allocation-free core of DoClipping. Box center/extents are computed from min/max exactly like
-- AxisAlignedBox3d.Center / Extents. Returns notAllClipped, t0, t1
local function DoClippingXYZ(
	t0: number,
	t1: number,
	ox: number,
	oy: number,
	oz: number,
	dx: number,
	dy: number,
	dz: number,
	minx: number,
	miny: number,
	minz: number,
	maxx: number,
	maxy: number,
	maxz: number
): (boolean, number, number)
	-- BOrigin = origin - box.Center; extent = box.Extents
	local bx, by, bz = ox - 0.5 * (minx + maxx), oy - 0.5 * (miny + maxy), oz - 0.5 * (minz + maxz)
	local ex, ey, ez = (maxx - minx) * 0.5, (maxy - miny) * 0.5, (maxz - minz) * 0.5
	local ok
	ok, t0, t1 = Clip(dx, -bx - ex, t0, t1)
	if not ok then
		return false, t0, t1
	end
	ok, t0, t1 = Clip(-dx, bx - ex, t0, t1)
	if not ok then
		return false, t0, t1
	end
	ok, t0, t1 = Clip(dy, -by - ey, t0, t1)
	if not ok then
		return false, t0, t1
	end
	ok, t0, t1 = Clip(-dy, by - ey, t0, t1)
	if not ok then
		return false, t0, t1
	end
	ok, t0, t1 = Clip(dz, -bz - ez, t0, t1)
	if not ok then
		return false, t0, t1
	end
	ok, t0, t1 = Clip(-dz, bz - ez, t0, t1)
	return ok, t0, t1
end
IntrLine3AxisAlignedBox3.DoClippingXYZ = DoClippingXYZ

-- C# static bool DoClipping(ref t0, ref t1, ref origin, ref direction, ref box, bool solid,
--                           ref quantity, ref point0, ref point1, ref intrType)
function IntrLine3AxisAlignedBox3.DoClipping(
	t0: number,
	t1: number,
	origin,
	direction,
	box,
	solid: boolean,
	quantity: number,
	point0,
	point1,
	intrType: number
)
	local saveT0, saveT1 = t0, t1
	local bmin, bmax = box.Min, box.Max
	local notAllClipped
	notAllClipped, t0, t1 = DoClippingXYZ(
		t0,
		t1,
		origin.x,
		origin.y,
		origin.z,
		direction.x,
		direction.y,
		direction.z,
		bmin.x,
		bmin.y,
		bmin.z,
		bmax.x,
		bmax.y,
		bmax.z
	)

	if notAllClipped and (solid or t0 ~= saveT0 or t1 ~= saveT1) then
		if t1 > t0 then
			intrType = IntersectionType.Segment
			quantity = 2
			point0 = origin + t0 * direction
			point1 = origin + t1 * direction
		else
			intrType = IntersectionType.Point
			quantity = 1
			point0 = origin + t0 * direction
		end
	else
		quantity = 0
		intrType = IntersectionType.Empty
	end

	return intrType ~= IntersectionType.Empty, t0, t1, quantity, point0, point1, intrType
end

return IntrLine3AxisAlignedBox3
