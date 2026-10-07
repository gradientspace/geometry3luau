--!native
-- Port of geometry3Sharp intersection/IntrRay3AxisAlignedBox3.cs (adapted from IntrRay3Box3)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau API notes:
--   IntrRay3AxisAlignedBox3.new(ray, box)    (Ray3d, AxisAlignedBox3d; stored by reference)
--   q:Find(), q:Compute(), q:Test(), q:IsSimpleIntersection()
--   fields: Quantity, Result, Type, RayParam0, RayParam1, Point0, Point1
--   C# properties Ray/Box -> q:Ray(), q:SetRay(r), q:Box(), q:SetBox(b)
--   static Intersects(ray, box, expandExtents?) -> bool
--   static FindRayIntersectT(ray, box) -> bool, rayT    (C# out RayParam; rayT = double.MaxValue on miss)

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local IntersectionResult = require(script.Parent.IntersectionResult)
local IntersectionType = require(script.Parent.IntersectionType)
local IntrLine3AxisAlignedBox3 = require(script.Parent.IntrLine3AxisAlignedBox3)

local abs = math.abs
local MaxDouble = MathUtil.MaxDouble
local DoClippingXYZ = IntrLine3AxisAlignedBox3.DoClippingXYZ

local IntrRay3AxisAlignedBox3 = {}
IntrRay3AxisAlignedBox3.__index = IntrRay3AxisAlignedBox3
IntrRay3AxisAlignedBox3.ClassName = "IntrRay3AxisAlignedBox3"

function IntrRay3AxisAlignedBox3.new(r, b)
	return setmetatable({
		ray = r,
		box = b,
		Quantity = 0,
		Result = IntersectionResult.NotComputed,
		Type = IntersectionType.Empty,
		RayParam0 = 0,
		RayParam1 = 0,
		Point0 = Vector3d.Zero,
		Point1 = Vector3d.Zero,
	}, IntrRay3AxisAlignedBox3)
end

function IntrRay3AxisAlignedBox3:Ray()
	return self.ray
end
function IntrRay3AxisAlignedBox3:SetRay(value)
	self.ray = value
	self.Result = IntersectionResult.NotComputed
end
function IntrRay3AxisAlignedBox3:Box()
	return self.box
end
function IntrRay3AxisAlignedBox3:SetBox(value)
	self.box = value
	self.Result = IntersectionResult.NotComputed
end

function IntrRay3AxisAlignedBox3:IsSimpleIntersection(): boolean
	return self.Result == IntersectionResult.Intersects and self.Type == IntersectionType.Point
end

function IntrRay3AxisAlignedBox3:Compute()
	self:Find()
	return self
end

function IntrRay3AxisAlignedBox3:Find(): boolean
	if self.Result ~= IntersectionResult.NotComputed then
		return self.Result == IntersectionResult.Intersects
	end

	-- [RMS] if either line direction is not a normalized vector,
	--   results are garbage, so fail query
	if self.ray.Direction:IsNormalized() == false then
		self.Type = IntersectionType.Empty
		self.Result = IntersectionResult.InvalidQuery
		return false
	end

	local _
	_, self.RayParam0, self.RayParam1, self.Quantity, self.Point0, self.Point1, self.Type = IntrLine3AxisAlignedBox3.DoClipping(
		0.0,
		MaxDouble,
		self.ray.Origin,
		self.ray.Direction,
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

function IntrRay3AxisAlignedBox3:Test(): boolean
	return IntrRay3AxisAlignedBox3.Intersects(self.ray, self.box)
end

-- test if ray intersects box.
-- expandExtents allows you to scale box for hit-testing purposes.
function IntrRay3AxisAlignedBox3.Intersects(ray, box, expandExtents: number?): boolean
	local expand = expandExtents or 0
	local RHS

	local c = box:Center()
	local e = box:Extents()
	local o, dir = ray.Origin, ray.Direction
	local diffx, diffy, diffz = o.x - c.x, o.y - c.y, o.z - c.z
	local extx, exty, extz = e.x + expand, e.y + expand, e.z + expand

	local WdUx = dir.x
	local AWdUx = abs(WdUx)
	local DdUx = diffx
	local ADdUx = abs(DdUx)
	if ADdUx > extx and DdUx * WdUx >= 0 then
		return false
	end

	local WdUy = dir.y
	local AWdUy = abs(WdUy)
	local DdUy = diffy
	local ADdUy = abs(DdUy)
	if ADdUy > exty and DdUy * WdUy >= 0 then
		return false
	end

	local WdUz = dir.z
	local AWdUz = abs(WdUz)
	local DdUz = diffz
	local ADdUz = abs(DdUz)
	if ADdUz > extz and DdUz * WdUz >= 0 then
		return false
	end

	-- WxD = ray.Direction.Cross(diff)
	local WxDx = dir.y * diffz - dir.z * diffy
	local WxDy = dir.z * diffx - dir.x * diffz
	local WxDz = dir.x * diffy - dir.y * diffx

	local AWxDdUx = abs(WxDx)
	RHS = exty * AWdUz + extz * AWdUy
	if AWxDdUx > RHS then
		return false
	end

	local AWxDdUy = abs(WxDy)
	RHS = extx * AWdUz + extz * AWdUx
	if AWxDdUy > RHS then
		return false
	end

	local AWxDdUz = abs(WxDz)
	RHS = extx * AWdUy + exty * AWdUx
	if AWxDdUz > RHS then
		return false
	end

	return true
end

-- Find intersection of ray with AABB, without having to construct any new classes.
-- Returns ray T-value of first intersection (or double.MaxVlaue on miss)
function IntrRay3AxisAlignedBox3.FindRayIntersectT(ray, box): (boolean, number)
	local o, d = ray.Origin, ray.Direction
	local bmin, bmax = box.Min, box.Max
	local notAllClipped, t0 = DoClippingXYZ(0.0, MaxDouble, o.x, o.y, o.z, d.x, d.y, d.z, bmin.x, bmin.y, bmin.z, bmax.x, bmax.y, bmax.z)
	-- solid == true, so Type != Empty iff notAllClipped
	if notAllClipped then
		return true, t0
	end
	return false, MaxDouble
end

return IntrRay3AxisAlignedBox3
