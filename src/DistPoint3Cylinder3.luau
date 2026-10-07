-- Port of geometry3Sharp distance/DistPoint3Cylinder3.cs (ported from GTEngine)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- However, code is modified to compute signed distance, instead of distance
-- to cylinder solid (which is 0 inside cylinder). If you want solid distance,
-- check IsInside, and if true, distance is 0 and point is input point.
-- SolidDistance will return this distance for you, but you have to do
-- the Point classification yourself.
--
-- DistanceSquared is always positive!!
--
-- Luau API notes:
--   DistPoint3Cylinder3.new(point, cylinder)        cylinder is a Cylinder3d
--   q:Compute() -> q, q:Get(), q:GetSquared(), q:get_squared_infinite()
--   fields: DistanceSquared (-1 until computed), SignedDistance (negative inside), CylinderClosest
--   properties -> methods: IsInside(), SolidDistance(), Point()/SetPoint(v), Cylinder()/SetCylinder(c)

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)

local V3 = Vector3d.new
local abs, sqrt = math.abs, math.sqrt

local DistPoint3Cylinder3 = {}
DistPoint3Cylinder3.__index = DistPoint3Cylinder3
DistPoint3Cylinder3.ClassName = "DistPoint3Cylinder3"

function DistPoint3Cylinder3.new(PointIn, CylinderIn)
	return setmetatable({
		point = PointIn,
		cylinder = CylinderIn,
		DistanceSquared = -1.0,
		SignedDistance = 0.0, -- negative on inside
		CylinderClosest = Vector3d.Zero,
	}, DistPoint3Cylinder3)
end

function DistPoint3Cylinder3:Point()
	return self.point
end
function DistPoint3Cylinder3:SetPoint(value)
	self.point = value
	self.DistanceSquared = -1.0
end
function DistPoint3Cylinder3:Cylinder()
	return self.cylinder
end
function DistPoint3Cylinder3:SetCylinder(value)
	self.cylinder = value
	self.DistanceSquared = -1.0
end

function DistPoint3Cylinder3:IsInside(): boolean
	return self.SignedDistance < 0
end
function DistPoint3Cylinder3:SolidDistance(): number
	return (self.SignedDistance < 0) and 0 or self.SignedDistance
end

function DistPoint3Cylinder3:Compute()
	self:GetSquared()
	return self
end

function DistPoint3Cylinder3:Get(): number
	return sqrt(self:GetSquared())
end

function DistPoint3Cylinder3:GetSquared(): number
	if self.DistanceSquared >= 0 then
		return self.DistanceSquared
	end
	local cylinder = self.cylinder

	if cylinder.Height >= MathUtil.MaxDouble then
		return self:get_squared_infinite()
	end

	-- Convert the point to the cylinder coordinate system.  In this system,
	-- the point believes (0,0,0) is the cylinder axis origin and (0,0,1) is
	-- the cylinder axis direction.
	local basis0 = cylinder.Axis.Direction
	local _, basis1, basis2 = Vector3d.ComputeOrthogonalComplement(1, basis0)
	local height = cylinder.Height / 2.0

	local delta = self.point - cylinder.Axis.Origin
	local P = V3(basis1:Dot(delta), basis2:Dot(delta), basis0:Dot(delta))

	local sqrRadius = cylinder.Radius * cylinder.Radius
	local sqrDistance = P.x * P.x + P.y * P.y

	-- The point is outside the infinite cylinder, or on the cylinder wall.
	local distance = sqrt(sqrDistance)
	local inf_distance = distance - cylinder.Radius
	local temp = cylinder.Radius / distance
	local inf_closest = V3(temp * P.x, temp * P.y, P.z)
	local bOutside = (sqrDistance >= sqrRadius)

	local result_closest = inf_closest
	local result_distance = inf_distance -- signed!

	if inf_closest.z >= height then
		result_closest = bOutside and inf_closest or P
		result_closest = V3(result_closest.x, result_closest.y, height)
		result_distance = result_closest:Distance(P) -- TODO: only compute sqr here
		bOutside = true
	elseif inf_closest.z <= -height then
		result_closest = bOutside and inf_closest or P
		result_closest = V3(result_closest.x, result_closest.y, -height)
		result_distance = result_closest:Distance(P) -- TODO: only compute sqr here
		bOutside = true
	elseif bOutside == false then
		if inf_closest.z > 0 and abs(inf_closest.z - height) < abs(inf_distance) then
			result_closest = V3(P.x, P.y, height)
			result_distance = result_closest:Distance(P) -- TODO: only compute sqr here
		elseif inf_closest.z < 0 and abs(inf_closest.z - -height) < abs(inf_distance) then
			result_closest = V3(P.x, P.y, -height)
			result_distance = result_closest:Distance(P) -- TODO: only compute sqr here
		end
	end
	self.SignedDistance = bOutside and abs(result_distance) or -abs(result_distance)

	-- Convert the closest point from the cylinder coordinate system to the
	-- original coordinate system.
	self.CylinderClosest = cylinder.Axis.Origin + result_closest.x * basis1 + result_closest.y * basis2 + result_closest.z * basis0

	self.DistanceSquared = result_distance * result_distance

	return self.DistanceSquared
end

function DistPoint3Cylinder3:get_squared_infinite(): number
	local cylinder = self.cylinder
	-- Convert the point to the cylinder coordinate system.  In this system,
	-- the point believes (0,0,0) is the cylinder axis origin and (0,0,1) is
	-- the cylinder axis direction.
	local basis0 = cylinder.Axis.Direction
	local _, basis1, basis2 = Vector3d.ComputeOrthogonalComplement(1, basis0)

	local delta = self.point - cylinder.Axis.Origin
	local P = V3(basis1:Dot(delta), basis2:Dot(delta), basis0:Dot(delta))

	local sqrDistance = P.x * P.x + P.y * P.y

	-- The point is outside the cylinder or on the cylinder wall.
	local distance = sqrt(sqrDistance)
	local result_distance = distance - cylinder.Radius
	local temp = cylinder.Radius / distance
	local result_closest = V3(temp * P.x, temp * P.y, P.z)

	-- Convert the closest point from the cylinder coordinate system to the
	-- original coordinate system.
	self.CylinderClosest = cylinder.Axis.Origin + result_closest.x * basis1 + result_closest.y * basis2 + result_closest.z * basis0
	self.SignedDistance = result_distance
	self.DistanceSquared = result_distance * result_distance
	return self.DistanceSquared
end

return DistPoint3Cylinder3
