-- Port of geometry3Sharp distance/DistPoint3Circle3.cs (ported from WildMagic 5)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau API notes:
--   DistPoint3Circle3.new(point, circle)            circle is a Circle3d
--   q:Compute() -> q, q:Get(), q:GetSquared()
--   fields: DistanceSquared (-1 until computed), CircleClosest, AllCirclePointsEquidistant
--   C# properties Point/Circle -> q:Point(), q:SetPoint(v), q:Circle(), q:SetCircle(c) (setters reset DistanceSquared)

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)

local DistPoint3Circle3 = {}
DistPoint3Circle3.__index = DistPoint3Circle3
DistPoint3Circle3.ClassName = "DistPoint3Circle3"

function DistPoint3Circle3.new(PointIn, circleIn)
	return setmetatable({
		point = PointIn,
		circle = circleIn,
		DistanceSquared = -1.0,
		CircleClosest = Vector3d.Zero,
		AllCirclePointsEquidistant = false,
	}, DistPoint3Circle3)
end

function DistPoint3Circle3:Point()
	return self.point
end
function DistPoint3Circle3:SetPoint(value)
	self.point = value
	self.DistanceSquared = -1.0
end
function DistPoint3Circle3:Circle()
	return self.circle
end
function DistPoint3Circle3:SetCircle(value)
	self.circle = value
	self.DistanceSquared = -1.0
end

function DistPoint3Circle3:Compute()
	self:GetSquared()
	return self
end

function DistPoint3Circle3:Get(): number
	return math.sqrt(self:GetSquared())
end

function DistPoint3Circle3:GetSquared(): number
	if self.DistanceSquared >= 0 then
		return self.DistanceSquared
	end
	local point, circle = self.point, self.circle

	-- Projection of P-C onto plane is Q-C = P-C - Dot(N,P-C)*N.
	local PmC = point - circle.Center
	local QmC = PmC - circle.Normal:Dot(PmC) * circle.Normal
	local lengthQmC = QmC:Length()
	if lengthQmC > MathUtil.Epsilon then
		self.CircleClosest = circle.Center + circle.Radius * QmC / lengthQmC
		self.AllCirclePointsEquidistant = false
	else
		-- All circle points are equidistant from P.  Return one of them.
		self.CircleClosest = circle.Center + circle.Radius * circle.PlaneX
		self.AllCirclePointsEquidistant = true
	end

	local diff = point - self.CircleClosest
	local sqrDistance = diff:Dot(diff)

	-- Account for numerical round-off error.
	if sqrDistance < 0 then
		sqrDistance = 0
	end
	self.DistanceSquared = sqrDistance
	return sqrDistance
end

return DistPoint3Circle3
