--!native
-- Port of geometry3Sharp math/Plane3.cs (Plane3d; 3D plane, based on WildMagic5 Wm5Plane3 class)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Mutable struct with fields Normal (Vector3d) and Constant (number). Use :Clone() where C# copies it.
-- Constructors:
--   Plane3d.new(normal, constant)          new Plane3d(Vector3d normal, double constant)
--   Plane3d.FromNormalPoint(normal, point) new Plane3d(Vector3d normal, Vector3d point)
--   Plane3d.FromPoints(p0, p1, p2)         new Plane3d(Vector3d p0, Vector3d p1, Vector3d p2)
--   Plane3d.new(...) also dispatches: (normal, number) / (normal, point) / (p0, p1, p2)

local Plane3d = {}
Plane3d.__index = Plane3d
Plane3d.ClassName = "Plane3d"

local function make(normal, constant: number)
	return setmetatable({ Normal = normal, Constant = constant }, Plane3d)
end

-- N is specified, c = Dot(N,P) where P is a point on the plane.
function Plane3d.FromNormalPoint(normal, point)
	return make(normal, normal:Dot(point))
end

-- N = Cross(P1-P0,P2-P0)/Length(Cross(P1-P0,P2-P0)), c = Dot(N,P0) where
-- P0, P1, P2 are points on the plane.
function Plane3d.FromPoints(p0, p1, p2)
	local edge1 = p1 - p0
	local edge2 = p2 - p0
	local normal = edge1:UnitCross(edge2)
	return make(normal, normal:Dot(p0))
end

function Plane3d.new(normal, b: any, c: any?)
	if c ~= nil then
		return Plane3d.FromPoints(normal, b, c)
	elseif type(b) == "number" then
		return make(normal, b)
	elseif b ~= nil then
		return Plane3d.FromNormalPoint(normal, b)
	end
	return make(normal, 0)
end

function Plane3d:Clone()
	return make(self.Normal, self.Constant)
end

-- Compute d = Dot(N,P)-c where N is the plane normal and c is the plane
-- constant.  This is a signed distance.  The sign of the return value is
-- positive if the point is on the positive side of the plane, negative if
-- the point is on the negative side, and zero if the point is on the
-- plane.
function Plane3d:DistanceTo(p): number
	return self.Normal:Dot(p) - self.Constant
end

-- The "positive side" of the plane is the half space to which the plane
-- normal points.  The "negative side" is the other half space.  The
-- function returns +1 when P is on the positive side, -1 when P is on the
-- the negative side, or 0 when P is on the plane.
function Plane3d:WhichSide(p): number
	local distance = self:DistanceTo(p)
	if distance < 0 then
		return -1
	elseif distance > 0 then
		return 1
	end
	return 0
end

-- project point p onto plane
function Plane3d:Project(p)
	local d = self.Normal:Dot(p) - self.Constant
	return p - d * self.Normal
end

return Plane3d
