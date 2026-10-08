--!native
-- Port of geometry3Sharp math/Ray3.cs (Ray3d; Ray3f is not ported, see PORTING.md)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Fields Origin, Direction (Vector3d). Ray3d.new(origin, direction, bIsNormalized?) normalizes
-- the direction unless bIsNormalized is true (like C#).

local Vector3d = require(script.Parent.Vector3d)

local Ray3d = {}
Ray3d.__index = Ray3d
Ray3d.ClassName = "Ray3d"

function Ray3d.new(origin, direction, bIsNormalized: boolean?)
	origin = origin or Vector3d.Zero
	direction = direction or Vector3d.AxisX
	if not bIsNormalized and not direction:IsNormalized() then
		direction = (direction:NormalizedAndLength())
	end
	return setmetatable({ Origin = origin, Direction = direction }, Ray3d)
end

function Ray3d.Clone(self)
	return setmetatable({ Origin = self.Origin, Direction = self.Direction }, Ray3d)
end

function Ray3d.PointAt(self, d: number)
	return self.Origin + d * self.Direction
end

function Ray3d.Project(self, p): number
	return (p - self.Origin):Dot(self.Direction)
end

function Ray3d.DistanceSquared(self, p): number
	local t = (p - self.Origin):Dot(self.Direction)
	if t < 0 then
		return self.Origin:DistanceSquared(p)
	end
	local proj = self.Origin + t * self.Direction
	return (proj - p):LengthSquared()
end

function Ray3d.Distance(self, p): number
	return math.sqrt(self:DistanceSquared(p))
end

function Ray3d.ClosestPoint(self, p)
	local t = (p - self.Origin):Dot(self.Direction)
	if t < 0 then
		return self.Origin
	end
	return self.Origin + t * self.Direction
end

return Ray3d
