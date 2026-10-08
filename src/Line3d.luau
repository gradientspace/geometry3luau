--!native
-- Port of geometry3Sharp math/Line3.cs (Line3d; Line3f is not ported, see PORTING.md)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Fields Origin, Direction (Vector3d).

local Line3d = {}
Line3d.__index = Line3d
Line3d.ClassName = "Line3d"

function Line3d.new(origin, direction)
	return setmetatable({ Origin = origin, Direction = direction }, Line3d)
end

function Line3d.Clone(self)
	return Line3d.new(self.Origin, self.Direction)
end

-- parameter is distance along ray
function Line3d.PointAt(self, d: number)
	return self.Origin + d * self.Direction
end

function Line3d.Project(self, p): number
	return (p - self.Origin):Dot(self.Direction)
end

function Line3d.DistanceSquared(self, p): number
	local t = (p - self.Origin):Dot(self.Direction)
	local proj = self.Origin + t * self.Direction
	return (proj - p):LengthSquared()
end

function Line3d.ClosestPoint(self, p)
	local t = (p - self.Origin):Dot(self.Direction)
	return self.Origin + t * self.Direction
end

return Line3d
