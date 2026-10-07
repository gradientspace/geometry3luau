--!native
-- Port of geometry3Sharp math/Line2.cs (Line2d; also used for Line2f)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Fields Origin, Direction (Vector2d).
--   Line2d.new(origin, direction), Line2d.FromPoints(p0, p1)
--   line:IntersectionPoint(other, dotThresh?) returns Vector2d.MaxValue if lines are parallel

local MathUtil = require(script.Parent.MathUtil)
local Vector2d = require(script.Parent.Vector2d)

local Line2d = {}
Line2d.__index = Line2d
Line2d.ClassName = "Line2d"

function Line2d.new(origin, direction)
	return setmetatable({ Origin = origin, Direction = direction }, Line2d)
end

function Line2d.FromPoints(p0, p1)
	return Line2d.new(p0, (p1 - p0):Normalized())
end

function Line2d:Clone()
	return Line2d.new(self.Origin, self.Direction)
end

-- parameter is distance along Line
function Line2d:PointAt(d: number)
	return self.Origin + d * self.Direction
end

function Line2d:Project(p): number
	return (p - self.Origin):Dot(self.Direction)
end

function Line2d:DistanceSquared(p): number
	local t = (p - self.Origin):Dot(self.Direction)
	local proj = self.Origin + t * self.Direction
	return (proj - p):LengthSquared()
end

-- Returns:
--   +1, on right of line
--   -1, on left of line
--    0, on the line
function Line2d:WhichSide(test, tol: number?): number
	local tl = tol or 0
	local x0 = test.x - self.Origin.x
	local y0 = test.y - self.Origin.y
	local x1 = self.Direction.x
	local y1 = self.Direction.y
	local det = x0 * y1 - x1 * y0
	return (det > tl) and 1 or ((det < -tl) and -1 or 0)
end

-- Calculate intersection point between this line and another one.
-- Returns Vector2d.MaxValue if lines are parallel.
function Line2d:IntersectionPoint(other, dotThresh: number?)
	local thresh = dotThresh or MathUtil.ZeroTolerance
	-- see IntrLine2Line2 for explanation of algorithm
	local diff = other.Origin - self.Origin
	local D0DotPerpD1 = self.Direction:DotPerp(other.Direction)
	if math.abs(D0DotPerpD1) > thresh then -- Lines intersect in a single point.
		local invD0DotPerpD1 = 1 / D0DotPerpD1
		local diffDotPerpD1 = diff:DotPerp(other.Direction)
		local s = diffDotPerpD1 * invD0DotPerpD1
		return self.Origin + s * self.Direction
	end
	-- Lines are parallel.
	return Vector2d.MaxValue
end

return Line2d
