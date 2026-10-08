--!native
-- Port of geometry3Sharp math/Segment2.cs (Segment2d; Segment2f is not ported, see PORTING.md)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Mutable struct with fields Center, Direction (unit Vector2d), Extent. Use :Clone() where C# copies it.
-- Constructors: Segment2d.new(p0, p1)  (from endpoints)
--               Segment2d.FromCenter(center, direction, extent)
-- C# properties -> methods: P0(), SetP0(v), P1(), SetP1(v), Length(), IsClosed(), ParamLength(),
--   HasArcLength(), ArcLength(), IsTransformable()
-- Overloads: DistanceSquared(p) / DistanceSquaredT(p) -> distSqr, DistanceParam  (C# out DistanceParam)
--   static Segment2d.WhichSideStatic(a, b, test, tol?)   (C# static WhichSide(ref a, ref b, ref test, tol))
--   static Segment2d.FastDistanceSquared(a, b, pt)
-- Transform(xform) uses a duck-typed ITransform2 (TransformP, TransformN, TransformScalar).
-- (The C# Segment2dBox reference wrapper is not ported: Luau tables are already references.)

local MathUtil = require(script.Parent.MathUtil)
local Interval1d = require(script.Parent.Interval1d)

local abs = math.abs

local Segment2d = {}
Segment2d.__index = Segment2d
Segment2d.ClassName = "Segment2d"

local function update_from_endpoints(self, p0, p1)
	self.Center = 0.5 * (p0 + p1)
	local dir, len = (p1 - p0):NormalizedAndLength()
	self.Direction = dir
	self.Extent = 0.5 * len
end

function Segment2d.new(p0, p1)
	local s = setmetatable({}, Segment2d)
	update_from_endpoints(s, p0, p1)
	return s
end

function Segment2d.FromCenter(center, direction, extent: number)
	return setmetatable({ Center = center, Direction = direction, Extent = extent }, Segment2d)
end

function Segment2d:Clone()
	return Segment2d.FromCenter(self.Center, self.Direction, self.Extent)
end

function Segment2d:P0()
	return self.Center - self.Extent * self.Direction
end
function Segment2d:SetP0(value)
	update_from_endpoints(self, value, self:P1())
end
function Segment2d:P1()
	return self.Center + self.Extent * self.Direction
end
function Segment2d:SetP1(value)
	update_from_endpoints(self, self:P0(), value)
end
function Segment2d:Length(): number
	return 2 * self.Extent
end

function Segment2d:Endpoint(i: number)
	return (i == 0) and (self.Center - self.Extent * self.Direction) or (self.Center + self.Extent * self.Direction)
end

-- DistanceParam is signed distance from segment center/origin
function Segment2d:PointAt(DistanceParam: number)
	return self.Center + DistanceParam * self.Direction
end

-- UnitParam ranges from [0,1] over [P0,P1]
function Segment2d:PointBetween(UnitParam: number)
	return self.Center + (2 * UnitParam - 1) * self.Extent * self.Direction
end

-- return squared distance from P to line segment
function Segment2d:DistanceSquared(p): number
	local t = (p - self.Center):Dot(self.Direction)
	if t >= self.Extent then
		return self:P1():DistanceSquared(p)
	elseif t <= -self.Extent then
		return self:P0():DistanceSquared(p)
	end
	local proj = self.Center + t * self.Direction
	return proj:DistanceSquared(p)
end

-- return squared distance from P to line segment, return clamped segment parameter in range [-Extent,Extent]
function Segment2d:DistanceSquaredT(p): (number, number)
	local DistanceParam = (p - self.Center):Dot(self.Direction)
	if DistanceParam >= self.Extent then
		DistanceParam = self.Extent
		return self:P1():DistanceSquared(p), DistanceParam
	elseif DistanceParam <= -self.Extent then
		DistanceParam = -self.Extent
		return self:P0():DistanceSquared(p), DistanceParam
	end
	local proj = self.Center + DistanceParam * self.Direction
	return proj:DistanceSquared(p), DistanceParam
end

-- return nearest point on segment to P
function Segment2d:NearestPoint(p)
	local t = (p - self.Center):Dot(self.Direction)
	if t >= self.Extent then
		return self:P1()
	end
	if t <= -self.Extent then
		return self:P0()
	end
	return self.Center + t * self.Direction
end

-- return distance from segment origin to closest point to P along segment line (*not* clamped to segment Extents!)
function Segment2d:Project(p): number
	return (p - self.Center):Dot(self.Direction)
end

-- find distance parameter in range [-Extent,Extent] of nearest segment point to Point
function Segment2d:ProjectToSegmentParam(Point): number
	local t = (Point - self.Center):Dot(self.Direction)
	return math.clamp(t, -self.Extent, self.Extent)
end

-- find unit parameter in range [0,1] of nearest segment point to Point
function Segment2d:ProjectToUnitParam(Point): number
	local t = (Point - self.Center):Dot(self.Direction)
	t = math.clamp(t, -self.Extent, self.Extent)
	return math.clamp((t + self.Extent) / (2 * self.Extent), 0, 1)
end

-- convert distance parameter to unit parameter along segment in range [0,1]   (clamps distance to [-Extent,Extent])
function Segment2d:DistanceToUnitParam(DistanceParam: number): number
	local t = math.clamp(DistanceParam, -self.Extent, self.Extent)
	return math.clamp((t + self.Extent) / (2 * self.Extent), 0, 1)
end

-- Returns:
--   +1, on right of line
--   -1, on left of line
--    0, on the line
function Segment2d:WhichSide(test, tol: number?): number
	local tl = tol or 0
	-- [TODO] subtract Center from test?
	local vec0 = self.Center + self.Extent * self.Direction
	local vec1 = self.Center - self.Extent * self.Direction
	local x0 = test.x - vec0.x
	local y0 = test.y - vec0.y
	local x1 = vec1.x - vec0.x
	local y1 = vec1.y - vec0.y
	local det = x0 * y1 - x1 * y0
	return (det > tl) and 1 or ((det < -tl) and -1 or 0)
end

-- IParametricCurve2d interface

function Segment2d:IsClosed(): boolean
	return false
end
function Segment2d:ParamLength(): number
	return 1.0
end
-- t in range[0,1] spans arc
function Segment2d:SampleT(t: number)
	return self.Center + (2 * t - 1) * self.Extent * self.Direction
end
function Segment2d:TangentT(t: number)
	return self.Direction
end
function Segment2d:HasArcLength(): boolean
	return true
end
function Segment2d:ArcLength(): number
	return 2 * self.Extent
end
function Segment2d:SampleArcLength(a: number)
	return self:P0() + a * self.Direction
end
function Segment2d:Reverse()
	update_from_endpoints(self, self:P1(), self:P0())
end
function Segment2d:IsTransformable(): boolean
	return true
end
function Segment2d:Transform(xform)
	self.Center = xform:TransformP(self.Center)
	self.Direction = xform:TransformN(self.Direction)
	self.Extent = xform:TransformScalar(self.Extent)
end

-- distance from pt to segment (a,b), with no square roots
function Segment2d.FastDistanceSquared(a, b, pt): number
	local vx, vy = b.x - a.x, b.y - a.y
	local len2 = vx * vx + vy * vy
	local dx, dy = pt.x - a.x, pt.y - a.y
	if len2 < 1e-13 then
		return dx * dx + dy * dy
	end
	local t = (dx * vx + dy * vy)
	if t <= 0 then
		return dx * dx + dy * dy
	elseif t >= len2 then
		dx = pt.x - b.x
		dy = pt.y - b.y
		return dx * dx + dy * dy
	end
	dx = pt.x - (a.x + ((t * vx) / len2))
	dy = pt.y - (a.y + ((t * vy) / len2))
	return dx * dx + dy * dy
end

-- Returns:
--   +1, on right of line
--   -1, on left of line
--    0, on the line
-- (C# static int WhichSide(ref a, ref b, ref test, tol))
function Segment2d.WhichSideStatic(a, b, test, tol: number?): number
	local tl = tol or 0
	local x0 = test.x - a.x
	local y0 = test.y - a.y
	local x1 = b.x - a.x
	local y1 = b.y - a.y
	local det = x0 * y1 - x1 * y0
	return (det > tl) and 1 or ((det < -tl) and -1 or 0)
end

-- Test if segments intersect. Returns true for parallel-line overlaps.
-- Returns same result as IntrSegment2Segment2.
-- (C# default dotThresh = double.Epsilon, the smallest positive double)
function Segment2d:Intersects(seg2, dotThresh: number?, intervalThresh: number?): boolean
	local dt = dotThresh or 4.9406564584124654e-324
	local it = intervalThresh or 0
	-- see IntrLine2Line2 and IntrSegment2Segment2 for details on this code

	local diff = seg2.Center - self.Center
	local D0DotPerpD1 = self.Direction:DotPerp(seg2.Direction)
	if abs(D0DotPerpD1) > dt then -- Lines intersect in a single point.
		local invD0DotPerpD1 = 1 / D0DotPerpD1
		local diffDotPerpD0 = diff:DotPerp(self.Direction)
		local diffDotPerpD1 = diff:DotPerp(seg2.Direction)
		local s = diffDotPerpD1 * invD0DotPerpD1
		local s2 = diffDotPerpD0 * invD0DotPerpD1
		return abs(s) <= (self.Extent + it) and abs(s2) <= (seg2.Extent + it)
	end

	-- Lines are parallel.
	diff = (diff:NormalizedAndLength())
	local diffNDotPerpD1 = diff:DotPerp(seg2.Direction)
	if abs(diffNDotPerpD1) <= dt then
		-- Compute the location of segment1 endpoints relative to segment0.
		diff = seg2.Center - self.Center
		local t1 = self.Direction:Dot(diff)
		local tmin = t1 - seg2.Extent
		local tmax = t1 + seg2.Extent
		local extents = Interval1d.new(-self.Extent, self.Extent)
		if extents:Overlaps(Interval1d.new(tmin, tmax)) then
			return true
		end
		return false
	end

	-- lines are parallel but not collinear
	return false
end

local _ = MathUtil
return Segment2d
