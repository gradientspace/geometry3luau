--!native
-- Port of geometry3Sharp math/Segment3.cs (Segment3d; also used for Segment3f)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Mutable struct with fields Center, Direction (unit), Extent. P0/P1 are methods (P0(), SetP0(v)).
-- Constructors: Segment3d.new(p0, p1)  (from endpoints)
--               Segment3d.FromCenter(center, direction, extent)

local Segment3d = {}
Segment3d.__index = Segment3d
Segment3d.ClassName = "Segment3d"

local function update_from_endpoints(self, p0, p1)
	self.Center = 0.5 * (p0 + p1)
	local dir, len = (p1 - p0):NormalizedAndLength()
	self.Direction = dir
	self.Extent = 0.5 * len
end

function Segment3d.new(p0, p1)
	local s = setmetatable({}, Segment3d)
	update_from_endpoints(s, p0, p1)
	return s
end

function Segment3d.FromCenter(center, direction, extent: number)
	return setmetatable({ Center = center, Direction = direction, Extent = extent }, Segment3d)
end

function Segment3d.Clone(self)
	return Segment3d.FromCenter(self.Center, self.Direction, self.Extent)
end

function Segment3d.SetEndpoints(self, p0, p1)
	update_from_endpoints(self, p0, p1)
end

function Segment3d.P0(self)
	return self.Center - self.Extent * self.Direction
end
function Segment3d.SetP0(self, value)
	update_from_endpoints(self, value, self:P1())
end
function Segment3d.P1(self)
	return self.Center + self.Extent * self.Direction
end
function Segment3d.SetP1(self, value)
	update_from_endpoints(self, self:P0(), value)
end

function Segment3d.Length(self): number
	return 2 * self.Extent
end

-- parameter is signed distance from center in direction
function Segment3d.PointAt(self, d: number)
	return self.Center + d * self.Direction
end

-- t ranges from [0,1] over [P0,P1]
function Segment3d.PointBetween(self, t: number)
	return self.Center + (2 * t - 1) * self.Extent * self.Direction
end

function Segment3d.DistanceSquared(self, p): number
	local t = (p - self.Center):Dot(self.Direction)
	if t >= self.Extent then
		return self:P1():DistanceSquared(p)
	elseif t <= -self.Extent then
		return self:P0():DistanceSquared(p)
	end
	local proj = self.Center + t * self.Direction
	return (proj - p):LengthSquared()
end

-- C# DistanceSquared(Vector3d p, out double t) -> returns distSqr, t
function Segment3d.DistanceSquaredT(self, p): (number, number)
	local t = (p - self.Center):Dot(self.Direction)
	if t >= self.Extent then
		return self:P1():DistanceSquared(p), self.Extent
	elseif t <= -self.Extent then
		return self:P0():DistanceSquared(p), -self.Extent
	end
	local proj = self.Center + t * self.Direction
	return (proj - p):LengthSquared(), t
end

function Segment3d.NearestPoint(self, p)
	local t = (p - self.Center):Dot(self.Direction)
	if t >= self.Extent then
		return self:P1()
	end
	if t <= -self.Extent then
		return self:P0()
	end
	return self.Center + t * self.Direction
end

function Segment3d.Project(self, p): number
	return (p - self.Center):Dot(self.Direction)
end

-- IParametricCurve3d interface
function Segment3d.IsClosed(self): boolean
	return false
end
function Segment3d.ParamLength(self): number
	return 1.0
end
function Segment3d.SampleT(self, t: number)
	return self.Center + (2 * t - 1) * self.Extent * self.Direction
end
function Segment3d.TangentT(self, t: number)
	return self.Direction
end
function Segment3d.HasArcLength(self): boolean
	return true
end
function Segment3d.ArcLength(self): number
	return 2 * self.Extent
end
function Segment3d.SampleArcLength(self, a: number)
	return self:P0() + a * self.Direction
end
function Segment3d.Reverse(self)
	update_from_endpoints(self, self:P1(), self:P0())
end

return Segment3d
