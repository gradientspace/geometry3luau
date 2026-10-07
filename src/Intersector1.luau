--!native
-- Port of geometry3Sharp intersection/Intersector1.cs (ported from WildMagic5)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- A class for intersection of intervals [u0,u1] and [v0,v1].  The end
-- points must be ordered:  u0 <= u1 and v0 <= v1.  Values of MAX_REAL
-- and -MAX_REAL are allowed, and degenerate intervals are allowed:
-- u0 = u1 or v0 = v1.
--
-- Luau API notes:
--   Intersector1.new(u0, u1, v0, v1)          Intersector1.FromIntervals(u, v)  (Interval1d)
--   fields U, V (Interval1d), NumIntersections; C# property Test -> method intr:Test()
--   intr:Find() -> bool, intr:GetIntersection(i) (i = 0 or 1)

local Interval1d = require(script.Parent.Interval1d)

local Intersector1 = {}
Intersector1.__index = Intersector1
Intersector1.ClassName = "Intersector1"

function Intersector1.new(u0: number, u1: number, v0: number, v1: number)
	-- [TODO] validate 0 < 1
	return setmetatable({
		U = Interval1d.new(u0, u1),
		V = Interval1d.new(v0, v1),
		NumIntersections = 0,
		Intersections = Interval1d.Zero(),
	}, Intersector1)
end

function Intersector1.FromIntervals(u, v)
	return setmetatable({
		U = u,
		V = v,
		NumIntersections = 0,
		Intersections = Interval1d.Zero(),
	}, Intersector1)
end

function Intersector1:Test(): boolean
	return self.U.a <= self.V.b and self.U.b >= self.V.a
end

function Intersector1:GetIntersection(i: number): number
	return (i == 0) and self.Intersections.a or self.Intersections.b
end

function Intersector1:Find(): boolean
	local U, V, I = self.U, self.V, self.Intersections
	if U.b < V.a or U.a > V.b then
		self.NumIntersections = 0
	elseif U.b > V.a then
		if U.a < V.b then
			self.NumIntersections = 2
			I.a = (U.a < V.a) and V.a or U.a
			I.b = (U.b > V.b) and V.b or U.b
			if I.a == I.b then
				self.NumIntersections = 1
			end
		else
			-- U.a == V.b
			self.NumIntersections = 1
			I.a = U.a
		end
	else
		-- U.b == V.a
		self.NumIntersections = 1
		I.a = U.b
	end
	return self.NumIntersections > 0
end

return Intersector1
