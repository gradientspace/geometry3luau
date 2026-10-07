--!native
-- Port of geometry3Sharp math/Interval1d.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Interval1d is a mutable struct in C# (Contain, Set). Use :Clone() where C# would copy it.
-- Constructors: Interval1d.new(a, b), Interval1d.Splat(f), Interval1d.Unsorted(x, y),
--               Interval1d.Zero(), Interval1d.Empty(), Interval1d.Infinite()

local MathUtil = require(script.Parent.MathUtil)

local Interval1d = {}
Interval1d.__index = Interval1d
Interval1d.ClassName = "Interval1d"

export type Interval1d = typeof(setmetatable({} :: { a: number, b: number }, Interval1d))

local function new(a: number, b: number): Interval1d
	return setmetatable({ a = a, b = b }, Interval1d) :: any
end
Interval1d.new = new

function Interval1d.Splat(f: number): Interval1d
	return new(f, f)
end
function Interval1d.Zero(): Interval1d
	return new(0, 0)
end
function Interval1d.Empty(): Interval1d
	return new(MathUtil.MaxDouble, -MathUtil.MaxDouble)
end
function Interval1d.Infinite(): Interval1d
	return new(-MathUtil.MaxDouble, MathUtil.MaxDouble)
end
function Interval1d.Unsorted(x: number, y: number): Interval1d
	if x < y then
		return new(x, y)
	end
	return new(y, x)
end

function Interval1d.Clone(self: Interval1d): Interval1d
	return new(self.a, self.b)
end

function Interval1d.Get(self: Interval1d, key: number): number
	return (key == 0) and self.a or self.b
end

function Interval1d.LengthSquared(self: Interval1d): number
	return (self.a - self.b) * (self.a - self.b)
end
function Interval1d.Length(self: Interval1d): number
	return self.b - self.a
end
function Interval1d.IsConstant(self: Interval1d): boolean
	return self.b == self.a
end
function Interval1d.Center(self: Interval1d): number
	return (self.b + self.a) * 0.5
end

function Interval1d.Contain(self: Interval1d, d: number)
	if d < self.a then
		self.a = d
	end
	if d > self.b then
		self.b = d
	end
end

function Interval1d.Contains(self: Interval1d, d: number): boolean
	return d >= self.a and d <= self.b
end

function Interval1d.Overlaps(self: Interval1d, o: Interval1d): boolean
	return not (o.a > self.b or o.b < self.a)
end

function Interval1d.SquaredDist(self: Interval1d, o: Interval1d): number
	if self.b < o.a then
		return (o.a - self.b) * (o.a - self.b)
	elseif self.a > o.b then
		return (self.a - o.b) * (self.a - o.b)
	end
	return 0
end

function Interval1d.Dist(self: Interval1d, o: Interval1d): number
	if self.b < o.a then
		return o.a - self.b
	elseif self.a > o.b then
		return self.a - o.b
	end
	return 0
end

function Interval1d.IntersectionWith(self: Interval1d, o: Interval1d): Interval1d
	if o.a > self.b or o.b < self.a then
		return Interval1d.Empty()
	end
	return new(math.max(self.a, o.a), math.min(self.b, o.b))
end

-- clamp value f to interval [a,b]
function Interval1d.Clamp(self: Interval1d, f: number): number
	return (f < self.a) and self.a or ((f > self.b) and self.b or f)
end

-- interpolate between a and b using value t in range [0,1]
function Interval1d.Interpolate(self: Interval1d, t: number): number
	return (1 - t) * self.a + t * self.b
end

-- Convert value into (clamped) t value in range [0,1]
function Interval1d.GetT(self: Interval1d, value: number): number
	if value <= self.a then
		return 0
	elseif value >= self.b then
		return 1
	elseif self.a == self.b then
		return 0.5
	end
	return (value - self.a) / (self.b - self.a)
end

-- Set(Interval1d o) or Set(a, b)
function Interval1d.Set(self: Interval1d, a: any, b: number?)
	if type(a) == "table" then
		self.a, self.b = a.a, a.b
	else
		self.a, self.b = a, b :: number
	end
end

Interval1d.__unm = function(v: Interval1d): Interval1d
	return new(-v.a, -v.b)
end
Interval1d.__add = function(v: Interval1d, f: number): Interval1d
	return new(v.a + f, v.b + f)
end
Interval1d.__sub = function(v: Interval1d, f: number): Interval1d
	return new(v.a - f, v.b - f)
end
Interval1d.__mul = function(v: any, f: any): Interval1d
	if type(v) == "number" then
		v, f = f, v
	end
	return new(v.a * f, v.b * f)
end
Interval1d.__tostring = function(v: Interval1d): string
	return string.format("[%.8f,%.8f]", v.a, v.b)
end

return Interval1d
