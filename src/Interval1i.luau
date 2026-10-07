--!native
-- Port of geometry3Sharp math/Interval1i.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Interval1i is a mutable struct in C#. Use :Clone() where C# would copy it.
-- C# `foreach (int i in interval)` -> `for i in interval:Indices() do` (or `for i = iv.a, iv.b do`)

local MathUtil = require(script.Parent.MathUtil)

local Interval1i = {}
Interval1i.__index = Interval1i
Interval1i.ClassName = "Interval1i"

export type Interval1i = typeof(setmetatable({} :: { a: number, b: number }, Interval1i))

local function new(a: number, b: number): Interval1i
	return setmetatable({ a = a, b = b }, Interval1i) :: any
end
Interval1i.new = new

function Interval1i.Splat(f: number): Interval1i
	return new(f, f)
end
function Interval1i.Zero(): Interval1i
	return new(0, 0)
end
function Interval1i.Empty(): Interval1i
	return new(MathUtil.MaxInt, -MathUtil.MaxInt)
end
function Interval1i.Infinite(): Interval1i
	return new(-MathUtil.MaxInt, MathUtil.MaxInt)
end

-- Range(N) is [0,N-1], or Range(start, N) is [start, start+N-1]
function Interval1i.Range(a: number, N: number?): Interval1i
	if N == nil then
		return new(0, a - 1)
	end
	return new(a, a + N - 1)
end
-- RangeInclusive(N) is [0,N]
function Interval1i.RangeInclusive(N: number): Interval1i
	return new(0, N)
end
function Interval1i.FromToInclusive(a: number, b: number): Interval1i
	return new(a, b)
end

function Interval1i.Clone(self: Interval1i): Interval1i
	return new(self.a, self.b)
end

function Interval1i.Get(self: Interval1i, key: number): number
	return (key == 0) and self.a or self.b
end

function Interval1i.LengthSquared(self: Interval1i): number
	return (self.a - self.b) * (self.a - self.b)
end
function Interval1i.Length(self: Interval1i): number
	return self.b - self.a
end
function Interval1i.Center(self: Interval1i): number
	return MathUtil.IntDiv(self.b + self.a, 2)
end

function Interval1i.Contain(self: Interval1i, d: number)
	if d < self.a then
		self.a = d
	end
	if d > self.b then
		self.b = d
	end
end

function Interval1i.Contains(self: Interval1i, d: number): boolean
	return d >= self.a and d <= self.b
end

function Interval1i.Overlaps(self: Interval1i, o: Interval1i): boolean
	return not (o.a > self.b or o.b < self.a)
end

function Interval1i.SquaredDist(self: Interval1i, o: Interval1i): number
	if self.b < o.a then
		return (o.a - self.b) * (o.a - self.b)
	elseif self.a > o.b then
		return (self.a - o.b) * (self.a - o.b)
	end
	return 0
end

function Interval1i.Dist(self: Interval1i, o: Interval1i): number
	if self.b < o.a then
		return o.a - self.b
	elseif self.a > o.b then
		return self.a - o.b
	end
	return 0
end

function Interval1i.Set(self: Interval1i, a: any, b: number?)
	if type(a) == "table" then
		self.a, self.b = a.a, a.b
	else
		self.a, self.b = a, b :: number
	end
end

-- IEnumerable<int>: iterate a..b inclusive
function Interval1i.Indices(self: Interval1i): () -> number?
	local i, last = self.a - 1, self.b
	return function()
		i += 1
		if i <= last then
			return i
		end
		return nil
	end
end

Interval1i.__unm = function(v: Interval1i): Interval1i
	return new(-v.a, -v.b)
end
Interval1i.__add = function(v: Interval1i, f: number): Interval1i
	return new(v.a + f, v.b + f)
end
Interval1i.__sub = function(v: Interval1i, f: number): Interval1i
	return new(v.a - f, v.b - f)
end
Interval1i.__mul = function(v: any, f: any): Interval1i
	if type(v) == "number" then
		v, f = f, v
	end
	return new(v.a * f, v.b * f)
end
Interval1i.__tostring = function(v: Interval1i): string
	return string.format("[%d,%d]", v.a, v.b)
end

return Interval1i
