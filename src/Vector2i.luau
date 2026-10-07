--!native
-- Port of geometry3Sharp math/Vector2i.cs (core subset)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Vector2i values are IMMUTABLE by convention.

local MathUtil = require(script.Parent.MathUtil)
local IntDiv = MathUtil.IntDiv

local Vector2i = {}
Vector2i.__index = Vector2i
Vector2i.ClassName = "Vector2i"

export type Vector2i = typeof(setmetatable({} :: { x: number, y: number }, Vector2i))

local function new(x: number, y: number): Vector2i
	return setmetatable({ x = x, y = y }, Vector2i) :: any
end
Vector2i.new = new

function Vector2i.Splat(f: number): Vector2i
	return new(f, f)
end

local function const(x, y)
	return table.freeze(new(x, y))
end
Vector2i.Zero = const(0, 0)
Vector2i.One = const(1, 1)
Vector2i.AxisX = const(1, 0)
Vector2i.AxisY = const(0, 1)

function Vector2i.Get(self: Vector2i, key: number): number
	return (key == 0) and self.x or self.y
end

function Vector2i.With(self: Vector2i, key: number, value: number): Vector2i
	if key == 0 then
		return new(value, self.y)
	end
	return new(self.x, value)
end

function Vector2i.LengthSquared(self: Vector2i): number
	return self.x * self.x + self.y * self.y
end

Vector2i.__unm = function(v: Vector2i): Vector2i
	return new(-v.x, -v.y)
end
Vector2i.__add = function(a: any, b: any): Vector2i
	if type(b) == "number" then
		return new(a.x + b, a.y + b)
	elseif type(a) == "number" then
		return new(a + b.x, a + b.y)
	end
	return new(a.x + b.x, a.y + b.y)
end
Vector2i.__sub = function(a: any, b: any): Vector2i
	if type(b) == "number" then
		return new(a.x - b, a.y - b)
	end
	return new(a.x - b.x, a.y - b.y)
end
Vector2i.__mul = function(a: any, b: any): Vector2i
	if type(a) == "number" then
		return new(a * b.x, a * b.y)
	elseif type(b) == "number" then
		return new(b * a.x, b * a.y)
	end
	return new(a.x * b.x, a.y * b.y)
end
Vector2i.__div = function(a: any, b: any): Vector2i
	if type(b) == "number" then
		return new(IntDiv(a.x, b), IntDiv(a.y, b))
	end
	return new(IntDiv(a.x, b.x), IntDiv(a.y, b.y))
end
Vector2i.__eq = function(a: Vector2i, b: Vector2i): boolean
	return a.x == b.x and a.y == b.y
end
Vector2i.__tostring = function(v: Vector2i): string
	return string.format("%d %d", v.x, v.y)
end

function Vector2i.HashKey(self: Vector2i): string
	return self.x .. "," .. self.y
end

return Vector2i
