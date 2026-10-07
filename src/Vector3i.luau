--!native
-- Port of geometry3Sharp math/Vector3i.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Vector3i values are IMMUTABLE by convention. Integer division follows C# (truncation toward zero).
-- Use v:HashKey() or a linear grid index (not the table itself) as a dictionary key.

local MathUtil = require(script.Parent.MathUtil)
local IntDiv = MathUtil.IntDiv

local Vector3i = {}
Vector3i.__index = Vector3i
Vector3i.ClassName = "Vector3i"

export type Vector3i = typeof(setmetatable({} :: { x: number, y: number, z: number }, Vector3i))

local function new(x: number, y: number, z: number): Vector3i
	return setmetatable({ x = x, y = y, z = z }, Vector3i) :: any
end
Vector3i.new = new

function Vector3i.Splat(f: number): Vector3i
	return new(f, f, f)
end

function Vector3i.FromArray(t: { number }, i0: number?): Vector3i
	local i = (i0 or 0)
	return new(t[i + 1], t[i + 2], t[i + 3])
end

-- explicit operator Vector3i(Vector3d v): truncates like C# (int) cast
function Vector3i.FromVector3d(v: any): Vector3i
	return new(MathUtil.TruncInt(v.x), MathUtil.TruncInt(v.y), MathUtil.TruncInt(v.z))
end

-- implicit operator Vector3i(Index3i v)
function Vector3i.FromIndex3i(t: { number }): Vector3i
	return new(t[1], t[2], t[3])
end

local function const(x, y, z)
	return table.freeze(new(x, y, z))
end
Vector3i.Zero = const(0, 0, 0)
Vector3i.One = const(1, 1, 1)
Vector3i.AxisX = const(1, 0, 0)
Vector3i.AxisY = const(0, 1, 0)
Vector3i.AxisZ = const(0, 0, 1)

function Vector3i.Get(self: Vector3i, key: number): number
	if key == 0 then
		return self.x
	elseif key == 1 then
		return self.y
	end
	return self.z
end

function Vector3i.With(self: Vector3i, key: number, value: number): Vector3i
	if key == 0 then
		return new(value, self.y, self.z)
	elseif key == 1 then
		return new(self.x, value, self.z)
	end
	return new(self.x, self.y, value)
end

-- C# .array property: returns {x, y, z}
function Vector3i.array(self: Vector3i): { number }
	return { self.x, self.y, self.z }
end

-- implicit operator Index3i(Vector3i v)
function Vector3i.ToIndex3i(self: Vector3i): { number }
	return { self.x, self.y, self.z }
end

-- explicit operator Vector3d(Vector3i v)
function Vector3i.ToVector3d(self: Vector3i)
	return require(script.Parent.Vector3d).new(self.x, self.y, self.z)
end

function Vector3i.Clone(self: Vector3i): Vector3i
	return new(self.x, self.y, self.z)
end

function Vector3i.LengthSquared(self: Vector3i): number
	return self.x * self.x + self.y * self.y + self.z * self.z
end

Vector3i.__unm = function(v: Vector3i): Vector3i
	return new(-v.x, -v.y, -v.z)
end
Vector3i.__add = function(a: any, b: any): Vector3i
	if type(b) == "number" then
		return new(a.x + b, a.y + b, a.z + b)
	elseif type(a) == "number" then
		return new(a + b.x, a + b.y, a + b.z)
	end
	return new(a.x + b.x, a.y + b.y, a.z + b.z)
end
Vector3i.__sub = function(a: any, b: any): Vector3i
	if type(b) == "number" then
		return new(a.x - b, a.y - b, a.z - b)
	end
	return new(a.x - b.x, a.y - b.y, a.z - b.z)
end
Vector3i.__mul = function(a: any, b: any): Vector3i
	if type(a) == "number" then
		return new(a * b.x, a * b.y, a * b.z)
	elseif type(b) == "number" then
		return new(b * a.x, b * a.y, b * a.z)
	end
	return new(a.x * b.x, a.y * b.y, a.z * b.z)
end
Vector3i.__div = function(a: any, b: any): Vector3i
	if type(b) == "number" then
		return new(IntDiv(a.x, b), IntDiv(a.y, b), IntDiv(a.z, b))
	elseif type(a) == "number" then
		return new(IntDiv(a, b.x), IntDiv(a, b.y), IntDiv(a, b.z))
	end
	return new(IntDiv(a.x, b.x), IntDiv(a.y, b.y), IntDiv(a.z, b.z))
end
Vector3i.__eq = function(a: Vector3i, b: Vector3i): boolean
	return a.x == b.x and a.y == b.y and a.z == b.z
end
Vector3i.__tostring = function(v: Vector3i): string
	return string.format("%d %d %d", v.x, v.y, v.z)
end

function Vector3i.Equals(self: Vector3i, other: Vector3i): boolean
	return self.x == other.x and self.y == other.y and self.z == other.z
end

function Vector3i.CompareTo(self: Vector3i, other: Vector3i): number
	if self.x ~= other.x then
		return self.x < other.x and -1 or 1
	elseif self.y ~= other.y then
		return self.y < other.y and -1 or 1
	elseif self.z ~= other.z then
		return self.z < other.z and -1 or 1
	end
	return 0
end

function Vector3i.HashKey(self: Vector3i): string
	return self.x .. "," .. self.y .. "," .. self.z
end

return Vector3i
