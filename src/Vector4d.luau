--!native
-- Port of geometry3Sharp math/Vector4d.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Vector4d values are IMMUTABLE by convention: never assign to .x/.y/.z/.w of an existing vector.
-- Operators: +, - (vector or scalar), * (scalar*v, v*scalar, v*v componentwise), / , unary -, ==
--
-- Constructors:
--   Vector4d.new(x, y, z, w)      new Vector4d(x, y, z, w)
--   Vector4d.Splat(f)             new Vector4d(f)
--   Vector4d.FromArray(t, i0?)    new Vector4d(double[] v) (reads t[i0+1..i0+4], i0 defaults to 0)
--
-- API differences:
--   * C# indexer get v[key] -> v:Get(key) (0-based); set v[key] = f -> v = v:With(key, f).
--   * C# `double len = v.Normalize(eps)` -> `v, len = v:NormalizedAndLength(eps)`.
--   * C# Round(nDecimals) (mutating) -> v = v:RoundFrac(nDecimals).
--   * C# GetHashCode -> v:HashKey() (string key for hash tables).

local MathUtil = require(script.Parent.MathUtil)

local sqrt, abs, acos = math.sqrt, math.abs, math.acos
local Epsilon = MathUtil.Epsilon
local ZeroTolerance = MathUtil.ZeroTolerance
local Rad2Deg = MathUtil.Rad2Deg

local Vector4d = {}
Vector4d.__index = Vector4d
Vector4d.ClassName = "Vector4d"

export type Vector4d = typeof(setmetatable({} :: { x: number, y: number, z: number, w: number }, Vector4d))

local function new(x: number, y: number, z: number, w: number): Vector4d
	return setmetatable({ x = x, y = y, z = z, w = w }, Vector4d) :: any
end
Vector4d.new = new

function Vector4d.Splat(f: number): Vector4d
	return new(f, f, f, f)
end

function Vector4d.FromArray(t: { number }, i0: number?): Vector4d
	local i = (i0 or 0)
	return new(t[i + 1], t[i + 2], t[i + 3], t[i + 4])
end

function Vector4d.IsVector4d(v: any): boolean
	return getmetatable(v) == Vector4d
end

local function const(x, y, z, w)
	return table.freeze(new(x, y, z, w))
end

Vector4d.Zero = const(0, 0, 0, 0)
Vector4d.One = const(1, 1, 1, 1)
Vector4d.UnitX = const(1, 0, 0, 0)
Vector4d.UnitY = const(0, 1, 0, 0)
Vector4d.UnitZ = const(0, 0, 1, 0)
Vector4d.UnitW = const(0, 0, 0, 1)

-- C# indexer get: v[key], key in 0..3
function Vector4d.Get(self: Vector4d, key: number): number
	if key < 2 then
		return (key == 0) and self.x or self.y
	end
	return (key == 2) and self.z or self.w
end

-- replacement for C# indexer set (v[key] = value): returns a new vector with component key replaced
function Vector4d.With(self: Vector4d, key: number, value: number): Vector4d
	if key == 0 then
		return new(value, self.y, self.z, self.w)
	elseif key == 1 then
		return new(self.x, value, self.z, self.w)
	elseif key == 2 then
		return new(self.x, self.y, value, self.w)
	end
	return new(self.x, self.y, self.z, value)
end

function Vector4d.Clone(self: Vector4d): Vector4d
	return new(self.x, self.y, self.z, self.w)
end

function Vector4d.LengthSquared(self: Vector4d): number
	local x, y, z, w = self.x, self.y, self.z, self.w
	return x * x + y * y + z * z + w * w
end

function Vector4d.Length(self: Vector4d): number
	local x, y, z, w = self.x, self.y, self.z, self.w
	return sqrt(x * x + y * y + z * z + w * w)
end

function Vector4d.LengthL1(self: Vector4d): number
	return abs(self.x) + abs(self.y) + abs(self.z) + abs(self.w)
end

-- C# `double len = v.Normalize(epsilon)` -> `v, len = v:NormalizedAndLength(epsilon)`
function Vector4d.NormalizedAndLength(self: Vector4d, epsilon: number?): (Vector4d, number)
	local length = self:Length()
	if length > (epsilon or Epsilon) then
		local invLength = 1.0 / length
		return new(self.x * invLength, self.y * invLength, self.z * invLength, self.w * invLength), length
	end
	return new(0, 0, 0, 0), 0
end

function Vector4d.Normalized(self: Vector4d): Vector4d
	local length = self:Length()
	if length > Epsilon then
		local invLength = 1.0 / length
		return new(self.x * invLength, self.y * invLength, self.z * invLength, self.w * invLength)
	end
	return Vector4d.Zero
end

function Vector4d.IsNormalized(self: Vector4d): boolean
	local x, y, z, w = self.x, self.y, self.z, self.w
	return abs((x * x + y * y + z * z + w * w) - 1) < ZeroTolerance
end

function Vector4d.IsFinite(self: Vector4d): boolean
	local f = self.x + self.y + self.z + self.w
	return f == f and f ~= math.huge and f ~= -math.huge
end

function Vector4d.RoundFrac(self: Vector4d, nDecimals: number): Vector4d
	return new(
		MathUtil.Round(self.x, nDecimals),
		MathUtil.Round(self.y, nDecimals),
		MathUtil.Round(self.z, nDecimals),
		MathUtil.Round(self.w, nDecimals)
	)
end

function Vector4d.Dot(self: Vector4d, v2: Vector4d): number
	return self.x * v2.x + self.y * v2.y + self.z * v2.z + self.w * v2.w
end

function Vector4d.AngleD(self: Vector4d, v2: Vector4d): number
	local fDot = MathUtil.Clamp(self:Dot(v2), -1, 1)
	return acos(fDot) * Rad2Deg
end

function Vector4d.AngleR(self: Vector4d, v2: Vector4d): number
	local fDot = MathUtil.Clamp(self:Dot(v2), -1, 1)
	return acos(fDot)
end

function Vector4d.DistanceSquared(self: Vector4d, v2: Vector4d): number
	local dx, dy, dz, dw = v2.x - self.x, v2.y - self.y, v2.z - self.z, v2.w - self.w
	return dx * dx + dy * dy + dz * dz + dw * dw
end

function Vector4d.Distance(self: Vector4d, v2: Vector4d): number
	local dx, dy, dz, dw = v2.x - self.x, v2.y - self.y, v2.z - self.z, v2.w - self.w
	return sqrt(dx * dx + dy * dy + dz * dz + dw * dw)
end

Vector4d.__unm = function(v: Vector4d): Vector4d
	return new(-v.x, -v.y, -v.z, -v.w)
end

Vector4d.__add = function(a: any, b: any): Vector4d
	if type(b) == "number" then
		return new(a.x + b, a.y + b, a.z + b, a.w + b)
	elseif type(a) == "number" then
		return new(a + b.x, a + b.y, a + b.z, a + b.w)
	end
	return new(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w)
end

Vector4d.__sub = function(a: any, b: any): Vector4d
	if type(b) == "number" then
		return new(a.x - b, a.y - b, a.z - b, a.w - b)
	end
	return new(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w)
end

Vector4d.__mul = function(a: any, b: any): Vector4d
	if type(a) == "number" then
		return new(a * b.x, a * b.y, a * b.z, a * b.w)
	elseif type(b) == "number" then
		return new(b * a.x, b * a.y, b * a.z, b * a.w)
	end
	return new(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w)
end

Vector4d.__div = function(a: any, b: any): Vector4d
	if type(b) == "number" then
		return new(a.x / b, a.y / b, a.z / b, a.w / b)
	elseif type(a) == "number" then
		return new(a / b.x, a / b.y, a / b.z, a / b.w)
	end
	return new(a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w)
end

Vector4d.__eq = function(a: Vector4d, b: Vector4d): boolean
	return a.x == b.x and a.y == b.y and a.z == b.z and a.w == b.w
end

Vector4d.__tostring = function(v: Vector4d): string
	return string.format("%.8f %.8f %.8f %.8f", v.x, v.y, v.z, v.w)
end

function Vector4d.ToString(self: Vector4d, fmt: string?): string
	if fmt then
		return string.format(fmt .. " " .. fmt .. " " .. fmt .. " " .. fmt, self.x, self.y, self.z, self.w)
	end
	return tostring(self)
end

function Vector4d.Equals(self: Vector4d, other: Vector4d): boolean
	return self.x == other.x and self.y == other.y and self.z == other.z and self.w == other.w
end

function Vector4d.CompareTo(self: Vector4d, other: Vector4d): number
	if self.x ~= other.x then
		return self.x < other.x and -1 or 1
	elseif self.y ~= other.y then
		return self.y < other.y and -1 or 1
	elseif self.z ~= other.z then
		return self.z < other.z and -1 or 1
	elseif self.w ~= other.w then
		return self.w < other.w and -1 or 1
	end
	return 0
end

function Vector4d.EpsilonEqual(self: Vector4d, v2: Vector4d, epsilon: number): boolean
	return abs(self.x - v2.x) <= epsilon
		and abs(self.y - v2.y) <= epsilon
		and abs(self.z - v2.z) <= epsilon
		and abs(self.w - v2.w) <= epsilon
end

-- string key for use in hash tables (C# GetHashCode/Equals for dictionary keys)
function Vector4d.HashKey(self: Vector4d): string
	return string.format("%.17g,%.17g,%.17g,%.17g", self.x, self.y, self.z, self.w)
end

return Vector4d
