--!native
-- Port of geometry3Sharp math/Vector2d.cs (Vector2d; Vector2f is not ported, see PORTING.md)
-- (Vector2d.GetInformation, the 2D point-set analysis helper, is not ported.)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Vector2d values are IMMUTABLE by convention: never assign to .x/.y of an existing vector.

local MathUtil = require(script.Parent.MathUtil)

local sqrt, abs, acos, max, min, cos, sin = math.sqrt, math.abs, math.acos, math.max, math.min, math.cos, math.sin

local Vector2d = {}
Vector2d.__index = Vector2d
Vector2d.ClassName = "Vector2d"

export type Vector2d = typeof(setmetatable({} :: { x: number, y: number }, Vector2d))

local function new(x: number, y: number): Vector2d
	return setmetatable({ x = x, y = y }, Vector2d) :: any
end
Vector2d.new = new

function Vector2d.Splat(f: number): Vector2d
	return new(f, f)
end

function Vector2d.FromArray(t: { number }, i0: number?): Vector2d
	local i = (i0 or 0)
	return new(t[i + 1], t[i + 2])
end

function Vector2d.FromVector(v: any): Vector2d
	if v.x ~= nil then
		return new(v.x, v.y)
	end
	return new(v.X, v.Y)
end

local function const(x, y)
	return table.freeze(new(x, y))
end
Vector2d.Zero = const(0, 0)
Vector2d.One = const(1, 1)
Vector2d.AxisX = const(1, 0)
Vector2d.AxisY = const(0, 1)
Vector2d.UnitX = Vector2d.AxisX
Vector2d.UnitY = Vector2d.AxisY
Vector2d.MaxValue = const(MathUtil.MaxDouble, MathUtil.MaxDouble)
Vector2d.MinValue = const(MathUtil.MinDouble, MathUtil.MinDouble)

function Vector2d.FromAngleRad(angle: number): Vector2d
	return new(cos(angle), sin(angle))
end
function Vector2d.FromAngleDeg(angle: number): Vector2d
	angle *= MathUtil.Deg2Rad
	return new(cos(angle), sin(angle))
end

function Vector2d.Get(self: Vector2d, key: number): number
	return (key == 0) and self.x or self.y
end

function Vector2d.With(self: Vector2d, key: number, value: number): Vector2d
	if key == 0 then
		return new(value, self.y)
	end
	return new(self.x, value)
end

function Vector2d.Clone(self: Vector2d): Vector2d
	return new(self.x, self.y)
end

function Vector2d.LengthSquared(self: Vector2d): number
	return self.x * self.x + self.y * self.y
end
function Vector2d.Length(self: Vector2d): number
	return sqrt(self.x * self.x + self.y * self.y)
end
function Vector2d.Max(self: Vector2d): number
	return max(self.x, self.y)
end
function Vector2d.Min(self: Vector2d): number
	return min(self.x, self.y)
end
function Vector2d.MaxAbs(self: Vector2d): number
	return max(abs(self.x), abs(self.y))
end
function Vector2d.MinAbs(self: Vector2d): number
	return min(abs(self.x), abs(self.y))
end
function Vector2d.Abs(self: Vector2d): Vector2d
	return new(abs(self.x), abs(self.y))
end

function Vector2d.NormalizedAndLength(self: Vector2d, epsilon: number?): (Vector2d, number)
	local length = sqrt(self.x * self.x + self.y * self.y)
	if length > (epsilon or MathUtil.Epsilon) then
		local invLength = 1.0 / length
		return new(self.x * invLength, self.y * invLength), length
	end
	return new(0, 0), 0
end

function Vector2d.Normalized(self: Vector2d): Vector2d
	local length = sqrt(self.x * self.x + self.y * self.y)
	if length > MathUtil.Epsilon then
		local invLength = 1.0 / length
		return new(self.x * invLength, self.y * invLength)
	end
	return Vector2d.Zero
end

function Vector2d.IsNormalized(self: Vector2d): boolean
	return abs((self.x * self.x + self.y * self.y) - 1) < MathUtil.ZeroTolerance
end

function Vector2d.IsFinite(self: Vector2d): boolean
	local f = self.x + self.y
	return f == f and f ~= math.huge and f ~= -math.huge
end

function Vector2d.RoundFrac(self: Vector2d, nDecimals: number): Vector2d
	return new(MathUtil.Round(self.x, nDecimals), MathUtil.Round(self.y, nDecimals))
end

function Vector2d.Dot(self: Vector2d, v2: Vector2d): number
	return self.x * v2.x + self.y * v2.y
end

function Vector2d.Cross(self: Vector2d, v2: Vector2d): number
	return self.x * v2.y - self.y * v2.x
end

function Vector2d.Perp(self: Vector2d): Vector2d
	return new(self.y, -self.x)
end

function Vector2d.UnitPerp(self: Vector2d): Vector2d
	return new(self.y, -self.x):Normalized()
end

function Vector2d.DotPerp(self: Vector2d, v2: Vector2d): number
	return self.x * v2.y - self.y * v2.x
end

function Vector2d.AngleD(self: Vector2d, v2: Vector2d): number
	local fDot = MathUtil.Clamp(self:Dot(v2), -1, 1)
	return acos(fDot) * MathUtil.Rad2Deg
end

function Vector2d.AngleR(self: Vector2d, v2: Vector2d): number
	local fDot = MathUtil.Clamp(self:Dot(v2), -1, 1)
	return acos(fDot)
end

function Vector2d.DistanceSquared(self: Vector2d, v2: Vector2d): number
	local dx, dy = v2.x - self.x, v2.y - self.y
	return dx * dx + dy * dy
end

function Vector2d.Distance(self: Vector2d, v2: Vector2d): number
	local dx, dy = v2.x - self.x, v2.y - self.y
	return sqrt(dx * dx + dy * dy)
end

Vector2d.__unm = function(v: Vector2d): Vector2d
	return new(-v.x, -v.y)
end
Vector2d.__add = function(a: any, b: any): Vector2d
	if type(b) == "number" then
		return new(a.x + b, a.y + b)
	elseif type(a) == "number" then
		return new(a + b.x, a + b.y)
	end
	return new(a.x + b.x, a.y + b.y)
end
Vector2d.__sub = function(a: any, b: any): Vector2d
	if type(b) == "number" then
		return new(a.x - b, a.y - b)
	end
	return new(a.x - b.x, a.y - b.y)
end
Vector2d.__mul = function(a: any, b: any): Vector2d
	if type(a) == "number" then
		return new(a * b.x, a * b.y)
	elseif type(b) == "number" then
		return new(b * a.x, b * a.y)
	end
	return new(a.x * b.x, a.y * b.y)
end
Vector2d.__div = function(a: any, b: any): Vector2d
	if type(b) == "number" then
		return new(a.x / b, a.y / b)
	elseif type(a) == "number" then
		return new(a / b.x, a / b.y)
	end
	return new(a.x / b.x, a.y / b.y)
end
Vector2d.__eq = function(a: Vector2d, b: Vector2d): boolean
	return a.x == b.x and a.y == b.y
end
Vector2d.__tostring = function(v: Vector2d): string
	return string.format("%.8f %.8f", v.x, v.y)
end

function Vector2d.Equals(self: Vector2d, other: Vector2d): boolean
	return self.x == other.x and self.y == other.y
end

function Vector2d.CompareTo(self: Vector2d, other: Vector2d): number
	if self.x ~= other.x then
		return self.x < other.x and -1 or 1
	elseif self.y ~= other.y then
		return self.y < other.y and -1 or 1
	end
	return 0
end

function Vector2d.EpsilonEqual(self: Vector2d, v2: Vector2d, epsilon: number): boolean
	return abs(self.x - v2.x) <= epsilon and abs(self.y - v2.y) <= epsilon
end

function Vector2d.Translated(self: Vector2d, dx: number, dy: number): Vector2d
	return new(self.x + dx, self.y + dy)
end

function Vector2d.Lerp(a: Vector2d, b: Vector2d, t: number): Vector2d
	local s = 1 - t
	return new(s * a.x + t * b.x, s * a.y + t * b.y)
end

function Vector2d.HashKey(self: Vector2d): string
	return string.format("%.17g,%.17g", self.x, self.y)
end

-- support x,y and x y
-- C# bool TryParse(string s, out Vector2d result) -> returns ok, result
function Vector2d.TryParse(s: string): (boolean, Vector2d)
	local ok, result4 = MathUtil.TryParseRealVector(s, 2)
	if ok then
		return true, new(result4.x, result4.y)
	end
	return false, Vector2d.Zero
end

return Vector2d
