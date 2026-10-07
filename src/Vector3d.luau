--!native
-- Port of geometry3Sharp math/Vector3d.cs (also used for Vector3f, see PORTING.md)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Vector3d values are IMMUTABLE by convention: never assign to .x/.y/.z of an existing vector.
-- Operators: +, - (vector or scalar), * (scalar*v, v*scalar, v*v componentwise), / , unary -, ==
--
-- Constructors:
--   Vector3d.new(x, y, z)         new Vector3d(x, y, z)
--   Vector3d.Splat(f)             new Vector3d(f)
--   Vector3d.FromArray(t, i0?)    new Vector3d(double[] v) (reads t[i0+1..i0+3], i0 defaults to 0)
--   Vector3d.FromVector(v)        from any {x,y,z} or Roblox-style {X,Y,Z} object

local MathUtil = require(script.Parent.MathUtil)

local sqrt, abs, acos, max, min = math.sqrt, math.abs, math.acos, math.max, math.min
local Epsilon = MathUtil.Epsilon
local ZeroTolerance = MathUtil.ZeroTolerance
local Rad2Deg = MathUtil.Rad2Deg

local Vector3d = {}
Vector3d.__index = Vector3d
Vector3d.ClassName = "Vector3d"

export type Vector3d = typeof(setmetatable({} :: { x: number, y: number, z: number }, Vector3d))

local function new(x: number, y: number, z: number): Vector3d
	return setmetatable({ x = x, y = y, z = z }, Vector3d) :: any
end
Vector3d.new = new

function Vector3d.Splat(f: number): Vector3d
	return new(f, f, f)
end

function Vector3d.FromArray(t: { number }, i0: number?): Vector3d
	local i = (i0 or 0)
	return new(t[i + 1], t[i + 2], t[i + 3])
end

function Vector3d.FromVector(v: any): Vector3d
	if v.x ~= nil then
		return new(v.x, v.y, v.z)
	end
	return new(v.X, v.Y, v.Z)
end

function Vector3d.IsVector3d(v: any): boolean
	return getmetatable(v) == Vector3d
end

local function const(x, y, z)
	return table.freeze(new(x, y, z))
end

Vector3d.Zero = const(0, 0, 0)
Vector3d.One = const(1, 1, 1)
Vector3d.OneNormalized = const(1 / sqrt(3), 1 / sqrt(3), 1 / sqrt(3))
-- use float so that LengthSquared is non-inf
Vector3d.Invalid = const(MathUtil.MaxFloat, MathUtil.MaxFloat, MathUtil.MaxFloat)
Vector3d.AxisX = const(1, 0, 0)
Vector3d.AxisY = const(0, 1, 0)
Vector3d.AxisZ = const(0, 0, 1)
Vector3d.UnitX = Vector3d.AxisX
Vector3d.UnitY = Vector3d.AxisY
Vector3d.UnitZ = Vector3d.AxisZ
Vector3d.MaxValue = const(MathUtil.MaxDouble, MathUtil.MaxDouble, MathUtil.MaxDouble)
Vector3d.MinValue = const(MathUtil.MinDouble, MathUtil.MinDouble, MathUtil.MinDouble)

-- C# indexer get: v[key], key in 0..2
function Vector3d.Get(self: Vector3d, key: number): number
	if key == 0 then
		return self.x
	elseif key == 1 then
		return self.y
	end
	return self.z
end

-- replacement for C# indexer set (v[key] = value): returns a new vector with component key replaced
function Vector3d.With(self: Vector3d, key: number, value: number): Vector3d
	if key == 0 then
		return new(value, self.y, self.z)
	elseif key == 1 then
		return new(self.x, value, self.z)
	end
	return new(self.x, self.y, value)
end

function Vector3d.Clone(self: Vector3d): Vector3d
	return new(self.x, self.y, self.z)
end

function Vector3d.xy(self: Vector3d)
	return require(script.Parent.Vector2d).new(self.x, self.y)
end
function Vector3d.xz(self: Vector3d)
	return require(script.Parent.Vector2d).new(self.x, self.z)
end
function Vector3d.yz(self: Vector3d)
	return require(script.Parent.Vector2d).new(self.y, self.z)
end

function Vector3d.LengthSquared(self: Vector3d): number
	local x, y, z = self.x, self.y, self.z
	return x * x + y * y + z * z
end

function Vector3d.Length(self: Vector3d): number
	local x, y, z = self.x, self.y, self.z
	return sqrt(x * x + y * y + z * z)
end

function Vector3d.LengthL1(self: Vector3d): number
	return abs(self.x) + abs(self.y) + abs(self.z)
end

function Vector3d.Max(self: Vector3d): number
	return max(self.x, max(self.y, self.z))
end

function Vector3d.Min(self: Vector3d): number
	return min(self.x, min(self.y, self.z))
end

function Vector3d.MaxAbs(self: Vector3d): number
	return max(abs(self.x), max(abs(self.y), abs(self.z)))
end

function Vector3d.MinAbs(self: Vector3d): number
	return min(abs(self.x), min(abs(self.y), abs(self.z)))
end

function Vector3d.Abs(self: Vector3d): Vector3d
	return new(abs(self.x), abs(self.y), abs(self.z))
end

-- C# `double len = v.Normalize(epsilon)` -> `v, len = v:NormalizedAndLength(epsilon)`
function Vector3d.NormalizedAndLength(self: Vector3d, epsilon: number?): (Vector3d, number)
	local x, y, z = self.x, self.y, self.z
	local length = sqrt(x * x + y * y + z * z)
	if length > (epsilon or Epsilon) then
		local invLength = 1.0 / length
		return new(x * invLength, y * invLength, z * invLength), length
	end
	return new(0, 0, 0), 0
end

function Vector3d.Normalized(self: Vector3d): Vector3d
	local x, y, z = self.x, self.y, self.z
	local length = sqrt(x * x + y * y + z * z)
	if length > Epsilon then
		local invLength = 1.0 / length
		return new(x * invLength, y * invLength, z * invLength)
	end
	return Vector3d.Zero
end

function Vector3d.IsNormalized(self: Vector3d): boolean
	local x, y, z = self.x, self.y, self.z
	return abs((x * x + y * y + z * z) - 1) < ZeroTolerance
end

function Vector3d.IsFinite(self: Vector3d): boolean
	local f = self.x + self.y + self.z
	return f == f and f ~= math.huge and f ~= -math.huge
end

function Vector3d.RoundFrac(self: Vector3d, nDecimals: number): Vector3d
	return new(
		MathUtil.Round(self.x, nDecimals),
		MathUtil.Round(self.y, nDecimals),
		MathUtil.Round(self.z, nDecimals)
	)
end

function Vector3d.Dot(self: Vector3d, v2: Vector3d): number
	return self.x * v2.x + self.y * v2.y + self.z * v2.z
end

function Vector3d.Cross(self: Vector3d, v2: Vector3d): Vector3d
	local x, y, z = self.x, self.y, self.z
	return new(y * v2.z - z * v2.y, z * v2.x - x * v2.z, x * v2.y - y * v2.x)
end

function Vector3d.UnitCross(self: Vector3d, v2: Vector3d): Vector3d
	local x, y, z = self.x, self.y, self.z
	local n = new(y * v2.z - z * v2.y, z * v2.x - x * v2.z, x * v2.y - y * v2.x)
	return (n:NormalizedAndLength())
end

function Vector3d.AngleD(self: Vector3d, v2: Vector3d): number
	local fDot = MathUtil.Clamp(self:Dot(v2), -1, 1)
	return acos(fDot) * Rad2Deg
end

function Vector3d.AngleR(self: Vector3d, v2: Vector3d): number
	local fDot = MathUtil.Clamp(self:Dot(v2), -1, 1)
	return acos(fDot)
end

function Vector3d.DistanceSquared(self: Vector3d, v2: Vector3d): number
	local dx, dy, dz = v2.x - self.x, v2.y - self.y, v2.z - self.z
	return dx * dx + dy * dy + dz * dz
end

function Vector3d.Distance(self: Vector3d, v2: Vector3d): number
	local dx, dy, dz = v2.x - self.x, v2.y - self.y, v2.z - self.z
	return sqrt(dx * dx + dy * dy + dz * dz)
end

Vector3d.__unm = function(v: Vector3d): Vector3d
	return new(-v.x, -v.y, -v.z)
end

Vector3d.__add = function(a: any, b: any): Vector3d
	if type(b) == "number" then
		return new(a.x + b, a.y + b, a.z + b)
	elseif type(a) == "number" then
		return new(a + b.x, a + b.y, a + b.z)
	end
	return new(a.x + b.x, a.y + b.y, a.z + b.z)
end

Vector3d.__sub = function(a: any, b: any): Vector3d
	if type(b) == "number" then
		return new(a.x - b, a.y - b, a.z - b)
	end
	return new(a.x - b.x, a.y - b.y, a.z - b.z)
end

Vector3d.__mul = function(a: any, b: any): Vector3d
	if type(a) == "number" then
		return new(a * b.x, a * b.y, a * b.z)
	elseif type(b) == "number" then
		return new(b * a.x, b * a.y, b * a.z)
	end
	return new(a.x * b.x, a.y * b.y, a.z * b.z)
end

Vector3d.__div = function(a: any, b: any): Vector3d
	if type(b) == "number" then
		return new(a.x / b, a.y / b, a.z / b)
	elseif type(a) == "number" then
		return new(a / b.x, a / b.y, a / b.z)
	end
	return new(a.x / b.x, a.y / b.y, a.z / b.z)
end

Vector3d.__eq = function(a: Vector3d, b: Vector3d): boolean
	return a.x == b.x and a.y == b.y and a.z == b.z
end

Vector3d.__tostring = function(v: Vector3d): string
	return string.format("%.8f %.8f %.8f", v.x, v.y, v.z)
end

function Vector3d.ToString(self: Vector3d, fmt: string?): string
	if fmt then
		return string.format(fmt .. " " .. fmt .. " " .. fmt, self.x, self.y, self.z)
	end
	return tostring(self)
end

function Vector3d.Equals(self: Vector3d, other: Vector3d): boolean
	return self.x == other.x and self.y == other.y and self.z == other.z
end

function Vector3d.CompareTo(self: Vector3d, other: Vector3d): number
	if self.x ~= other.x then
		return self.x < other.x and -1 or 1
	elseif self.y ~= other.y then
		return self.y < other.y and -1 or 1
	elseif self.z ~= other.z then
		return self.z < other.z and -1 or 1
	end
	return 0
end

function Vector3d.EpsilonEqual(self: Vector3d, v2: Vector3d, epsilon: number): boolean
	return abs(self.x - v2.x) <= epsilon and abs(self.y - v2.y) <= epsilon and abs(self.z - v2.z) <= epsilon
end

function Vector3d.Lerp(a: Vector3d, b: Vector3d, t: number): Vector3d
	local s = 1 - t
	return new(s * a.x + t * b.x, s * a.y + t * b.y, s * a.z + t * b.z)
end

-- string key for use in hash tables (C# GetHashCode/Equals for dictionary keys)
function Vector3d.HashKey(self: Vector3d): string
	return string.format("%.17g,%.17g,%.17g", self.x, self.y, self.z)
end

---------------------------------------------------------------------------------------------------
-- complicated functions go down here...

-- Gram-Schmidt orthonormalization of the input vectors.
-- C#: static double Orthonormalize(ref u, ref v, ref w) -> returns minLength, u, v, w
function Vector3d.Orthonormalize(u: Vector3d, v: Vector3d, w: Vector3d): (number, Vector3d, Vector3d, Vector3d)
	-- If the input vectors are v0, v1, and v2, then the Gram-Schmidt
	-- orthonormalization produces vectors u0, u1, and u2 as follows,
	--
	--   u0 = v0/|v0|
	--   u1 = (v1-(u0*v1)u0)/|v1-(u0*v1)u0|
	--   u2 = (v2-(u0*v2)u0-(u1*v2)u1)/|v2-(u0*v2)u0-(u1*v2)u1|
	local minLength
	u, minLength = u:NormalizedAndLength()

	local dot0 = u:Dot(v)
	v = v - dot0 * u
	local l
	v, l = v:NormalizedAndLength()
	if l < minLength then
		minLength = l
	end

	local dot1 = v:Dot(w)
	dot0 = u:Dot(w)
	w = w - (dot0 * u + dot1 * v)
	w, l = w:NormalizedAndLength()
	if l < minLength then
		minLength = l
	end
	return minLength, u, v, w
end

-- Input W must be a unit-length vector. The output vectors {U,V} are unit length and mutually
-- perpendicular, and {U,V,W} is an orthonormal basis. ported from WildMagic5
-- C#: GenerateComplementBasis(ref u, ref v, Vector3d w) -> returns u, v
function Vector3d.GenerateComplementBasis(w: Vector3d): (Vector3d, Vector3d)
	local invLength
	if abs(w.x) >= abs(w.y) then
		-- W.x or W.z is the largest magnitude component, swap them
		invLength = MathUtil.InvSqrt(w.x * w.x + w.z * w.z)
		local ux, uy, uz = -w.z * invLength, 0, w.x * invLength
		return new(ux, uy, uz), new(w.y * uz, w.z * ux - w.x * uz, -w.y * ux)
	else
		-- W.y or W.z is the largest magnitude component, swap them
		invLength = MathUtil.InvSqrt(w.y * w.y + w.z * w.z)
		local ux, uy, uz = 0, w.z * invLength, -w.y * invLength
		return new(ux, uy, uz), new(w.y * uz - w.z * uy, -w.x * uz, w.x * uy)
	end
end

-- ported from GTEngine. Compute a right-handed orthonormal basis for the orthogonal complement
-- of the input vectors. numInputs must be 1 or 2.
-- C#: ComputeOrthogonalComplement(int numInputs, Vector3d v0, ref v1, ref v2) -> returns minLength, v1, v2
function Vector3d.ComputeOrthogonalComplement(numInputs: number, v0: Vector3d, v1: Vector3d?, v2: Vector3d?): (number, Vector3d, Vector3d)
	if numInputs == 1 then
		if abs(v0.x) > abs(v0.y) then
			v1 = new(-v0.z, 0.0, v0.x)
		else
			v1 = new(0.0, v0.z, -v0.y)
		end
		numInputs = 2
	end
	if numInputs == 2 then
		v2 = Vector3d.Cross(v0, v1 :: Vector3d)
		local minLen, _, nv1, nv2 = Vector3d.Orthonormalize(v0, v1 :: Vector3d, v2 :: Vector3d)
		return minLen, nv1, nv2
	end
	return 0, v1 :: Vector3d, v2 :: Vector3d
end

-- Returns two vectors perpendicular to n, as efficiently as possible.
-- Duff et al method, from https://graphics.pixar.com/library/OrthonormalB/paper.pdf
-- C#: MakePerpVectors(n, out b1, out b2) -> returns b1, b2
function Vector3d.MakePerpVectors(n: Vector3d): (Vector3d, Vector3d)
	if n.z < 0.0 then
		local a = 1.0 / (1.0 - n.z)
		local b = n.x * n.y * a
		return new(1.0 - n.x * n.x * a, -b, n.x), new(b, n.y * n.y * a - 1.0, -n.y)
	else
		local a = 1.0 / (1.0 + n.z)
		local b = -n.x * n.y * a
		return new(1.0 - n.x * n.x * a, b, -n.x), new(b, 1.0 - n.y * n.y * a, -n.y)
	end
end

-- support x,y,z and x y z
function Vector3d.TryParse(s: string): (boolean, Vector3d)
	local vals = {}
	for token in string.gmatch(s, "[^%s,]+") do
		table.insert(vals, tonumber(token))
	end
	if #vals == 3 and vals[1] and vals[2] and vals[3] then
		return true, new(vals[1], vals[2], vals[3])
	end
	return false, Vector3d.Zero
end

return Vector3d
