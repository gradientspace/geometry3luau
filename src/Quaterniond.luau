--!native
-- Port of geometry3Sharp math/Quaterniond.cs (Quaterniond; Quaternionf is not ported, see PORTING.md)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Quaterniond values are IMMUTABLE by convention. C# mutating Set* methods become constructors:
--   Quaterniond.new(x, y, z, w)
--   Quaterniond.Identity / Quaterniond.Zero          (frozen constants)
--   Quaterniond.AxisAngleD(axis, angleDeg)           new Quaterniond(Vector3d axis, double AngleDeg) / SetAxisAngleD
--   Quaterniond.AxisAngleR(axis, angleRad)
--   Quaterniond.FromTo(vFrom, vTo)                   new Quaterniond(vFrom, vTo) / SetFromTo
--   Quaterniond.Slerp(p, q, t)                       new Quaterniond(p, q, t) / SetToSlerp
--   Quaterniond.FromRotationMatrix(mat)              new Quaterniond(Matrix3d) / SetFromRotationMatrix
-- Operators: q*q, q*scalar, scalar*q, q*Vector3d (rotate), q+q, q-q, -q, ==

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local V3 = Vector3d.new

local sqrt, abs = math.sqrt, math.abs

local Quaterniond = {}
Quaterniond.__index = Quaterniond
Quaterniond.ClassName = "Quaterniond"

export type Quaterniond = typeof(setmetatable({} :: { x: number, y: number, z: number, w: number }, Quaterniond))

local function new(x: number, y: number, z: number, w: number): Quaterniond
	return setmetatable({ x = x, y = y, z = z, w = w }, Quaterniond) :: any
end
Quaterniond.new = new

Quaterniond.Zero = table.freeze(new(0, 0, 0, 0))
Quaterniond.Identity = table.freeze(new(0, 0, 0, 1))

function Quaterniond.Get(self: Quaterniond, key: number): number
	if key == 0 then
		return self.x
	elseif key == 1 then
		return self.y
	elseif key == 2 then
		return self.z
	end
	return self.w
end

function Quaterniond.LengthSquared(self: Quaterniond): number
	return self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w
end

function Quaterniond.Length(self: Quaterniond): number
	return sqrt(self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w)
end

-- C# `double len = q.Normalize(eps)` -> `q, len = q:NormalizedAndLength(eps)`
function Quaterniond.NormalizedAndLength(self: Quaterniond, epsilon: number?): (Quaterniond, number)
	local length = self:Length()
	if length > (epsilon or 0) then
		local invLength = 1.0 / length
		return new(self.x * invLength, self.y * invLength, self.z * invLength, self.w * invLength), length
	end
	return new(0, 0, 0, 0), 0
end

function Quaterniond.Normalized(self: Quaterniond): Quaterniond
	return (self:NormalizedAndLength())
end

function Quaterniond.Dot(self: Quaterniond, q2: Quaterniond): number
	return self.x * q2.x + self.y * q2.y + self.z * q2.z + self.w * q2.w
end

Quaterniond.__unm = function(q: Quaterniond): Quaterniond
	return new(-q.x, -q.y, -q.z, -q.w)
end

local function rotate(q: Quaterniond, v: any)
	--return q.ToRotationMatrix() * v;
	-- inline-expansion of above:
	local twoX = 2 * q.x
	local twoY = 2 * q.y
	local twoZ = 2 * q.z
	local twoWX = twoX * q.w
	local twoWY = twoY * q.w
	local twoWZ = twoZ * q.w
	local twoXX = twoX * q.x
	local twoXY = twoY * q.x
	local twoXZ = twoZ * q.x
	local twoYY = twoY * q.y
	local twoYZ = twoZ * q.y
	local twoZZ = twoZ * q.z
	return V3(
		v.x * (1 - (twoYY + twoZZ)) + v.y * (twoXY - twoWZ) + v.z * (twoXZ + twoWY),
		v.x * (twoXY + twoWZ) + v.y * (1 - (twoXX + twoZZ)) + v.z * (twoYZ - twoWX),
		v.x * (twoXZ - twoWY) + v.y * (twoYZ + twoWX) + v.z * (1 - (twoXX + twoYY))
	)
end

Quaterniond.__mul = function(a: any, b: any): any
	if type(a) == "number" then
		return new(a * b.x, a * b.y, a * b.z, a * b.w)
	elseif type(b) == "number" then
		return new(b * a.x, b * a.y, b * a.z, b * a.w)
	elseif getmetatable(b) == Quaterniond then
		local w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
		local x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y
		local y = a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z
		local z = a.w * b.z + a.z * b.w + a.x * b.y - a.y * b.x
		return new(x, y, z, w)
	end
	return rotate(a, b)
end

Quaterniond.__add = function(q1: Quaterniond, q2: Quaterniond): Quaterniond
	return new(q1.x + q2.x, q1.y + q2.y, q1.z + q2.z, q1.w + q2.w)
end

Quaterniond.__sub = function(q1: Quaterniond, q2: Quaterniond): Quaterniond
	return new(q1.x - q2.x, q1.y - q2.y, q1.z - q2.z, q1.w - q2.w)
end

Quaterniond.__eq = function(a: Quaterniond, b: Quaterniond): boolean
	return a.x == b.x and a.y == b.y and a.z == b.z and a.w == b.w
end

-- q * v (rotate vector)
function Quaterniond.Multiply(self: Quaterniond, v: Vector3d.Vector3d): Vector3d.Vector3d
	return rotate(self, v)
end

-- Inverse() * v
function Quaterniond.InverseMultiply(self: Quaterniond, v: Vector3d.Vector3d): Vector3d.Vector3d
	local norm = self:LengthSquared()
	if norm > 0 then
		local invNorm = 1.0 / norm
		local qx, qy, qz, qw = -self.x * invNorm, -self.y * invNorm, -self.z * invNorm, self.w * invNorm
		local twoX = 2 * qx
		local twoY = 2 * qy
		local twoZ = 2 * qz
		local twoWX = twoX * qw
		local twoWY = twoY * qw
		local twoWZ = twoZ * qw
		local twoXX = twoX * qx
		local twoXY = twoY * qx
		local twoXZ = twoZ * qx
		local twoYY = twoY * qy
		local twoYZ = twoZ * qy
		local twoZZ = twoZ * qz
		return V3(
			v.x * (1 - (twoYY + twoZZ)) + v.y * (twoXY - twoWZ) + v.z * (twoXZ + twoWY),
			v.x * (twoXY + twoWZ) + v.y * (1 - (twoXX + twoZZ)) + v.z * (twoYZ - twoWX),
			v.x * (twoXZ - twoWY) + v.y * (twoYZ + twoWX) + v.z * (1 - (twoXX + twoYY))
		)
	end
	return Vector3d.Zero
end

-- these multiply quaternion by (1,0,0), (0,1,0), (0,0,1), respectively.
-- faster than full multiply, because of all the zeros
function Quaterniond.AxisX(self: Quaterniond): Vector3d.Vector3d
	local x, y, z, w = self.x, self.y, self.z, self.w
	local twoY = 2 * y
	local twoZ = 2 * z
	local twoWY = twoY * w
	local twoWZ = twoZ * w
	local twoXY = twoY * x
	local twoXZ = twoZ * x
	local twoYY = twoY * y
	local twoZZ = twoZ * z
	return V3(1 - (twoYY + twoZZ), twoXY + twoWZ, twoXZ - twoWY)
end

function Quaterniond.AxisY(self: Quaterniond): Vector3d.Vector3d
	local x, y, z, w = self.x, self.y, self.z, self.w
	local twoX = 2 * x
	local twoY = 2 * y
	local twoZ = 2 * z
	local twoWX = twoX * w
	local twoWZ = twoZ * w
	local twoXX = twoX * x
	local twoXY = twoY * x
	local twoYZ = twoZ * y
	local twoZZ = twoZ * z
	return V3(twoXY - twoWZ, 1 - (twoXX + twoZZ), twoYZ + twoWX)
end

function Quaterniond.AxisZ(self: Quaterniond): Vector3d.Vector3d
	local x, y, z, w = self.x, self.y, self.z, self.w
	local twoX = 2 * x
	local twoY = 2 * y
	local twoZ = 2 * z
	local twoWX = twoX * w
	local twoWY = twoY * w
	local twoXX = twoX * x
	local twoXZ = twoZ * x
	local twoYY = twoY * y
	local twoYZ = twoZ * y
	return V3(twoXZ + twoWY, twoYZ - twoWX, 1 - (twoXX + twoYY))
end

-- C# GetXYZ(out X, out Y, out Z) -> returns X, Y, Z
function Quaterniond.GetXYZ(self: Quaterniond): (Vector3d.Vector3d, Vector3d.Vector3d, Vector3d.Vector3d)
	return self:AxisX(), self:AxisY(), self:AxisZ()
end

function Quaterniond.Inverse(self: Quaterniond): Quaterniond
	local norm = self:LengthSquared()
	if norm > 0 then
		local invNorm = 1.0 / norm
		return new(-self.x * invNorm, -self.y * invNorm, -self.z * invNorm, self.w * invNorm)
	end
	return Quaterniond.Zero
end

function Quaterniond.Conjugate(self: Quaterniond): Quaterniond
	return new(-self.x, -self.y, -self.z, self.w)
end

function Quaterniond.ToRotationMatrix(self: Quaterniond)
	local Matrix3d = require(script.Parent.Matrix3d)
	local x, y, z, w = self.x, self.y, self.z, self.w
	local twoX = 2 * x
	local twoY = 2 * y
	local twoZ = 2 * z
	local twoWX = twoX * w
	local twoWY = twoY * w
	local twoWZ = twoZ * w
	local twoXX = twoX * x
	local twoXY = twoY * x
	local twoXZ = twoZ * x
	local twoYY = twoY * y
	local twoYZ = twoZ * y
	local twoZZ = twoZ * z
	return Matrix3d.new(
		1 - (twoYY + twoZZ),
		twoXY - twoWZ,
		twoXZ + twoWY,
		twoXY + twoWZ,
		1 - (twoXX + twoZZ),
		twoYZ - twoWX,
		twoXZ - twoWY,
		twoYZ + twoWX,
		1 - (twoXX + twoYY)
	)
end

function Quaterniond.AxisAngleD(axis: Vector3d.Vector3d, AngleDeg: number): Quaterniond
	local angle_rad = MathUtil.Deg2Rad * AngleDeg
	local halfAngle = 0.5 * angle_rad
	local sn = math.sin(halfAngle)
	return new(sn * axis.x, sn * axis.y, sn * axis.z, math.cos(halfAngle))
end

function Quaterniond.AxisAngleR(axis: Vector3d.Vector3d, angleRad: number): Quaterniond
	return Quaterniond.AxisAngleD(axis, angleRad * MathUtil.Rad2Degf)
end

-- this function can take non-normalized vectors vFrom and vTo (normalizes internally)
function Quaterniond.FromTo(vFrom: Vector3d.Vector3d, vTo: Vector3d.Vector3d): Quaterniond
	-- [TODO] this page seems to have optimized version:
	--    http://lolengine.net/blog/2013/09/18/beautiful-maths-quaternion-from-vectors

	-- [RMS] not ideal to explicitly normalize here, but if we don't,
	--   output quaternion is not normalized and this causes problems,
	--   eg like drift if we do repeated SetFromTo()
	local from, to = vFrom:Normalized(), vTo:Normalized()
	local bisector = (from + to):Normalized()
	local x, y, z
	local w = from:Dot(bisector)
	if w ~= 0 then
		local cross = from:Cross(bisector)
		x, y, z = cross.x, cross.y, cross.z
	else
		local invLength
		if abs(from.x) >= abs(from.y) then
			-- V1.x or V1.z is the largest magnitude component.
			invLength = 1.0 / sqrt(from.x * from.x + from.z * from.z)
			x = -from.z * invLength
			y = 0
			z = from.x * invLength
		else
			-- V1.y or V1.z is the largest magnitude component.
			invLength = 1.0 / sqrt(from.y * from.y + from.z * from.z)
			x = 0
			y = from.z * invLength
			z = -from.y * invLength
		end
	end
	return (new(x, y, z, w):NormalizedAndLength()) -- aaahhh just to be safe...
end

function Quaterniond.FromToConstrained(vFrom: Vector3d.Vector3d, vTo: Vector3d.Vector3d, vAround: Vector3d.Vector3d): Quaterniond
	local fAngle = MathUtil.PlaneAngleSignedD(vFrom, vTo, vAround)
	return Quaterniond.AxisAngleD(vAround, fAngle)
end

function Quaterniond.Slerp(p: Quaterniond, q: Quaterniond, t: number): Quaterniond
	local cs = p:Dot(q)
	local angle = math.acos(cs)
	if abs(angle) >= MathUtil.ZeroTolerance then
		local sn = math.sin(angle)
		local invSn = 1 / sn
		local tAngle = t * angle
		local coeff0 = math.sin(angle - tAngle) * invSn
		local coeff1 = math.sin(tAngle) * invSn
		return new(coeff0 * p.x + coeff1 * q.x, coeff0 * p.y + coeff1 * q.y, coeff0 * p.z + coeff1 * q.z, coeff0 * p.w + coeff1 * q.w)
	end
	return new(p.x, p.y, p.z, p.w)
end

function Quaterniond.FromRotationMatrix(rot): Quaterniond
	-- Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
	-- article "Quaternion Calculus and Fast Animation".
	local nextIdx = { 1, 2, 0 }
	local trace = rot:Get(0, 0) + rot:Get(1, 1) + rot:Get(2, 2)
	local root
	local x, y, z, w
	if trace > 0 then
		-- |w| > 1/2, may as well choose w > 1/2
		root = sqrt(trace + 1.0) -- 2w
		w = 0.5 * root
		root = 0.5 / root -- 1/(4w)
		x = (rot:Get(2, 1) - rot:Get(1, 2)) * root
		y = (rot:Get(0, 2) - rot:Get(2, 0)) * root
		z = (rot:Get(1, 0) - rot:Get(0, 1)) * root
	else
		-- |w| <= 1/2
		local i = 0
		if rot:Get(1, 1) > rot:Get(0, 0) then
			i = 1
		end
		if rot:Get(2, 2) > rot:Get(i, i) then
			i = 2
		end
		local j = nextIdx[i + 1]
		local k = nextIdx[j + 1]

		root = sqrt(rot:Get(i, i) - rot:Get(j, j) - rot:Get(k, k) + 1.0)

		local quat = { 0, 0, 0 }
		quat[i + 1] = 0.5 * root
		root = 0.5 / root
		w = (rot:Get(k, j) - rot:Get(j, k)) * root
		quat[j + 1] = (rot:Get(j, i) + rot:Get(i, j)) * root
		quat[k + 1] = (rot:Get(k, i) + rot:Get(i, k)) * root
		x, y, z = quat[1], quat[2], quat[3]
	end
	return (new(x, y, z, w):NormalizedAndLength()) -- we prefer normalized quaternions...
end

function Quaterniond.Equals(self: Quaterniond, other: Quaterniond): boolean
	return self.x == other.x and self.y == other.y and self.z == other.z and self.w == other.w
end

function Quaterniond.CompareTo(self: Quaterniond, other: Quaterniond): number
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

function Quaterniond.EpsilonEqual(self: Quaterniond, q2: Quaterniond, epsilon: number): boolean
	return abs(self.x - q2.x) <= epsilon
		and abs(self.y - q2.y) <= epsilon
		and abs(self.z - q2.z) <= epsilon
		and abs(self.w - q2.w) <= epsilon
end

Quaterniond.__tostring = function(q: Quaterniond): string
	return string.format("%.8f %.8f %.8f %.8f", q.x, q.y, q.z, q.w)
end

return Quaterniond
