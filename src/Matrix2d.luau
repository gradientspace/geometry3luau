--!native
-- Port of geometry3Sharp math/Matrix2d.cs (C# Matrix2d; Matrix2f in math/FloatTypes.cs is not ported, see PORTING.md)
-- some functions ported from WildMagic5 Matrix2
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Matrix2d is a C# class with public fields m00, m01, m10, m11, and it is MUTABLE (SetToDiagonal,
-- SetToRotationRad/Deg and Orthonormalize modify it in place). The static constants Matrix2d.Identity,
-- Matrix2d.Zero and Matrix2d.One are frozen: mutating them errors (in C# that would silently corrupt them).
--
-- Constructors:
--   Matrix2d.new(m00, m01, m10, m11)                new Matrix2d(double m00, double m01, double m10, double m11)
--   Matrix2d.FromIdentity(bIdentity)                new Matrix2d(bool bIdentity)  (identity or zero)
--   Matrix2d.Diagonal(m00, m11)                     new Matrix2d(double m00, double m11)
--   Matrix2d.FromAngle(angle, bDegrees?)            new Matrix2d(double angle, bool bDegrees = false)  (rotation, CCW)
--   Matrix2d.FromVectors(u, v, columns)             new Matrix2d(Vector2d u, Vector2d v, bool columns)
--   Matrix2d.OuterProduct(u, v)                     new Matrix2d(Vector2d u, Vector2d v)  (tensor product u*v^T)
--   m:Clone()
--
-- Luau API notes:
--   * this[r, c] -> m:Get(r, c) (0-based r, c). Determinant is a method: m:Determinant().
--   * m:Inverse(epsilon?) returns the frozen Matrix2d.Zero when the determinant is <= epsilon (as in C#).
--   * EigenDecomposition(ref rot, ref diag) -> local rot, diag = m:EigenDecomposition(rot?, diag?)
--     (fills rot/diag when passed, otherwise creates them).
--   * Operators: -m, m + m, m + f, m - m, m - f, m * f, f * m, m / f, m * v (row*vector, also m:Multiply(v)).
--     C# v * m (vector*column) -> Matrix2d.VectorMultiply(v, m), since Vector2d's * does not know Matrix2d.
--   * The implicit/explicit conversions to/from Matrix2f are not ported (no float types).

local MathUtil = require(script.Parent.MathUtil)
local Vector2d = require(script.Parent.Vector2d)

local V2 = Vector2d.new
local abs, sqrt, cos, sin, atan2 = math.abs, math.sqrt, math.cos, math.sin, math.atan2

local Matrix2d = {}
Matrix2d.__index = Matrix2d
Matrix2d.ClassName = "Matrix2d"

export type Matrix2d = typeof(setmetatable({} :: { m00: number, m01: number, m10: number, m11: number }, Matrix2d))

local function new(m00: number, m01: number, m10: number, m11: number): Matrix2d
	return setmetatable({ m00 = m00, m01 = m01, m10 = m10, m11 = m11 }, Matrix2d) :: any
end
Matrix2d.new = new

function Matrix2d.FromIdentity(bIdentity: boolean): Matrix2d
	if bIdentity then
		return new(1, 0, 0, 1)
	end
	return new(0, 0, 0, 0)
end

Matrix2d.Identity = table.freeze(new(1, 0, 0, 1))
Matrix2d.Zero = table.freeze(new(0, 0, 0, 0))
Matrix2d.One = table.freeze(new(1, 1, 1, 1))

function Matrix2d.Diagonal(m00: number, m11: number): Matrix2d
	return new(m00, 0, 0, m11)
end

-- Create a rotation matrix (positive angle -> counterclockwise).
function Matrix2d.FromAngle(angle: number, bDegrees: boolean?): Matrix2d
	local m = new(0, 0, 0, 0)
	if bDegrees then
		m:SetToRotationDeg(angle)
	else
		m:SetToRotationRad(angle)
	end
	return m
end

-- Create matrices based on vector input.  The bool is interpreted as
--   true: vectors are columns of the matrix
--   false: vectors are rows of the matrix
function Matrix2d.FromVectors(u: Vector2d.Vector2d, v: Vector2d.Vector2d, columns: boolean): Matrix2d
	if columns then
		return new(u.x, v.x, u.y, v.y)
	end
	return new(u.x, u.y, v.x, v.y)
end

-- Create a tensor product U*V^T.
function Matrix2d.OuterProduct(u: Vector2d.Vector2d, v: Vector2d.Vector2d): Matrix2d
	return new(u.x * v.x, u.x * v.y, u.y * v.x, u.y * v.y)
end

function Matrix2d.Clone(self: Matrix2d): Matrix2d
	return new(self.m00, self.m01, self.m10, self.m11)
end

-- this[r, c]
function Matrix2d.Get(self: Matrix2d, r: number, c: number): number
	if r == 0 then
		return (c == 0) and self.m00 or self.m01
	end
	return (c == 0) and self.m10 or self.m11
end

function Matrix2d.SetToDiagonal(self: Matrix2d, m00: number, m11: number)
	self.m00 = m00
	self.m11 = m11
	self.m01 = 0
	self.m10 = 0
end

function Matrix2d.SetToRotationRad(self: Matrix2d, angleRad: number)
	local c = cos(angleRad)
	self.m00 = c
	self.m11 = c
	self.m10 = sin(angleRad)
	self.m01 = -self.m10
end

function Matrix2d.SetToRotationDeg(self: Matrix2d, angleDeg: number)
	self:SetToRotationRad(MathUtil.Deg2Rad * angleDeg)
end

-- row*vector multiply
function Matrix2d.Multiply(self: Matrix2d, v: Vector2d.Vector2d): Vector2d.Vector2d
	return V2(self.m00 * v.x + self.m01 * v.y, self.m10 * v.x + self.m11 * v.y)
end

-- vector*column multiply (C# operator*(Vector2d v, Matrix2d m))
function Matrix2d.VectorMultiply(v: Vector2d.Vector2d, m: Matrix2d): Vector2d.Vector2d
	return V2(v.x * m.m00 + v.y * m.m10, v.x * m.m01 + v.y * m.m11)
end

-- u^T*M*v
function Matrix2d.QForm(self: Matrix2d, u: Vector2d.Vector2d, v: Vector2d.Vector2d): number
	return u:Dot(self:Multiply(v))
end

function Matrix2d.Transpose(self: Matrix2d): Matrix2d
	return new(self.m00, self.m10, self.m01, self.m11)
end

-- Other operations.
function Matrix2d.Inverse(self: Matrix2d, epsilon: number?): Matrix2d
	local eps = epsilon or 0
	local m00, m01, m10, m11 = self.m00, self.m01, self.m10, self.m11
	local det = m00 * m11 - m10 * m01
	if abs(det) > eps then
		local invDet = 1.0 / det
		return new(m11 * invDet, -m01 * invDet, -m10 * invDet, m00 * invDet)
	end
	return Matrix2d.Zero
end

function Matrix2d.Adjoint(self: Matrix2d): Matrix2d
	return new(self.m11, -self.m01, -self.m10, self.m00)
end

function Matrix2d.Determinant(self: Matrix2d): number
	return self.m00 * self.m11 - self.m01 * self.m10
end

function Matrix2d.ExtractAngle(self: Matrix2d): number
	-- assert:  'this' matrix represents a rotation
	return atan2(self.m10, self.m00)
end

function Matrix2d.Row(self: Matrix2d, i: number): Vector2d.Vector2d
	return (i == 0) and V2(self.m00, self.m01) or V2(self.m10, self.m11)
end

function Matrix2d.Column(self: Matrix2d, i: number): Vector2d.Vector2d
	return (i == 0) and V2(self.m00, self.m10) or V2(self.m01, self.m11)
end

function Matrix2d.Orthonormalize(self: Matrix2d)
	-- Algorithm uses Gram-Schmidt orthogonalization.  If 'this' matrix is
	-- M = [m0|m1], then orthonormal output matrix is Q = [q0|q1],
	--
	--   q0 = m0/|m0|
	--   q1 = (m1-(q0*m1)q0)/|m1-(q0*m1)q0|
	--
	-- where |V| indicates length of vector V and A*B indicates dot
	-- product of vectors A and B.

	-- Compute q0.
	local invLength = 1.0 / sqrt(self.m00 * self.m00 + self.m10 * self.m10)

	self.m00 *= invLength
	self.m10 *= invLength

	-- Compute q1.
	local dot0 = self.m00 * self.m01 + self.m10 * self.m11
	self.m01 -= dot0 * self.m00
	self.m11 -= dot0 * self.m10

	invLength = 1.0 / sqrt(self.m01 * self.m01 + self.m11 * self.m11)

	self.m01 *= invLength
	self.m11 *= invLength
end

-- C# EigenDecomposition(ref Matrix2d rot, ref Matrix2d diag). Returns rot, diag.
function Matrix2d.EigenDecomposition(self: Matrix2d, rot: Matrix2d?, diag: Matrix2d?): (Matrix2d, Matrix2d)
	local r: Matrix2d = rot or new(0, 0, 0, 0)
	local d: Matrix2d = diag or new(0, 0, 0, 0)
	local m00, m01, m11 = self.m00, self.m01, self.m11

	local sum = abs(m00) + abs(m11)
	if abs(m01) + sum == sum then
		-- The matrix M is diagonal (within numerical round-off).
		r.m00 = 1
		r.m01 = 0
		r.m10 = 0
		r.m11 = 1
		d.m00 = m00
		d.m01 = 0
		d.m10 = 0
		d.m11 = m11
		return r, d
	end

	local trace = m00 + m11
	local diff = m00 - m11
	local discr = sqrt(diff * diff + 4 * m01 * m01)
	local eigVal0 = 0.5 * (trace - discr)
	local eigVal1 = 0.5 * (trace + discr)
	d:SetToDiagonal(eigVal0, eigVal1)

	local cs, sn
	if diff >= 0.0 then
		cs = m01
		sn = eigVal0 - m00
	else
		cs = eigVal0 - m11
		sn = m01
	end
	local invLength = 1.0 / sqrt(cs * cs + sn * sn)
	cs *= invLength
	sn *= invLength

	r.m00 = cs
	r.m01 = -sn
	r.m10 = sn
	r.m11 = cs
	return r, d
end

Matrix2d.__unm = function(v: Matrix2d): Matrix2d
	return new(-v.m00, -v.m01, -v.m10, -v.m11)
end

-- operator+: Matrix + Matrix, Matrix + scalar
Matrix2d.__add = function(a: any, o: any): Matrix2d
	if type(o) == "number" then
		return new(a.m00 + o, a.m01 + o, a.m10 + o, a.m11 + o)
	end
	return new(a.m00 + o.m00, a.m01 + o.m01, a.m10 + o.m10, a.m11 + o.m11)
end

-- operator-: Matrix - Matrix, Matrix - scalar
Matrix2d.__sub = function(a: any, o: any): Matrix2d
	if type(o) == "number" then
		return new(a.m00 - o, a.m01 - o, a.m10 - o, a.m11 - o)
	end
	return new(a.m00 - o.m00, a.m01 - o.m01, a.m10 - o.m10, a.m11 - o.m11)
end

-- operator*: Matrix*scalar, scalar*Matrix, Matrix*Vector2d (row*vector)
Matrix2d.__mul = function(a: any, b: any): any
	if type(a) == "number" then
		a, b = b, a
	end
	if type(b) == "number" then
		return new(a.m00 * b, a.m01 * b, a.m10 * b, a.m11 * b)
	end
	return Matrix2d.Multiply(a, b)
end

Matrix2d.__div = function(a: Matrix2d, f: number): Matrix2d
	return new(a.m00 / f, a.m01 / f, a.m10 / f, a.m11 / f)
end

Matrix2d.__tostring = function(m: Matrix2d): string
	return string.format("[%.8f %.8f] [%.8f %.8f]", m.m00, m.m01, m.m10, m.m11)
end

return Matrix2d
