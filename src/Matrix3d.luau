--!native
-- Port of geometry3Sharp math/Matrix3d.cs (also used for Matrix3f)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Matrix3d values are IMMUTABLE by convention (rows are Vector3d).
-- Constructors:
--   Matrix3d.new(m00, m01, m02, m10, m11, m12, m20, m21, m22)
--   Matrix3d.Identity / Matrix3d.Zero               (frozen constants)
--   Matrix3d.FromArray(t)                           new Matrix3d(double[] mat), t is 1-based {m00..m22}
--   Matrix3d.FromFunc(f)                            new Matrix3d(Func<int,double>) (0-based buffer index)
--   Matrix3d.FromFunc2(f)                           new Matrix3d(Func<int,int,double>)
--   Matrix3d.Diagonal(m00, m11, m22)                new Matrix3d(double m00, double m11, double m22)
--   Matrix3d.FromDiagonal(vdiag)                    new Matrix3d(Vector3d diag)
--   Matrix3d.FromVectors(v1, v2, v3, bRows)         new Matrix3d(v1, v2, v3, bool bRows)
--   Matrix3d.OuterProduct(u, v)                     new Matrix3d(ref Vector3d u, ref Vector3d v)

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local V3 = Vector3d.new

local Matrix3d = {}
Matrix3d.__index = Matrix3d
Matrix3d.ClassName = "Matrix3d"

export type Matrix3d = typeof(setmetatable({} :: { Row0: Vector3d.Vector3d, Row1: Vector3d.Vector3d, Row2: Vector3d.Vector3d }, Matrix3d))

local function fromRows(r0, r1, r2): Matrix3d
	return setmetatable({ Row0 = r0, Row1 = r1, Row2 = r2 }, Matrix3d) :: any
end
Matrix3d.FromRows = fromRows

local function new(m00, m01, m02, m10, m11, m12, m20, m21, m22): Matrix3d
	return fromRows(V3(m00, m01, m02), V3(m10, m11, m12), V3(m20, m21, m22))
end
Matrix3d.new = new

Matrix3d.Identity = table.freeze(fromRows(Vector3d.AxisX, Vector3d.AxisY, Vector3d.AxisZ))
Matrix3d.Zero = table.freeze(fromRows(Vector3d.Zero, Vector3d.Zero, Vector3d.Zero))

function Matrix3d.FromArray(t: { number }): Matrix3d
	return new(t[1], t[2], t[3], t[4], t[5], t[6], t[7], t[8], t[9])
end

function Matrix3d.FromFunc(f: (number) -> number): Matrix3d
	return new(f(0), f(1), f(2), f(3), f(4), f(5), f(6), f(7), f(8))
end

function Matrix3d.FromFunc2(f: (number, number) -> number): Matrix3d
	return new(f(0, 0), f(0, 1), f(0, 2), f(1, 0), f(1, 1), f(1, 2), f(2, 0), f(2, 1), f(2, 2))
end

function Matrix3d.Diagonal(m00: number, m11: number, m22: number): Matrix3d
	return new(m00, 0, 0, 0, m11, 0, 0, 0, m22)
end

function Matrix3d.FromDiagonal(diag: Vector3d.Vector3d): Matrix3d
	return new(diag.x, 0, 0, 0, diag.y, 0, 0, 0, diag.z)
end

function Matrix3d.FromVectors(v1: Vector3d.Vector3d, v2: Vector3d.Vector3d, v3: Vector3d.Vector3d, bRows: boolean): Matrix3d
	if bRows then
		return fromRows(v1, v2, v3)
	end
	return new(v1.x, v2.x, v3.x, v1.y, v2.y, v3.y, v1.z, v2.z, v3.z)
end

-- outer-product of u*v^T
function Matrix3d.OuterProduct(u: Vector3d.Vector3d, v: Vector3d.Vector3d): Matrix3d
	return new(u.x * v.x, u.x * v.y, u.x * v.z, u.y * v.x, u.y * v.y, u.y * v.z, u.z * v.x, u.z * v.y, u.z * v.z)
end

-- this[r, c]
function Matrix3d.Get(self: Matrix3d, r: number, c: number): number
	local row = (r == 0) and self.Row0 or ((r == 1) and self.Row1 or self.Row2)
	return row:Get(c)
end

-- this[i], i in 0..8
function Matrix3d.GetLinear(self: Matrix3d, i: number): number
	if i > 5 then
		return self.Row2:Get(i % 3)
	elseif i > 2 then
		return self.Row1:Get(i % 3)
	end
	return self.Row0:Get(i % 3)
end

-- replacement for this[r, c] = value: returns new matrix
function Matrix3d.With(self: Matrix3d, r: number, c: number, value: number): Matrix3d
	if r == 0 then
		return fromRows(self.Row0:With(c, value), self.Row1, self.Row2)
	elseif r == 1 then
		return fromRows(self.Row0, self.Row1:With(c, value), self.Row2)
	end
	return fromRows(self.Row0, self.Row1, self.Row2:With(c, value))
end

function Matrix3d.Row(self: Matrix3d, i: number): Vector3d.Vector3d
	return (i == 0) and self.Row0 or ((i == 1) and self.Row1 or self.Row2)
end

function Matrix3d.Column(self: Matrix3d, i: number): Vector3d.Vector3d
	if i == 0 then
		return V3(self.Row0.x, self.Row1.x, self.Row2.x)
	elseif i == 1 then
		return V3(self.Row0.y, self.Row1.y, self.Row2.y)
	end
	return V3(self.Row0.z, self.Row1.z, self.Row2.z)
end

-- returns 1-based array of 9 values (row-major)
function Matrix3d.ToBuffer(self: Matrix3d): { number }
	local r0, r1, r2 = self.Row0, self.Row1, self.Row2
	return { r0.x, r0.y, r0.z, r1.x, r1.y, r1.z, r2.x, r2.y, r2.z }
end

-- Multiply(Vector3d v)
function Matrix3d.Multiply(self: Matrix3d, v: Vector3d.Vector3d): Vector3d.Vector3d
	local r0, r1, r2 = self.Row0, self.Row1, self.Row2
	return V3(r0.x * v.x + r0.y * v.y + r0.z * v.z, r1.x * v.x + r1.y * v.y + r1.z * v.z, r2.x * v.x + r2.y * v.y + r2.z * v.z)
end

local function matmul(mat1: Matrix3d, mat2: Matrix3d): Matrix3d
	local a0, a1, a2 = mat1.Row0, mat1.Row1, mat1.Row2
	local b0, b1, b2 = mat2.Row0, mat2.Row1, mat2.Row2
	local m00 = a0.x * b0.x + a0.y * b1.x + a0.z * b2.x
	local m01 = a0.x * b0.y + a0.y * b1.y + a0.z * b2.y
	local m02 = a0.x * b0.z + a0.y * b1.z + a0.z * b2.z
	local m10 = a1.x * b0.x + a1.y * b1.x + a1.z * b2.x
	local m11 = a1.x * b0.y + a1.y * b1.y + a1.z * b2.y
	local m12 = a1.x * b0.z + a1.y * b1.z + a1.z * b2.z
	local m20 = a2.x * b0.x + a2.y * b1.x + a2.z * b2.x
	local m21 = a2.x * b0.y + a2.y * b1.y + a2.z * b2.y
	local m22 = a2.x * b0.z + a2.y * b1.z + a2.z * b2.z
	return new(m00, m01, m02, m10, m11, m12, m20, m21, m22)
end

-- operator*: Matrix*scalar, scalar*Matrix, Matrix*Vector3d, Matrix*Matrix
Matrix3d.__mul = function(a: any, b: any): any
	if type(a) == "number" then
		a, b = b, a
	end
	if type(b) == "number" then
		local f = b
		local r0, r1, r2 = a.Row0, a.Row1, a.Row2
		return new(r0.x * f, r0.y * f, r0.z * f, r1.x * f, r1.y * f, r1.z * f, r2.x * f, r2.y * f, r2.z * f)
	end
	if getmetatable(b) == Matrix3d then
		return matmul(a, b)
	end
	return Matrix3d.Multiply(a, b)
end

Matrix3d.__unm = function(v: Matrix3d): Matrix3d
	return fromRows(-v.Row0, -v.Row1, -v.Row2)
end
Matrix3d.__add = function(m1: Matrix3d, m2: Matrix3d): Matrix3d
	return fromRows(m1.Row0 + m2.Row0, m1.Row1 + m2.Row1, m1.Row2 + m2.Row2)
end
Matrix3d.__sub = function(m1: Matrix3d, m2: Matrix3d): Matrix3d
	return fromRows(m1.Row0 - m2.Row0, m1.Row1 - m2.Row1, m1.Row2 - m2.Row2)
end

function Matrix3d.InnerProduct(self: Matrix3d, m2: Matrix3d): number
	return self.Row0:Dot(m2.Row0) + self.Row1:Dot(m2.Row1) + self.Row2:Dot(m2.Row2)
end

function Matrix3d.Determinant(self: Matrix3d): number
	local a11, a12, a13 = self.Row0.x, self.Row0.y, self.Row0.z
	local a21, a22, a23 = self.Row1.x, self.Row1.y, self.Row1.z
	local a31, a32, a33 = self.Row2.x, self.Row2.y, self.Row2.z
	local i00 = a33 * a22 - a32 * a23
	local i01 = -(a33 * a12 - a32 * a13)
	local i02 = a23 * a12 - a22 * a13
	return a11 * i00 + a21 * i01 + a31 * i02
end

function Matrix3d.Inverse(self: Matrix3d): Matrix3d
	local a11, a12, a13 = self.Row0.x, self.Row0.y, self.Row0.z
	local a21, a22, a23 = self.Row1.x, self.Row1.y, self.Row1.z
	local a31, a32, a33 = self.Row2.x, self.Row2.y, self.Row2.z
	local i00 = a33 * a22 - a32 * a23
	local i01 = -(a33 * a12 - a32 * a13)
	local i02 = a23 * a12 - a22 * a13

	local i10 = -(a33 * a21 - a31 * a23)
	local i11 = a33 * a11 - a31 * a13
	local i12 = -(a23 * a11 - a21 * a13)

	local i20 = a32 * a21 - a31 * a22
	local i21 = -(a32 * a11 - a31 * a12)
	local i22 = a22 * a11 - a21 * a12

	local det = a11 * i00 + a21 * i01 + a31 * i02
	if math.abs(det) < 4.94065645841247e-324 then -- double.Epsilon
		error("Matrix3d.Inverse: matrix is not invertible")
	end
	det = 1.0 / det
	return new(i00 * det, i01 * det, i02 * det, i10 * det, i11 * det, i12 * det, i20 * det, i21 * det, i22 * det)
end

function Matrix3d.Transpose(self: Matrix3d): Matrix3d
	local r0, r1, r2 = self.Row0, self.Row1, self.Row2
	return new(r0.x, r1.x, r2.x, r0.y, r1.y, r2.y, r0.z, r1.z, r2.z)
end

function Matrix3d.ToQuaternion(self: Matrix3d)
	return require(script.Parent.Quaterniond).FromRotationMatrix(self)
end

function Matrix3d.EpsilonEqual(self: Matrix3d, m2: Matrix3d, epsilon: number): boolean
	return self.Row0:EpsilonEqual(m2.Row0, epsilon) and self.Row1:EpsilonEqual(m2.Row1, epsilon) and self.Row2:EpsilonEqual(m2.Row2, epsilon)
end

function Matrix3d.RotateX(Angle: number, bIsDegrees: boolean): Matrix3d
	local angle = bIsDegrees and (Angle * MathUtil.Deg2Rad) or Angle
	local cs, sn = math.cos(angle), math.sin(angle)
	return new(1, 0, 0, 0, cs, -sn, 0, sn, cs)
end

function Matrix3d.RotateY(Angle: number, bIsDegrees: boolean): Matrix3d
	local angle = bIsDegrees and (Angle * MathUtil.Deg2Rad) or Angle
	local cs, sn = math.cos(angle), math.sin(angle)
	return new(cs, 0, sn, 0, 1, 0, -sn, 0, cs)
end

function Matrix3d.RotateZ(Angle: number, bIsDegrees: boolean): Matrix3d
	local angle = bIsDegrees and (Angle * MathUtil.Deg2Rad) or Angle
	local cs, sn = math.cos(angle), math.sin(angle)
	return new(cs, -sn, 0, sn, cs, 0, 0, 0, 1)
end

-- Scale(uniform) or Scale(sx, sy, sz)
function Matrix3d.Scale(sx: number, sy: number?, sz: number?): Matrix3d
	return Matrix3d.Diagonal(sx, sy or sx, sz or sx)
end

function Matrix3d.AxisAngleD(axis: Vector3d.Vector3d, angleDeg: number): Matrix3d
	local angle = angleDeg * MathUtil.Deg2Rad
	local cs = math.cos(angle)
	local sn = math.sin(angle)
	local oneMinusCos = 1.0 - cs
	local x2 = axis.x * axis.x
	local y2 = axis.y * axis.y
	local z2 = axis.z * axis.z
	local xym = axis.x * axis.y * oneMinusCos
	local xzm = axis.x * axis.z * oneMinusCos
	local yzm = axis.y * axis.z * oneMinusCos
	local xSin = axis.x * sn
	local ySin = axis.y * sn
	local zSin = axis.z * sn
	return new(
		x2 * oneMinusCos + cs,
		xym - zSin,
		xzm + ySin,
		xym + zSin,
		y2 * oneMinusCos + cs,
		yzm - xSin,
		xzm - ySin,
		yzm + xSin,
		z2 * oneMinusCos + cs
	)
end

Matrix3d.__tostring = function(m: Matrix3d): string
	return string.format("[%s] [%s] [%s]", tostring(m.Row0), tostring(m.Row1), tostring(m.Row2))
end

return Matrix3d
