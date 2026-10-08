--!native
-- Port of geometry3Sharp math/Matrix4d.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Matrix4d values are IMMUTABLE by convention (rows are Vector4d).
-- Constructors:
--   Matrix4d.new(m00, m01, ..., m33)               (16 values, row-major)
--   Matrix4d.Identity / Matrix4d.Zero               (frozen constants) new Matrix4d(bool bIdentity)
--   Matrix4d.FromRows(r0, r1, r2, r3)              (rows are Vector4d)
--   Matrix4d.FromArray(t)                          new Matrix4d(double[] / float[] / Span mat), t is 1-based {m00..m33}
--   Matrix4d.FromArray2D(t)                        new Matrix4d(double[,] / float[,] mat), t[r+1][c+1] (row-major)
--   Matrix4d.FromFunc(f)                           new Matrix4d(Func<int,double>) (0-based buffer index)
--   Matrix4d.FromFunc2(f)                          new Matrix4d(Func<int,int,double>)
--   Matrix4d.Diagonal(m00, m11, m22, m33)          new Matrix4d(double m00, double m11, double m22, double m33)
--   Matrix4d.FromVectors(v0, v1, v2, v3, bRows)    new Matrix4d(Vector4d v0, v1, v2, v3, bool bRows)
--   Matrix4d.FromMatrix3(mat3x3, translation?)     new Matrix4d(Matrix3d) / new Matrix4d(Matrix3d, Vector3d Translation)
--   Matrix4d.FromQuaternion(rotation, translation) new Matrix4d(Quaterniond Rotation, Vector3d Translation)
--   Matrix4d.FromFrame(frame)                      new Matrix4d(Frame3d Frame)
--   Matrix4d.OuterProduct(u, v)                    new Matrix4d(ref Vector4d u, ref Vector4d v)
--
-- API differences:
--   * C# indexers: m[r, c] -> m:Get(r, c), m[i] -> m:GetLinear(i) (0-based). Setters return a new matrix:
--     m:With(r, c, value), m:WithLinear(i, value).
--   * Properties IsAffine / IsIdentity are methods: m:IsAffine(), m:IsIdentity().
--   * GetColumns(out c0..c3) -> local c0, c1, c2, c3 = m:GetColumns().
--   * ToBuffer() returns a 1-based array of 16 values; ToBuffer(buf) fills buf[1..16].
--   * GetAffineNormalTransform(out NormalTransform) -> local ok, NormalTransform = m:GetAffineNormalTransform().
--   * Multiply(v, out vOut) is the same function as Multiply(v).
--   * The Vector3f overloads of TransformPointAffine / TransformVectorAffine are the same as the Vector3d ones.
--   * The C# outer-product constructor assigns Row2 twice and leaves Row3 zero (a typo); here the fourth row
--     is u.w * v, as the comment "Mij = u_i * v_j" describes.
--   * C# Determinant() and the non-affine branch of Inverse() throw NotImplementedException. Here they are
--     implemented with cofactor expansion (Inverse errors if the matrix is singular, like Matrix3d.Inverse).

local Vector3d = require(script.Parent.Vector3d)
local Vector4d = require(script.Parent.Vector4d)
local Matrix3d = require(script.Parent.Matrix3d)

local V3 = Vector3d.new
local V4 = Vector4d.new

local Matrix4d = {}
Matrix4d.__index = Matrix4d
Matrix4d.ClassName = "Matrix4d"

export type Matrix4d = typeof(setmetatable(
	{} :: { Row0: Vector4d.Vector4d, Row1: Vector4d.Vector4d, Row2: Vector4d.Vector4d, Row3: Vector4d.Vector4d },
	Matrix4d
))

local function fromRows(r0, r1, r2, r3): Matrix4d
	return setmetatable({ Row0 = r0, Row1 = r1, Row2 = r2, Row3 = r3 }, Matrix4d) :: any
end
Matrix4d.FromRows = fromRows

local function new(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33): Matrix4d
	return fromRows(V4(m00, m01, m02, m03), V4(m10, m11, m12, m13), V4(m20, m21, m22, m23), V4(m30, m31, m32, m33))
end
Matrix4d.new = new

Matrix4d.Identity = table.freeze(fromRows(Vector4d.UnitX, Vector4d.UnitY, Vector4d.UnitZ, Vector4d.UnitW))
Matrix4d.Zero = table.freeze(fromRows(Vector4d.Zero, Vector4d.Zero, Vector4d.Zero, Vector4d.Zero))

-- assumes input is row-major...
function Matrix4d.FromArray(t: { number }): Matrix4d
	return new(t[1], t[2], t[3], t[4], t[5], t[6], t[7], t[8], t[9], t[10], t[11], t[12], t[13], t[14], t[15], t[16])
end

-- assumes input is row-major: t[r+1][c+1]
function Matrix4d.FromArray2D(t: { { number } }): Matrix4d
	return fromRows(
		V4(t[1][1], t[1][2], t[1][3], t[1][4]),
		V4(t[2][1], t[2][2], t[2][3], t[2][4]),
		V4(t[3][1], t[3][2], t[3][3], t[3][4]),
		V4(t[4][1], t[4][2], t[4][3], t[4][4])
	)
end

function Matrix4d.FromFunc(f: (number) -> number): Matrix4d
	return new(f(0), f(1), f(2), f(3), f(4), f(5), f(6), f(7), f(8), f(9), f(10), f(11), f(12), f(13), f(14), f(15))
end

function Matrix4d.FromFunc2(f: (number, number) -> number): Matrix4d
	return fromRows(
		V4(f(0, 0), f(0, 1), f(0, 2), f(0, 3)),
		V4(f(1, 0), f(1, 1), f(1, 2), f(1, 3)),
		V4(f(2, 0), f(2, 1), f(2, 2), f(2, 3)),
		V4(f(3, 0), f(3, 1), f(3, 2), f(3, 3))
	)
end

function Matrix4d.Diagonal(m00: number, m11: number, m22: number, m33: number): Matrix4d
	return new(m00, 0, 0, 0, 0, m11, 0, 0, 0, 0, m22, 0, 0, 0, 0, m33)
end

function Matrix4d.FromVectors(
	v0: Vector4d.Vector4d,
	v1: Vector4d.Vector4d,
	v2: Vector4d.Vector4d,
	v3: Vector4d.Vector4d,
	bRows: boolean
): Matrix4d
	if bRows then
		return fromRows(v0, v1, v2, v3)
	end
	return fromRows(V4(v0.x, v1.x, v2.x, v3.x), V4(v0.y, v1.y, v2.y, v3.y), V4(v0.z, v1.z, v2.z, v3.z), V4(v0.w, v1.w, v2.w, v3.w))
end

-- C# set_mat_vec
local function set_mat_vec(mat3x3: Matrix3d.Matrix3d, translation: Vector3d.Vector3d): Matrix4d
	local r0, r1, r2 = mat3x3.Row0, mat3x3.Row1, mat3x3.Row2
	return fromRows(
		V4(r0.x, r0.y, r0.z, translation.x),
		V4(r1.x, r1.y, r1.z, translation.y),
		V4(r2.x, r2.y, r2.z, translation.z),
		V4(0, 0, 0, 1)
	)
end

function Matrix4d.FromMatrix3(mat3x3: Matrix3d.Matrix3d, Translation: Vector3d.Vector3d?): Matrix4d
	return set_mat_vec(mat3x3, Translation or Vector3d.Zero)
end

function Matrix4d.FromQuaternion(Rotation: any, Translation: Vector3d.Vector3d): Matrix4d
	return set_mat_vec(Rotation:ToRotationMatrix(), Translation)
end

function Matrix4d.FromFrame(Frame: any): Matrix4d
	return set_mat_vec(Frame.Rotation:ToRotationMatrix(), Frame.Origin)
end

-- Construct outer-product of u*transpose(v) of u and v
-- result is that Mij = u_i * v_j
function Matrix4d.OuterProduct(u: Vector4d.Vector4d, v: Vector4d.Vector4d): Matrix4d
	return fromRows(
		V4(u.x * v.x, u.x * v.y, u.x * v.z, u.x * v.w),
		V4(u.y * v.x, u.y * v.y, u.y * v.z, u.y * v.w),
		V4(u.z * v.x, u.z * v.y, u.z * v.z, u.z * v.w),
		V4(u.w * v.x, u.w * v.y, u.w * v.z, u.w * v.w)
	)
end

-- this[r, c]
function Matrix4d.Get(self: Matrix4d, r: number, c: number): number
	if r == 0 then
		return self.Row0:Get(c)
	elseif r == 1 then
		return self.Row1:Get(c)
	elseif r == 2 then
		return self.Row2:Get(c)
	end
	return self.Row3:Get(c)
end

-- this[i], i in 0..15
function Matrix4d.GetLinear(self: Matrix4d, i: number): number
	if i > 11 then
		return self.Row3:Get(i % 4)
	elseif i > 7 then
		return self.Row2:Get(i % 4)
	elseif i > 3 then
		return self.Row1:Get(i % 4)
	end
	return self.Row0:Get(i % 4)
end

-- replacement for this[r, c] = value: returns new matrix
function Matrix4d.With(self: Matrix4d, r: number, c: number, value: number): Matrix4d
	if r == 0 then
		return fromRows(self.Row0:With(c, value), self.Row1, self.Row2, self.Row3)
	elseif r == 1 then
		return fromRows(self.Row0, self.Row1:With(c, value), self.Row2, self.Row3)
	elseif r == 2 then
		return fromRows(self.Row0, self.Row1, self.Row2:With(c, value), self.Row3)
	end
	return fromRows(self.Row0, self.Row1, self.Row2, self.Row3:With(c, value))
end

-- replacement for this[i] = value: returns new matrix
function Matrix4d.WithLinear(self: Matrix4d, i: number, value: number): Matrix4d
	if i > 11 then
		return self:With(3, i % 4, value)
	elseif i > 7 then
		return self:With(2, i % 4, value)
	elseif i > 3 then
		return self:With(1, i % 4, value)
	end
	return self:With(0, i % 4, value)
end

function Matrix4d.Row(self: Matrix4d, i: number): Vector4d.Vector4d
	return (i == 0) and self.Row0 or ((i == 1) and self.Row1 or ((i == 2) and self.Row2 or self.Row3))
end

function Matrix4d.Column(self: Matrix4d, i: number): Vector4d.Vector4d
	local r0, r1, r2, r3 = self.Row0, self.Row1, self.Row2, self.Row3
	if i == 0 then
		return V4(r0.x, r1.x, r2.x, r3.x)
	elseif i == 1 then
		return V4(r0.y, r1.y, r2.y, r3.y)
	elseif i == 2 then
		return V4(r0.z, r1.z, r2.z, r3.z)
	end
	return V4(r0.w, r1.w, r2.w, r3.w)
end

-- C# GetColumns(out Column0, out Column1, out Column2, out Column3)
function Matrix4d.GetColumns(self: Matrix4d): (Vector4d.Vector4d, Vector4d.Vector4d, Vector4d.Vector4d, Vector4d.Vector4d)
	local r0, r1, r2, r3 = self.Row0, self.Row1, self.Row2, self.Row3
	return V4(r0.x, r1.x, r2.x, r3.x), V4(r0.y, r1.y, r2.y, r3.y), V4(r0.z, r1.z, r2.z, r3.z), V4(r0.w, r1.w, r2.w, r3.w)
end

-- returns 1-based array of 16 values (row-major); if buf is given, fills buf[1..16] and returns it
function Matrix4d.ToBuffer(self: Matrix4d, buf: { number }?): { number }
	local r0, r1, r2, r3 = self.Row0, self.Row1, self.Row2, self.Row3
	if buf == nil then
		return { r0.x, r0.y, r0.z, r0.w, r1.x, r1.y, r1.z, r1.w, r2.x, r2.y, r2.z, r2.w, r3.x, r3.y, r3.z, r3.w }
	end
	local b = buf :: { number }
	b[1], b[2], b[3], b[4] = r0.x, r0.y, r0.z, r0.w
	b[5], b[6], b[7], b[8] = r1.x, r1.y, r1.z, r1.w
	b[9], b[10], b[11], b[12] = r2.x, r2.y, r2.z, r2.w
	b[13], b[14], b[15], b[16] = r3.x, r3.y, r3.z, r3.w
	return b
end

-- Multiply(Vector4d v) / Multiply(Vector4d v, out vOut)
function Matrix4d.Multiply(self: Matrix4d, v: Vector4d.Vector4d): Vector4d.Vector4d
	local r0, r1, r2, r3 = self.Row0, self.Row1, self.Row2, self.Row3
	return V4(
		r0.x * v.x + r0.y * v.y + r0.z * v.z + r0.w * v.w,
		r1.x * v.x + r1.y * v.y + r1.z * v.z + r1.w * v.w,
		r2.x * v.x + r2.y * v.y + r2.z * v.z + r2.w * v.w,
		r3.x * v.x + r3.y * v.y + r3.z * v.z + r3.w * v.w
	)
end

function Matrix4d.TransformPointAffine(self: Matrix4d, v: Vector3d.Vector3d): Vector3d.Vector3d
	-- Vector4d vv = new(v.x, v.y, v.z, 1.0)
	local r0, r1, r2 = self.Row0, self.Row1, self.Row2
	return V3(
		r0.x * v.x + r0.y * v.y + r0.z * v.z + r0.w,
		r1.x * v.x + r1.y * v.y + r1.z * v.z + r1.w,
		r2.x * v.x + r2.y * v.y + r2.z * v.z + r2.w
	)
end

function Matrix4d.TransformVectorAffine(self: Matrix4d, v: Vector3d.Vector3d): Vector3d.Vector3d
	-- Vector4d vv = new(v.x, v.y, v.z, 0.0)
	local r0, r1, r2 = self.Row0, self.Row1, self.Row2
	return V3(r0.x * v.x + r0.y * v.y + r0.z * v.z, r1.x * v.x + r1.y * v.y + r1.z * v.z, r2.x * v.x + r2.y * v.y + r2.z * v.z)
end

local function matmul(mat1: Matrix4d, mat2: Matrix4d): Matrix4d
	local Col0, Col1, Col2, Col3 = mat2:GetColumns()
	local a0, a1, a2, a3 = mat1.Row0, mat1.Row1, mat1.Row2, mat1.Row3
	return fromRows(
		V4(a0:Dot(Col0), a0:Dot(Col1), a0:Dot(Col2), a0:Dot(Col3)),
		V4(a1:Dot(Col0), a1:Dot(Col1), a1:Dot(Col2), a1:Dot(Col3)),
		V4(a2:Dot(Col0), a2:Dot(Col1), a2:Dot(Col2), a2:Dot(Col3)),
		V4(a3:Dot(Col0), a3:Dot(Col1), a3:Dot(Col2), a3:Dot(Col3))
	)
end

-- operator*: Matrix*scalar, scalar*Matrix, Matrix*Vector4d, Matrix*Matrix
Matrix4d.__mul = function(a: any, b: any): any
	if type(a) == "number" then
		a, b = b, a
	end
	if type(b) == "number" then
		return fromRows(a.Row0 * b, a.Row1 * b, a.Row2 * b, a.Row3 * b)
	end
	if getmetatable(b) == Matrix4d then
		return matmul(a, b)
	end
	return Matrix4d.Multiply(a, b)
end

Matrix4d.__add = function(m1: Matrix4d, m2: Matrix4d): Matrix4d
	return fromRows(m1.Row0 + m2.Row0, m1.Row1 + m2.Row1, m1.Row2 + m2.Row2, m1.Row3 + m2.Row3)
end
Matrix4d.__sub = function(m1: Matrix4d, m2: Matrix4d): Matrix4d
	return fromRows(m1.Row0 - m2.Row0, m1.Row1 - m2.Row1, m1.Row2 - m2.Row2, m1.Row3 - m2.Row3)
end
Matrix4d.__eq = function(m1: Matrix4d, m2: Matrix4d): boolean
	return m1.Row0 == m2.Row0 and m1.Row1 == m2.Row1 and m1.Row2 == m2.Row2 and m1.Row3 == m2.Row3
end

function Matrix4d.InnerProduct(self: Matrix4d, m2: Matrix4d): number
	return self.Row0:Dot(m2.Row0) + self.Row1:Dot(m2.Row1) + self.Row2:Dot(m2.Row2) + self.Row3:Dot(m2.Row3)
end

-- cofactors of the matrix (shared by Determinant and Inverse). Returns the determinant and the
-- 16 entries of the adjugate (transposed cofactor matrix), row-major.
local function cofactors(m: Matrix4d)
	local a, b, c, d = m.Row0.x, m.Row0.y, m.Row0.z, m.Row0.w
	local e, f, g, h = m.Row1.x, m.Row1.y, m.Row1.z, m.Row1.w
	local i, j, k, l = m.Row2.x, m.Row2.y, m.Row2.z, m.Row2.w
	local mm, n, o, p = m.Row3.x, m.Row3.y, m.Row3.z, m.Row3.w

	-- 2x2 sub-determinants of the bottom two rows (for cofactors of rows 0,1)
	local kp_lo, jp_ln, jo_kn = k * p - l * o, j * p - l * n, j * o - k * n
	local ip_lm, io_km, in_jm = i * p - l * mm, i * o - k * mm, i * n - j * mm
	-- 2x2 sub-determinants of the top two rows (for cofactors of rows 2,3)
	local ch_dg, bh_df, bg_cf = c * h - d * g, b * h - d * f, b * g - c * f
	local ah_de, ag_ce, af_be = a * h - d * e, a * g - c * e, a * f - b * e

	local c00 = f * kp_lo - g * jp_ln + h * jo_kn
	local c01 = -(e * kp_lo - g * ip_lm + h * io_km)
	local c02 = e * jp_ln - f * ip_lm + h * in_jm
	local c03 = -(e * jo_kn - f * io_km + g * in_jm)
	local det = a * c00 + b * c01 + c * c02 + d * c03

	local c10 = -(b * kp_lo - c * jp_ln + d * jo_kn)
	local c11 = a * kp_lo - c * ip_lm + d * io_km
	local c12 = -(a * jp_ln - b * ip_lm + d * in_jm)
	local c13 = a * jo_kn - b * io_km + c * in_jm

	local c20 = n * ch_dg - o * bh_df + p * bg_cf
	local c21 = -(mm * ch_dg - o * ah_de + p * ag_ce)
	local c22 = mm * bh_df - n * ah_de + p * af_be
	local c23 = -(mm * bg_cf - n * ag_ce + o * af_be)

	local c30 = -(j * ch_dg - k * bh_df + l * bg_cf)
	local c31 = i * ch_dg - k * ah_de + l * ag_ce
	local c32 = -(i * bh_df - j * ah_de + l * af_be)
	local c33 = i * bg_cf - j * ag_ce + k * af_be

	-- adjugate = transpose of cofactor matrix
	return det, c00, c10, c20, c30, c01, c11, c21, c31, c02, c12, c22, c32, c03, c13, c23, c33
end

-- C# throws NotImplementedException; implemented here (cofactor expansion)
function Matrix4d.Determinant(self: Matrix4d): number
	return (cofactors(self))
end

function Matrix4d.IsAffine(self: Matrix4d): boolean
	return self.Row3 == Vector4d.UnitW
end

function Matrix4d.IsIdentity(self: Matrix4d): boolean
	return self.Row0 == Vector4d.UnitX and self.Row1 == Vector4d.UnitY and self.Row2 == Vector4d.UnitZ and self.Row3 == Vector4d.UnitW
end

function Matrix4d.Inverse(self: Matrix4d): Matrix4d
	if self:IsAffine() then
		-- can invert more efficiently
		-- https://stackoverflow.com/questions/2624422/efficient-4x4-matrix-inverse-affine-transform
		local M = self:GetAffineTransform()
		local b = self:GetAffineTranslation()
		M = M:Inverse()
		b = -(M * b)
		return Matrix4d.FromMatrix3(M, b)
	else
		-- C# throws NotImplementedException here; general inverse via the adjugate
		local det, m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33 = cofactors(self)
		if math.abs(det) < 4.94065645841247e-324 then -- double.Epsilon
			error("Matrix4d.Inverse: matrix is not invertible")
		end
		local s = 1.0 / det
		return new(
			m00 * s,
			m01 * s,
			m02 * s,
			m03 * s,
			m10 * s,
			m11 * s,
			m12 * s,
			m13 * s,
			m20 * s,
			m21 * s,
			m22 * s,
			m23 * s,
			m30 * s,
			m31 * s,
			m32 * s,
			m33 * s
		)
	end
end

function Matrix4d.Transpose(self: Matrix4d): Matrix4d
	return Matrix4d.FromVectors(self.Row0, self.Row1, self.Row2, self.Row3, false)
end

function Matrix4d.GetAffineTransform(self: Matrix4d): Matrix3d.Matrix3d
	local r0, r1, r2 = self.Row0, self.Row1, self.Row2
	return Matrix3d.new(r0.x, r0.y, r0.z, r1.x, r1.y, r1.z, r2.x, r2.y, r2.z)
end

function Matrix4d.GetAffineTranslation(self: Matrix4d): Vector3d.Vector3d
	return V3(self.Row0.w, self.Row1.w, self.Row2.w)
end

-- C# bool GetAffineNormalTransform(out Matrix3d NormalTransform) -> returns ok, NormalTransform
function Matrix4d.GetAffineNormalTransform(self: Matrix4d): (boolean, Matrix3d.Matrix3d)
	if self:IsAffine() == false then
		return false, Matrix3d.Identity
	end
	local XForm = self:GetAffineTransform()
	return true, XForm:Inverse():Transpose()
end

function Matrix4d.EpsilonEqual(self: Matrix4d, m2: Matrix4d, epsilon: number): boolean
	return self.Row0:EpsilonEqual(m2.Row0, epsilon)
		and self.Row1:EpsilonEqual(m2.Row1, epsilon)
		and self.Row2:EpsilonEqual(m2.Row2, epsilon)
		and self.Row3:EpsilonEqual(m2.Row3, epsilon)
end

function Matrix4d.AxisAngleDeg(axis: Vector3d.Vector3d, angleDeg: number): Matrix4d
	local rot = Matrix3d.AxisAngleD(axis, angleDeg)
	return Matrix4d.FromMatrix3(rot)
end

function Matrix4d.Translation(Translation: Vector3d.Vector3d): Matrix4d
	return Matrix4d.FromVectors(Vector4d.UnitX, Vector4d.UnitY, Vector4d.UnitZ, V4(Translation.x, Translation.y, Translation.z, 1.0), false)
end

function Matrix4d.Scale(Scale: Vector3d.Vector3d): Matrix4d
	return Matrix4d.Diagonal(Scale.x, Scale.y, Scale.z, 1.0)
end

function Matrix4d.Affine(AffineTransform: Matrix3d.Matrix3d): Matrix4d
	return Matrix4d.FromMatrix3(AffineTransform)
end

Matrix4d.__tostring = function(m: Matrix4d): string
	return string.format("[%s] [%s] [%s] [%s]", tostring(m.Row0), tostring(m.Row1), tostring(m.Row2), tostring(m.Row3))
end

function Matrix4d.ToString(self: Matrix4d, fmt: string?): string
	if fmt == nil then
		return tostring(self)
	end
	return string.format(
		"[%s] [%s] [%s] [%s]",
		self.Row0:ToString(fmt),
		self.Row1:ToString(fmt),
		self.Row2:ToString(fmt),
		self.Row3:ToString(fmt)
	)
end

return Matrix4d
