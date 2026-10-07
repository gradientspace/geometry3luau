--!native
-- Port of geometry3Sharp mesh/Reducer.cs (struct QuadricError)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Stores quadratic function that evaluates distance to plane,
-- in minimal 10-coefficient form, following http://mgarland.org/files/papers/qtheory.pdf
-- - symmetric matrix A
-- - vector b
-- - constant c
--
-- Luau port notes:
--   * QuadricError is a C# struct; here it is a MUTABLE table (Add() modifies it). Clone() where C# copies.
--   * Constructors:
--       QuadricError.FromPlane(n, p)         new QuadricError(ref Vector3d n, ref Vector3d p)
--       QuadricError.FromSum(a, b)           new QuadricError(ref QuadricError a, ref QuadricError b)
--       QuadricError.Zero()                  QuadricError.Zero (returns a fresh zero quadric)
--   * OptimalPoint() -> ok, Vector3d result   (C# bool OptimalPoint(ref Vector3d result))
--   * Reducer stores quadrics in flat number arrays (10 coefficients per element, in the field order
--     Axx, Axy, Axz, Ayy, Ayz, Azz, bx, by, bz, c). The static helpers below operate on such arrays:
--       QuadricError.ToArray(q, arr, offset) / QuadricError.FromArray(arr, offset)
--       QuadricError.EvaluateFlat(arr, o, x, y, z) -> number
--       QuadricError.OptimalPointFlat(arr, o) -> ok, x, y, z

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)

local abs = math.abs
local DET_TOL = 1000.0 * MathUtil.Epsilon

local QuadricError = {}
QuadricError.__index = QuadricError
QuadricError.ClassName = "QuadricError"

local function make(Axx, Axy, Axz, Ayy, Ayz, Azz, bx, by, bz, c)
	return setmetatable(
		{ Axx = Axx, Axy = Axy, Axz = Axz, Ayy = Ayy, Ayz = Ayz, Azz = Azz, bx = bx, by = by, bz = bz, c = c },
		QuadricError
	)
end

function QuadricError.Zero()
	return make(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
end

function QuadricError.FromPlane(n, p)
	local Axx = n.x * n.x
	local Axy = n.x * n.y
	local Axz = n.x * n.z
	local Ayy = n.y * n.y
	local Ayz = n.y * n.z
	local Azz = n.z * n.z
	-- v = multiplyA(p)
	local vx = Axx * p.x + Axy * p.y + Axz * p.z
	local vy = Axy * p.x + Ayy * p.y + Ayz * p.z
	local vz = Axz * p.x + Ayz * p.y + Azz * p.z
	return make(Axx, Axy, Axz, Ayy, Ayz, Azz, -vx, -vy, -vz, p.x * vx + p.y * vy + p.z * vz)
end

function QuadricError.FromSum(a, b)
	return make(
		a.Axx + b.Axx,
		a.Axy + b.Axy,
		a.Axz + b.Axz,
		a.Ayy + b.Ayy,
		a.Ayz + b.Ayz,
		a.Azz + b.Azz,
		a.bx + b.bx,
		a.by + b.by,
		a.bz + b.bz,
		a.c + b.c
	)
end

function QuadricError:Clone()
	return make(self.Axx, self.Axy, self.Axz, self.Ayy, self.Ayz, self.Azz, self.bx, self.by, self.bz, self.c)
end

function QuadricError:Add(w: number, b)
	self.Axx += w * b.Axx
	self.Axy += w * b.Axy
	self.Axz += w * b.Axz
	self.Ayy += w * b.Ayy
	self.Ayz += w * b.Ayz
	self.Azz += w * b.Azz
	self.bx += w * b.bx
	self.by += w * b.by
	self.bz += w * b.bz
	self.c += w * b.c
end

-- returns pAp + 2*dot(p,b) + c
function QuadricError:Evaluate(pt): number
	local x = self.Axx * pt.x + self.Axy * pt.y + self.Axz * pt.z
	local y = self.Axy * pt.x + self.Ayy * pt.y + self.Ayz * pt.z
	local z = self.Axz * pt.x + self.Ayz * pt.y + self.Azz * pt.z
	return (pt.x * x + pt.y * y + pt.z * z) + 2.0 * (pt.x * self.bx + pt.y * self.by + pt.z * self.bz) + self.c
end

function QuadricError:multiplyA(pt)
	local x = self.Axx * pt.x + self.Axy * pt.y + self.Axz * pt.z
	local y = self.Axy * pt.x + self.Ayy * pt.y + self.Ayz * pt.z
	local z = self.Axz * pt.x + self.Ayz * pt.y + self.Azz * pt.z
	return Vector3d.new(x, y, z)
end

-- shared optimal point solve on scalars -> ok, x, y, z
local function optimal_point(Axx, Axy, Axz, Ayy, Ayz, Azz, bx, by, bz): (boolean, number, number, number)
	local a11 = Azz * Ayy - Ayz * Ayz
	local a12 = Axz * Ayz - Azz * Axy
	local a13 = Axy * Ayz - Axz * Ayy
	local a22 = Azz * Axx - Axz * Axz
	local a23 = Axy * Axz - Axx * Ayz
	local a33 = Axx * Ayy - Axy * Axy
	local det = (Axx * a11) + (Axy * a12) + (Axz * a13)
	-- [RMS] not sure what we should be using for this threshold...have seen
	--  det less than 10^-9 on "normal" meshes.
	if abs(det) > DET_TOL then
		det = 1.0 / det
		a11 *= det
		a12 *= det
		a13 *= det
		a22 *= det
		a23 *= det
		a33 *= det
		local x = a11 * bx + a12 * by + a13 * bz
		local y = a12 * bx + a22 * by + a23 * bz
		local z = a13 * bx + a23 * by + a33 * bz
		return true, -x, -y, -z
	end
	return false, 0, 0, 0
end

-- C# bool OptimalPoint(ref Vector3d result) -> ok, result (result is nil if not ok)
function QuadricError:OptimalPoint()
	local ok, x, y, z = optimal_point(self.Axx, self.Axy, self.Axz, self.Ayy, self.Ayz, self.Azz, self.bx, self.by, self.bz)
	if ok then
		return true, Vector3d.new(x, y, z)
	end
	return false, nil
end

-- flat-array helpers ------------------------------------------------------------------------------

function QuadricError.ToArray(q, arr: { number }, o: number)
	arr[o + 1], arr[o + 2], arr[o + 3], arr[o + 4], arr[o + 5] = q.Axx, q.Axy, q.Axz, q.Ayy, q.Ayz
	arr[o + 6], arr[o + 7], arr[o + 8], arr[o + 9], arr[o + 10] = q.Azz, q.bx, q.by, q.bz, q.c
end

function QuadricError.FromArray(arr: { number }, o: number)
	return make(arr[o + 1], arr[o + 2], arr[o + 3], arr[o + 4], arr[o + 5], arr[o + 6], arr[o + 7], arr[o + 8], arr[o + 9], arr[o + 10])
end

function QuadricError.EvaluateFlat(q: { number }, o: number, px: number, py: number, pz: number): number
	local Axx, Axy, Axz, Ayy, Ayz, Azz = q[o + 1], q[o + 2], q[o + 3], q[o + 4], q[o + 5], q[o + 6]
	local x = Axx * px + Axy * py + Axz * pz
	local y = Axy * px + Ayy * py + Ayz * pz
	local z = Axz * px + Ayz * py + Azz * pz
	return (px * x + py * y + pz * z) + 2.0 * (px * q[o + 7] + py * q[o + 8] + pz * q[o + 9]) + q[o + 10]
end

function QuadricError.OptimalPointFlat(q: { number }, o: number): (boolean, number, number, number)
	return optimal_point(q[o + 1], q[o + 2], q[o + 3], q[o + 4], q[o + 5], q[o + 6], q[o + 7], q[o + 8], q[o + 9])
end

return QuadricError
