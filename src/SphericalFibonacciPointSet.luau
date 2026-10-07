--!native
-- Port of geometry3Sharp comp_geom/SphericalFibonacciPointSet.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- A Spherical Fibonacci Point Set is a set of points that are roughly evenly distributed on
-- a sphere. Basically the points lie on a spiral, see pdf below.
-- The i-th SF point of an N-point set can be calculated directly.
-- For a given (normalized) point P, finding the nearest SF point (ie mapping back to i)
-- can be done in constant time.
--
-- math from http://lgdv.cs.fau.de/uploads/publications/spherical_fibonacci_mapping_opt.pdf
--
-- Luau port notes:
--   * SphericalFibonacciPointSet.new(n = 64); field N; Count() method; C# indexer this[i] -> Get(i) (= Point(i)).
--   * Point indices are 0-based like the C#.
--   * The 2x2 matrix math (C# Matrix2d B, B.Inverse(), invB * c, B.Row(1)) is inlined with scalars.
--   * C# Math.Round (round-half-to-even) -> MathUtil.RoundToEven.

local MathUtil = require(script.Parent.MathUtil)
local Util = require(script.Parent.Util)
local Vector3d = require(script.Parent.Vector3d)

local floor, sqrt, cos, sin, acos, atan2, log, pi = math.floor, math.sqrt, math.cos, math.sin, math.acos, math.atan2, math.log, math.pi

local PHI = (sqrt(5.0) + 1.0) / 2.0

local SphericalFibonacciPointSet = {}
SphericalFibonacciPointSet.__index = SphericalFibonacciPointSet
SphericalFibonacciPointSet.ClassName = "SphericalFibonacciPointSet"
SphericalFibonacciPointSet.PHI = PHI

function SphericalFibonacciPointSet.new(n: number?)
	return setmetatable({ N = n or 64 }, SphericalFibonacciPointSet)
end

function SphericalFibonacciPointSet:Count(): number
	return self.N
end

-- Compute i'th spherical point
function SphericalFibonacciPointSet:Point(i: number)
	Util.gDevAssert(i < self.N)
	local N = self.N
	local div = i / PHI
	local phi = MathUtil.TwoPI * (div - floor(div))
	local cos_phi, sin_phi = cos(phi), sin(phi)

	local z = 1.0 - (2.0 * i + 1.0) / N
	local theta = acos(z)
	local sin_theta = sin(theta)

	return Vector3d.new(cos_phi * sin_theta, sin_phi * sin_theta, z)
end

-- C# indexer this[i]
function SphericalFibonacciPointSet:Get(i: number)
	return self:Point(i)
end

-- Find index of nearest point-set point for input arbitrary point
function SphericalFibonacciPointSet:NearestPoint(p, bIsNormalized: boolean?): number
	if bIsNormalized then
		return self:inverseSF(p)
	end
	return self:inverseSF(p:Normalized())
end

local function madfrac(a: number, b: number): number
	--#define madfrac(A,B) mad((A),(B),-floor((A)*(B)))
	return a * b + -floor(a * b)
end

-- This computes mapping from p to i. Note that the code in the original PDF is HLSL shader code.
-- I have ported here to comparable C# functions. *However* the PDF also explains some assumptions
-- made about what certain operators return in different cases (particularly NaN handling).
-- I have not yet tested these cases to make sure C# behavior is the same (not sure when they happen).
function SphericalFibonacciPointSet:inverseSF(p): number
	local N = self.N
	local phi = math.min(atan2(p.y, p.x), pi)
	local cosTheta = p.z
	local k = math.max(2.0, floor(log(N * pi * sqrt(5.0) * (1.0 - cosTheta * cosTheta)) / log(PHI * PHI)))
	local Fk = PHI ^ k / sqrt(5.0)

	--double F0 = round(Fk), F1 = round(Fk * PHI);
	local F0, F1 = MathUtil.RoundToEven(Fk), MathUtil.RoundToEven(Fk * PHI)

	-- Matrix2d B = new Matrix2d(b00, b01, b10, b11)  (row-major)
	local b00 = 2 * pi * madfrac(F0 + 1, PHI - 1) - 2 * pi * (PHI - 1)
	local b01 = 2 * pi * madfrac(F1 + 1, PHI - 1) - 2 * pi * (PHI - 1)
	local b10 = -2 * F0 / N
	local b11 = -2 * F1 / N
	-- Matrix2d invB = B.Inverse()
	local det = b00 * b11 - b01 * b10
	local invDet = 1.0 / det
	local i00, i01, i10, i11 = b11 * invDet, -b01 * invDet, -b10 * invDet, b00 * invDet

	--Vector2d c = floor(mul(invB, double2(phi, cosTheta - (1 - 1.0/N))));
	local cx0, cy0 = phi, cosTheta - (1 - 1.0 / N)
	local cx = floor(i00 * cx0 + i01 * cy0)
	local cy = floor(i10 * cx0 + i11 * cy0)

	local d, j = math.huge, 0
	for s = 0, 3 do
		-- Vector2d cosTheta_second = new Vector2d(s%2, s/2) + c;
		local sx, sy = (s % 2) + cx, (s // 2) + cy
		cosTheta = b10 * sx + b11 * sy + (1 - 1.0 / N)
		cosTheta = MathUtil.Clamp(cosTheta, -1.0, 1.0) * 2.0 - cosTheta
		local i = floor(N * 0.5 - cosTheta * N * 0.5)
		phi = 2.0 * pi * madfrac(i, PHI - 1)
		cosTheta = 1.0 - (2.0 * i + 1.0) * (1.0 / N) -- rcp(n);
		local sinTheta = sqrt(1.0 - cosTheta * cosTheta)
		local qx, qy, qz = cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta
		local dx, dy, dz = qx - p.x, qy - p.y, qz - p.z
		local squaredDistance = dx * dx + dy * dy + dz * dz
		if squaredDistance < d then
			d = squaredDistance
			j = i
		end
	end

	-- [TODO] should we be clamping this??
	return MathUtil.TruncInt(j)
end

return SphericalFibonacciPointSet
