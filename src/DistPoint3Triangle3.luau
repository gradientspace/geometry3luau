--!native
-- Port of geometry3Sharp distance/DistPoint3Triangle3.cs (ported from WildMagic 5)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau API notes:
--   DistPoint3Triangle3.new(point, triangle)      triangle is a Triangle3d (stored by reference, not copied)
--   q:Compute() -> q, q:Get(), q:GetSquared()
--   fields: DistanceSquared (-1 until computed), TriangleClosest, TriangleBaryCoords (Vector3d)
--   C# properties Point/Triangle -> q:Point(), q:SetPoint(v), q:Triangle(), q:SetTriangle(t)
--     (setters reset DistanceSquared like C#)
--   static DistanceSqr(point, triangle) -> sqrDist, closestPoint, baryCoords
--     (C# DistanceSqr(ref point, ref triangle, out closestPoint, out baryCoords))
--   static DistanceSqrXYZ(px,py,pz, ax,ay,az, bx,by,bz, cx,cy,cz) -> sqrDist, s, t
--     allocation-free variant (closest = V0 + s*(V1-V0) + t*(V2-V0), bary = (1-s-t, s, t))

local Vector3d = require(script.Parent.Vector3d)

local V3 = Vector3d.new
local abs = math.abs

local DistPoint3Triangle3 = {}
DistPoint3Triangle3.__index = DistPoint3Triangle3
DistPoint3Triangle3.ClassName = "DistPoint3Triangle3"

function DistPoint3Triangle3.new(PointIn, TriangleIn)
	return setmetatable({
		point = PointIn,
		triangle = TriangleIn,
		DistanceSquared = -1.0,
		TriangleClosest = Vector3d.Zero,
		TriangleBaryCoords = Vector3d.Zero,
	}, DistPoint3Triangle3)
end

function DistPoint3Triangle3:Point()
	return self.point
end
function DistPoint3Triangle3:SetPoint(value)
	self.point = value
	self.DistanceSquared = -1.0
end
function DistPoint3Triangle3:Triangle()
	return self.triangle
end
function DistPoint3Triangle3:SetTriangle(value)
	self.triangle = value
	self.DistanceSquared = -1.0
end

function DistPoint3Triangle3:Compute()
	self:GetSquared()
	return self
end

function DistPoint3Triangle3:Get(): number
	return math.sqrt(self:GetSquared())
end

function DistPoint3Triangle3:GetSquared(): number
	if self.DistanceSquared >= 0 then
		return self.DistanceSquared
	end
	local d, closest, bary = DistPoint3Triangle3.DistanceSqr(self.point, self.triangle)
	self.DistanceSquared = d
	self.TriangleClosest = closest
	self.TriangleBaryCoords = bary
	return d
end

-- core of DistanceSqr on scalars. Returns (unclamped) sqrDistance, s, t
local function distance_sqr_xyz(
	px: number,
	py: number,
	pz: number,
	v0x: number,
	v0y: number,
	v0z: number,
	v1x: number,
	v1y: number,
	v1z: number,
	v2x: number,
	v2y: number,
	v2z: number
): (number, number, number)
	local dx, dy, dz = v0x - px, v0y - py, v0z - pz -- diff = V0 - point
	local e0x, e0y, e0z = v1x - v0x, v1y - v0y, v1z - v0z -- edge0 = V1 - V0
	local e1x, e1y, e1z = v2x - v0x, v2y - v0y, v2z - v0z -- edge1 = V2 - V0
	local a00 = e0x * e0x + e0y * e0y + e0z * e0z
	local a01 = e0x * e1x + e0y * e1y + e0z * e1z
	local a11 = e1x * e1x + e1y * e1y + e1z * e1z
	local b0 = dx * e0x + dy * e0y + dz * e0z
	local b1 = dx * e1x + dy * e1y + dz * e1z
	local c = dx * dx + dy * dy + dz * dz
	local det = abs(a00 * a11 - a01 * a01)
	local s = a01 * b1 - a11 * b0
	local t = a01 * b0 - a00 * b1
	local sqrDistance

	if s + t <= det then
		if s < 0 then
			if t < 0 then -- region 4
				if b0 < 0 then
					t = 0
					if -b0 >= a00 then
						s = 1
						sqrDistance = a00 + 2 * b0 + c
					else
						s = -b0 / a00
						sqrDistance = b0 * s + c
					end
				else
					s = 0
					if b1 >= 0 then
						t = 0
						sqrDistance = c
					elseif -b1 >= a11 then
						t = 1
						sqrDistance = a11 + 2 * b1 + c
					else
						t = -b1 / a11
						sqrDistance = b1 * t + c
					end
				end
			else -- region 3
				s = 0
				if b1 >= 0 then
					t = 0
					sqrDistance = c
				elseif -b1 >= a11 then
					t = 1
					sqrDistance = a11 + 2 * b1 + c
				else
					t = -b1 / a11
					sqrDistance = b1 * t + c
				end
			end
		elseif t < 0 then -- region 5
			t = 0
			if b0 >= 0 then
				s = 0
				sqrDistance = c
			elseif -b0 >= a00 then
				s = 1
				sqrDistance = a00 + 2 * b0 + c
			else
				s = -b0 / a00
				sqrDistance = b0 * s + c
			end
		else -- region 0
			-- minimum at interior point
			local invDet = 1 / det
			s *= invDet
			t *= invDet
			sqrDistance = s * (a00 * s + a01 * t + 2 * b0) + t * (a01 * s + a11 * t + 2 * b1) + c
		end
	else
		local tmp0, tmp1, numer, denom
		if s < 0 then -- region 2
			tmp0 = a01 + b0
			tmp1 = a11 + b1
			if tmp1 > tmp0 then
				numer = tmp1 - tmp0
				denom = a00 - 2 * a01 + a11
				if numer >= denom then
					s = 1
					t = 0
					sqrDistance = a00 + 2 * b0 + c
				else
					s = numer / denom
					t = 1 - s
					sqrDistance = s * (a00 * s + a01 * t + 2 * b0) + t * (a01 * s + a11 * t + 2 * b1) + c
				end
			else
				s = 0
				if tmp1 <= 0 then
					t = 1
					sqrDistance = a11 + 2 * b1 + c
				elseif b1 >= 0 then
					t = 0
					sqrDistance = c
				else
					t = -b1 / a11
					sqrDistance = b1 * t + c
				end
			end
		elseif t < 0 then -- region 6
			tmp0 = a01 + b1
			tmp1 = a00 + b0
			if tmp1 > tmp0 then
				numer = tmp1 - tmp0
				denom = a00 - 2 * a01 + a11
				if numer >= denom then
					t = 1
					s = 0
					sqrDistance = a11 + 2 * b1 + c
				else
					t = numer / denom
					s = 1 - t
					sqrDistance = s * (a00 * s + a01 * t + 2 * b0) + t * (a01 * s + a11 * t + 2 * b1) + c
				end
			else
				t = 0
				if tmp1 <= 0 then
					s = 1
					sqrDistance = a00 + 2 * b0 + c
				elseif b0 >= 0 then
					s = 0
					sqrDistance = c
				else
					s = -b0 / a00
					sqrDistance = b0 * s + c
				end
			end
		else -- region 1
			numer = a11 + b1 - a01 - b0
			if numer <= 0 then
				s = 0
				t = 1
				sqrDistance = a11 + 2 * b1 + c
			else
				denom = a00 - 2 * a01 + a11
				if numer >= denom then
					s = 1
					t = 0
					sqrDistance = a00 + 2 * b0 + c
				else
					s = numer / denom
					t = 1 - s
					sqrDistance = s * (a00 * s + a01 * t + 2 * b0) + t * (a01 * s + a11 * t + 2 * b1) + c
				end
			end
		end
	end
	return sqrDistance, s, t
end

-- allocation-free variant: returns clamped sqrDistance and the s,t parameters
function DistPoint3Triangle3.DistanceSqrXYZ(
	px: number,
	py: number,
	pz: number,
	v0x: number,
	v0y: number,
	v0z: number,
	v1x: number,
	v1y: number,
	v1z: number,
	v2x: number,
	v2y: number,
	v2z: number
): (number, number, number)
	local d, s, t = distance_sqr_xyz(px, py, pz, v0x, v0y, v0z, v1x, v1y, v1z, v2x, v2y, v2z)
	-- Account for numerical round-off error. (Math.Max(sqrDistance, 0))
	if d < 0 then
		d = 0
	end
	return d, s, t
end
DistPoint3Triangle3.distance_sqr_xyz = distance_sqr_xyz

-- C# static double DistanceSqr(ref point, ref triangle, out closestPoint, out baryCoords)
function DistPoint3Triangle3.DistanceSqr(point, triangle)
	local V0, V1, V2 = triangle.V0, triangle.V1, triangle.V2
	local v0x, v0y, v0z = V0.x, V0.y, V0.z
	local d, s, t = distance_sqr_xyz(point.x, point.y, point.z, v0x, v0y, v0z, V1.x, V1.y, V1.z, V2.x, V2.y, V2.z)
	local e0x, e0y, e0z = V1.x - v0x, V1.y - v0y, V1.z - v0z
	local e1x, e1y, e1z = V2.x - v0x, V2.y - v0y, V2.z - v0z
	-- closestPoint = triangle.V0 + s * edge0 + t * edge1
	local closestPoint = V3(v0x + s * e0x + t * e1x, v0y + s * e0y + t * e1y, v0z + s * e0z + t * e1z)
	local baryCoords = V3(1 - s - t, s, t)
	-- Account for numerical round-off error.
	if d < 0 then
		d = 0
	end
	return d, closestPoint, baryCoords
end

return DistPoint3Triangle3
