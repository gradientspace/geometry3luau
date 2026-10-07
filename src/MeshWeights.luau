--!native
-- Port of geometry3Sharp mesh/MeshWeights.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau port notes: static functions, called as MeshWeights.CotanCentroid(mesh, vid) etc.
-- The centroid functions are used per-vertex in Remesher smoothing passes, so they read the mesh
-- buffers directly (same arithmetic as the C# vector expressions).

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)

local V3 = Vector3d.new
local sqrt, abs = math.sqrt, math.abs
local Epsilon = MathUtil.Epsilon
local ZeroTolerance = MathUtil.ZeroTolerance
local InvalidID = -1

local MeshWeights = {}

function MeshWeights.OneRingCentroid(mesh, vID: number)
	local sx, sy, sz = 0, 0, 0
	local nCount = 0
	local d = mesh.vertices.data
	for nbr in mesh:VtxVerticesItr(vID) do
		local k = 3 * nbr
		sx += d[k + 1]
		sy += d[k + 2]
		sz += d[k + 3]
		nCount += 1
	end
	if nCount == 0 then
		return mesh:GetVertex(vID)
	end
	local m = 1.0 / nCount
	return V3(sx * m, sy * m, sz * m)
end

-- normalize (x,y,z) like Vector3d.Normalized
local function normalize(x: number, y: number, z: number): (number, number, number)
	local len = sqrt(x * x + y * y + z * z)
	if len > Epsilon then
		local s = 1.0 / len
		return x * s, y * s, z * s
	end
	return 0, 0, 0
end

-- MathUtil.VectorCot on scalar vectors
local function vector_cot(ax: number, ay: number, az: number, bx: number, by: number, bz: number): number
	local fDot = ax * bx + ay * by + az * bz
	local lensqr1 = ax * ax + ay * ay + az * az
	local lensqr2 = bx * bx + by * by + bz * bz
	local d = lensqr1 * lensqr2 - fDot * fDot
	if d < 0 then
		d = 0
	end
	if d < ZeroTolerance then
		return 0
	end
	return fDot / sqrt(d)
end

-- Compute cotan-weighted neighbour sum around a vertex.
-- These weights are numerically unstable if any of the triangles are degenerate.
-- We catch these problems and return input vertex as centroid
-- http://www.geometry.caltech.edu/pubs/DMSB_III.pdf
function MeshWeights.CotanCentroid(mesh, v_i: number)
	local sx, sy, sz = 0, 0, 0
	local wSum = 0
	local d = mesh.vertices.data
	local ki = 3 * v_i
	local vix, viy, viz = d[ki + 1], d[ki + 2], d[ki + 3]

	local bAborted = false
	for eid in mesh:VtxEdgesItr(v_i) do
		local v_j, opp_v1, opp_v2 = mesh:GetVtxNbrhood(eid, v_i)
		local kj = 3 * v_j
		local vjx, vjy, vjz = d[kj + 1], d[kj + 2], d[kj + 3]

		local k1 = 3 * opp_v1
		local o1x, o1y, o1z = d[k1 + 1], d[k1 + 2], d[k1 + 3]
		local ax, ay, az = normalize(vix - o1x, viy - o1y, viz - o1z)
		local bx, by, bz = normalize(vjx - o1x, vjy - o1y, vjz - o1z)
		local cot_alpha_ij = vector_cot(ax, ay, az, bx, by, bz)
		if cot_alpha_ij == 0 then
			bAborted = true
			break
		end
		local w_ij = cot_alpha_ij

		if opp_v2 ~= InvalidID then
			local k2 = 3 * opp_v2
			local o2x, o2y, o2z = d[k2 + 1], d[k2 + 2], d[k2 + 3]
			local cx, cy, cz = normalize(vix - o2x, viy - o2y, viz - o2z)
			local ex, ey, ez = normalize(vjx - o2x, vjy - o2y, vjz - o2z)
			local cot_beta_ij = vector_cot(cx, cy, cz, ex, ey, ez)
			if cot_beta_ij == 0 then
				bAborted = true
				break
			end
			w_ij += cot_beta_ij
		end

		sx += w_ij * vjx
		sy += w_ij * vjy
		sz += w_ij * vjz
		wSum += w_ij
	end
	if bAborted or abs(wSum) < ZeroTolerance then
		return V3(vix, viy, viz)
	end
	return V3(sx / wSum, sy / wSum, sz / wSum)
end

-- from http://www.geometry.caltech.edu/pubs/DMSB_III.pdf
function MeshWeights.VoronoiArea(mesh, v_i: number): number
	local areaSum = 0
	local Vi = mesh:GetVertex(v_i)

	for tid in mesh:VtxTrianglesItr(v_i) do
		local t = mesh:GetTriangle(tid)
		local ti = (t[1] == v_i) and 0 or ((t[2] == v_i) and 1 or 2)
		local Vj = mesh:GetVertex(t[((ti + 1) % 3) + 1])
		local Vk = mesh:GetVertex(t[((ti + 2) % 3) + 1])

		if MathUtil.IsObtuse(Vi, Vj, Vk) then
			local Vij = (Vj - Vi):Normalized()
			local Vik = (Vk - Vi):Normalized()
			local areaT = 0.5 * Vij:Cross(Vik):Length()
			if Vector3d.AngleR(Vij, Vik) > MathUtil.HalfPI then
				areaSum += (areaT * 0.5) -- obtuse at v_i
			else
				areaSum += (areaT * 0.25) -- not obtuse
			end
		else
			-- voronoi area
			local Vji, dist_ji = (Vi - Vj):NormalizedAndLength()
			local Vki, dist_ki = (Vi - Vk):NormalizedAndLength()
			local Vkj = (Vj - Vk):Normalized()

			local cot_alpha_ij = MathUtil.VectorCot(Vki, Vkj)
			local cot_alpha_ik = MathUtil.VectorCot(Vji, -Vkj)
			areaSum += dist_ji * dist_ji * cot_alpha_ij * 0.125
			areaSum += dist_ki * dist_ki * cot_alpha_ik * 0.125
		end
	end
	return areaSum
end

-- tan(theta/2) = +/- sqrt( (1-cos(theta)) / (1+cos(theta)) )
-- (in context above we never want negative value!)
function MeshWeights.VectorTanHalfAngle(a, b): number
	local cosAngle = a:Dot(b)
	local sqr = (1 - cosAngle) / (1 + cosAngle)
	sqr = MathUtil.Clamp(sqr, 0, MathUtil.MaxDouble)
	return sqrt(sqr)
end

local function tan_half_angle(ax: number, ay: number, az: number, bx: number, by: number, bz: number): number
	local cosAngle = ax * bx + ay * by + az * bz
	local sqr = (1 - cosAngle) / (1 + cosAngle)
	-- MathUtil.Clamp(sqr, 0, double.MaxValue)
	if sqr < 0 then
		sqr = 0
	elseif sqr > MathUtil.MaxDouble then
		sqr = MathUtil.MaxDouble
	end
	return sqrt(sqr)
end

-- http://128.148.32.110/courses/cs224/papers/mean_value.pdf
function MeshWeights.MeanValueCentroid(mesh, v_i: number)
	local sx, sy, sz = 0, 0, 0
	local wSum = 0
	local d = mesh.vertices.data
	local ki = 3 * v_i
	local vix, viy, viz = d[ki + 1], d[ki + 2], d[ki + 3]

	for eid in mesh:VtxEdgesItr(v_i) do
		local v_j, opp_v1, opp_v2 = mesh:GetVtxNbrhood(eid, v_i)

		local kj = 3 * v_j
		local vjx, vjy, vjz = d[kj + 1], d[kj + 2], d[kj + 3]
		local dx, dy, dz = vjx - vix, vjy - viy, vjz - viz
		local len_vVj = sqrt(dx * dx + dy * dy + dz * dz)
		-- vVj.Normalize()
		if len_vVj > Epsilon then
			local s = 1.0 / len_vVj
			dx, dy, dz = dx * s, dy * s, dz * s
		else
			dx, dy, dz, len_vVj = 0, 0, 0, 0
		end
		-- [RMS] is this the right thing to do? if vertices are coincident,
		--   weight of this vertex should be very high!
		if len_vVj >= ZeroTolerance then
			local k1 = 3 * opp_v1
			local ex, ey, ez = normalize(d[k1 + 1] - vix, d[k1 + 2] - viy, d[k1 + 3] - viz)
			local w_ij = tan_half_angle(dx, dy, dz, ex, ey, ez)

			if opp_v2 ~= InvalidID then
				local k2 = 3 * opp_v2
				local gx, gy, gz = normalize(d[k2 + 1] - vix, d[k2 + 2] - viy, d[k2 + 3] - viz)
				w_ij += tan_half_angle(dx, dy, dz, gx, gy, gz)
			end

			w_ij /= len_vVj

			sx += w_ij * vjx
			sy += w_ij * vjy
			sz += w_ij * vjz
			wSum += w_ij
		end
	end
	if wSum < ZeroTolerance then
		return V3(vix, viy, viz)
	end
	return V3(sx / wSum, sy / wSum, sz / wSum)
end

return MeshWeights
