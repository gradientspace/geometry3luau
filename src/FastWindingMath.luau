--!native
-- Port of geometry3Sharp math/FastWindingMath.cs (static classes FastTriWinding and FastPointWinding)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Formulas for triangle / point-set winding number approximation (Barill et al 2018, "Fast Winding Numbers").
--
-- Module returns { FastTriWinding = ..., FastPointWinding = ... }.
--
-- Luau API notes (ref/out parameters become return values):
--   FastTriWinding.ComputeCoeffs(mesh, triangles, triCache?) -> p, r, order1, order2
--       triangles: 1-based array of triangle ids, or an iterator function
--       p/order1 are Vector3d, r a number, order2 a Matrix3d
--   FastTriWinding.ComputeCoeffsFlat(mesh, triList, nTris, triCache, out, off)
--       allocation-light variant used by DMeshAABBTree3: writes 16 numbers into out[off+1 .. off+16]:
--       center xyz, r, order1 xyz, order2 (row-major 3x3)
--   FastTriWinding.EvaluateOrder1Approx(center, order1Coeff, q)
--   FastTriWinding.EvaluateOrder2Approx(center, order1Coeff, order2Coeff, q)
--   FastTriWinding.EvaluateOrder2ApproxFlat(t, off, qx, qy, qz)       (reads ComputeCoeffsFlat layout)
--   FastTriWinding.EvaluateOrder1ApproxFlat(t, off, qx, qy, qz)
--   FastTriWinding.Order1Approx(t, p, xn, xA, q), FastTriWinding.Order2Approx(t, p, xn, xA, q)
--   FastPointWinding.ComputeCoeffs(pointSet, points, pointAreas) -> p, r, order1, order2
--       pointAreas is a 0-based-by-id table: pointAreas[vid + 1] (C# double[] pointAreas[vid])
--       pointSet must implement HasVertexNormals(), GetVertex(vid), GetVertexNormal(vid) (IPointSet)
--   FastPointWinding.EvaluateOrder1Approx / EvaluateOrder2Approx / ExactEval / Order1Approx / Order2Approx

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local Matrix3d = require(script.Parent.Matrix3d)

local sqrt, max = math.sqrt, math.max
local FourPI = MathUtil.FourPI
local Epsilon = MathUtil.Epsilon
local V3 = Vector3d.new

local function toList(items: any): ({ number }, number)
	if type(items) == "function" then
		local list = {}
		for v in items do
			table.insert(list, v)
		end
		return list, #list
	end
	return items, #items
end

---------------------------------------------------------------------------------------------------
local FastTriWinding = {}

-- precompute constant coefficients of triangle winding number approximation (flat output, see header)
function FastTriWinding.ComputeCoeffsFlat(mesh, triList: { number }, nTris: number, triCache, out: { number }, off: number)
	local px, py, pz = 0, 0, 0
	local o1x, o1y, o1z = 0, 0, 0
	local m00, m01, m02, m10, m11, m12, m20, m21, m22 = 0, 0, 0, 0, 0, 0, 0, 0, 0
	local r = 0

	local tris, verts = mesh.triangles.data, mesh.vertices.data

	-- compute area-weighted centroid of triangles, we use this as the expansion point
	local sum_area = 0
	if triCache ~= nil then
		local areas, centroids = triCache.Areas.data, triCache.Centroids.data
		for k = 1, nTris do
			local tid = triList[k]
			local area = areas[tid + 1]
			sum_area += area
			local c = centroids[tid + 1]
			px, py, pz = px + area * c.x, py + area * c.y, pz + area * c.z
		end
	else
		for k = 1, nTris do
			local tid = triList[k]
			local ti = 3 * tid
			local a, b, c = 3 * tris[ti + 1], 3 * tris[ti + 2], 3 * tris[ti + 3]
			local x0, y0, z0 = verts[a + 1], verts[a + 2], verts[a + 3]
			local x1, y1, z1 = verts[b + 1], verts[b + 2], verts[b + 3]
			local x2, y2, z2 = verts[c + 1], verts[c + 2], verts[c + 3]
			-- MathUtil.Area(P0, P1, P2) = 0.5 * (P1-P0).Cross(P2-P0).Length
			local e1x, e1y, e1z = x1 - x0, y1 - y0, z1 - z0
			local e2x, e2y, e2z = x2 - x0, y2 - y0, z2 - z0
			local cx, cy, cz = e1y * e2z - e1z * e2y, e1z * e2x - e1x * e2z, e1x * e2y - e1y * e2x
			local area = 0.5 * sqrt(cx * cx + cy * cy + cz * cz)
			sum_area += area
			-- p += area * ((P0 + P1 + P2) / 3.0)
			px += area * ((x0 + x1 + x2) / 3.0)
			py += area * ((y0 + y1 + y2) / 3.0)
			pz += area * ((z0 + z1 + z2) / 3.0)
		end
	end
	px, py, pz = px / sum_area, py / sum_area, pz / sum_area

	-- compute first and second-order coefficients of FWN taylor expansion, as well as
	-- 'radius' value r, which is max dist from any tri vertex to p
	local normals, areas, centroids
	if triCache ~= nil then
		normals, areas, centroids = triCache.Normals.data, triCache.Areas.data, triCache.Centroids.data
	end
	for k = 1, nTris do
		local tid = triList[k]
		local ti = 3 * tid
		local ia, ib, ic = 3 * tris[ti + 1], 3 * tris[ti + 2], 3 * tris[ti + 3]
		local x0, y0, z0 = verts[ia + 1], verts[ia + 2], verts[ia + 3]
		local x1, y1, z1 = verts[ib + 1], verts[ib + 2], verts[ib + 3]
		local x2, y2, z2 = verts[ic + 1], verts[ic + 2], verts[ic + 3]

		local nx, ny, nz, a, cx, cy, cz
		if triCache == nil then
			-- c = (1.0 / 3.0) * (P0 + P1 + P2)
			local f = 1.0 / 3.0
			cx, cy, cz = f * (x0 + x1 + x2), f * (y0 + y1 + y2), f * (z0 + z1 + z2)
			-- n = MathUtil.FastNormalArea(ref P0, ref P1, ref P2, out a)
			local e1x, e1y, e1z = x1 - x0, y1 - y0, z1 - z0
			local e2x, e2y, e2z = x2 - x0, y2 - y0, z2 - z0
			nx, ny, nz = e1y * e2z - e1z * e2y, e1z * e2x - e1x * e2z, e1x * e2y - e1y * e2x
			local len = sqrt(nx * nx + ny * ny + nz * nz)
			if len > Epsilon then
				local invLen = 1.0 / len
				nx, ny, nz = nx * invLen, ny * invLen, nz * invLen
			else
				nx, ny, nz, len = 0, 0, 0, 0
			end
			a = 0.5 * len
		else
			local n, c = normals[tid + 1], centroids[tid + 1]
			nx, ny, nz = n.x, n.y, n.z
			a = areas[tid + 1]
			cx, cy, cz = c.x, c.y, c.z
		end

		-- order1 += a * n
		o1x, o1y, o1z = o1x + a * nx, o1y + a * ny, o1z + a * nz

		-- order2 += a * new Matrix3d(ref dcp, ref n)   (outer product dcp * n^T)
		local dx, dy, dz = cx - px, cy - py, cz - pz
		m00 += a * (dx * nx)
		m01 += a * (dx * ny)
		m02 += a * (dx * nz)
		m10 += a * (dy * nx)
		m11 += a * (dy * ny)
		m12 += a * (dy * nz)
		m20 += a * (dz * nx)
		m21 += a * (dz * ny)
		m22 += a * (dz * nz)

		-- this is just for return value...
		local d0 = (px - x0) * (px - x0) + (py - y0) * (py - y0) + (pz - z0) * (pz - z0)
		local d1 = (px - x1) * (px - x1) + (py - y1) * (py - y1) + (pz - z1) * (pz - z1)
		local d2 = (px - x2) * (px - x2) + (py - y2) * (py - y2) + (pz - z2) * (pz - z2)
		local maxdist = max(d0, max(d1, d2))
		r = max(r, sqrt(maxdist))
	end

	out[off + 1], out[off + 2], out[off + 3], out[off + 4] = px, py, pz, r
	out[off + 5], out[off + 6], out[off + 7] = o1x, o1y, o1z
	out[off + 8], out[off + 9], out[off + 10] = m00, m01, m02
	out[off + 11], out[off + 12], out[off + 13] = m10, m11, m12
	out[off + 14], out[off + 15], out[off + 16] = m20, m21, m22
end

-- precompute constant coefficients of triangle winding number approximation
-- p: 'center' of expansion for triangles (area-weighted centroid avg)
-- r: max distance from p to triangles
-- order1: first-order vector coeff
-- order2: second-order matrix coeff
-- triCache: optional precomputed triangle centroid/normal/area
-- C# ComputeCoeffs(mesh, triangles, ref p, ref r, ref order1, ref order2, triCache) -> returns p, r, order1, order2
function FastTriWinding.ComputeCoeffs(mesh, triangles: any, triCache: any?)
	local list, n = toList(triangles)
	local out = table.create(16, 0)
	FastTriWinding.ComputeCoeffsFlat(mesh, list, n, triCache, out, 0)
	local p = V3(out[1], out[2], out[3])
	local order1 = V3(out[5], out[6], out[7])
	local order2 = Matrix3d.new(out[8], out[9], out[10], out[11], out[12], out[13], out[14], out[15], out[16])
	return p, out[4], order1, order2
end

-- Evaluate first-order FWN approximation at point q, relative to center c
function FastTriWinding.EvaluateOrder1Approx(center, order1Coeff, q): number
	local dpq = center - q
	local len = dpq:Length()
	return (1.0 / FourPI) * order1Coeff:Dot(dpq / (len * len * len))
end

-- Evaluate second-order FWN approximation at point q, relative to center c
function FastTriWinding.EvaluateOrder2Approx(center, order1Coeff, order2Coeff, q): number
	local dx, dy, dz = center.x - q.x, center.y - q.y, center.z - q.z
	local len = sqrt(dx * dx + dy * dy + dz * dz)
	local len3 = len * len * len
	local fourPi_len3 = 1.0 / (FourPI * len3)

	local order1 = fourPi_len3 * (order1Coeff.x * dx + order1Coeff.y * dy + order1Coeff.z * dz)

	-- second-order hessian \grad^2(G)
	local c = -3.0 / (FourPI * len3 * len * len)

	-- expanded-out version below avoids extra constructors
	local r0, r1, r2 = order2Coeff.Row0, order2Coeff.Row1, order2Coeff.Row2
	local order2 = (r0.x * (fourPi_len3 + c * dx * dx) + r0.y * (c * dx * dy) + r0.z * (c * dx * dz))
		+ (r1.x * (c * dy * dx) + r1.y * (fourPi_len3 + c * dy * dy) + r1.z * (c * dy * dz))
		+ (r2.x * (c * dz * dx) + r2.y * (c * dz * dy) + r2.z * (fourPi_len3 + c * dz * dz))

	return order1 + order2
end

-- EvaluateOrder2Approx reading coefficients from flat layout (see ComputeCoeffsFlat)
local function EvaluateOrder2ApproxFlat(t: { number }, off: number, qx: number, qy: number, qz: number): number
	local dx, dy, dz = t[off + 1] - qx, t[off + 2] - qy, t[off + 3] - qz
	local len = sqrt(dx * dx + dy * dy + dz * dz)
	local len3 = len * len * len
	local fourPi_len3 = 1.0 / (FourPI * len3)
	local order1 = fourPi_len3 * (t[off + 5] * dx + t[off + 6] * dy + t[off + 7] * dz)
	local c = -3.0 / (FourPI * len3 * len * len)
	local order2 = (t[off + 8] * (fourPi_len3 + c * dx * dx) + t[off + 9] * (c * dx * dy) + t[off + 10] * (c * dx * dz))
		+ (t[off + 11] * (c * dy * dx) + t[off + 12] * (fourPi_len3 + c * dy * dy) + t[off + 13] * (c * dy * dz))
		+ (t[off + 14] * (c * dz * dx) + t[off + 15] * (c * dz * dy) + t[off + 16] * (fourPi_len3 + c * dz * dz))
	return order1 + order2
end
FastTriWinding.EvaluateOrder2ApproxFlat = EvaluateOrder2ApproxFlat

local function EvaluateOrder1ApproxFlat(t: { number }, off: number, qx: number, qy: number, qz: number): number
	local dx, dy, dz = t[off + 1] - qx, t[off + 2] - qy, t[off + 3] - qz
	local len = sqrt(dx * dx + dy * dy + dz * dz)
	local l3 = len * len * len
	return (1.0 / FourPI) * (t[off + 5] * (dx / l3) + t[off + 6] * (dy / l3) + t[off + 7] * (dz / l3))
end
FastTriWinding.EvaluateOrder1ApproxFlat = EvaluateOrder1ApproxFlat

-- triangle-winding-number first-order approximation.
-- t is triangle, p is 'center' of cluster of dipoles, q is evaluation point
-- (This is really just for testing)
function FastTriWinding.Order1Approx(t, p, xn, xA: number, q): number
	local at0 = xA * xn
	local dpq = p - q
	local len = dpq:Length()
	return (1.0 / FourPI) * at0:Dot(dpq / (len * len * len))
end

-- triangle-winding-number second-order approximation
-- t is triangle, p is 'center' of cluster of dipoles, q is evaluation point
-- (This is really just for testing)
function FastTriWinding.Order2Approx(t, p, xn, xA: number, q): number
	local dpq = p - q
	local len = dpq:Length()
	local len3 = len * len * len

	-- first-order approximation - integrated_normal_area * \grad(G)
	local order1 = (xA / FourPI) * xn:Dot(dpq / len3)

	-- second-order hessian \grad^2(G)
	local xqxq = Matrix3d.OuterProduct(dpq, dpq)
	xqxq = xqxq * (3.0 / (FourPI * len3 * len * len))
	local diag = 1 / (FourPI * len3)
	local hessian = Matrix3d.Diagonal(diag, diag, diag) - xqxq

	-- second-order LHS - integrated second-order area matrix (formula 26)
	local centroid = V3((t.V0.x + t.V1.x + t.V2.x) / 3.0, (t.V0.y + t.V1.y + t.V2.y) / 3.0, (t.V0.z + t.V1.z + t.V2.z) / 3.0)
	local dcp = centroid - p
	local o2_lhs = Matrix3d.OuterProduct(dcp, xn)
	local order2 = xA * o2_lhs:InnerProduct(hessian)

	return order1 + order2
end

---------------------------------------------------------------------------------------------------
local FastPointWinding = {}

-- precompute constant coefficients of point winding number approximation
-- pointAreas must be provided, and pointSet must have vertex normals!
-- C# ComputeCoeffs(pointSet, points, pointAreas, ref p, ref r, ref order1, ref order2) -> returns p, r, order1, order2
function FastPointWinding.ComputeCoeffs(pointSet, points: any, pointAreas: { number })
	if pointSet:HasVertexNormals() == false then
		error("FastPointWinding.ComputeCoeffs: point set does not have normals!")
	end
	local list, n = toList(points)

	local p = Vector3d.Zero
	local order1 = Vector3d.Zero
	local order2 = Matrix3d.Zero
	local r = 0

	-- compute area-weighted centroid of points, we use this as the expansion point
	local sum_area = 0
	for k = 1, n do
		local vid = list[k]
		sum_area += pointAreas[vid + 1]
		p = p + pointAreas[vid + 1] * pointSet:GetVertex(vid)
	end
	p = p / sum_area

	-- compute first and second-order coefficients of FWN taylor expansion, as well as
	-- 'radius' value r, which is max dist from any tri vertex to p
	for k = 1, n do
		local vid = list[k]
		local p_i = pointSet:GetVertex(vid)
		local n_i = pointSet:GetVertexNormal(vid)
		local a_i = pointAreas[vid + 1]

		order1 = order1 + a_i * n_i

		local dcp = p_i - p
		order2 = order2 + a_i * Matrix3d.OuterProduct(dcp, n_i)

		-- this is just for return value...
		r = max(r, p_i:Distance(p))
	end
	return p, r, order1, order2
end

-- Evaluate first-order FWN approximation at point q, relative to center c
FastPointWinding.EvaluateOrder1Approx = FastTriWinding.EvaluateOrder1Approx

-- Evaluate second-order FWN approximation at point q, relative to center c
FastPointWinding.EvaluateOrder2Approx = FastTriWinding.EvaluateOrder2Approx

function FastPointWinding.ExactEval(x, xn, xA: number, q): number
	local dv = x - q
	local len = dv:Length()
	return (xA / FourPI) * xn:Dot(dv / (len * len * len))
end

-- point-winding-number first-order approximation.
-- x is dipole point, p is 'center' of cluster of dipoles, q is evaluation point
function FastPointWinding.Order1Approx(x, p, xn, xA: number, q): number
	local dpq = p - q
	local len = dpq:Length()
	return (xA / FourPI) * xn:Dot(dpq / (len * len * len))
end

-- point-winding-number second-order approximation
-- x is dipole point, p is 'center' of cluster of dipoles, q is evaluation point
function FastPointWinding.Order2Approx(x, p, xn, xA: number, q): number
	local dpq = p - q
	local dxp = x - p

	local len = dpq:Length()
	local len3 = len * len * len

	-- first-order approximation - area*normal*\grad(G)
	local order1 = (xA / FourPI) * xn:Dot(dpq / len3)

	-- second-order hessian \grad^2(G)
	local xqxq = Matrix3d.OuterProduct(dpq, dpq)
	xqxq = xqxq * (3.0 / (FourPI * len3 * len * len))
	local diag = 1 / (FourPI * len3)
	local hessian = Matrix3d.Diagonal(diag, diag, diag) - xqxq

	-- second-order LHS area * \outer(x-p, normal)
	local o2_lhs = Matrix3d.OuterProduct(dxp, xn)
	local order2 = xA * o2_lhs:InnerProduct(hessian)

	return order1 + order2
end

return {
	FastTriWinding = FastTriWinding,
	FastPointWinding = FastPointWinding,
}
