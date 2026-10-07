--!native
-- Port of geometry3Sharp mesh/MeshUtil.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau port notes (see PORTING.md):
--   * static functions: MeshUtil.UniformSmooth(mesh, vid, t), ...
--   * GetEdgeFlipTris(mesh, eID) -> orig_t0, orig_t1, flip_t0, flip_t1   (Index3i arrays, C# out params)
--   * GetEdgeFlipNormals(mesh, eID) -> n1, n2, on1, on2                   (C# out params)
--   * ExtractLoopV(mesh, vertices) accepts a 1-based Luau array of vertex ids (C# int[]) or an
--     iterator function (C# IEnumerable<int>).

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local IndexUtil = require(script.Parent.IndexUtil)
local MeshWeights = require(script.Parent.MeshWeights)
local Util = require(script.Parent.Util)

local V3 = Vector3d.new

local MeshUtil = {}

-- t in range [0,1]
function MeshUtil.UniformSmooth(mesh, vID: number, t: number)
	local d = mesh.vertices.data
	local k = 3 * vID
	local vx, vy, vz = d[k + 1], d[k + 2], d[k + 3]
	local c = mesh:VtxOneRingCentroid(vID)
	local s = 1.0 - t
	return V3(s * vx + t * c.x, s * vy + t * c.y, s * vz + t * c.z)
end

-- t in range [0,1]
function MeshUtil.MeanValueSmooth(mesh, vID: number, t: number)
	local v = mesh:GetVertex(vID)
	local c = MeshWeights.MeanValueCentroid(mesh, vID)
	return (1 - t) * v + t * c
end

-- t in range [0,1]
function MeshUtil.CotanSmooth(mesh, vID: number, t: number)
	local v = mesh:GetVertex(vID)
	local c = MeshWeights.CotanCentroid(mesh, vID)
	return (1 - t) * v + t * c
end

function MeshUtil.ScaleMesh(mesh, f, vScale)
	for vid in mesh:VertexIndices() do
		local v = mesh:GetVertex(vid)
		local vScaledInF = f:ToFrameP(v) * vScale
		local vNew = f:FromFrameP(vScaledInF)
		mesh:SetVertex(vid, vNew)
		-- TODO: normals
	end
end

-- computes opening angle between the two triangles connected to edge
function MeshUtil.OpeningAngleD(mesh, eid: number): number
	local et = mesh:GetEdgeT(eid)
	if et[2] == -1 then
		return MathUtil.MaxDouble -- boundary edge!!
	end
	local n0 = mesh:GetTriNormal(et[1])
	local n1 = mesh:GetTriNormal(et[2])
	return Vector3d.AngleD(n0, n1)
end

-- computes sum of opening-angles in triangles around vid, minus 2pi.
-- This is zero on flat areas.
function MeshUtil.DiscreteGaussCurvature(mesh, vid: number): number
	local angle_sum = 0
	for tid in mesh:VtxTrianglesItr(vid) do
		local et = mesh:GetTriangle(tid)
		local idx = IndexUtil.find_tri_index(vid, et)
		angle_sum += mesh:GetTriInternalAngleR(tid, idx)
	end
	return angle_sum - MathUtil.TwoPI
end

-- Check if collapsing edge edgeID to point newv will flip normal of any attached face
function MeshUtil.CheckIfCollapseCreatesFlip(mesh, edgeID: number, newv): boolean
	local edge_info = mesh:GetEdge(edgeID)
	local tc, td = edge_info[3], edge_info[4]

	for j = 0, 1 do
		local vid = edge_info[j + 1]
		local vother = edge_info[((j + 1) % 2) + 1]

		for tid in mesh:VtxTrianglesItr(vid) do
			if tid ~= tc and tid ~= td then
				local curt = mesh:GetTriangle(tid)
				if curt[1] == vother or curt[2] == vother or curt[3] == vother then
					return true -- invalid nbrhood for collapse
				end
				local va = mesh:GetVertex(curt[1])
				local vb = mesh:GetVertex(curt[2])
				local vc = mesh:GetVertex(curt[3])
				local ncur = (vb - va):Cross(vc - va)
				local sign = 0
				if curt[1] == vid then
					local nnew = (vb - newv):Cross(vc - newv)
					sign = ncur:Dot(nnew)
				elseif curt[2] == vid then
					local nnew = (newv - va):Cross(vc - va)
					sign = ncur:Dot(nnew)
				elseif curt[3] == vid then
					local nnew = (vb - va):Cross(newv - va)
					sign = ncur:Dot(nnew)
				else
					error("MeshUtil.CheckIfCollapseCreatesFlip: should never be here!")
				end
				if sign <= 0.0 then
					return true
				end
			end
		end
	end
	return false
end

local function edge_flip_metric(n0, n1, flip_dot_tol: number): number
	if flip_dot_tol == 0 then
		return n0:Dot(n1)
	end
	return n0:Normalized():Dot(n1:Normalized())
end

-- if before a flip we have normals (n1,n2) and after we have (m1,m2), check if
-- the dot between any of the 4 pairs changes sign after the flip, or is
-- less than the dot-product tolerance (ie angle tolerance)
function MeshUtil.CheckIfEdgeFlipCreatesFlip(mesh, eID: number, flip_dot_tol: number?): boolean
	local tol = flip_dot_tol or 0.0
	Util.gDevAssert(mesh:IsBoundaryEdge(eID) == false)
	local einfo = mesh:GetEdge(eID)
	local ov = mesh:GetEdgeOpposingV(eID)

	local a, b, c, d = einfo[1], einfo[2], ov[1], ov[2]
	local t0 = einfo[3]

	local vC, vD = mesh:GetVertex(c), mesh:GetVertex(d)
	local tri_v = mesh:GetTriangle(t0)
	local _, oa, ob = IndexUtil.orient_tri_edge(a, b, tri_v)
	local vOA, vOB = mesh:GetVertex(oa), mesh:GetVertex(ob)
	local n0 = MathUtil.FastNormalDirection(vOA, vOB, vC)
	local n1 = MathUtil.FastNormalDirection(vOB, vOA, vD)
	local f0 = MathUtil.FastNormalDirection(vC, vD, vOB)
	if edge_flip_metric(n0, f0, tol) <= tol or edge_flip_metric(n1, f0, tol) <= tol then
		return true
	end
	local f1 = MathUtil.FastNormalDirection(vD, vC, vOA)
	if edge_flip_metric(n0, f1, tol) <= tol or edge_flip_metric(n1, f1, tol) <= tol then
		return true
	end
	return false
end

-- For given edge, return it's triangles and the triangles that would
-- be created if it was flipped (used in edge-flip optimizers)
-- C# GetEdgeFlipTris(mesh, eID, out orig_t0, out orig_t1, out flip_t0, out flip_t1)
function MeshUtil.GetEdgeFlipTris(mesh, eID: number)
	local einfo = mesh:GetEdge(eID)
	local ov = mesh:GetEdgeOpposingV(eID)
	local a, b, c, d = einfo[1], einfo[2], ov[1], ov[2]
	local t0 = einfo[3]
	local tri_v = mesh:GetTriangle(t0)
	local _, oa, ob = IndexUtil.orient_tri_edge(a, b, tri_v)
	return { oa, ob, c }, { ob, oa, d }, { c, d, ob }, { d, c, oa }
end

-- For given edge, return normals of it's two triangles, and normals
-- of the triangles created if edge is flipped (used in edge-flip optimizers)
-- C# GetEdgeFlipNormals(mesh, eID, out n1, out n2, out on1, out on2)
function MeshUtil.GetEdgeFlipNormals(mesh, eID: number)
	local einfo = mesh:GetEdge(eID)
	local ov = mesh:GetEdgeOpposingV(eID)
	local a, b, c, d = einfo[1], einfo[2], ov[1], ov[2]
	local t0 = einfo[3]
	local vC, vD = mesh:GetVertex(c), mesh:GetVertex(d)
	local tri_v = mesh:GetTriangle(t0)
	local _, oa, ob = IndexUtil.orient_tri_edge(a, b, tri_v)
	local vOA, vOB = mesh:GetVertex(oa), mesh:GetVertex(ob)
	local n1 = MathUtil.Normal(vOA, vOB, vC)
	local n2 = MathUtil.Normal(vOB, vOA, vD)
	local on1 = MathUtil.Normal(vC, vD, vOB)
	local on2 = MathUtil.Normal(vD, vC, vOA)
	return n1, n2, on1, on2
end

-- ExtractLoopV(IMesh mesh, IEnumerable<int> vertices) / ExtractLoopV(IMesh mesh, int[] vertices)
function MeshUtil.ExtractLoopV(mesh, vertices: any)
	local DCurve3 = require(script.Parent.DCurve3)
	local curve = DCurve3.new()
	if type(vertices) == "function" then
		for vid in vertices do
			curve:AppendVertex(mesh:GetVertex(vid))
		end
	else
		for i = 1, #vertices do
			curve:AppendVertex(mesh:GetVertex(vertices[i]))
		end
	end
	curve.Closed = true
	return curve
end

return MeshUtil
