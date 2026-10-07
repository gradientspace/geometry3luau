--!native
-- Port of geometry3Sharp mesh/MeshRefinerBase.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Base class of Remesher and Reducer (metatable inheritance, see PORTING.md).
--
-- Luau port notes:
--   * MeshRefinerBase.new(mesh); subclasses call MeshRefinerBase.init(self, mesh).
--   * protected fields keep their C# names: self.mesh, self.constraints, self.edge_flip_tol.
--   * properties: Mesh(), Constraints(), EdgeFlipTolerance() / SetEdgeFlipTolerance(value).
--   * Progress is a ProgressCancel (or nil). Cancelled() polls it.
--   * C# out params become extra return values:
--       can_collapse_constraints(eid, a, b, c, d, tc, td) -> ok, collapse_to
--       can_collapse_vtx(eid, a, b) -> ok, collapse_to
--       get_vertex_constraint(vid) -> VertexConstraint (Unconstrained if none)
--       try_get_vertex_constraint(vid) -> found, vc   (C# bool get_vertex_constraint(vid, ref vc))
--   * edge_flip_metric(n0, n1) (instance) and MeshRefinerBase.edge_flip_metric_tol(n0, n1, tol) (static).
--   * collapse_creates_flip_or_invalid(vid, vother, newv, tc, td) (instance) and the static
--     MeshRefinerBase.collapse_creates_flip_or_invalid_mesh(mesh, vid, vother, newv, tc, td, tol?)
--     The scalar variant collapse_creates_flip_or_invalid_xyz(vid, vother, nx, ny, nz, tc, td) avoids
--     allocating the new position (used by Reducer/Remesher hot paths); results are identical.

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local IndexUtil = require(script.Parent.IndexUtil)
local MeshConstraints = require(script.Parent.MeshConstraints)

local EdgeConstraint = MeshConstraints.EdgeConstraint
local VertexConstraint = MeshConstraints.VertexConstraint
local VertexUnconstrained = VertexConstraint.Unconstrained

local sqrt = math.sqrt
local Epsilon = MathUtil.Epsilon
local InvalidID = -1

local MeshRefinerBase = {}
MeshRefinerBase.__index = MeshRefinerBase
MeshRefinerBase.ClassName = "MeshRefinerBase"

function MeshRefinerBase.init(self, mesh)
	self.mesh = mesh
	self.constraints = nil
	-- if true, then when two Fixed vertices have the same non-invalid SetID,
	-- we treat them as not fixed and allow collapse
	self.AllowCollapseFixedVertsWithSameSetID = true
	-- If normals dot product is less than this, we consider it a normal flip. default = 0
	self.edge_flip_tol = 0.0
	-- Set this to be able to cancel running remesher
	self.Progress = nil
	return self
end

function MeshRefinerBase.new(mesh)
	return MeshRefinerBase.init(setmetatable({}, MeshRefinerBase), mesh)
end

function MeshRefinerBase:EdgeFlipTolerance(): number
	return self.edge_flip_tol
end
function MeshRefinerBase:SetEdgeFlipTolerance(value: number)
	self.edge_flip_tol = MathUtil.Clamp(value, -1.0, 1.0)
end

function MeshRefinerBase:Mesh()
	return self.mesh
end

function MeshRefinerBase:Constraints()
	return self.constraints
end

--! This object will be modified !!!
function MeshRefinerBase:SetExternalConstraints(cons)
	self.constraints = cons
end

-- if this returns true, abort computation.
function MeshRefinerBase:Cancelled(): boolean
	local p = self.Progress
	if p == nil then
		return false
	end
	return p:Cancelled()
end

local function normalized_dot(ax, ay, az, bx, by, bz): number
	local la = sqrt(ax * ax + ay * ay + az * az)
	if la > Epsilon then
		local s = 1.0 / la
		ax, ay, az = ax * s, ay * s, az * s
	else
		ax, ay, az = 0, 0, 0
	end
	local lb = sqrt(bx * bx + by * by + bz * bz)
	if lb > Epsilon then
		local s = 1.0 / lb
		bx, by, bz = bx * s, by * s, bz * s
	else
		bx, by, bz = 0, 0, 0
	end
	return ax * bx + ay * by + az * bz
end

-- scalar edge_flip_metric(n0, n1, tol)
local function flip_metric_xyz(ax, ay, az, bx, by, bz, tol: number): number
	if tol == 0 then
		return ax * bx + ay * by + az * bz
	end
	return normalized_dot(ax, ay, az, bx, by, bz)
end
MeshRefinerBase.flip_metric_xyz = flip_metric_xyz

local SmallListSet = require(script.Parent.SmallListSet)
local SLS_Null = -1
local SLS_BLOCKSIZE = SmallListSet.BLOCKSIZE
local SLS_BLOCK_LIST_OFFSET = SmallListSet.BLOCK_LIST_OFFSET

-- Fill buf[1..n] with the one-ring triangles of vertex vid, in exactly the order of
-- mesh:VtxTrianglesItr(vid), without allocating. Returns n. (vid must be a valid vertex)
local function gather_vtx_triangles(mesh, vid: number, buf: { number }): number
	local sls = mesh.vertex_edges
	local block_ptr = sls.list_heads[vid + 1]
	if block_ptr == nil or block_ptr == SLS_Null then
		return 0
	end
	local ed = mesh.edges.data
	local t = mesh.triangles.data
	local bs = sls.block_store
	local N = bs[block_ptr + 1]
	local n = 0
	local iEnd = block_ptr + ((N < SLS_BLOCKSIZE) and N or SLS_BLOCKSIZE)
	local i = block_ptr + 2
	local cur_ptr = (N > SLS_BLOCKSIZE) and bs[block_ptr + SLS_BLOCK_LIST_OFFSET + 1] or SLS_Null
	local ls = sls.linked_store
	while true do
		local eid
		if i <= iEnd + 1 then
			eid = bs[i]
			i += 1
		elseif cur_ptr ~= SLS_Null then
			eid = ls[cur_ptr + 1]
			cur_ptr = ls[cur_ptr + 2]
		else
			break
		end
		local ei = 4 * eid
		local ev0 = ed[ei + 1]
		local vOther = (ev0 == vid) and ed[ei + 2] or ev0
		-- tri_has_sequential_v(et0, vid, vOther)
		local et0 = ed[ei + 3]
		local ti = 3 * et0
		local v0, v1, v2 = t[ti + 1], t[ti + 2], t[ti + 3]
		if (v0 == vid and v1 == vOther) or (v1 == vid and v2 == vOther) or (v2 == vid and v0 == vOther) then
			n += 1
			buf[n] = et0
		end
		local et1 = ed[ei + 4]
		if et1 ~= InvalidID then
			ti = 3 * et1
			v0, v1, v2 = t[ti + 1], t[ti + 2], t[ti + 3]
			if (v0 == vid and v1 == vOther) or (v1 == vid and v2 == vOther) or (v2 == vid and v0 == vOther) then
				n += 1
				buf[n] = et1
			end
		end
	end
	return n
end
MeshRefinerBase.gather_vtx_triangles = gather_vtx_triangles

local tri_scratch = {}

function MeshRefinerBase:edge_flip_metric(n0, n1): number
	if self.edge_flip_tol == 0 then
		return n0:Dot(n1)
	end
	return n0:Normalized():Dot(n1:Normalized())
end

-- C# static double edge_flip_metric(ref Vector3d n0, ref Vector3d n1, double edge_flip_tol)
function MeshRefinerBase.edge_flip_metric_tol(n0, n1, edge_flip_tol: number): number
	if edge_flip_tol == 0 then
		return n0:Dot(n1)
	end
	return n0:Normalized():Dot(n1:Normalized())
end

-- check if edge collapse will create a face-normal flip.
-- Also checks if collapse would violate link condition, since
-- we are iterating over one-ring anyway.
--
-- This only checks one-ring of vid, so you have to call it twice,
-- with vid and vother reversed, to check both one-rings
-- (scalar implementation; newv given as nx, ny, nz)
local function collapse_creates_flip_or_invalid_xyz(
	mesh,
	vid: number,
	vother: number,
	nx: number,
	ny: number,
	nz: number,
	tc: number,
	td: number,
	edge_flip_tol: number
): boolean
	local vd = mesh.vertices.data
	local td_ = mesh.triangles.data
	if not mesh.vertices_refcount:isValid(vid) then
		return false
	end
	local buf = tri_scratch
	local nt = gather_vtx_triangles(mesh, vid, buf)
	for k = 1, nt do
		local tid = buf[k]
		if tid == tc or tid == td then
			continue
		end
		local ti = 3 * tid
		local ca, cb, cc = td_[ti + 1], td_[ti + 2], td_[ti + 3]
		if ca == vother or cb == vother or cc == vother then
			return true -- invalid nbrhood for collapse
		end
		local ia, ib, ic = 3 * ca, 3 * cb, 3 * cc
		local ax, ay, az = vd[ia + 1], vd[ia + 2], vd[ia + 3]
		local bx, by, bz = vd[ib + 1], vd[ib + 2], vd[ib + 3]
		local cx, cy, cz = vd[ic + 1], vd[ic + 2], vd[ic + 3]
		-- ncur = (vb - va).Cross(vc - va)
		local e1x, e1y, e1z = bx - ax, by - ay, bz - az
		local e2x, e2y, e2z = cx - ax, cy - ay, cz - az
		local ncx, ncy, ncz = e1y * e2z - e1z * e2y, e1z * e2x - e1x * e2z, e1x * e2y - e1y * e2x
		local sign
		if ca == vid then
			-- nnew = (vb - newv).Cross(vc - newv)
			local f1x, f1y, f1z = bx - nx, by - ny, bz - nz
			local f2x, f2y, f2z = cx - nx, cy - ny, cz - nz
			sign = flip_metric_xyz(ncx, ncy, ncz, f1y * f2z - f1z * f2y, f1z * f2x - f1x * f2z, f1x * f2y - f1y * f2x, edge_flip_tol)
		elseif cb == vid then
			-- nnew = (newv - va).Cross(vc - va)
			local f1x, f1y, f1z = nx - ax, ny - ay, nz - az
			sign = flip_metric_xyz(ncx, ncy, ncz, f1y * e2z - f1z * e2y, f1z * e2x - f1x * e2z, f1x * e2y - f1y * e2x, edge_flip_tol)
		elseif cc == vid then
			-- nnew = (vb - va).Cross(newv - va)
			local f2x, f2y, f2z = nx - ax, ny - ay, nz - az
			sign = flip_metric_xyz(ncx, ncy, ncz, e1y * f2z - e1z * f2y, e1z * f2x - e1x * f2z, e1x * f2y - e1y * f2x, edge_flip_tol)
		else
			error("MeshRefinerBase.collapse_creates_flip_or_invalid: should never be here!")
		end
		if sign <= edge_flip_tol then
			return true
		end
	end
	return false
end

-- C# public static bool collapse_creates_flip_or_invalid(DMesh3 mesh, int vid, int vother, ref Vector3d newv, int tc, int td, double edge_flip_tol = 0)
function MeshRefinerBase.collapse_creates_flip_or_invalid_mesh(
	mesh,
	vid: number,
	vother: number,
	newv,
	tc: number,
	td: number,
	edge_flip_tol: number?
): boolean
	return collapse_creates_flip_or_invalid_xyz(mesh, vid, vother, newv.x, newv.y, newv.z, tc, td, edge_flip_tol or 0)
end

function MeshRefinerBase:collapse_creates_flip_or_invalid(vid: number, vother: number, newv, tc: number, td: number): boolean
	return collapse_creates_flip_or_invalid_xyz(self.mesh, vid, vother, newv.x, newv.y, newv.z, tc, td, self.edge_flip_tol)
end

function MeshRefinerBase:collapse_creates_flip_or_invalid_xyz(
	vid: number,
	vother: number,
	nx: number,
	ny: number,
	nz: number,
	tc: number,
	td: number
): boolean
	return collapse_creates_flip_or_invalid_xyz(self.mesh, vid, vother, nx, ny, nz, tc, td, self.edge_flip_tol)
end

-- FastNormalDirection(v1, v2, v3) = (v2 - v1).Cross(v3 - v1)
local function fnd(x1, y1, z1, x2, y2, z2, x3, y3, z3)
	local ex, ey, ez = x2 - x1, y2 - y1, z2 - z1
	local fx, fy, fz = x3 - x1, y3 - y1, z3 - z1
	return ey * fz - ez * fy, ez * fx - ex * fz, ex * fy - ey * fx
end

-- Check if edge flip might reverse normal direction.
--
-- Not entirely clear on how to implement this test.
-- Currently checking if any normal-pairs are reversed.
function MeshRefinerBase:flip_inverts_normals(a: number, b: number, c: number, d: number, t0: number): boolean
	local mesh = self.mesh
	local vd = mesh.vertices.data
	local tdata = mesh.triangles.data
	local tol = self.edge_flip_tol
	local ti = 3 * t0
	-- IndexUtil.orient_tri_edge(ref oa, ref ob, ref tri_v) (inlined)
	local t0a, t0b, t0c = tdata[ti + 1], tdata[ti + 2], tdata[ti + 3]
	local oa, ob = a, b
	if (a == t0a and t0c == b) or (a == t0b and t0a == b) or (a == t0c and t0b == b) then
		oa, ob = b, a
	end
	local ic, id_, ioa, iob = 3 * c, 3 * d, 3 * oa, 3 * ob
	local cx, cy, cz = vd[ic + 1], vd[ic + 2], vd[ic + 3]
	local dx, dy, dz = vd[id_ + 1], vd[id_ + 2], vd[id_ + 3]
	local oax, oay, oaz = vd[ioa + 1], vd[ioa + 2], vd[ioa + 3]
	local obx, oby, obz = vd[iob + 1], vd[iob + 2], vd[iob + 3]

	local n0x, n0y, n0z = fnd(oax, oay, oaz, obx, oby, obz, cx, cy, cz)
	local n1x, n1y, n1z = fnd(obx, oby, obz, oax, oay, oaz, dx, dy, dz)
	local f0x, f0y, f0z = fnd(cx, cy, cz, dx, dy, dz, obx, oby, obz)
	if flip_metric_xyz(n0x, n0y, n0z, f0x, f0y, f0z, tol) <= tol or flip_metric_xyz(n1x, n1y, n1z, f0x, f0y, f0z, tol) <= tol then
		return true
	end
	local f1x, f1y, f1z = fnd(dx, dy, dz, cx, cy, cz, oax, oay, oaz)
	if flip_metric_xyz(n0x, n0y, n0z, f1x, f1y, f1z, tol) <= tol or flip_metric_xyz(n1x, n1y, n1z, f1x, f1y, f1z, tol) <= tol then
		return true
	end

	-- this only checks if output faces are pointing towards eachother, which seems
	-- to still result in normal-flips in some cases
	--if (f0.Dot(f1) < 0)
	--    return true;

	return false
end

-- Figure out if we can collapse edge eid=[a,b] under current constraint set.
-- First we resolve vertex constraints using can_collapse_vtx(). However this
-- does not catch some topological cases at the edge-constraint level, which
-- which we will only be able to detect once we know if we are losing a or b.
-- See comments on can_collapse_vtx() for what collapse_to is for.
-- returns ok, collapse_to
function MeshRefinerBase:can_collapse_constraints(
	eid: number,
	a: number,
	b: number,
	c: number,
	d: number,
	tc: number,
	td: number
): (boolean, number)
	local constraints = self.constraints
	if constraints == nil then
		return true, -1
	end
	local bVtx, collapse_to = self:can_collapse_vtx(eid, a, b)
	if bVtx == false then
		return false, collapse_to
	end

	-- when we lose a vtx in a collapse, we also lose two edges [iCollapse,c] and [iCollapse,d].
	-- If either of those edges is constrained, we would lose that constraint.
	-- This would be bad.
	local mesh = self.mesh
	local iCollapse = (collapse_to == a) and b or a
	if c ~= InvalidID then
		local ec = mesh:FindEdgeFromTri(iCollapse, c, tc)
		if constraints:GetEdgeConstraint(ec):IsUnconstrained() == false then
			return false, collapse_to
		end
	end
	if d ~= InvalidID then
		local ed = mesh:FindEdgeFromTri(iCollapse, d, td)
		if constraints:GetEdgeConstraint(ed):IsUnconstrained() == false then
			return false, collapse_to
		end
	end

	return true, collapse_to
end

-- resolve vertex constraints for collapsing edge eid=[a,b]. Generally we would
-- collapse a to b, and set the new position as 0.5*(v_a+v_b). However if a *or* b
-- are constrained, then we want to keep that vertex and collapse to its position.
-- This vertex (a or b) will be returned in collapse_to, which is -1 otherwise.
-- If a *and* b are constrained, then things are complicated (and documented below).
-- returns ok, collapse_to
function MeshRefinerBase:can_collapse_vtx(eid: number, a: number, b: number): (boolean, number)
	local constraints = self.constraints
	if constraints == nil then
		return true, -1
	end
	local ca = constraints:GetVertexConstraint(a)
	local cb = constraints:GetVertexConstraint(b)

	-- no constraint at all
	if ca.Fixed == false and cb.Fixed == false and ca.Target == nil and cb.Target == nil then
		return true, -1
	end

	-- handle a or b fixed
	if ca.Fixed == true and cb.Fixed == false then
		-- if b is fixed to a target, and it is different than a's target, we can't collapse
		if cb.Target ~= nil and cb.Target ~= ca.Target then
			return false, -1
		end
		return true, a
	end
	if cb.Fixed == true and ca.Fixed == false then
		if ca.Target ~= nil and ca.Target ~= cb.Target then
			return false, -1
		end
		return true, b
	end
	-- if both fixed, and options allow, treat this edge as unconstrained (eg collapse to midpoint)
	-- [RMS] tried picking a or b here, but something weird happens, where
	--   eg cylinder cap will entirely erode away. Somehow edge lengths stay below threshold??
	if self.AllowCollapseFixedVertsWithSameSetID and ca.FixedSetID >= 0 and ca.FixedSetID == cb.FixedSetID then
		return true, -1
	end

	-- handle a or b w/ target
	if ca.Target ~= nil and cb.Target == nil then
		return true, a
	end
	if cb.Target ~= nil and ca.Target == nil then
		return true, b
	end
	-- if both vertices are on the same target, and the edge is on that target,
	-- then we can collapse to either and use the midpoint (which will be projected
	-- to the target). *However*, if the edge is not on the same target, then we
	-- cannot collapse because we would be changing the constraint topology!
	if cb.Target ~= nil and ca.Target ~= nil and ca.Target == cb.Target then
		if constraints:GetEdgeConstraint(eid).Target == ca.Target then
			return true, -1
		end
	end

	return false, -1
end

function MeshRefinerBase:vertex_is_fixed(vid: number): boolean
	local constraints = self.constraints
	if constraints ~= nil and constraints:GetVertexConstraint(vid).Fixed then
		return true
	end
	return false
end

function MeshRefinerBase:vertex_is_constrained(vid: number): boolean
	local constraints = self.constraints
	if constraints ~= nil then
		local vc = constraints.Vertices[vid]
		if vc ~= nil and (vc.Fixed or vc.Target ~= nil) then
			return true
		end
	end
	return false
end

function MeshRefinerBase:get_vertex_constraint(vid: number)
	local constraints = self.constraints
	if constraints ~= nil then
		return constraints:GetVertexConstraint(vid)
	end
	return VertexUnconstrained
end

-- C# bool get_vertex_constraint(int vid, ref VertexConstraint vc) -> found, vc
function MeshRefinerBase:try_get_vertex_constraint(vid: number)
	local constraints = self.constraints
	if constraints == nil then
		return false, nil
	end
	return constraints:TryGetVertexConstraint(vid)
end

return MeshRefinerBase
