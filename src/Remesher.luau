--!native
-- Port of geometry3Sharp mesh/Remesher.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Isotropic remeshing by edge split / flip / collapse + smoothing + projection (Botsch & Kobbelt style).
--
-- Luau port notes (see PORTING.md):
--   * Remesher.new(mesh). Remesher inherits MeshRefinerBase; RemesherPro (and RegionRemesher) inherit
--     Remesher. Subclasses call Remesher.init(self, mesh) (mesh may be nil for the C# protected
--     Remesher() constructor; set self.mesh later).
--   * Public options are fields with the C# names and defaults: EnableFlips, EnableCollapses,
--     EnableSplits, EnableSmoothing, PreventNormalFlips, MinEdgeLength, MaxEdgeLength, SmoothSpeedT,
--     SmoothType (Remesher.SmoothTypes), CustomSmoothF(mesh, vid, t) -> Vector3d,
--     VertexControlF(vid) -> Remesher.VertexControl flags, ProjectionMode (Remesher.TargetProjectionMode),
--     EnableParallelProjection, EnableParallelSmooth, EnableSmoothInPlace (parallel options are accepted
--     but everything runs sequentially, see gParallel), ENABLE_PROFILING, ENABLE_DEBUG_CHECKS,
--     DebugEdges, ModifiedEdgesLastPass, Progress (ProgressCancel).
--     (C# initializes MinEdgeLength/MaxEdgeLength/SmoothSpeedT from float literals; plain doubles are used.)
--   * ProjectionTarget() / SetProjectionTarget(target). IProjectionTarget is duck typed:
--       target:Project(vPoint, identifier?) -> Vector3d [, normal]
--     (IOrientedProjectionTarget returns the normal as a second value; see BasicProjectionTargets.luau.)
--   * next_edge(cur_eid) -> new_eid, bDone   (C# out bool bDone)
--   * ComputeSmoothedVertexPos(vID, smoothFunc) -> Vector3d, bModified  (C# out bool bModified)
--   * update_after_split(edgeID, va, vb, splitInfo): splitInfo is the table returned by DMesh3:SplitEdge.
--   * Smoothing buffer (protected): self.vBufferV is a flat number array (vBufferV[3*vid + k], k = 1..3)
--     and self.vModifiedV a boolean array (vModifiedV[vid + 1]) instead of DVector<Vector3d>/BitArray.
--   * When the smoothing function is the default uniform smooth and ComputeSmoothedVertexPos is not
--     overridden, the smoothing pass uses an allocation-free scalar implementation with identical results.
--   * RuntimeDebugCheck (C# breaks into the debugger) prints via Util.DebugPrint when Util.DebugChecks is on;
--     DebugCheckVertexConstraints (ENABLE_DEBUG_CHECKS) errors where C# calls gBreakToDebugger.

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local Util = require(script.Parent.Util)
local MeshResult = require(script.Parent.MeshResult)
local MeshUtil = require(script.Parent.MeshUtil)
local MeshRefinerBase = require(script.Parent.MeshRefinerBase)
local MeshConstraints = require(script.Parent.MeshConstraints)
local SmallListSet = require(script.Parent.SmallListSet)

local EdgeConstraint = MeshConstraints.EdgeConstraint
local VertexConstraint = MeshConstraints.VertexConstraint
local EdgeUnconstrained = EdgeConstraint.Unconstrained
local VertexUnconstrained = VertexConstraint.Unconstrained

local V3 = Vector3d.new
local abs = math.abs
local band = bit32.band
local InvalidID = -1
local MeshResult_Ok = MeshResult.Ok

local SLS_Null = -1
local SLS_BLOCKSIZE = SmallListSet.BLOCKSIZE
local SLS_BLOCK_LIST_OFFSET = SmallListSet.BLOCK_LIST_OFFSET

local Remesher = setmetatable({}, { __index = MeshRefinerBase })
Remesher.__index = Remesher
Remesher.ClassName = "Remesher"

Remesher.SmoothTypes = table.freeze({
	Uniform = 0,
	Cotan = 1,
	MeanValue = 2,
})
local SmoothTypes = Remesher.SmoothTypes

-- Sometimes we need to have very granular control over what happens to
-- specific vertices. This function allows client to specify such behavior.
-- Somewhat redundant w/ VertexConstraints, but simpler to code.
-- [Flags]
Remesher.VertexControl = table.freeze({
	AllowAll = 0,
	NoSmooth = 1,
	NoProject = 2,
	NoMovement = 3, -- NoSmooth | NoProject
})
local VC_NoSmooth = 1
local VC_NoProject = 2

-- if Target is set, we can project onto it in different ways
Remesher.TargetProjectionMode = table.freeze({
	NoProjection = 0, -- disable projection
	AfterRefinement = 1, -- do all projection after the refine/smooth pass
	Inline = 2, -- project after each vertex update. Better results but more
	-- expensive because eg we might create a vertex with
	-- split, then project, then smooth, then project again.
})
local TargetProjectionMode = Remesher.TargetProjectionMode

Remesher.ProcessResult = table.freeze({
	Ok_Collapsed = 0,
	Ok_Flipped = 1,
	Ok_Split = 2,
	Ignored_EdgeIsFine = 3,
	Ignored_EdgeIsFullyConstrained = 4,
	Failed_OpNotSuccessful = 5,
	Failed_NotAnEdge = 6,
})
local ProcessResult = Remesher.ProcessResult

-- We are using a modulo-index loop to break symmetry/pathological conditions.
-- For example in a highly tessellated minimal cylinder, if the top/bottom loops have
-- sequential edge IDs, and all edges are < min edge length, then we can easily end
-- up successively collapsing each tiny edge, and eroding away the entire mesh!
-- By using modulo-index loop we jump around and hence this is unlikely to happen.
local nPrime = 31337 -- any prime will do...
Remesher.nPrime = nPrime

-- minimal Stopwatch replacement for profiling
local function stopwatch()
	return { elapsed = 0, start = nil }
end
local function sw_start(w)
	if w and w.start == nil then
		w.start = os.clock()
	end
end
local function sw_stop(w)
	if w and w.start ~= nil then
		w.elapsed += os.clock() - w.start
		w.start = nil
	end
end
Remesher._stopwatch = stopwatch
Remesher._sw_start = sw_start
Remesher._sw_stop = sw_stop

function Remesher.init(self, m)
	MeshRefinerBase.init(self, m)
	self.target = nil

	self.EnableFlips = true
	self.EnableCollapses = true
	self.EnableSplits = true
	self.EnableSmoothing = true

	self.PreventNormalFlips = false

	self.MinEdgeLength = 0.001
	self.MaxEdgeLength = 0.1

	self.SmoothSpeedT = 0.1
	self.SmoothType = SmoothTypes.Uniform

	-- this overrides default smoothing if provided
	self.CustomSmoothF = nil

	self.VertexControlF = nil

	-- [RMS] this is a debugging aid, will break to debugger if these edges are touched, in debug builds
	self.DebugEdges = {}

	self.ProjectionMode = TargetProjectionMode.AfterRefinement

	-- enable parallel projection. Only applied in AfterRefinement mode
	self.EnableParallelProjection = true

	-- Enable parallel smoothing. This will produce slightly different results
	-- across runs because we smooth in-place and hence there will be order side-effects.
	self.EnableParallelSmooth = true

	-- if smoothing is done in-place, we don't need an extra buffer, but also
	-- there will some randomness introduced in results. Probably worse.
	self.EnableSmoothInPlace = false

	self.ENABLE_PROFILING = false
	self.ENABLE_DEBUG_CHECKS = false

	-- glboal mesh info that, if known, lets us avoid work in remesh
	self.MeshIsClosed = false

	-- Number of edges that were modified in previous Remesh pass.
	-- If this number gets small relative to edge count, you have probably converged (ish)
	self.ModifiedEdgesLastPass = 0

	self.nMaxEdgeID = 0

	self.vBufferV = {}
	self.vBufferVSize = 0
	self.vModifiedV = table.create(4096, false)
	self.vModifiedVLength = 4096

	self.COUNT_SPLITS = 0
	self.COUNT_COLLAPSES = 0
	self.COUNT_FLIPS = 0
	return self
end

function Remesher.new(m)
	return Remesher.init(setmetatable({}, Remesher), m)
end

function Remesher:ProjectionTarget()
	return self.target
end

function Remesher:SetProjectionTarget(target)
	self.target = target
end

-- Set min/max edge-lengths to sane values for given target edge length
function Remesher:SetTargetEdgeLength(fLength: number)
	-- from Botsch paper
	--MinEdgeLength = fLength * (4.0/5.0);
	--MaxEdgeLength = fLength * (4.0/3.0);
	-- much nicer!! makes sense as when we split, edges are both > min !
	self.MinEdgeLength = fLength * 0.66
	self.MaxEdgeLength = fLength * 1.33
end

-- this just lets us write more concise tests below
function Remesher:EnableInlineProjection(): boolean
	return self.ProjectionMode == TargetProjectionMode.Inline
end

-- we can vastly speed things up if we precompute some invariants.
-- You need to re-run this if you are changing the mesh externally
-- between remesh passes, otherwise you will get weird results.
-- But you will probably still come out ahead, computation-time-wise
function Remesher:Precompute()
	-- if we know mesh is closed, we can skip is-boundary checks, which makes
	-- the flip-valence tests much faster!
	local mesh = self.mesh
	local ed = mesh.edges.data
	self.MeshIsClosed = true
	for eid in mesh:EdgeIndices() do
		if ed[4 * eid + 4] == InvalidID then
			self.MeshIsClosed = false
			break
		end
	end
end

-- Linear edge-refinement pass, followed by smoothing and projection
-- - Edges are processed in prime-modulo-order to break symmetry
-- - smoothing is done in parallel if EnableParallelSmooth = true
-- - Projection pass if ProjectionMode == AfterRefinement
-- - number of modified edges returned in ModifiedEdgesLastPass
function Remesher:BasicRemeshPass()
	local mesh = self.mesh
	if mesh:TriangleCount() == 0 then -- badness if we don't catch this...
		return
	end

	self:begin_pass()

	-- Iterate over all edges in the mesh at start of pass.
	-- Some may be removed, so we skip those.
	-- However, some old eid's may also be re-used, so we will touch
	-- some new edges. Can't see how we could efficiently prevent this.
	--
	self:begin_ops()

	local edges_rc = mesh.edges_refcount
	local cur_eid = self:start_edges()
	local done = false
	self.ModifiedEdgesLastPass = 0
	repeat
		if edges_rc:isValid(cur_eid) then
			local result = self:ProcessEdge(cur_eid)
			if result == ProcessResult.Ok_Collapsed or result == ProcessResult.Ok_Flipped or result == ProcessResult.Ok_Split then
				self.ModifiedEdgesLastPass += 1
			end
		end
		if self:Cancelled() then -- expensive to check every iter?
			return
		end
		cur_eid, done = self:next_edge(cur_eid)
	until done
	self:end_ops()

	if self:Cancelled() then
		return
	end

	self:begin_smooth()
	if self.EnableSmoothing and self.SmoothSpeedT > 0 then
		if self.EnableSmoothInPlace then
			self:FullSmoothPass_InPlace(self.EnableParallelSmooth)
		else
			self:FullSmoothPass_Buffer(self.EnableParallelSmooth)
		end
		self:DoDebugChecks()
	end
	self:end_smooth()

	if self:Cancelled() then
		return
	end

	self:begin_project()
	if self.target ~= nil and self.ProjectionMode == TargetProjectionMode.AfterRefinement then
		self:FullProjectionPass()
		self:DoDebugChecks()
	end
	self:end_project()

	if self:Cancelled() then
		return
	end

	self:end_pass()
end

-- subclasses can override these to implement custom behavior...
function Remesher:OnEdgeSplit(edgeID: number, va: number, vb: number, splitInfo)
	-- this is for subclasses...
end

function Remesher:OnEdgeCollapse(edgeID: number, va: number, vb: number, collapseInfo)
	-- this is for subclasses...
end

-- start_edges() and next_edge() control the iteration over edges that will be refined.
-- Default here is to iterate over entire mesh.
-- Subclasses can override these two functions to restrict the affected edges (eg EdgeLoopRemesher)
function Remesher:start_edges(): number
	self.nMaxEdgeID = self.mesh:MaxEdgeID()
	return 0
end

-- C# next_edge(int cur_eid, out bool bDone) -> new_eid, bDone
function Remesher:next_edge(cur_eid: number): (number, boolean)
	local new_eid = (cur_eid + nPrime) % self.nMaxEdgeID
	return new_eid, (new_eid == 0)
end

function Remesher:smooth_vertices(): () -> number?
	return self.mesh:VertexIndices()
end
function Remesher:project_vertices(): () -> number?
	return self.mesh:VertexIndices()
end

-- number of edges at vertex (vertex_edges list count); vertex must be valid
local function vtx_valence(sls, vid: number): number
	local block_ptr = sls.list_heads[vid + 1]
	if block_ptr == nil or block_ptr == SLS_Null then
		return 0
	end
	return sls.block_store[block_ptr + 1]
end

-- other vertex of triangle tid that is not a or b
local function tri_other_vtx(tdata, tid: number, a: number, b: number): number
	local ti = 3 * tid
	local x, y, z = tdata[ti + 1], tdata[ti + 2], tdata[ti + 3]
	if (x == a and y == b) or (x == b and y == a) then
		return z
	elseif (y == a and z == b) or (y == b and z == a) then
		return x
	elseif (z == a and x == b) or (z == b and x == a) then
		return y
	end
	return InvalidID
end

function Remesher:ProcessEdge(edgeID: number): number
	self:RuntimeDebugCheck(edgeID)

	local mesh = self.mesh
	local constraints = self.constraints
	local constraint = (constraints == nil) and EdgeUnconstrained or constraints:GetEdgeConstraint(edgeID)
	local rflags = constraint.refineFlags
	if band(rflags, 7) == 7 then -- constraint.NoModifications
		return ProcessResult.Ignored_EdgeIsFullyConstrained
	end

	-- look up verts and tris for this edge
	if not mesh.edges_refcount:isValid(edgeID) then
		return ProcessResult.Failed_NotAnEdge
	end
	local ed = mesh.edges.data
	local ei = 4 * edgeID
	local a, b, t0, t1 = ed[ei + 1], ed[ei + 2], ed[ei + 3], ed[ei + 4]
	local bIsBoundaryEdge = (t1 == InvalidID)

	-- look up 'other' verts c (from t0) and d (from t1, if it exists)
	-- (mesh.GetEdgeOpposingV(edgeID))
	local tdata = mesh.triangles.data
	local c = tri_other_vtx(tdata, t0, a, b)
	local d = bIsBoundaryEdge and InvalidID or tri_other_vtx(tdata, t1, a, b)

	local vd = mesh.vertices.data
	local ia, ib = 3 * a, 3 * b
	local ax, ay, az = vd[ia + 1], vd[ia + 2], vd[ia + 3]
	local bx, by, bz = vd[ib + 1], vd[ib + 2], vd[ib + 3]
	-- vA.DistanceSquared(vB)
	local dx, dy, dz = bx - ax, by - ay, bz - az
	local edge_len_sqr = dx * dx + dy * dy + dz * dz

	self:begin_collapse()

	-- check if we should collapse, and also find which vertex we should collapse to,
	-- in cases where we have constraints/etc
	local collapse_to = -1
	local MinEdgeLength = self.MinEdgeLength
	local bCanCollapse = false
	if self.EnableCollapses and band(rflags, 4) == 0 and edge_len_sqr < MinEdgeLength * MinEdgeLength then
		bCanCollapse, collapse_to = self:can_collapse_constraints(edgeID, a, b, c, d, t0, t1)
	end

	-- optimization: if edge cd exists, we cannot collapse or flip. look that up here?
	--  funcs will do it internally...
	--  (or maybe we can collapse if cd exists? edge-collapse doesn't check for it explicitly...)

	-- if edge length is too short, we want to collapse it
	local bTriedCollapse = false
	if bCanCollapse then
		local iKeep, iCollapse = b, a
		local vNewPos

		-- if either vtx is fixed, collapse to that position
		if collapse_to == b then
			vNewPos = V3(bx, by, bz)
		elseif collapse_to == a then
			iKeep, iCollapse = a, b
			vNewPos = V3(ax, ay, az)
		else
			-- (vA + vB) * 0.5
			vNewPos = self:get_projected_collapse_position(iKeep, V3((ax + bx) * 0.5, (ay + by) * 0.5, (az + bz) * 0.5))
		end

		-- if new position would flip normal of one of the existing triangles
		-- either one-ring, don't allow it
		local bAbort = false
		if self.PreventNormalFlips then
			local nx, ny, nz = vNewPos.x, vNewPos.y, vNewPos.z
			if
				self:collapse_creates_flip_or_invalid_xyz(a, b, nx, ny, nz, t0, t1)
				or self:collapse_creates_flip_or_invalid_xyz(b, a, nx, ny, nz, t0, t1)
			then
				bAbort = true -- goto abort_collapse
			end
		end

		if not bAbort then
			-- TODO be smart about picking b (keep vtx).
			--    - swap if one is bdry vtx, for example?
			-- lots of cases where we cannot collapse, but we should just let
			-- mesh sort that out, right?
			self.COUNT_COLLAPSES += 1
			local result, collapseInfo = mesh:CollapseEdge(iKeep, iCollapse)
			if result == MeshResult_Ok then
				mesh:SetVertexXYZ(iKeep, vNewPos.x, vNewPos.y, vNewPos.z)
				if constraints ~= nil then
					constraints:ClearEdgeConstraint(edgeID)
					constraints:ClearEdgeConstraint(collapseInfo.eRemoved0)
					if collapseInfo.eRemoved1 ~= InvalidID then
						constraints:ClearEdgeConstraint(collapseInfo.eRemoved1)
					end
					constraints:ClearVertexConstraint(iCollapse)
				end
				self:OnEdgeCollapse(edgeID, iKeep, iCollapse, collapseInfo)
				self:DoDebugChecks()

				return ProcessResult.Ok_Collapsed
			else
				bTriedCollapse = true
			end
		end
	end
	-- abort_collapse:

	self:end_collapse()
	self:begin_flip()

	-- if this is not a boundary edge, maybe we want to flip
	local bTriedFlip = false
	if self.EnableFlips and band(rflags, 1) == 0 and bIsBoundaryEdge == false then
		-- can we do this more efficiently somehow?
		local MeshIsClosed = self.MeshIsClosed
		local a_is_boundary_vtx = (not MeshIsClosed) and (bIsBoundaryEdge or mesh:IsBoundaryVertex(a))
		local b_is_boundary_vtx = (not MeshIsClosed) and (bIsBoundaryEdge or mesh:IsBoundaryVertex(b))
		local c_is_boundary_vtx = (not MeshIsClosed) and mesh:IsBoundaryVertex(c)
		local d_is_boundary_vtx = (not MeshIsClosed) and mesh:IsBoundaryVertex(d)
		local sls = mesh.vertex_edges
		local valence_a, valence_b = vtx_valence(sls, a), vtx_valence(sls, b)
		local valence_c, valence_d = vtx_valence(sls, c), vtx_valence(sls, d)
		local valence_a_target = a_is_boundary_vtx and valence_a or 6
		local valence_b_target = b_is_boundary_vtx and valence_b or 6
		local valence_c_target = c_is_boundary_vtx and valence_c or 6
		local valence_d_target = d_is_boundary_vtx and valence_d or 6

		-- if total valence error improves by flip, we want to do it
		local curr_err = abs(valence_a - valence_a_target)
			+ abs(valence_b - valence_b_target)
			+ abs(valence_c - valence_c_target)
			+ abs(valence_d - valence_d_target)
		local flip_err = abs((valence_a - 1) - valence_a_target)
			+ abs((valence_b - 1) - valence_b_target)
			+ abs((valence_c + 1) - valence_c_target)
			+ abs((valence_d + 1) - valence_d_target)

		local bTryFlip = flip_err < curr_err
		if bTryFlip and self.PreventNormalFlips and self:flip_inverts_normals(a, b, c, d, t0) then
			bTryFlip = false
		end

		if bTryFlip then
			self.COUNT_FLIPS += 1
			local result = mesh:FlipEdge(edgeID)
			if result == MeshResult_Ok then
				self:DoDebugChecks()
				return ProcessResult.Ok_Flipped
			else
				bTriedFlip = true
			end
		end
	end

	self:end_flip()
	self:begin_split()

	-- if edge length is too long, we want to split it
	local bTriedSplit = false
	local MaxEdgeLength = self.MaxEdgeLength
	if self.EnableSplits and band(rflags, 2) == 0 and edge_len_sqr > MaxEdgeLength * MaxEdgeLength then
		self.COUNT_SPLITS += 1
		local result, splitInfo = mesh:SplitEdge(edgeID)
		if result == MeshResult_Ok then
			self:update_after_split(edgeID, a, b, splitInfo)
			self:OnEdgeSplit(edgeID, a, b, splitInfo)
			self:DoDebugChecks()
			return ProcessResult.Ok_Split
		else
			bTriedSplit = true
		end
	end

	self:end_split()

	if bTriedFlip or bTriedSplit or bTriedCollapse then
		return ProcessResult.Failed_OpNotSuccessful
	end
	return ProcessResult.Ignored_EdgeIsFine
end

-- After we split an edge, we have created a new edge and a new vertex.
-- The edge needs to inherit the constraint on the other pre-existing edge that we kept.
-- In addition, if the edge vertices were both constrained, then we /might/
-- want to also constrain this new vertex, possibly project to constraint target.
function Remesher:update_after_split(edgeID: number, va: number, vb: number, splitInfo)
	local bPositionFixed = false
	local constraints = self.constraints
	if constraints ~= nil and constraints:HasEdgeConstraint(edgeID) then
		-- inherit edge constraint
		constraints:SetOrUpdateEdgeConstraint(splitInfo.eNewBN, constraints:GetEdgeConstraint(edgeID))

		-- [RMS] update vertex constraints. Note that there is some ambiguity here.
		--   Both verts being constrained doesn't inherently mean that the edge is on
		--   a constraint, that's why these checks are only applied if edge is constrained.
		--   But constrained edge doesn't necessarily mean we want to inherit vert constraints!!
		--
		--   although, pretty safe to assume that we would at least disable flips
		--   if both vertices are constrained to same line/curve. So, maybe this makes sense...
		--
		--   (perhaps edge constraint should be explicitly tagged to resolve this ambiguity??)

		-- vert inherits Fixed if both orig edge verts Fixed, and both tagged with same SetID
		local ca = constraints:GetVertexConstraint(va)
		local cb = constraints:GetVertexConstraint(vb)
		if ca.Fixed and cb.Fixed then
			local nSetID = (ca.FixedSetID > 0 and ca.FixedSetID == cb.FixedSetID) and ca.FixedSetID or VertexConstraint.InvalidSetID
			constraints:SetOrUpdateVertexConstraint(splitInfo.vNew, VertexConstraint.new(true, nSetID))
			bPositionFixed = true
		end

		-- vert inherits Target if:
		--  1) both source verts and edge have same Target, and is same as edge target
		--  2) either vert has same target as edge, and other vert is fixed
		if ca.Target ~= nil or cb.Target ~= nil then
			local edge_target = constraints:GetEdgeConstraint(edgeID).Target
			local set_target = nil
			if ca.Target == cb.Target and ca.Target == edge_target then
				set_target = edge_target
			elseif ca.Target == edge_target and cb.Fixed then
				set_target = edge_target
			elseif cb.Target == edge_target and ca.Fixed then
				set_target = edge_target
			end
			if set_target ~= nil then
				constraints:SetOrUpdateVertexConstraint(splitInfo.vNew, VertexConstraint.FromTarget(set_target))
				self:project_vertex(splitInfo.vNew, set_target)
				bPositionFixed = true
			end
		end
	end

	if self.ProjectionMode == TargetProjectionMode.Inline and bPositionFixed == false and self.target ~= nil then
		self:project_vertex(splitInfo.vNew, self.target)
	end
end

function Remesher:project_vertex(vID: number, targetIn)
	local mesh = self.mesh
	local curpos = mesh:GetVertex(vID)
	local projected = targetIn:Project(curpos, vID)
	mesh:SetVertex(vID, projected)
end

-- used by collapse-edge to get projected position for new vertex
function Remesher:get_projected_collapse_position(vid: number, vNewPos)
	local constraints = self.constraints
	if constraints ~= nil then
		local vc = constraints:GetVertexConstraint(vid)
		if vc.Target ~= nil then
			return (vc.Target:Project(vNewPos, vid))
		end
		if vc.Fixed then
			return vNewPos
		end
	end
	-- no constraint applied, so if we have a target surface, project to that
	if self.ProjectionMode == TargetProjectionMode.Inline and self.target ~= nil then
		if self.VertexControlF == nil or band(self.VertexControlF(vid), VC_NoProject) == 0 then
			return (self.target:Project(vNewPos, vid))
		end
	end
	return vNewPos
end

-- returns the smoothing function selected by CustomSmoothF / SmoothType
function Remesher:get_smooth_func()
	if self.CustomSmoothF ~= nil then
		return self.CustomSmoothF
	end
	if self.SmoothType == SmoothTypes.MeanValue then
		return MeshUtil.MeanValueSmooth
	elseif self.SmoothType == SmoothTypes.Cotan then
		return MeshUtil.CotanSmooth
	end
	return MeshUtil.UniformSmooth
end

function Remesher:FullSmoothPass_InPlace(bParallel: boolean)
	local smoothFunc = self:get_smooth_func()
	local mesh = self.mesh
	for vID in self:smooth_vertices() do
		local vSmoothed, bModified = self:ComputeSmoothedVertexPos(vID, smoothFunc)
		if bModified then
			mesh:SetVertex(vID, vSmoothed)
		end
	end
end

-- allocation-free uniform smoothing of vertex vID: (1-t)*v + t*onering_centroid  (MeshUtil.UniformSmooth)
local function uniform_smooth_xyz(mesh, vID: number, t: number): (number, number, number)
	local d = mesh.vertices.data
	local k = 3 * vID
	local vx, vy, vz = d[k + 1], d[k + 2], d[k + 3]
	-- mesh.VtxOneRingCentroid(vID) (inlined iteration over vertex_edges)
	local cx, cy, cz = 0, 0, 0
	local n = 0
	local ed = mesh.edges.data
	local sls = mesh.vertex_edges
	local block_ptr = sls.list_heads[vID + 1]
	if block_ptr ~= nil and block_ptr ~= SLS_Null then
		local bs = sls.block_store
		local N = bs[block_ptr + 1]
		local iEnd = block_ptr + ((N < SLS_BLOCKSIZE) and N or SLS_BLOCKSIZE)
		for i = block_ptr + 2, iEnd + 1 do
			local eid = bs[i]
			local ev0 = ed[4 * eid + 1]
			local other = (ev0 == vID) and ed[4 * eid + 2] or ev0
			local oi = 3 * other
			cx += d[oi + 1]
			cy += d[oi + 2]
			cz += d[oi + 3]
			n += 1
		end
		if N > SLS_BLOCKSIZE then
			local ls = sls.linked_store
			local cur_ptr = bs[block_ptr + SLS_BLOCK_LIST_OFFSET + 1]
			while cur_ptr ~= SLS_Null do
				local eid = ls[cur_ptr + 1]
				local ev0 = ed[4 * eid + 1]
				local other = (ev0 == vID) and ed[4 * eid + 2] or ev0
				local oi = 3 * other
				cx += d[oi + 1]
				cy += d[oi + 2]
				cz += d[oi + 3]
				n += 1
				cur_ptr = ls[cur_ptr + 2]
			end
		end
		if n > 0 then
			local f = 1.0 / n
			cx *= f
			cy *= f
			cz *= f
		end
	end
	local s = 1.0 - t
	return s * vx + t * cx, s * vy + t * cy, s * vz + t * cz
end
Remesher.uniform_smooth_xyz = uniform_smooth_xyz

-- true if the smoothing pass can use uniform_smooth_xyz instead of ComputeSmoothedVertexPos
function Remesher:can_use_fast_uniform_smooth(smoothFunc): boolean
	return smoothFunc == MeshUtil.UniformSmooth
		and self.ComputeSmoothedVertexPos == Remesher.ComputeSmoothedVertexPos
		and (self.target == nil or self.ProjectionMode ~= TargetProjectionMode.Inline)
		and self.VertexControlF == nil
end

function Remesher:FullSmoothPass_Buffer(bParallel: boolean)
	self:InitializeVertexBufferForPass()

	local smoothFunc = self:get_smooth_func()
	local vBufferV, vModifiedV = self.vBufferV, self.vModifiedV

	if self:can_use_fast_uniform_smooth(smoothFunc) then
		-- same result as ComputeSmoothedVertexPos with MeshUtil.UniformSmooth
		local mesh = self.mesh
		local constraints = self.constraints
		local t = self.SmoothSpeedT
		local cv = constraints and constraints.Vertices
		for vID in self:smooth_vertices() do
			local vc = cv and cv[vID]
			if vc ~= nil and vc.Fixed then
				continue
			end
			local x, y, z = uniform_smooth_xyz(mesh, vID, t)
			if vc ~= nil and vc.Target ~= nil then
				local p = vc.Target:Project(V3(x, y, z), vID)
				x, y, z = p.x, p.y, p.z
			end
			vModifiedV[vID + 1] = true
			local k = 3 * vID
			vBufferV[k + 1], vBufferV[k + 2], vBufferV[k + 3] = x, y, z
		end
	else
		for vID in self:smooth_vertices() do
			local vSmoothed, bModified = self:ComputeSmoothedVertexPos(vID, smoothFunc)
			if bModified then
				vModifiedV[vID + 1] = true
				local k = 3 * vID
				vBufferV[k + 1], vBufferV[k + 2], vBufferV[k + 3] = vSmoothed.x, vSmoothed.y, vSmoothed.z
			end
		end
	end

	self:ApplyVertexBuffer(bParallel)
end

function Remesher:InitializeVertexBufferForPass()
	local NV = self.mesh:MaxVertexID()
	if self.vBufferVSize < NV then
		local newSize = NV + NV // 5
		local buf = self.vBufferV
		for i = 3 * self.vBufferVSize + 1, 3 * newSize do
			buf[i] = 0
		end
		self.vBufferVSize = newSize
	end
	if self.vModifiedVLength < NV then
		self.vModifiedVLength = 2 * NV
		self.vModifiedV = table.create(2 * NV, false)
	else
		local m = self.vModifiedV
		for i = 1, self.vModifiedVLength do
			m[i] = false
		end
	end
end

function Remesher:ApplyVertexBuffer(bParallel: boolean)
	local mesh = self.mesh
	local vBufferV, vModifiedV = self.vBufferV, self.vModifiedV
	if bParallel then
		local NV = mesh:MaxVertexID()
		for vid = 0, NV - 1 do
			if vModifiedV[vid + 1] then
				local k = 3 * vid
				mesh:SetVertexXYZ(vid, vBufferV[k + 1], vBufferV[k + 2], vBufferV[k + 3])
			end
		end
	else
		for vid in mesh:VertexIndices() do
			if vModifiedV[vid + 1] then
				local k = 3 * vid
				mesh:SetVertexXYZ(vid, vBufferV[k + 1], vBufferV[k + 2], vBufferV[k + 3])
			end
		end
	end
end

-- This computes smoothed positions w/ proper constraints/etc.
-- Does not modify mesh.
-- C# ComputeSmoothedVertexPos(vID, smoothFunc, out bModified) -> vSmoothed, bModified
function Remesher:ComputeSmoothedVertexPos(vID: number, smoothFunc): (any, boolean)
	local mesh = self.mesh
	local constraints = self.constraints
	local vConstraint = VertexUnconstrained
	if constraints ~= nil then
		local found, vc = constraints:TryGetVertexConstraint(vID)
		if found then
			vConstraint = vc
		end
	end
	if vConstraint.Fixed then
		return mesh:GetVertex(vID), false
	end
	local vControl = (self.VertexControlF == nil) and 0 or self.VertexControlF(vID)
	if band(vControl, VC_NoSmooth) ~= 0 then
		return mesh:GetVertex(vID), false
	end

	local vSmoothed = smoothFunc(mesh, vID, self.SmoothSpeedT)
	Util.gDevAssert(vSmoothed:IsFinite()) -- this will really catch a lot of bugs...

	-- project onto either vtx constraint target, or surface target
	if vConstraint.Target ~= nil then
		vSmoothed = vConstraint.Target:Project(vSmoothed, vID)
	elseif self.ProjectionMode == TargetProjectionMode.Inline and self.target ~= nil then
		if band(vControl, VC_NoProject) == 0 then
			vSmoothed = self.target:Project(vSmoothed, vID)
		end
	end

	return vSmoothed, true
end

-- Project vertices onto projection target.
-- We can do projection in parallel if we have .net
function Remesher:FullProjectionPass()
	local mesh = self.mesh
	local target = self.target
	local VertexControlF = self.VertexControlF
	for vID in self:project_vertices() do
		if self:vertex_is_constrained(vID) then
			continue
		end
		if VertexControlF ~= nil and band(VertexControlF(vID), VC_NoProject) ~= 0 then
			continue
		end
		local curpos = mesh:GetVertex(vID)
		local projected = target:Project(curpos, vID)
		mesh:SetVertexXYZ(vID, projected.x, projected.y, projected.z)
	end
end

-- [Conditional("DEBUG")]
function Remesher:RuntimeDebugCheck(eid: number)
	if Util.DebugChecks and #self.DebugEdges > 0 and table.find(self.DebugEdges, eid) ~= nil then
		Util.DebugPrint("Remesher: touched debug edge " .. eid)
	end
end

function Remesher:DoDebugChecks()
	if self.ENABLE_DEBUG_CHECKS == false then
		return
	end
	self:DebugCheckVertexConstraints()
end

function Remesher:DebugCheckVertexConstraints()
	local constraints = self.constraints
	if constraints == nil then
		return
	end
	local mesh = self.mesh
	for vid, vc in constraints:VertexConstraintsItr() do
		if vc.Target ~= nil then
			local curpos = mesh:GetVertex(vid)
			local projected = vc.Target:Project(curpos, vid)
			if curpos:DistanceSquared(projected) > 0.0001 then
				error("Remesher.DebugCheckVertexConstraints: vertex " .. vid .. " is not on its constraint target")
			end
		end
	end
end

--
-- profiling functions, turn on ENABLE_PROFILING to see output in console
--
function Remesher:begin_pass()
	if self.ENABLE_PROFILING then
		self.COUNT_SPLITS, self.COUNT_COLLAPSES, self.COUNT_FLIPS = 0, 0, 0
		self.AllOpsW = stopwatch()
		self.SmoothW = stopwatch()
		self.ProjectW = stopwatch()
		self.FlipW = stopwatch()
		self.SplitW = stopwatch()
		self.CollapseW = stopwatch()
	end
end

function Remesher:end_pass()
	if self.ENABLE_PROFILING then
		local mesh = self.mesh
		print(
			string.format(
				"RemeshPass: T %d V %d splits %d flips %d collapses %d",
				mesh:TriangleCount(),
				mesh:VertexCount(),
				self.COUNT_SPLITS,
				self.COUNT_FLIPS,
				self.COUNT_COLLAPSES
			)
		)
		print(
			string.format(
				"           Timing1:  ops %.3f smooth %.3f project %.3f",
				self.AllOpsW.elapsed,
				self.SmoothW.elapsed,
				self.ProjectW.elapsed
			)
		)
		print(
			string.format(
				"           Timing2:  collapse %.3f flip %.3f split %.3f",
				self.CollapseW.elapsed,
				self.FlipW.elapsed,
				self.SplitW.elapsed
			)
		)
	end
end

local function profiler(startName: string, stopName: string, field: string)
	Remesher[startName] = function(self)
		if self.ENABLE_PROFILING then
			sw_start(self[field])
		end
	end
	Remesher[stopName] = function(self)
		if self.ENABLE_PROFILING then
			sw_stop(self[field])
		end
	end
end
profiler("begin_ops", "end_ops", "AllOpsW")
profiler("begin_smooth", "end_smooth", "SmoothW")
profiler("begin_project", "end_project", "ProjectW")
profiler("begin_collapse", "end_collapse", "CollapseW")
profiler("begin_flip", "end_flip", "FlipW")
profiler("begin_split", "end_split", "SplitW")

return Remesher
