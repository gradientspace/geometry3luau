--!native
-- Port of geometry3Sharp mesh/RemesherPro.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Extension to Remesher that is smarter about which edges/vertices to touch:
--  - queue tracks edges that were affected on last pass, and hence might need to be updated
--  - FastSplitIteration() just does splits, to reach target edge length as quickly as possible
--  - RemeshIteration() applies remesh pass for modified edges
--  - TrackedSmoothPass() smooths all vertices but only adds to queue if edge changes enough
--  - TrackedProjectionPass() same
--
-- Luau port notes (see PORTING.md):
--   * RemesherPro.new(mesh); inherits Remesher (metatable inheritance).
--   * FastestRemesh(nMaxIterations = 25, bDoFastSplits = true)
--     SharpEdgeReprojectionRemesh(nRemeshIterations, nTuneIterations, bDoFastSplits = true)
--   * The C# HashSet<int> modified_edges is an insertion-ordered set (self.modified_edges =
--     { list = {...}, set = {[eid] = true} }), which reproduces the C# HashSet enumeration order.
--   * EdgesIterator() returns an iterator function.
--   * ComputeProjectedVertexPos(vID) -> Vector3d, bModified  (C# out bool bModified)
--   * Face-aligned projection (UseFaceAlignedProjection, SharpEdgeReprojectionRemesh,
--     TrackedFaceProjectionPass) needs an IOrientedProjectionTarget: an object with
--     IsOrientedProjectionTarget == true whose target:Project(vPoint, identifier?) returns
--     (projectedPoint, normal) (see BasicProjectionTargets.luau).
--   * vBufferVWeights is a plain array (vBufferVWeights[vid + 1]).

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local Frame3d = require(script.Parent.Frame3d)
local Remesher = require(script.Parent.Remesher)
local IndexUtil = require(script.Parent.IndexUtil)

local V3 = Vector3d.new
local find_tri_other_vtx_dv = IndexUtil.find_tri_other_vtx_dv
local sqrt = math.sqrt
local band = bit32.band
local TargetProjectionMode = Remesher.TargetProjectionMode
local ProcessResult = Remesher.ProcessResult
local VC_NoProject = Remesher.VertexControl.NoProject
local ZeroTolerancef = MathUtil.ZeroTolerancef

local RemesherPro = setmetatable({}, { __index = Remesher })
RemesherPro.__index = RemesherPro
RemesherPro.ClassName = "RemesherPro"

function RemesherPro.init(self, m)
	Remesher.init(self, m)
	self.UseFaceAlignedProjection = false
	self.FaceProjectionPassesPerIteration = 1
	self.modified_edges = nil
	self.SplitF = nil
	self.edges_buffer = {}
	self.vBufferVWeights = {}
	self.vBufferVWeightsSize = 0
	self.stateStack = {}
	return self
end

function RemesherPro.new(m)
	return RemesherPro.init(setmetatable({}, RemesherPro), m)
end

local function new_edge_set()
	return { list = {}, set = {} }
end

local function edge_set_add(s, eid: number)
	local set = s.set
	if not set[eid] then
		set[eid] = true
		local list = s.list
		list[#list + 1] = eid
	end
end

local function edge_set_clear(s)
	table.clear(s.list)
	table.clear(s.set)
end

-- iterator over all edges in prime-modulo order (start_edges / next_edge)
function RemesherPro:EdgesIterator(): () -> number?
	local started = false
	local done = false
	local cur_eid = 0
	return function()
		if not started then
			started = true
			cur_eid = self:start_edges()
			return cur_eid
		end
		if done then
			return nil
		end
		cur_eid, done = self:next_edge(cur_eid)
		if done then
			return nil
		end
		return cur_eid
	end
end

-- iterate the vertex_edges of vid and add them to modified_edges
function RemesherPro:queue_one_ring(vid: number)
	local mesh = self.mesh
	if mesh:IsVertex(vid) then
		local s = self.modified_edges
		for eid in mesh:VtxEdgesItr(vid) do
			edge_set_add(s, eid)
		end
	end
end
RemesherPro.queue_one_ring_safe = RemesherPro.queue_one_ring

function RemesherPro:queue_edge(eid: number)
	edge_set_add(self.modified_edges, eid)
end
RemesherPro.queue_edge_safe = RemesherPro.queue_edge

function RemesherPro:OnEdgeSplit(edgeID: number, va: number, vb: number, splitInfo)
	local SplitF = self.SplitF
	if SplitF ~= nil then
		SplitF(edgeID, va, vb, splitInfo.vNew)
	end
end

-- Converge on remeshed result as quickly as possible
function RemesherPro:FastestRemesh(nMaxIterations: number?, bDoFastSplits: boolean?)
	local nMax = nMaxIterations or 25
	self:ResetQueue()

	-- first we do fast splits to hit edge length target
	-- ?? should we do project in fastsplit? will result in more splits, and
	--    we are going to project in first remesh pass anyway...
	--    (but that might result in larger queue in first remesh pass?)
	local fastsplit_i = 0
	local max_fastsplits = nMax
	if bDoFastSplits ~= false then
		if self:Cancelled() then
			return
		end

		local bContinue = true
		while bContinue do
			local nSplits = self:FastSplitIteration()
			local cur = fastsplit_i
			fastsplit_i += 1
			if cur > max_fastsplits then
				bContinue = false
			end
			if nSplits / self.mesh:EdgeCount() < 0.01 then
				bContinue = false
			end
			if self:Cancelled() then
				return
			end
		end
		self:ResetQueue()
	end

	-- should we do a fast collapse pass? more dangerous...

	-- now do queued remesh iterations.
	-- disable projection every other iteration to improve speed
	local saveMode = self.ProjectionMode
	for k = 0, nMax - 2 do
		if self:Cancelled() then
			break
		end
		self.ProjectionMode = (k % 2 == 0) and TargetProjectionMode.NoProjection or saveMode
		self:RemeshIteration()
	end

	-- final pass w/ full projection
	self.ProjectionMode = saveMode

	if self:Cancelled() then
		return
	end

	self:RemeshIteration()
end

local function is_oriented_target(t): boolean
	return t ~= nil and t.IsOrientedProjectionTarget == true
end

-- This is a remesh that tries to recover sharp edges by aligning triangles to face normals
-- of our projection target (similar to Ohtake RZN-flow).
function RemesherPro:SharpEdgeReprojectionRemesh(nRemeshIterations: number, nTuneIterations: number, bDoFastSplits: boolean?)
	if self:ProjectionTarget() == nil or is_oriented_target(self:ProjectionTarget()) == false then
		error("RemesherPro.SharpEdgeReprojectionRemesh: cannot call this without a ProjectionTarget that has normals")
	end

	self:ResetQueue()

	-- first we do fast splits to hit edge length target
	local fastsplit_i = 0
	local max_fastsplits = nRemeshIterations
	if bDoFastSplits ~= false then
		if self:Cancelled() then
			return
		end

		local bContinue = true
		while bContinue do
			local nSplits = self:FastSplitIteration()
			local cur = fastsplit_i
			fastsplit_i += 1
			if cur > max_fastsplits then
				bContinue = false
			end
			if nSplits / self.mesh:EdgeCount() < 0.01 then
				bContinue = false
			end
			if self:Cancelled() then
				return
			end
		end
		self:ResetQueue()
	end

	local save_use_face_aligned = self.UseFaceAlignedProjection
	self.UseFaceAlignedProjection = true
	self.FaceProjectionPassesPerIteration = 1

	-- should we do a fast collapse pass? more dangerous but would get rid of all the tiny
	-- edges we might have just created, and/or get us closer to target resolution

	-- now do queued remesh iterations. As we proceed we slowly step
	-- down the smoothing factor, this helps us get triangles closer
	-- to where they will ultimately want to go
	local smooth_speed = self.SmoothSpeedT
	for k = 0, nRemeshIterations - 1 do
		if self:Cancelled() then
			break
		end
		self:RemeshIteration()
		if k > nRemeshIterations // 2 then
			self.SmoothSpeedT *= 0.9
		end
	end

	-- [TODO] would like to still do splits and maybe sometimes flips here.
	-- Perhaps this could be something more combinatorial? Like, test all the
	-- edges we queued in the projection pass, if we can get better alignment
	-- after a with flip or split, do it
	for _ = 0, nTuneIterations - 1 do
		if self:Cancelled() then
			break
		end
		self:TrackedFaceProjectionPass()
	end

	self.SmoothSpeedT = smooth_speed
	self.UseFaceAlignedProjection = save_use_face_aligned
end

-- Reset tracked-edges queue. Should be called if mesh is modified by external functions
-- between passes, and also between different types of passes (eg FastSplitIteration vs RemeshIteration)
function RemesherPro:ResetQueue()
	if self.modified_edges ~= nil then
		edge_set_clear(self.modified_edges)
		self.modified_edges = nil
	end
end

-- returns the edge iterator for this pass, and resets modified_edges (C# code shared by
-- FastSplitIteration and RemeshIteration)
function RemesherPro:begin_tracked_edges(): () -> number?
	if self.modified_edges == nil then
		self.modified_edges = new_edge_set()
		return self:EdgesIterator()
	end
	local buf = self.edges_buffer
	table.clear(buf)
	table.move(self.modified_edges.list, 1, #self.modified_edges.list, 1, buf)
	edge_set_clear(self.modified_edges)
	local i = 0
	return function()
		i += 1
		return buf[i]
	end
end

-- This pass only does edge splits. Returns number of split edges.
-- Tracks previously-split
function RemesherPro:FastSplitIteration(): number
	local mesh = self.mesh
	if mesh:TriangleCount() == 0 then -- badness if we don't catch this...
		return 0
	end

	self:PushState()
	self.EnableFlips = false
	self.EnableCollapses = false
	self.EnableSmoothing = false
	self.ProjectionMode = TargetProjectionMode.NoProjection

	self:begin_pass()

	-- Iterate over all edges in the mesh at start of pass.
	-- Some may be removed, so we skip those.
	-- However, some old eid's may also be re-used, so we will touch
	-- some new edges. Can't see how we could efficiently prevent this.
	--
	self:begin_ops()

	local edgesItr = self:begin_tracked_edges()

	local splitEdges = 0

	-- When we split an edge, we need to check it and the adjacent ones we added.
	-- Because of overhead in ProcessEdge, it is worth it to do a distance-check here
	local max_edge_len_sqr = self.MaxEdgeLength * self.MaxEdgeLength
	local vd = mesh.vertices.data
	local ed = mesh.edges.data
	local modified = self.modified_edges
	self.SplitF = function(edgeID, a, b, vNew)
		local k = 3 * vNew
		local vx, vy, vz = vd[k + 1], vd[k + 2], vd[k + 3]
		for eid in mesh:VtxEdgesItr(vNew) do
			local ev0, ev1 = ed[4 * eid + 1], ed[4 * eid + 2]
			local othervid = (ev0 == vNew) and ev1 or ev0
			local o = 3 * othervid
			local dx, dy, dz = vx - vd[o + 1], vy - vd[o + 2], vz - vd[o + 3]
			if dx * dx + dy * dy + dz * dz > max_edge_len_sqr then
				edge_set_add(modified, eid)
			end
		end
		--queue_one_ring(vNew);
	end

	self.ModifiedEdgesLastPass = 0
	local edges_rc = mesh.edges_refcount
	local aborted = false
	for cur_eid in edgesItr do
		if self:Cancelled() then
			aborted = true
			break -- goto abort_compute
		end

		if edges_rc:isValid(cur_eid) then
			local result = self:ProcessEdge(cur_eid)
			if result == ProcessResult.Ok_Split then
				-- new edges queued by SplitF
				self.ModifiedEdgesLastPass += 1
				splitEdges += 1
			end
		end
	end
	if not aborted then
		self:end_ops()
	end

	-- abort_compute:
	self.SplitF = nil
	self:PopState()

	self:end_pass()

	return splitEdges
end

function RemesherPro:RemeshIteration()
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

	local edgesItr = self:begin_tracked_edges()

	self.ModifiedEdgesLastPass = 0
	local edges_rc = mesh.edges_refcount
	local ed = mesh.edges.data
	for cur_eid in edgesItr do
		if self:Cancelled() then
			return
		end

		if edges_rc:isValid(cur_eid) then
			-- ev = mesh.GetEdgeV(cur_eid); ov = mesh.GetEdgeOpposingV(cur_eid)
			local ei = 4 * cur_eid
			local eva, evb, t0, t1 = ed[ei + 1], ed[ei + 2], ed[ei + 3], ed[ei + 4]
			local ova = find_tri_other_vtx_dv(eva, evb, mesh.triangles, t0)
			local ovb = (t1 ~= -1) and find_tri_other_vtx_dv(eva, evb, mesh.triangles, t1) or -1

			-- TODO: optimize the queuing here, are over-doing it!
			-- TODO: be able to queue w/o flip (eg queue from smooth never requires flip check)

			local result = self:ProcessEdge(cur_eid)
			if result == ProcessResult.Ok_Collapsed or result == ProcessResult.Ok_Split or result == ProcessResult.Ok_Flipped then
				self:queue_one_ring(eva)
				self:queue_one_ring(evb)
				self:queue_one_ring(ova)
				self:queue_one_ring(ovb)
				self.ModifiedEdgesLastPass += 1
			end
		end
	end
	self:end_ops()

	if self:Cancelled() then
		return
	end

	self:begin_smooth()
	if self.EnableSmoothing and self.SmoothSpeedT > 0 then
		self:TrackedSmoothPass(self.EnableParallelSmooth)
		self:DoDebugChecks()
	end
	self:end_smooth()

	if self:Cancelled() then
		return
	end

	self:begin_project()
	if self:ProjectionTarget() ~= nil and self.ProjectionMode == TargetProjectionMode.AfterRefinement then
		--FullProjectionPass();

		if self.UseFaceAlignedProjection then
			for _ = 1, self.FaceProjectionPassesPerIteration do
				self:TrackedFaceProjectionPass()
			end
		else
			self:TrackedProjectionPass(self.EnableParallelProjection)
		end
		self:DoDebugChecks()
	end
	self:end_project()

	self:end_pass()
end

-- queue edges of vID whose length after moving vID to (nx,ny,nz) is outside [MinEdgeLength, MaxEdgeLength]
local function queue_changed_edges(self, vID: number, nx: number, ny: number, nz: number)
	local mesh = self.mesh
	local vd = mesh.vertices.data
	local ed = mesh.edges.data
	local MinEdgeLength, MaxEdgeLength = self.MinEdgeLength, self.MaxEdgeLength
	local modified = self.modified_edges
	for eid in mesh:VtxEdgesItr(vID) do
		local ev0, ev1 = ed[4 * eid + 1], ed[4 * eid + 2]
		local othervid = (ev0 == vID) and ev1 or ev0
		local o = 3 * othervid
		-- new_len = vNew.Distance(otherv)
		local dx, dy, dz = vd[o + 1] - nx, vd[o + 2] - ny, vd[o + 3] - nz
		local new_len = sqrt(dx * dx + dy * dy + dz * dz)
		if new_len < MinEdgeLength or new_len > MaxEdgeLength then
			edge_set_add(modified, eid)
		end
	end
end

function RemesherPro:TrackedSmoothPass(bParallel: boolean)
	self:InitializeVertexBufferForPass()

	local smoothFunc = self:get_smooth_func()
	local vBufferV, vModifiedV = self.vBufferV, self.vModifiedV
	local mesh = self.mesh

	if self:can_use_fast_uniform_smooth(smoothFunc) then
		-- same result as ComputeSmoothedVertexPos with MeshUtil.UniformSmooth
		local constraints = self.constraints
		local cv = constraints and constraints.Vertices
		local t = self.SmoothSpeedT
		local uniform_smooth_xyz = Remesher.uniform_smooth_xyz
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
			queue_changed_edges(self, vID, x, y, z)
		end
	else
		for vID in self:smooth_vertices() do
			local vSmoothed, bModified = self:ComputeSmoothedVertexPos(vID, smoothFunc)
			--if (vCur.EpsilonEqual(vSmoothed, MathUtil.ZeroTolerancef))
			--    bModified = false;
			if bModified then
				vModifiedV[vID + 1] = true
				local k = 3 * vID
				vBufferV[k + 1], vBufferV[k + 2], vBufferV[k + 3] = vSmoothed.x, vSmoothed.y, vSmoothed.z
				queue_changed_edges(self, vID, vSmoothed.x, vSmoothed.y, vSmoothed.z)
			end
		end
	end

	self:ApplyVertexBuffer(bParallel)
end

-- [TODO] projection pass
--   - only project vertices modified by smooth pass?
--   - and/or verts in set of modified edges?
function RemesherPro:TrackedProjectionPass(bParallel: boolean)
	self:InitializeVertexBufferForPass()

	local vBufferV, vModifiedV = self.vBufferV, self.vModifiedV
	local mesh = self.mesh
	local vd = mesh.vertices.data
	for vID in self:smooth_vertices() do
		local k = 3 * vID
		local cx, cy, cz = vd[k + 1], vd[k + 2], vd[k + 3]
		local vProjected, bModified = self:ComputeProjectedVertexPos(vID)
		-- vCur.EpsilonEqual(vProjected, MathUtil.ZeroTolerancef)
		if
			math.abs(cx - vProjected.x) <= ZeroTolerancef
			and math.abs(cy - vProjected.y) <= ZeroTolerancef
			and math.abs(cz - vProjected.z) <= ZeroTolerancef
		then
			bModified = false
		end
		if bModified then
			vModifiedV[vID + 1] = true
			vBufferV[k + 1], vBufferV[k + 2], vBufferV[k + 3] = vProjected.x, vProjected.y, vProjected.z
			queue_changed_edges(self, vID, vProjected.x, vProjected.y, vProjected.z)
		end
	end

	self:ApplyVertexBuffer(bParallel)
end

-- This computes projected position w/ proper constraints/etc.
-- Does not modify mesh.
-- C# ComputeProjectedVertexPos(vID, out bModified) -> projected, bModified
function RemesherPro:ComputeProjectedVertexPos(vID: number): (any, boolean)
	local mesh = self.mesh
	if self:vertex_is_constrained(vID) then
		return mesh:GetVertex(vID), false
	end
	if self.VertexControlF ~= nil and band(self.VertexControlF(vID), VC_NoProject) ~= 0 then
		return mesh:GetVertex(vID), false
	end

	local curpos = mesh:GetVertex(vID)
	local projected = self:ProjectionTarget():Project(curpos, vID)
	return projected, true
end

--
--  Implementation of face-aligned projection. Combined with rest of remesh
--  this is basically an RZN-flow-type algorithm.
--

function RemesherPro:InitializeBuffersForFacePass()
	Remesher.InitializeVertexBufferForPass(self)
	if self.vBufferVWeightsSize < self.vBufferVSize then
		local w = self.vBufferVWeights
		for i = self.vBufferVWeightsSize + 1, self.vBufferVSize do
			w[i] = 0
		end
		self.vBufferVWeightsSize = self.vBufferVSize
	end

	local NV = self.mesh:MaxVertexID()
	local vBufferV, vBufferVWeights = self.vBufferV, self.vBufferVWeights
	for i = 0, NV - 1 do
		local k = 3 * i
		vBufferV[k + 1], vBufferV[k + 2], vBufferV[k + 3] = 0, 0, 0
		vBufferVWeights[i + 1] = 0
	end
end

-- [TODO] projection pass
--   - only project vertices modified by smooth pass?
--   - and/or verts in set of modified edges?
function RemesherPro:TrackedFaceProjectionPass()
	local normalTarget = self:ProjectionTarget()
	if not is_oriented_target(normalTarget) then
		error("RemesherPro.TrackedFaceProjectionPass: projection target does not have normals!")
	end

	self:InitializeBuffersForFacePass()

	local mesh = self.mesh
	local vBufferV, vBufferVWeights, vModifiedV = self.vBufferV, self.vBufferVWeights, self.vModifiedV

	-- this function computes rotated position of triangle, such that it
	-- aligns with face normal on target surface. We accumulate weighted-average
	-- of vertex positions, which we will then use further down where possible.
	for tid in mesh:TriangleIndices() do
		local normal, area, centroid = mesh:GetTriInfo(tid)

		local projPos, projNormal = normalTarget:Project(centroid)

		local tv = mesh:GetTriangle(tid)
		local v0, v1, v2 = mesh:GetVertex(tv[1]), mesh:GetVertex(tv[2]), mesh:GetVertex(tv[3])

		-- ugh could probably do this more efficiently...
		local triF = Frame3d.FromZ(centroid, normal)
		v0 = triF:ToFrameP(v0)
		v1 = triF:ToFrameP(v1)
		v2 = triF:ToFrameP(v2)
		triF:AlignAxis(2, projNormal)
		triF.Origin = projPos
		v0 = triF:FromFrameP(v0)
		v1 = triF:FromFrameP(v1)
		v2 = triF:FromFrameP(v2)

		local dot = normal:Dot(projNormal)
		dot = MathUtil.Clamp(dot, 0, 1.0)
		local w = area * (dot * dot * dot)

		local ka, kb, kc = 3 * tv[1], 3 * tv[2], 3 * tv[3]
		vBufferV[ka + 1] += w * v0.x
		vBufferV[ka + 2] += w * v0.y
		vBufferV[ka + 3] += w * v0.z
		vBufferVWeights[tv[1] + 1] += w
		vBufferV[kb + 1] += w * v1.x
		vBufferV[kb + 2] += w * v1.y
		vBufferV[kb + 3] += w * v1.z
		vBufferVWeights[tv[2] + 1] += w
		vBufferV[kc + 1] += w * v2.x
		vBufferV[kc + 2] += w * v2.y
		vBufferV[kc + 3] += w * v2.z
		vBufferVWeights[tv[3] + 1] += w
	end

	-- ok now we filter out all the positions we can't change, as well as vertices that
	-- did not actually move. We also queue any edges that moved far enough to fall
	-- under min/max edge length thresholds
	local vd = mesh.vertices.data
	local VertexControlF = self.VertexControlF
	for vID in mesh:VertexIndices() do
		vModifiedV[vID + 1] = false
		local wsum = vBufferVWeights[vID + 1]
		if wsum < MathUtil.ZeroTolerance then
			continue
		end
		if self:vertex_is_constrained(vID) then
			continue
		end
		if VertexControlF ~= nil and band(VertexControlF(vID), VC_NoProject) ~= 0 then
			continue
		end

		local k = 3 * vID
		local cx, cy, cz = vd[k + 1], vd[k + 2], vd[k + 3]
		-- projPos = vBufferV[vID] / vBufferVWeights[vID]
		local px, py, pz = vBufferV[k + 1] / wsum, vBufferV[k + 2] / wsum, vBufferV[k + 3] / wsum
		if math.abs(cx - px) <= ZeroTolerancef and math.abs(cy - py) <= ZeroTolerancef and math.abs(cz - pz) <= ZeroTolerancef then
			continue
		end

		vModifiedV[vID + 1] = true
		vBufferV[k + 1], vBufferV[k + 2], vBufferV[k + 3] = px, py, pz

		if self.modified_edges ~= nil then
			queue_changed_edges(self, vID, px, py, pz)
		end
	end

	-- update vertices
	self:ApplyVertexBuffer(true)
end

-- settings stack ------------------------------------------------------------------------------------

function RemesherPro:PushState()
	table.insert(self.stateStack, {
		EnableFlips = self.EnableFlips,
		EnableCollapses = self.EnableCollapses,
		EnableSplits = self.EnableSplits,
		EnableSmoothing = self.EnableSmoothing,
		MinEdgeLength = self.MinEdgeLength,
		MaxEdgeLength = self.MaxEdgeLength,
		SmoothSpeedT = self.SmoothSpeedT,
		SmoothType = self.SmoothType,
		ProjectionMode = self.ProjectionMode,
	})
end

function RemesherPro:PopState()
	local s = table.remove(self.stateStack)
	self.EnableFlips = s.EnableFlips
	self.EnableCollapses = s.EnableCollapses
	self.EnableSplits = s.EnableSplits
	self.EnableSmoothing = s.EnableSmoothing
	self.MinEdgeLength = s.MinEdgeLength
	self.MaxEdgeLength = s.MaxEdgeLength
	self.SmoothSpeedT = s.SmoothSpeedT
	self.SmoothType = s.SmoothType
	self.ProjectionMode = s.ProjectionMode
end

return RemesherPro
