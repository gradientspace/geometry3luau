--!native
-- Port of geometry3Sharp mesh/Reducer.cs (class Reducer; struct QuadricError is in QuadricError.luau)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Mesh Simplication - implementation of Garland & Heckbert Quadric Error Metric (QEM) Simplification
--
-- Luau port notes (see PORTING.md):
--   * Reducer.new(mesh). Reducer inherits MeshRefinerBase (metatable inheritance); subclasses can
--     override the virtual methods (InitializeVertexQuadrics, InitializeQueue, UpdateNeighbours,
--     Reproject, Precompute, OnEdgeCollapse, CollapseEdge, FullProjectionPass, project_vertices, ...).
--   * Public options are fields: MinimizeQuadricPositionError, PreserveBoundaryShape, ProjectionMode
--     (Reducer.TargetProjectionMode), ENABLE_PROFILING, ENABLE_DEBUG_CHECKS, DebugEdges, Progress.
--   * SetProjectionTarget(target): target is an IProjectionTarget (duck typed):
--       target:Project(vPoint, identifier?) -> Vector3d   (an IOrientedProjectionTarget may return the
--       normal as a second value; it is ignored here)
--   * CollapseEdge(edgeID, vNewPos) -> ProcessResult, collapseToV   (C# out int collapseToV)
--   * FastCollapsePass(fMinEdgeLength, nRounds = 1, MeshIsClosedHint = false)
--   * Internal storage (protected members, for subclasses):
--       self.vertQuadrics   flat array, 10 coefficients per vertex: vertQuadrics[10*vid + k], k = 1..10
--                           (QuadricError field order Axx, Axy, Axz, Ayy, Ayz, Azz, bx, by, bz, c)
--       self.EdgeQuadrics   flat array, QEdge.q for each edge: EdgeQuadrics[10*eid + k]
--       self.EdgeCollapsePt flat array, QEdge.collapse_pt for each edge: EdgeCollapsePt[3*eid + k], k = 1..3
--       self.EdgeQueue      IndexPriorityQueue
--       self.IsBoundaryVtxCache  boolean array, IsBoundaryVtxCache[vid + 1]
--     Use GetVertexQuadric(vid) / GetEdgeQuadric(eid) to get QuadricError objects.
--   * OptimalPoint(eid, q, ea, eb) takes a QuadricError and returns a Vector3d, as in C#; the queue code
--     uses the equivalent flat-array version optimal_point_flat(eid, arr, offset, ea, eb) -> x, y, z.
--   * Edge-queue priorities are doubles (C# float), and the initial sorted queue insert breaks ties by
--     edge id (C# Array.Sort is unstable), so collapse order can differ from C# for (near-)equal errors.
--   * RuntimeDebugCheck (C# breaks into the debugger) prints via Util.DebugPrint when Util.DebugChecks is on.
--     DebugCheckVertexConstraints (ENABLE_DEBUG_CHECKS) errors where C# calls gBreakToDebugger.

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local Util = require(script.Parent.Util)
local MeshResult = require(script.Parent.MeshResult)
local MeshRefinerBase = require(script.Parent.MeshRefinerBase)
local MeshConstraints = require(script.Parent.MeshConstraints)
local IndexPriorityQueue = require(script.Parent.IndexPriorityQueue)
local QuadricError = require(script.Parent.QuadricError)

local EdgeConstraint = MeshConstraints.EdgeConstraint
local EdgeUnconstrained = EdgeConstraint.Unconstrained
local optimal_point_flat_q = QuadricError.OptimalPointFlat
local evaluate_flat = QuadricError.EvaluateFlat

local V3 = Vector3d.new
local sqrt = math.sqrt
local band = bit32.band
local Epsilon = MathUtil.Epsilon
local InvalidID = -1
local MeshResult_Ok = MeshResult.Ok

local Reducer = setmetatable({}, { __index = MeshRefinerBase })
Reducer.__index = Reducer
Reducer.ClassName = "Reducer"

-- if Target is set, we can project onto it in different ways
Reducer.TargetProjectionMode = table.freeze({
	NoProjection = 0, -- disable projection
	AfterRefinement = 1, -- do all projection after reducing
	Inline = 2, -- projection is computed before evaluating quadrics, so
	-- we are properly evaluating QEM error of result
})
local TargetProjectionMode = Reducer.TargetProjectionMode

Reducer.TargetModes = table.freeze({
	TriangleCount = 0,
	VertexCount = 1,
	MinEdgeLength = 2,
})
local TargetModes = Reducer.TargetModes

Reducer.ProcessResult = table.freeze({
	Ok_Collapsed = 0,
	Ignored_CannotCollapse = 1,
	Ignored_EdgeIsFullyConstrained = 2,
	Ignored_EdgeTooLong = 3,
	Ignored_Constrained = 4,
	Ignored_CreatesFlip = 5,
	Failed_OpNotSuccessful = 6,
	Failed_NotAnEdge = 7,
})
local ProcessResult = Reducer.ProcessResult

-- We are using a modulo-index loop to break symmetry/pathological conditions.
local nPrime = 31337 -- any prime will do...

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

function Reducer.init(self, m)
	MeshRefinerBase.init(self, m)
	self.target = nil

	-- if true, we try to find position for collapsed vertices that
	-- minimizes quadrice error. If false we just use midpoints.
	-- Note: using midpoints is *significantly* slower, because it results
	-- in may more points that would cause a triangle flip, which are then rejected.
	-- (Also results in more invalid collapses, not sure why though...)
	self.MinimizeQuadricPositionError = true

	-- if true, we try to keep boundary vertices on boundary. You probably want this.
	self.PreserveBoundaryShape = true

	-- [RMS] this is a debugging aid, will break to debugger if these edges are touched, in debug builds
	self.DebugEdges = {}

	self.ProjectionMode = TargetProjectionMode.AfterRefinement

	-- set to true to print profiling info to console
	self.ENABLE_PROFILING = false
	self.ENABLE_DEBUG_CHECKS = false

	self.MinEdgeLength = MathUtil.MaxDouble
	self.TargetCount = MathUtil.MaxInt
	self.ReduceMode = TargetModes.TriangleCount

	self.HaveBoundary = false
	self.IsBoundaryVtxCache = {}

	self.vertQuadrics = nil
	self.EdgeQuadrics = nil
	self.EdgeCollapsePt = nil
	self.EdgeQueue = nil

	self.nMaxEdgeID = 0
	self.COUNT_COLLAPSES = 0
	self.COUNT_ITERATIONS = 0
	return self
end

function Reducer.new(m)
	return Reducer.init(setmetatable({}, Reducer), m)
end

function Reducer:SetProjectionTarget(target)
	self.target = target
end

-- this just lets us write more concise tests below
function Reducer:EnableInlineProjection(): boolean
	return self.ProjectionMode == TargetProjectionMode.Inline
end

function Reducer:DoReduce()
	local mesh = self.mesh
	if mesh:TriangleCount() == 0 then -- badness if we don't catch this...
		return
	end

	self:begin_pass()

	self:begin_setup()
	self:Precompute()
	if self:Cancelled() then
		return
	end
	self:InitializeVertexQuadrics()
	if self:Cancelled() then
		return
	end
	self:InitializeQueue()
	if self:Cancelled() then
		return
	end
	self:end_setup()

	self:begin_ops()

	self:begin_collapse()
	local EdgeQueue = self.EdgeQueue
	local edges_rc = mesh.edges_refcount
	local bVertexMode = (self.ReduceMode == TargetModes.VertexCount)
	local TargetCount = self.TargetCount
	local hasProgress = self.Progress ~= nil
	while EdgeQueue.num_nodes > 0 do
		-- termination criteria
		if bVertexMode then
			if mesh:VertexCount() <= TargetCount then
				break
			end
		else
			if mesh:TriangleCount() <= TargetCount then
				break
			end
		end

		self.COUNT_ITERATIONS += 1
		local eid = EdgeQueue:Dequeue()
		if not edges_rc:isValid(eid) then
			continue
		end
		if hasProgress and self:Cancelled() then
			return
		end

		local ept = self.EdgeCollapsePt
		local pi = 3 * eid
		local result, vKept = self:CollapseEdge(eid, V3(ept[pi + 1], ept[pi + 2], ept[pi + 3]))
		if result == ProcessResult.Ok_Collapsed then
			-- vertQuadrics[vKept] = EdgeQuadrics[eid].q
			local vq, eq = self.vertQuadrics, self.EdgeQuadrics
			local ov, oe = 10 * vKept, 10 * eid
			for k = 1, 10 do
				vq[ov + k] = eq[oe + k]
			end
			self:UpdateNeighbours(vKept)
		end
	end
	self:end_collapse()
	self:end_ops()

	if self:Cancelled() then
		return
	end

	self:Reproject()

	self:end_pass()
end

function Reducer:ReduceToTriangleCount(nCount: number)
	self.ReduceMode = TargetModes.TriangleCount
	self.TargetCount = math.max(1, nCount)
	self.MinEdgeLength = MathUtil.MaxDouble
	self:DoReduce()
end

function Reducer:ReduceToVertexCount(nCount: number)
	self.ReduceMode = TargetModes.VertexCount
	self.TargetCount = math.max(3, nCount)
	self.MinEdgeLength = MathUtil.MaxDouble
	self:DoReduce()
end

function Reducer:ReduceToEdgeLength(minEdgeLen: number)
	self.ReduceMode = TargetModes.MinEdgeLength
	self.TargetCount = 1
	self.MinEdgeLength = minEdgeLen
	self:DoReduce()
end

function Reducer:FastCollapsePass(fMinEdgeLength: number, nRounds: number?, MeshIsClosedHint: boolean?)
	local mesh = self.mesh
	if mesh:TriangleCount() == 0 then -- badness if we don't catch this...
		return
	end
	local rounds = nRounds or 1

	self.MinEdgeLength = fMinEdgeLength
	local min_sqr = self.MinEdgeLength * self.MinEdgeLength

	-- we don't collapse on the boundary
	self.HaveBoundary = false

	self:begin_pass()

	self:begin_setup()
	self:Precompute(MeshIsClosedHint == true)
	if self:Cancelled() then
		return
	end
	self:end_setup()

	self:begin_ops()

	self:begin_collapse()

	local N = mesh:MaxEdgeID()
	local num_last_pass = 0
	local vd = mesh.vertices.data
	local ed = mesh.edges.data
	local edges_rc = mesh.edges_refcount
	for _ = 0, rounds - 1 do
		num_last_pass = 0

		for eid = 0, N - 1 do
			if not edges_rc:isValid(eid) then
				continue
			end
			if ed[4 * eid + 4] == InvalidID then -- IsBoundaryEdge
				continue
			end
			if self:Cancelled() then
				return
			end

			local ia, ib = 3 * ed[4 * eid + 1], 3 * ed[4 * eid + 2]
			local ax, ay, az = vd[ia + 1], vd[ia + 2], vd[ia + 3]
			local bx, by, bz = vd[ib + 1], vd[ib + 2], vd[ib + 3]
			local dx, dy, dz = bx - ax, by - ay, bz - az
			if dx * dx + dy * dy + dz * dz > min_sqr then
				continue
			end

			self.COUNT_ITERATIONS += 1

			-- midpoint = (va + vb) * 0.5
			local midpoint = V3((ax + bx) * 0.5, (ay + by) * 0.5, (az + bz) * 0.5)
			local result = self:CollapseEdge(eid, midpoint)
			if result == ProcessResult.Ok_Collapsed then
				num_last_pass += 1
			end
		end

		if num_last_pass == 0 then -- converged
			break
		end
	end
	self:end_collapse()
	self:end_ops()

	if self:Cancelled() then
		return
	end

	self:Reproject()

	self:end_pass()
end

function Reducer:InitializeVertexQuadrics()
	local mesh = self.mesh
	local NT = mesh:MaxTriangleID()
	-- triQuadrics[10*tid + k] are the per-triangle plane quadrics, triAreas[tid + 1] the areas
	local triQuadrics = table.create(10 * NT, 0)
	local triAreas = table.create(NT, 0)
	local tris_rc = mesh.triangles_refcount
	for tid = 0, NT - 1 do
		if tris_rc:isValid(tid) then
			-- mesh.GetTriInfo(tid, out n, out triAreas[tid], out c)
			local ax, ay, az, bx, by, bz, cx, cy, cz = mesh:GetTriVerticesXYZ(tid)
			local third = 1.0 / 3.0
			local px, py, pz = third * (ax + bx + cx), third * (ay + by + cy), third * (az + bz + cz)
			local e1x, e1y, e1z = bx - ax, by - ay, bz - az
			local e2x, e2y, e2z = cx - ax, cy - ay, cz - az
			local nx, ny, nz = e1y * e2z - e1z * e2y, e1z * e2x - e1x * e2z, e1x * e2y - e1y * e2x
			local len = sqrt(nx * nx + ny * ny + nz * nz)
			if len > Epsilon then
				local s = 1.0 / len
				nx, ny, nz = nx * s, ny * s, nz * s
			else
				nx, ny, nz, len = 0, 0, 0, 0
			end
			triAreas[tid + 1] = 0.5 * len
			-- triQuadrics[tid] = new QuadricError(ref n, ref c)
			local Axx, Axy, Axz = nx * nx, nx * ny, nx * nz
			local Ayy, Ayz, Azz = ny * ny, ny * nz, nz * nz
			local vx = Axx * px + Axy * py + Axz * pz
			local vy = Axy * px + Ayy * py + Ayz * pz
			local vz = Axz * px + Ayz * py + Azz * pz
			local o = 10 * tid
			triQuadrics[o + 1], triQuadrics[o + 2], triQuadrics[o + 3] = Axx, Axy, Axz
			triQuadrics[o + 4], triQuadrics[o + 5], triQuadrics[o + 6] = Ayy, Ayz, Azz
			triQuadrics[o + 7], triQuadrics[o + 8], triQuadrics[o + 9] = -vx, -vy, -vz
			triQuadrics[o + 10] = px * vx + py * vy + pz * vz
		end
	end

	local NV = mesh:MaxVertexID()
	local vq = table.create(10 * NV, 0)
	self.vertQuadrics = vq
	local verts_rc = mesh.vertices_refcount
	local gather_vtx_triangles = MeshRefinerBase.gather_vtx_triangles
	local tbuf = {}
	for vid = 0, NV - 1 do
		if verts_rc:isValid(vid) then
			local o = 10 * vid
			local nt = gather_vtx_triangles(mesh, vid, tbuf)
			for ti = 1, nt do
				local tid = tbuf[ti]
				-- vertQuadrics[vid].Add(triAreas[tid], ref triQuadrics[tid])
				local w = triAreas[tid + 1]
				local t = 10 * tid
				for k = 1, 10 do
					vq[o + k] += w * triQuadrics[t + k]
				end
			end
		end
	end
end

-- compute QEdge for eid=[a,b]: stores Q = vq[a] + vq[b] into EdgeQuadrics, the optimal point into
-- EdgeCollapsePt, and returns the error Q.Evaluate(opt)
local function compute_qedge(self, eid: number, a: number, b: number): number
	local vq, eq, ept = self.vertQuadrics, self.EdgeQuadrics, self.EdgeCollapsePt
	local o, oa, ob = 10 * eid, 10 * a, 10 * b
	for k = 1, 10 do
		eq[o + k] = vq[oa + k] + vq[ob + k]
	end
	local x, y, z = self:optimal_point_flat(eid, eq, o, a, b)
	local pi = 3 * eid
	ept[pi + 1], ept[pi + 2], ept[pi + 3] = x, y, z
	return evaluate_flat(eq, o, x, y, z)
end

function Reducer:InitializeQueue()
	local mesh = self.mesh
	local MaxEID = mesh:MaxEdgeID()

	self.EdgeQuadrics = table.create(10 * MaxEID, 0)
	self.EdgeCollapsePt = table.create(3 * MaxEID, 0)
	self.EdgeQueue = IndexPriorityQueue.new(MaxEID)
	local edgeErrors = table.create(MaxEID, 0)

	local ed = mesh.edges.data
	local edges_rc = mesh.edges_refcount
	local indices = {}
	for eid = 0, MaxEID - 1 do
		if edges_rc:isValid(eid) then
			local a, b = ed[4 * eid + 1], ed[4 * eid + 2]
			edgeErrors[eid + 1] = compute_qedge(self, eid, a, b)
			table.insert(indices, eid)
		end
	end

	-- sorted pq insert is faster, so sort edge errors array and index map
	-- (C# Array.Sort is unstable; here ties are broken by edge id so the result is deterministic)
	table.sort(indices, function(i, j)
		local ei, ej = edgeErrors[i + 1], edgeErrors[j + 1]
		if ei ~= ej then
			return ei < ej
		end
		return i < j
	end)

	-- now do inserts
	local EdgeQueue = self.EdgeQueue
	for i = 1, #indices do
		local eid = indices[i]
		EdgeQueue:Insert(eid, edgeErrors[eid + 1])
	end
end

-- return point that minimizes quadric error for edge [ea,eb]  (flat-array version: quadric at q[o+1..o+10])
function Reducer:optimal_point_flat(eid: number, q: { number }, o: number, ea: number, eb: number): (number, number, number)
	local mesh = self.mesh
	local vd = mesh.vertices.data
	local ia, ib = 3 * ea, 3 * eb

	-- if we would like to preserve boundary, we need to know that here
	-- so that we properly score these edges
	if self.HaveBoundary and self.PreserveBoundaryShape then
		if mesh.edges.data[4 * eid + 4] == InvalidID then -- mesh.IsBoundaryEdge(eid)
			return (vd[ia + 1] + vd[ib + 1]) * 0.5, (vd[ia + 2] + vd[ib + 2]) * 0.5, (vd[ia + 3] + vd[ib + 3]) * 0.5
		else
			local cache = self.IsBoundaryVtxCache
			if cache[ea + 1] then
				return vd[ia + 1], vd[ia + 2], vd[ia + 3]
			elseif cache[eb + 1] then
				return vd[ib + 1], vd[ib + 2], vd[ib + 3]
			end
		end
	end

	-- [TODO] if we have constraints, we should apply them here, for same reason as bdry above...

	if self.MinimizeQuadricPositionError == false then
		return self:project_xyz((vd[ia + 1] + vd[ib + 1]) * 0.5, (vd[ia + 2] + vd[ib + 2]) * 0.5, (vd[ia + 3] + vd[ib + 3]) * 0.5)
	else
		local ok, rx, ry, rz = optimal_point_flat_q(q, o)
		if ok then
			return self:project_xyz(rx, ry, rz)
		end

		-- degenerate matrix, evaluate quadric at edge end and midpoints
		-- (could do line search here...)
		local ax, ay, az = vd[ia + 1], vd[ia + 2], vd[ia + 3]
		local bx, by, bz = vd[ib + 1], vd[ib + 2], vd[ib + 3]
		local cx, cy, cz = self:project_xyz((ax + bx) * 0.5, (ay + by) * 0.5, (az + bz) * 0.5)
		local fa = evaluate_flat(q, o, ax, ay, az)
		local fb = evaluate_flat(q, o, bx, by, bz)
		local fc = evaluate_flat(q, o, cx, cy, cz)
		local m = MathUtil.Min(fa, fb, fc)
		if m == fa then
			return ax, ay, az
		elseif m == fb then
			return bx, by, bz
		end
		return cx, cy, cz
	end
end

-- return point that minimizes quadric error for edge [ea,eb]
-- C# protected Vector3d OptimalPoint(int eid, ref QuadricError q, int ea, int eb)
function Reducer:OptimalPoint(eid: number, q, ea: number, eb: number)
	local arr = table.create(10, 0)
	QuadricError.ToArray(q, arr, 0)
	return V3(self:optimal_point_flat(eid, arr, 0, ea, eb))
end

function Reducer:project_xyz(x: number, y: number, z: number): (number, number, number)
	if self.ProjectionMode == TargetProjectionMode.Inline and self.target ~= nil then
		local p = self.target:Project(V3(x, y, z))
		return p.x, p.y, p.z
	end
	return x, y, z
end

function Reducer:project(pos)
	if self.ProjectionMode == TargetProjectionMode.Inline and self.target ~= nil then
		return (self.target:Project(pos))
	end
	return pos
end

-- update queue weight for each edge in vertex one-ring
function Reducer:UpdateNeighbours(vid: number)
	local mesh = self.mesh
	local ed = mesh.edges.data
	local EdgeQueue = self.EdgeQueue
	for eid in mesh:VtxEdgesItr(vid) do
		local a, b = ed[4 * eid + 1], ed[4 * eid + 2]
		local err = compute_qedge(self, eid, a, b)
		if EdgeQueue:Contains(eid) then
			EdgeQueue:Update(eid, err)
		else
			EdgeQueue:Insert(eid, err)
		end
	end
end

function Reducer:Reproject()
	self:begin_project()
	if self.target ~= nil and self.ProjectionMode == TargetProjectionMode.AfterRefinement then
		self:FullProjectionPass()
		self:DoDebugChecks()
	end
	self:end_project()
end

function Reducer:Precompute(bMeshIsClosed: boolean?)
	local mesh = self.mesh
	self.HaveBoundary = false
	local cache = table.create(mesh:MaxVertexID(), false)
	self.IsBoundaryVtxCache = cache
	if bMeshIsClosed ~= true then
		for eid in mesh:BoundaryEdgeIndices() do
			local ev = mesh:GetEdgeV(eid)
			cache[ev[1] + 1] = true
			cache[ev[2] + 1] = true
			self.HaveBoundary = true
		end
	end
end

function Reducer:IsBoundaryV(vid: number): boolean
	return self.IsBoundaryVtxCache[vid + 1] == true
end

-- subclasses can override these to implement custom behavior...
function Reducer:OnEdgeCollapse(edgeID: number, va: number, vb: number, collapseInfo)
	-- this is for subclasses...
end

-- start_edges() and next_edge() control the iteration over edges that will be refined.
-- Default here is to iterate over entire mesh.
-- Subclasses can override these two functions to restrict the affected edges (eg EdgeLoopRemesher)
function Reducer:start_edges(): number
	self.nMaxEdgeID = self.mesh:MaxEdgeID()
	return 0
end

-- C# next_edge(cur_eid, out bDone) -> new_eid, bDone
function Reducer:next_edge(cur_eid: number): (number, boolean)
	local new_eid = (cur_eid + nPrime) % self.nMaxEdgeID
	return new_eid, (new_eid == 0)
end

function Reducer:project_vertices(): () -> number?
	return self.mesh:VertexIndices()
end

-- C# CollapseEdge(int edgeID, Vector3d vNewPos, out int collapseToV) -> ProcessResult, collapseToV
function Reducer:CollapseEdge(edgeID: number, vNewPos): (number, number)
	local collapseToV = InvalidID
	self:RuntimeDebugCheck(edgeID)

	local mesh = self.mesh
	local constraints = self.constraints
	local constraint = (constraints == nil) and EdgeUnconstrained or constraints:GetEdgeConstraint(edgeID)
	local rflags = constraint.refineFlags
	if band(rflags, 7) == 7 then -- constraint.NoModifications
		return ProcessResult.Ignored_EdgeIsFullyConstrained, collapseToV
	end
	if band(rflags, 4) ~= 0 then -- constraint.CanCollapse == false
		return ProcessResult.Ignored_EdgeIsFullyConstrained, collapseToV
	end

	-- look up verts and tris for this edge
	if not mesh.edges_refcount:isValid(edgeID) then
		return ProcessResult.Failed_NotAnEdge, collapseToV
	end
	local ed = mesh.edges.data
	local ei = 4 * edgeID
	local a, b, t0, t1 = ed[ei + 1], ed[ei + 2], ed[ei + 3], ed[ei + 4]
	local bIsBoundaryEdge = (t1 == InvalidID)

	-- look up 'other' verts c (from t0) and d (from t1, if it exists)
	local tdata = mesh.triangles.data
	local c, d
	do
		local ti = 3 * t0
		local x, y, z = tdata[ti + 1], tdata[ti + 2], tdata[ti + 3]
		-- IndexUtil.find_tri_other_vtx(a, b, T0tv)
		if (x == a and y == b) or (x == b and y == a) then
			c = z
		elseif (y == a and z == b) or (y == b and z == a) then
			c = x
		elseif (z == a and x == b) or (z == b and x == a) then
			c = y
		else
			c = InvalidID
		end
	end
	if bIsBoundaryEdge then
		d = InvalidID
	else
		local ti = 3 * t1
		local x, y, z = tdata[ti + 1], tdata[ti + 2], tdata[ti + 3]
		if (x == a and y == b) or (x == b and y == a) then
			d = z
		elseif (y == a and z == b) or (y == b and z == a) then
			d = x
		elseif (z == a and x == b) or (z == b and x == a) then
			d = y
		else
			d = InvalidID
		end
	end

	local vd = mesh.vertices.data
	local ia, ib = 3 * a, 3 * b
	local ax, ay, az = vd[ia + 1], vd[ia + 2], vd[ia + 3]
	local bx, by, bz = vd[ib + 1], vd[ib + 2], vd[ib + 3]
	local dx, dy, dz = ax - bx, ay - by, az - bz
	local edge_len_sqr = dx * dx + dy * dy + dz * dz
	local MinEdgeLength = self.MinEdgeLength
	if edge_len_sqr > MinEdgeLength * MinEdgeLength then
		return ProcessResult.Ignored_EdgeTooLong, collapseToV
	end

	self:begin_collapse()

	-- check if we should collapse, and also find which vertex we should collapse to,
	-- in cases where we have constraints/etc
	local collapse_to = -1
	local bCanCollapse
	bCanCollapse, collapse_to = self:can_collapse_constraints(edgeID, a, b, c, d, t0, t1)
	if bCanCollapse == false then
		return ProcessResult.Ignored_Constrained, collapseToV
	end

	-- if we have a boundary, we want to collapse to boundary
	if self.PreserveBoundaryShape and self.HaveBoundary then
		local cache = self.IsBoundaryVtxCache
		local bBdryA, bBdryB = cache[a + 1] == true, cache[b + 1] == true
		if collapse_to ~= -1 then
			if (bBdryB and collapse_to ~= b) or (bBdryA and collapse_to ~= a) then
				return ProcessResult.Ignored_Constrained, collapseToV
			end
		end
		if bBdryB then
			collapse_to = b
		elseif bBdryA then
			collapse_to = a
		end
	end

	-- optimization: if edge cd exists, we cannot collapse or flip. look that up here?
	--  funcs will do it internally...
	--  (or maybe we can collapse if cd exists? edge-collapse doesn't check for it explicitly...)
	local retVal = ProcessResult.Failed_OpNotSuccessful

	local iKeep, iCollapse = b, a
	local nx, ny, nz

	-- if either vtx is fixed, collapse to that position
	if collapse_to == b then
		nx, ny, nz = bx, by, bz
	elseif collapse_to == a then
		iKeep, iCollapse = a, b
		nx, ny, nz = ax, ay, az
	else
		local p = self:get_projected_collapse_position(iKeep, vNewPos)
		nx, ny, nz = p.x, p.y, p.z
	end

	-- check if this collapse will create a normal flip. Also checks
	-- for invalid collapse nbrhood, since we are doing one-ring iter anyway.
	-- [TODO] could we skip this one-ring check in CollapseEdge? pass in hints?
	if
		self:collapse_creates_flip_or_invalid_xyz(a, b, nx, ny, nz, t0, t1)
		or self:collapse_creates_flip_or_invalid_xyz(b, a, nx, ny, nz, t0, t1)
	then
		retVal = ProcessResult.Ignored_CreatesFlip
	else
		-- lots of cases where we cannot collapse, but we should just let
		-- mesh sort that out, right?
		self.COUNT_COLLAPSES += 1
		local result, collapseInfo = mesh:CollapseEdge(iKeep, iCollapse)
		if result == MeshResult_Ok then
			collapseToV = iKeep
			mesh:SetVertexXYZ(iKeep, nx, ny, nz)
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

			retVal = ProcessResult.Ok_Collapsed
		end
	end

	self:end_collapse()
	return retVal, collapseToV
end

function Reducer:project_vertex(vID: number, targetIn)
	local mesh = self.mesh
	local curpos = mesh:GetVertex(vID)
	local projected = targetIn:Project(curpos, vID)
	mesh:SetVertex(vID, projected)
end

-- used by collapse-edge to get projected position for new vertex
function Reducer:get_projected_collapse_position(vid: number, vNewPos)
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

	-- we don't need to do inline projection to target surface here because we
	-- already did it in OptimalPoint()

	return vNewPos
end

-- we can do projection in parallel if we have .net
function Reducer:FullProjectionPass()
	local mesh = self.mesh
	local target = self.target
	for vID in self:project_vertices() do
		if self:vertex_is_constrained(vID) then
			continue
		end
		local curpos = mesh:GetVertex(vID)
		local projected = target:Project(curpos, vID)
		mesh:SetVertex(vID, projected)
	end
end

-- [Conditional("DEBUG")]
function Reducer:RuntimeDebugCheck(eid: number)
	if Util.DebugChecks and #self.DebugEdges > 0 and table.find(self.DebugEdges, eid) ~= nil then
		Util.DebugPrint("Reducer: touched debug edge " .. eid)
	end
end

function Reducer:DoDebugChecks()
	if self.ENABLE_DEBUG_CHECKS == false then
		return
	end
	self:DebugCheckVertexConstraints()
end

function Reducer:DebugCheckVertexConstraints()
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
				error("Reducer.DebugCheckVertexConstraints: vertex " .. vid .. " is not on its constraint target")
			end
		end
	end
end

-- QuadricError accessors for the flat storage
function Reducer:GetVertexQuadric(vid: number)
	return QuadricError.FromArray(self.vertQuadrics, 10 * vid)
end
function Reducer:GetEdgeQuadric(eid: number)
	return QuadricError.FromArray(self.EdgeQuadrics, 10 * eid)
end

--
-- profiling functions, turn on ENABLE_PROFILING to see output in console
--
function Reducer:begin_pass()
	if self.ENABLE_PROFILING then
		self.COUNT_COLLAPSES = 0
		self.COUNT_ITERATIONS = 0
		self.AllOpsW = stopwatch()
		self.SetupW = stopwatch()
		self.ProjectW = stopwatch()
		self.CollapseW = stopwatch()
	end
end

function Reducer:end_pass()
	if self.ENABLE_PROFILING then
		local mesh = self.mesh
		print(
			string.format(
				"ReducePass: T %d V %d collapses %d  iterations %d",
				mesh:TriangleCount(),
				mesh:VertexCount(),
				self.COUNT_COLLAPSES,
				self.COUNT_ITERATIONS
			)
		)
		print(
			string.format(
				"           Timing1: setup %.3f ops %.3f project %.3f",
				self.SetupW.elapsed,
				self.AllOpsW.elapsed,
				self.ProjectW.elapsed
			)
		)
	end
end

function Reducer:begin_ops()
	if self.ENABLE_PROFILING then
		sw_start(self.AllOpsW)
	end
end
function Reducer:end_ops()
	if self.ENABLE_PROFILING then
		sw_stop(self.AllOpsW)
	end
end
function Reducer:begin_setup()
	if self.ENABLE_PROFILING then
		sw_start(self.SetupW)
	end
end
function Reducer:end_setup()
	if self.ENABLE_PROFILING then
		sw_stop(self.SetupW)
	end
end
function Reducer:begin_project()
	if self.ENABLE_PROFILING then
		sw_start(self.ProjectW)
	end
end
function Reducer:end_project()
	if self.ENABLE_PROFILING then
		sw_stop(self.ProjectW)
	end
end
function Reducer:begin_collapse()
	if self.ENABLE_PROFILING then
		sw_start(self.CollapseW)
	end
end
function Reducer:end_collapse()
	if self.ENABLE_PROFILING then
		sw_stop(self.CollapseW)
	end
end

return Reducer
