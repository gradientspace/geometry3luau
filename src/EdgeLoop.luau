-- Port of geometry3Sharp mesh/EdgeLoop.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Sequential set of vertices/edges in a mesh, that form a closed loop.
-- If all you have are the vertices, use EdgeLoop.VertexLoopToEdgeLoop() to construct an EdgeLoop
--
-- Luau port notes (see PORTING.md):
--   * Vertices, Edges, BowtieVertices are 1-based Luau arrays of 0-based mesh ids (C# int[]).
--     Positions/indices into these arrays used by the API (GetVertex(i), FindVertexIndex, ...) are
--     0-based like C#.
--   * Constructors:
--       EdgeLoop.new(mesh)                                    new EdgeLoop(DMesh3)
--       EdgeLoop.FromArrays(mesh, vertices, edges, bCopyArrays) new EdgeLoop(mesh, int[], int[], bool)
--       EdgeLoop.FromCopy(copy)                               new EdgeLoop(EdgeLoop copy)
--       EdgeLoop.FromEdges(mesh, edges)
--       EdgeLoop.FromVertices(mesh, vertices, bAutoOrient?)   C# has two overloads; the 2-argument call
--           (bAutoOrient == nil) is the overload without auto-orientation (C# overload resolution picks
--           that one for FromVertices(mesh, list)); pass true to auto-orient.
--   * VertexCount()/EdgeCount() are methods.
--   * CountWithinTolerance(v, tol) -> count, last_in_tol

local Vector3d = require(script.Parent.Vector3d)
local AxisAlignedBox3d = require(script.Parent.AxisAlignedBox3d)
local IndexUtil = require(script.Parent.IndexUtil)
local MathUtil = require(script.Parent.MathUtil)
local Util = require(script.Parent.Util)

local InvalidID = -1
local FailMode = Util.FailMode

local EdgeLoop = {}
EdgeLoop.__index = EdgeLoop
EdgeLoop.ClassName = "EdgeLoop"

local function reverse(a: { number })
	local n = #a
	for i = 1, n // 2 do
		a[i], a[n - i + 1] = a[n - i + 1], a[i]
	end
end

function EdgeLoop.new(mesh)
	return setmetatable({ Mesh = mesh, Vertices = nil, Edges = nil, BowtieVertices = nil }, EdgeLoop)
end

function EdgeLoop.FromArrays(mesh, vertices: { number }, edges: { number }, bCopyArrays: boolean)
	local loop = EdgeLoop.new(mesh)
	if bCopyArrays then
		loop.Vertices = table.clone(vertices)
		loop.Edges = table.clone(edges)
	else
		loop.Vertices = vertices
		loop.Edges = edges
	end
	return loop
end

function EdgeLoop.FromCopy(copy)
	local loop = EdgeLoop.new(copy.Mesh)
	loop.Vertices = table.clone(copy.Vertices)
	loop.Edges = table.clone(copy.Edges)
	if copy.BowtieVertices ~= nil then
		loop.BowtieVertices = table.clone(copy.BowtieVertices)
	end
	return loop
end

-- construct EdgeLoop from a list of edges of mesh
function EdgeLoop.FromEdges(mesh, edges: { number })
	local Edges = table.clone(edges)
	local N = #Edges
	local Vertices = table.create(N, 0)
	local start_ev = mesh:GetEdgeV(Edges[1])
	local prev_ev = start_ev
	for i = 1, N - 1 do
		local next_ev = mesh:GetEdgeV(Edges[(i % N) + 1])
		Vertices[i + 1] = IndexUtil.find_shared_edge_v(prev_ev, next_ev)
		prev_ev = next_ev
	end
	Vertices[1] = IndexUtil.find_edge_other_v(start_ev, Vertices[2])
	return EdgeLoop.FromArrays(mesh, Vertices, Edges, false)
end

-- construct EdgeLoop from a list of vertices of mesh.
-- If bAutoOrient is true and the loop is a boundary loop, orientation is corrected.
function EdgeLoop.FromVertices(mesh, vertices: { number }, bAutoOrient: boolean?)
	local Vertices = table.clone(vertices)
	local NV = #Vertices
	if bAutoOrient == nil then
		-- C# FromVertices(DMesh3, IList<int>)
		local Edges = table.create(NV, 0)
		for i = 0, NV - 1 do
			Edges[i + 1] = mesh:FindEdge(Vertices[i + 1], Vertices[((i + 1) % NV) + 1])
			if Edges[i + 1] == InvalidID then
				error("EdgeLoop.FromVertices: vertices are not connected by edge!")
			end
		end
		return EdgeLoop.FromArrays(mesh, Vertices, Edges, false)
	end

	if bAutoOrient then
		local a, b = Vertices[1], Vertices[2]
		local eid = mesh:FindEdge(a, b)
		if mesh:IsBoundaryEdge(eid) then
			local ev = mesh:GetOrientedBoundaryEdgeV(eid)
			if ev[1] == b and ev[2] == a then
				reverse(Vertices)
			end
		end
	end

	local Edges = table.create(NV, 0)
	for i = 0, NV - 1 do
		local a, b = Vertices[i + 1], Vertices[((i + 1) % NV) + 1]
		Edges[i + 1] = mesh:FindEdge(a, b)
		if Edges[i + 1] == InvalidID then
			error("EdgeLoop.FromVertices: invalid edge [" .. a .. "," .. b .. "]")
		end
	end
	return EdgeLoop.FromArrays(mesh, Vertices, Edges, false)
end

function EdgeLoop:VertexCount(): number
	return #self.Vertices
end
function EdgeLoop:EdgeCount(): number
	return #self.Edges
end

function EdgeLoop:GetVertex(i: number)
	return self.Mesh:GetVertex(self.Vertices[i + 1])
end

function EdgeLoop:GetBounds()
	local box = AxisAlignedBox3d.Empty()
	for i = 1, #self.Vertices do
		box:Contain(self.Mesh:GetVertex(self.Vertices[i]))
	end
	return box
end

function EdgeLoop:ToCurve(sourceMesh)
	local MeshUtil = require(script.Parent.MeshUtil)
	if sourceMesh == nil then
		sourceMesh = self.Mesh
	end
	local curve = MeshUtil.ExtractLoopV(sourceMesh, self.Vertices)
	curve.Closed = true
	return curve
end

-- if this is a border edge-loop, we can check that it is oriented correctly, and
-- if not, reverse it. Returns true if we reversed orientation.
function EdgeLoop:CorrectOrientation(): boolean
	local a, b = self.Vertices[1], self.Vertices[2]
	local eid = self.Mesh:FindEdge(a, b)
	if self.Mesh:IsBoundaryEdge(eid) then
		local ev = self.Mesh:GetOrientedBoundaryEdgeV(eid)
		if ev[1] == b and ev[2] == a then
			self:Reverse()
			return true
		end
	end
	return false
end

function EdgeLoop:Reverse()
	reverse(self.Vertices)
	reverse(self.Edges)
end

-- check if all edges of this loop are internal edges (ie none on boundary)
function EdgeLoop:IsInternalLoop(): boolean
	local V = self.Vertices
	local NV = #V
	for i = 0, NV - 1 do
		local eid = self.Mesh:FindEdge(V[i + 1], V[((i + 1) % NV) + 1])
		Util.gDevAssert(eid ~= InvalidID)
		if self.Mesh:IsBoundaryEdge(eid) then
			return false
		end
	end
	return true
end

-- Check if all edges of this loop are boundary edges.
-- If testMesh != null, will check that mesh instead of internal Mesh
function EdgeLoop:IsBoundaryLoop(testMesh): boolean
	local useMesh = (testMesh ~= nil) and testMesh or self.Mesh
	local V = self.Vertices
	local NV = #V
	for i = 0, NV - 1 do
		local eid = useMesh:FindEdge(V[i + 1], V[((i + 1) % NV) + 1])
		Util.gDevAssert(eid ~= InvalidID)
		if useMesh:IsBoundaryEdge(eid) == false then
			return false
		end
	end
	return true
end

-- find index of vertex vID in Vertices list, or -1 if not found
function EdgeLoop:FindVertexIndex(vID: number): number
	local V = self.Vertices
	for i = 1, #V do
		if V[i] == vID then
			return i - 1
		end
	end
	return -1
end

-- find index of vertices of loop that is closest to point v
function EdgeLoop:FindNearestVertex(v): number
	local iNear = -1
	local fNearSqr = MathUtil.MaxDouble
	local V = self.Vertices
	for i = 0, #V - 1 do
		local lv = self.Mesh:GetVertex(V[i + 1])
		local d2 = v:DistanceSquared(lv)
		if d2 < fNearSqr then
			fNearSqr = d2
			iNear = i
		end
	end
	return iNear
end

-- count # of vertices in loop that are within tol of v
-- C# CountWithinTolerance(v, tol, out last_in_tol) -> returns count, last_in_tol
function EdgeLoop:CountWithinTolerance(v, tol: number): (number, number)
	local last_in_tol = -1
	local count = 0
	local V = self.Vertices
	for i = 0, #V - 1 do
		local lv = self.Mesh:GetVertex(V[i + 1])
		if v:Distance(lv) < tol then
			count += 1
			last_in_tol = i
		end
	end
	return count, last_in_tol
end

-- Check if Loop2 is the same set of positions on another mesh.
-- Does not require the indexing to be the same
-- Currently doesn't handle loop-reversal
function EdgeLoop:IsSameLoop(Loop2, bReverse2: boolean?, tolerance: number?): boolean
	local tol = tolerance or MathUtil.ZeroTolerance
	-- find a duplicate starting vertex
	local N = #self.Vertices
	local N2 = #Loop2.Vertices
	if N ~= N2 then
		return false
	end

	local Mesh2 = Loop2.Mesh

	local start_i, start_j = 0, -1

	-- try to find a unique same-vertex on each loop. Do not
	-- use vertices that have duplicate positions.
	local bFoundGoodStart = false
	while not bFoundGoodStart and start_i < N do
		-- [RMS] note: C# uses Mesh.GetVertex(start_i) here (vertex id, not loop index); kept as-is
		local start_v = self.Mesh:GetVertex(start_i)
		local count
		count, start_j = Loop2:CountWithinTolerance(start_v, tol)
		if count == 1 then
			bFoundGoodStart = true
		else
			start_i += 1
		end
	end
	if not bFoundGoodStart then
		return false -- no within-tolerance duplicate vtx to start at
	end

	for ii = 0, N - 1 do
		local i = (start_i + ii) % N
		local j = bReverse2 and MathUtil.WrapSignedIndex(start_j - ii, N2) or (start_j + ii) % N2
		local v = self.Mesh:GetVertex(self.Vertices[i + 1])
		local v2 = Mesh2:GetVertex(Loop2.Vertices[j + 1])
		if v:Distance(v2) > tol then
			return false
		end
	end

	return true
end

-- stores vertices [starti, starti+1, ... starti+count-1] in span, and returns span, or nil if invalid range
-- (span is a 1-based Luau array; it is filled starting at span[1])
function EdgeLoop:GetVertexSpan(starti: number, count: number, span: { number }, reverseSpan: boolean?): { number }?
	local V = self.Vertices
	local N = #V
	if starti < 0 or starti >= N or count > N - 1 then
		return nil
	end
	if reverseSpan then
		for k = 0, count - 1 do
			span[count - k] = V[((starti + k) % N) + 1]
		end
	else
		for k = 0, count - 1 do
			span[k + 1] = V[((starti + k) % N) + 1]
		end
	end
	return span
end

-- Exhaustively check that verts and edges of this EdgeLoop are consistent. Not for production use.
function EdgeLoop:CheckValidity(eFailMode: number?): boolean
	local mode = eFailMode or FailMode.Throw
	local is_ok = true
	local function CheckOrFailF(b: boolean)
		if mode == FailMode.Throw then
			if b == false then
				error("EdgeLoop.CheckValidity: check failed")
			end
		elseif mode == FailMode.DebugAssert or mode == FailMode.gDevAssert then
			Util.gDevAssert(b)
			is_ok = is_ok and b
		else
			is_ok = is_ok and b
		end
	end

	local Mesh = self.Mesh
	local V, E = self.Vertices, self.Edges
	CheckOrFailF(#V == #E)
	for ei = 0, #E - 1 do
		local ev = Mesh:GetEdgeV(E[ei + 1])
		CheckOrFailF(Mesh:IsVertex(ev[1]))
		CheckOrFailF(Mesh:IsVertex(ev[2]))
		CheckOrFailF(Mesh:FindEdge(ev[1], ev[2]) ~= InvalidID)
		CheckOrFailF(V[ei + 1] == ev[1] or V[ei + 1] == ev[2])
		local vn = V[((ei + 1) % #E) + 1]
		CheckOrFailF(vn == ev[1] or vn == ev[2])
	end
	for vi = 0, #V - 1 do
		local a, b = V[vi + 1], V[((vi + 1) % #V) + 1]
		CheckOrFailF(Mesh:IsVertex(a))
		CheckOrFailF(Mesh:IsVertex(b))
		CheckOrFailF(Mesh:FindEdge(a, b) ~= InvalidID)
		local n, edge_before_b, edge_after_b = 0, E[vi + 1], E[((vi + 1) % #V) + 1]
		for nbr_e in Mesh:VtxEdgesItr(b) do
			if nbr_e == edge_before_b or nbr_e == edge_after_b then
				n += 1
			end
		end
		CheckOrFailF(n == 2)
	end
	return is_ok
end

-- Convert a vertex loop to an edge loop. This should be somewhere else...
function EdgeLoop.VertexLoopToEdgeLoop(mesh, vertex_loop: { number }): { number }
	local NV = #vertex_loop
	local edges = table.create(NV, 0)
	for i = 0, NV - 1 do
		local v0 = vertex_loop[i + 1]
		local v1 = vertex_loop[((i + 1) % NV) + 1]
		edges[i + 1] = mesh:FindEdge(v0, v1)
	end
	return edges
end

return EdgeLoop
