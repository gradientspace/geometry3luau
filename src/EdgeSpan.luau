-- Port of geometry3Sharp mesh/EdgeSpan.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- An EdgeSpan is a continous set of edges in a Mesh that is *not* closed (that would be an EdgeLoop)
--
-- Luau port notes (see PORTING.md):
--   * Vertices, Edges, BowtieVertices are 1-based Luau arrays of 0-based mesh ids (C# int[]).
--     Indices into these arrays used by the API are 0-based like C#.
--   * Constructors:
--       EdgeSpan.new(mesh)                                     new EdgeSpan(DMesh3)
--       EdgeSpan.FromArrays(mesh, vertices, edges, bCopyArrays)  new EdgeSpan(mesh, int[], int[], bool)
--       EdgeSpan.FromEdges(mesh, edges), EdgeSpan.FromVertices(mesh, vertices)
--   * VertexCount()/EdgeCount() are methods. CountWithinTolerance(v, tol) -> count, last_in_tol
--   * IsSameSpan errors with "todo!" exactly like the C# (NotImplementedException).

local AxisAlignedBox3d = require(script.Parent.AxisAlignedBox3d)
local IndexUtil = require(script.Parent.IndexUtil)
local MathUtil = require(script.Parent.MathUtil)
local Util = require(script.Parent.Util)

local InvalidID = -1
local FailMode = Util.FailMode

local EdgeSpan = {}
EdgeSpan.__index = EdgeSpan
EdgeSpan.ClassName = "EdgeSpan"

function EdgeSpan.new(mesh)
	return setmetatable({ Mesh = mesh, Vertices = nil, Edges = nil, BowtieVertices = nil }, EdgeSpan)
end

function EdgeSpan.FromArrays(mesh, vertices: { number }, edges: { number }, bCopyArrays: boolean)
	local span = EdgeSpan.new(mesh)
	if bCopyArrays then
		span.Vertices = table.clone(vertices)
		span.Edges = table.clone(edges)
	else
		span.Vertices = vertices
		span.Edges = edges
	end
	return span
end

-- construct EdgeSpan from a list of edges of mesh
function EdgeSpan.FromEdges(mesh, edges: { number })
	local Edges = table.clone(edges)
	local NE = #Edges
	local Vertices = table.create(NE + 1, 0)
	local start_ev = mesh:GetEdgeV(Edges[1])
	local prev_ev = start_ev
	if NE > 1 then
		for i = 1, NE - 1 do
			local next_ev = mesh:GetEdgeV(Edges[i + 1])
			Vertices[i + 1] = IndexUtil.find_shared_edge_v(prev_ev, next_ev)
			prev_ev = next_ev
		end
		Vertices[1] = IndexUtil.find_edge_other_v(start_ev, Vertices[2])
		local NV = #Vertices
		Vertices[NV] = IndexUtil.find_edge_other_v(prev_ev, Vertices[NV - 1])
	else
		Vertices[1] = start_ev[1]
		Vertices[2] = start_ev[2]
	end
	return EdgeSpan.FromArrays(mesh, Vertices, Edges, false)
end

-- construct EdgeSpan from a list of vertices of mesh
function EdgeSpan.FromVertices(mesh, vertices: { number })
	local Vertices = table.clone(vertices)
	local NV = #Vertices
	local NE = NV - 1
	local Edges = table.create(NE, 0)
	for i = 0, NE - 1 do
		Edges[i + 1] = mesh:FindEdge(Vertices[i + 1], Vertices[i + 2])
		if Edges[i + 1] == InvalidID then
			error("EdgeSpan.FromVertices: vertices are not connected by edge!")
		end
	end
	return EdgeSpan.FromArrays(mesh, Vertices, Edges, false)
end

function EdgeSpan:VertexCount(): number
	return #self.Vertices
end
function EdgeSpan:EdgeCount(): number
	return #self.Edges
end

function EdgeSpan:GetVertex(i: number)
	return self.Mesh:GetVertex(self.Vertices[i + 1])
end

function EdgeSpan:GetBounds()
	local box = AxisAlignedBox3d.Empty()
	for i = 1, #self.Vertices do
		box:Contain(self.Mesh:GetVertex(self.Vertices[i]))
	end
	return box
end

function EdgeSpan:ToCurve(sourceMesh)
	local MeshUtil = require(script.Parent.MeshUtil)
	if sourceMesh == nil then
		sourceMesh = self.Mesh
	end
	local curve = MeshUtil.ExtractLoopV(sourceMesh, self.Vertices)
	curve.Closed = false
	return curve
end

function EdgeSpan:IsInternalSpan(): boolean
	local V = self.Vertices
	local NV = #V
	for i = 0, NV - 2 do
		local eid = self.Mesh:FindEdge(V[i + 1], V[i + 2])
		Util.gDevAssert(eid ~= InvalidID)
		if self.Mesh:IsBoundaryEdge(eid) then
			return false
		end
	end
	return true
end

function EdgeSpan:IsBoundarySpan(testMesh): boolean
	local useMesh = (testMesh ~= nil) and testMesh or self.Mesh
	local V = self.Vertices
	local NV = #V
	for i = 0, NV - 2 do
		local eid = useMesh:FindEdge(V[i + 1], V[i + 2])
		Util.gDevAssert(eid ~= InvalidID)
		if useMesh:IsBoundaryEdge(eid) == false then
			return false
		end
	end
	return true
end

function EdgeSpan:FindNearestVertex(v): number
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

-- count # of vertices in span that are within tol of v -> count, last_in_tol
function EdgeSpan:CountWithinTolerance(v, tol: number): (number, number)
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

-- Check if Spanw is the same set of positions on another mesh.
function EdgeSpan:IsSameSpan(Spanw, bReverse2: boolean?, tolerance: number?): boolean
	-- [RMS] this is much easier than for a loop, because it has to have
	--   same endpoints. But don't have time right now.
	error("EdgeSpan.IsSameSpan: todo!")
end

-- Exhaustively check that verts and edges of this EdgeSpan are consistent. Not for production use.
function EdgeSpan:CheckValidity(eFailMode: number?): boolean
	local mode = eFailMode or FailMode.Throw
	local is_ok = true
	local function CheckOrFailF(b: boolean)
		if mode == FailMode.Throw then
			if b == false then
				error("EdgeSpan.CheckValidity: check failed")
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
	CheckOrFailF(#V == #E + 1)
	for ei = 0, #E - 1 do
		local ev = Mesh:GetEdgeV(E[ei + 1])
		CheckOrFailF(Mesh:IsVertex(ev[1]))
		CheckOrFailF(Mesh:IsVertex(ev[2]))
		CheckOrFailF(Mesh:FindEdge(ev[1], ev[2]) ~= InvalidID)
		CheckOrFailF(V[ei + 1] == ev[1] or V[ei + 1] == ev[2])
		CheckOrFailF(V[ei + 2] == ev[1] or V[ei + 2] == ev[2])
	end
	for vi = 0, #V - 2 do
		local a, b = V[vi + 1], V[vi + 2]
		CheckOrFailF(Mesh:IsVertex(a))
		CheckOrFailF(Mesh:IsVertex(b))
		CheckOrFailF(Mesh:FindEdge(a, b) ~= InvalidID)
		if vi < #V - 2 then
			local n, edge_before_b, edge_after_b = 0, E[vi + 1], E[vi + 2]
			for nbr_e in Mesh:VtxEdgesItr(b) do
				if nbr_e == edge_before_b or nbr_e == edge_after_b then
					n += 1
				end
			end
			CheckOrFailF(n == 2)
		end
	end
	-- [RMS] C# returns true here regardless of is_ok
	return true
end

-- Convert vertex span to list of edges. This should be somewhere else.
function EdgeSpan.VerticesToEdges(mesh, vertex_span: { number }): { number }
	local NV = #vertex_span
	local edges = table.create(math.max(NV - 1, 0), 0)
	for i = 0, NV - 2 do
		edges[i + 1] = mesh:FindEdge(vertex_span[i + 1], vertex_span[i + 2])
	end
	return edges
end

return EdgeSpan
