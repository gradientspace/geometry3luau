-- Port of geometry3Sharp mesh_selection/MeshVertexSelection.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- A set of selected vertices of a mesh.
--
-- Luau port notes (see PORTING.md):
--   * Constructors:
--       MeshVertexSelection.new(mesh)
--       MeshVertexSelection.FromFaceSelection(mesh, convertT)   C# MeshVertexSelection(mesh, MeshFaceSelection)
--       MeshVertexSelection.FromEdgeSelection(mesh, convertE)   C# MeshVertexSelection(mesh, MeshEdgeSelection)
--   * IEnumerable<int>: sel:GetEnumerator() iterator (also `for vid in sel do`). Order unspecified.
--   * sel:Count(); selected set is sel.Selected = {[vid] = true}; ExtractSelected() returns (and detaches) that set.
--   * Select/Deselect/SelectTriangleVertices/SelectEdgeVertices/DeselectEdges accept an id (where C# has an
--     int overload), an array, a set {[id]=true}, an iterator function, or a selection object.
--   * ExpandToOneRingNeighbours(FilterF?) / ExpandToOneRingNeighbours(nRings, FilterF?)
--   * FloodFill(vSeed or Seeds array, VertIncludedF?) -- a nil VertIncludedF includes all vertices
--     (the C# code would throw a NullReferenceException).

local Util = require(script.Parent.Util)

local MeshVertexSelection = {}
MeshVertexSelection.__index = MeshVertexSelection
MeshVertexSelection.ClassName = "MeshVertexSelection"

local function toItr(src: any): () -> number?
	if type(src) == "function" then
		return src
	end
	local mt = getmetatable(src)
	if mt ~= nil and type(mt) == "table" and mt.GetEnumerator ~= nil then
		return src:GetEnumerator()
	end
	if type(src[1]) == "number" or next(src) == nil then
		local i = 0
		return function()
			i += 1
			return src[i]
		end
	end
	local k = nil
	return function()
		k = next(src, k)
		return k
	end
end

function MeshVertexSelection.new(mesh)
	local self = setmetatable({}, MeshVertexSelection)
	self.Mesh = mesh
	self.Selected = {}
	self.count = 0
	self.temp = {}
	return self
end

-- convert face selection to vertex selection.
function MeshVertexSelection.FromFaceSelection(mesh, convertT)
	local self = MeshVertexSelection.new(mesh)
	for tid in toItr(convertT) do
		local tv = mesh:GetTriangle(tid)
		self:add(tv[1])
		self:add(tv[2])
		self:add(tv[3])
	end
	return self
end

-- convert edge selection to vertex selection.
function MeshVertexSelection.FromEdgeSelection(mesh, convertE)
	local self = MeshVertexSelection.new(mesh)
	for eid in toItr(convertE) do
		local ev = mesh:GetEdgeV(eid)
		self:add(ev[1])
		self:add(ev[2])
	end
	return self
end

function MeshVertexSelection:ExtractSelected(): { [number]: boolean }
	local ret = self.Selected
	self.Selected = {}
	self.count = 0
	return ret
end

function MeshVertexSelection:GetEnumerator(): () -> number?
	local k = nil
	local sel = self.Selected
	return function()
		k = next(sel, k)
		return k
	end
end
MeshVertexSelection.__iter = function(self)
	return self:GetEnumerator()
end

function MeshVertexSelection:add(vID: number)
	if self.Selected[vID] == nil then
		self.Selected[vID] = true
		self.count += 1
	end
end
function MeshVertexSelection:remove(vID: number)
	if self.Selected[vID] ~= nil then
		self.Selected[vID] = nil
		self.count -= 1
	end
end

function MeshVertexSelection:Count(): number
	return self.count
end

function MeshVertexSelection:IsSelected(vID: number): boolean
	return self.Selected[vID] == true
end

-- Select(int vID) / Select(int[]) / Select(IEnumerable<int>)
function MeshVertexSelection:Select(x: any)
	local Mesh = self.Mesh
	if type(x) == "number" then
		Util.gDevAssert(Mesh:IsVertex(x))
		if Mesh:IsVertex(x) then
			self:add(x)
		end
		return
	end
	for vID in toItr(x) do
		if Mesh:IsVertex(vID) then
			self:add(vID)
		end
	end
end

-- SelectTriangleVertices(int[] / IEnumerable<int> / MeshFaceSelection)
function MeshVertexSelection:SelectTriangleVertices(triangles: any)
	local Mesh = self.Mesh
	for tid in toItr(triangles) do
		local tri = Mesh:GetTriangle(tid)
		self:add(tri[1])
		self:add(tri[2])
		self:add(tri[3])
	end
end

-- for each vertex of input triangle set, select vertex if all
-- one-ring triangles are contained in triangle set (ie vertex is not on boundary of triangle set).
function MeshVertexSelection:SelectInteriorVertices(triangles)
	local Mesh = self.Mesh
	local borderv = {}
	for tid in toItr(triangles) do
		local tv = Mesh:GetTriangle(tid)
		for j = 1, 3 do
			local vid = tv[j]
			if self.Selected[vid] or borderv[vid] then
				continue
			end
			local full_ring = true
			for ring_tid in Mesh:VtxTrianglesItr(vid) do
				if triangles:IsSelected(ring_tid) == false then
					full_ring = false
					break
				end
			end
			if full_ring then
				self:add(vid)
			else
				borderv[vid] = true
			end
		end
	end
end

-- Select set of boundary vertices connected to vSeed.
function MeshVertexSelection:SelectConnectedBoundaryV(vSeed: number)
	local Mesh = self.Mesh
	if not Mesh:IsBoundaryVertex(vSeed) then
		error("MeshConnectedComponents.FindConnectedBoundaryV: vSeed is not a boundary vertex")
	end

	local found = {}
	found[vSeed] = true
	local queue = self.temp
	table.clear(queue)
	table.insert(queue, vSeed)
	while #queue > 0 do
		local vid = queue[#queue]
		queue[#queue] = nil
		for nbrid in Mesh:VtxVerticesItr(vid) do
			if Mesh:IsBoundaryVertex(nbrid) and found[nbrid] == nil then
				found[nbrid] = true
				table.insert(queue, nbrid)
			end
		end
	end
	for vid in found do
		self:add(vid)
	end
	table.clear(self.temp)
end

-- SelectEdgeVertices(int[] / IEnumerable<int>)
function MeshVertexSelection:SelectEdgeVertices(edges: any)
	local Mesh = self.Mesh
	for eid in toItr(edges) do
		local ev = Mesh:GetEdgeV(eid)
		self:add(ev[1])
		self:add(ev[2])
	end
end

-- Deselect(int vID) / Deselect(int[]) / Deselect(IEnumerable<int>)
function MeshVertexSelection:Deselect(x: any)
	if type(x) == "number" then
		self:remove(x)
		return
	end
	if x == self then
		table.clear(self.Selected)
		self.count = 0
		return
	end
	for vid in toItr(x) do
		self:remove(vid)
	end
end

function MeshVertexSelection:DeselectEdge(eid: number)
	local ev = self.Mesh:GetEdgeV(eid)
	self:remove(ev[1])
	self:remove(ev[2])
end

function MeshVertexSelection:DeselectEdges(edges: any)
	local Mesh = self.Mesh
	for eid in toItr(edges) do
		local ev = Mesh:GetEdgeV(eid)
		self:remove(ev[1])
		self:remove(ev[2])
	end
end

function MeshVertexSelection:ToArray(): { number }
	local verts = table.create(self.count)
	for vid in self.Selected do
		table.insert(verts, vid)
	end
	return verts
end

-- Add all one-ring neighbours of current selection to set.
-- Return false from FilterF to prevent vertices from being included.
-- ExpandToOneRingNeighbours(FilterF?) / ExpandToOneRingNeighbours(nRings, FilterF?)
-- [TODO] should do this more efficiently, like MeshFaceSelection
function MeshVertexSelection:ExpandToOneRingNeighbours(a: any, b: any)
	if type(a) == "number" then
		for k = 1, a do
			self:ExpandToOneRingNeighbours(b)
		end
		return
	end
	local FilterF = a
	local Mesh = self.Mesh
	local temp = self.temp
	table.clear(temp)

	for vid in self.Selected do
		for nbr_vid in Mesh:VtxVerticesItr(vid) do
			if FilterF ~= nil and FilterF(nbr_vid) == false then
				continue
			end
			if self:IsSelected(nbr_vid) == false then
				table.insert(temp, nbr_vid)
			end
		end
	end

	for i = 1, #temp do
		self:add(temp[i])
	end
end

-- Grow selection outwards from seed vertex (or array of seeds), until it hits boundaries defined by vertex filter.
function MeshVertexSelection:FloodFill(Seeds: any, VertIncludedF: ((number) -> boolean)?)
	if type(Seeds) == "number" then
		Seeds = { Seeds }
	end
	local Mesh = self.Mesh
	local stack = table.clone(Seeds)
	for k = 1, #Seeds do
		self:add(Seeds[k])
	end
	while #stack > 0 do
		local vID = stack[#stack]
		stack[#stack] = nil

		for nbr_vid in Mesh:VtxVerticesItr(vID) do
			if self:IsSelected(nbr_vid) == true or (VertIncludedF ~= nil and VertIncludedF(nbr_vid) == false) then
				continue
			end
			self:add(nbr_vid)
			table.insert(stack, nbr_vid)
		end
	end
end

return MeshVertexSelection
