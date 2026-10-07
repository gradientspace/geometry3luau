-- Port of geometry3Sharp mesh_selection/MeshEdgeSelection.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- A set of selected edges of a mesh.
--
-- Luau port notes (see PORTING.md):
--   * Constructors:
--       MeshEdgeSelection.new(mesh)
--       MeshEdgeSelection.FromCopy(copy)                                   C# MeshEdgeSelection(MeshEdgeSelection)
--       MeshEdgeSelection.FromVertexSelection(mesh, convertV, minCount=2)  C# MeshEdgeSelection(mesh, MeshVertexSelection, int)
--       MeshEdgeSelection.FromFaceSelection(mesh, convertT, minCount=1)    C# MeshEdgeSelection(mesh, MeshFaceSelection, int)
--   * IEnumerable<int>: sel:GetEnumerator() iterator (also `for eid in sel do`). Order unspecified.
--   * sel:Count(); the selected set is sel.Selected = {[eid] = true}.
--   * Select(x)/Deselect(x): id, array, set, iterator function or selection. C# Select(Func<int,bool>) is SelectF(selectF).

local Util = require(script.Parent.Util)
local MathUtil = require(script.Parent.MathUtil)
local BitArray = require(script.Parent.BitArray)

local MeshEdgeSelection = {}
MeshEdgeSelection.__index = MeshEdgeSelection
MeshEdgeSelection.ClassName = "MeshEdgeSelection"

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

function MeshEdgeSelection.new(mesh)
	local self = setmetatable({}, MeshEdgeSelection)
	self.Mesh = mesh
	self.Selected = {}
	self.count = 0
	self.temp = {}
	self.tempBits = nil
	return self
end

function MeshEdgeSelection.FromCopy(copy)
	local self = MeshEdgeSelection.new(copy.Mesh)
	self.Selected = table.clone(copy.Selected)
	self.count = copy.count
	return self
end

-- protected Bitmap property
function MeshEdgeSelection:Bitmap()
	if self.tempBits == nil then
		self.tempBits = BitArray.new(self.Mesh:MaxEdgeID())
	end
	return self.tempBits
end

-- convert vertex selection to edge selection. Require at least minCount verts of edge to be selected
function MeshEdgeSelection.FromVertexSelection(mesh, convertV, minCount: number?)
	local self = MeshEdgeSelection.new(mesh)
	local mc = MathUtil.Clamp(minCount or 2, 1, 2)

	-- [TODO] if minCount == 1, and convertV is small, it is faster to iterate over convertV!!

	for eid in mesh:EdgeIndices() do
		local ev = mesh:GetEdgeV(eid)
		local n = (convertV:IsSelected(ev[1]) and 1 or 0) + (convertV:IsSelected(ev[2]) and 1 or 0)
		if n >= mc then
			self:add(eid)
		end
	end
	return self
end

-- convert face selection to edge selection. Require at least minCount tris of edge to be selected
function MeshEdgeSelection.FromFaceSelection(mesh, convertT, minCount: number?)
	local self = MeshEdgeSelection.new(mesh)
	local mc = MathUtil.Clamp(minCount or 1, 1, 2)

	if mc == 1 then
		for tid in toItr(convertT) do
			local te = mesh:GetTriEdges(tid)
			self:add(te[1])
			self:add(te[2])
			self:add(te[3])
		end
	else
		for eid in mesh:EdgeIndices() do
			local et = mesh:GetEdgeT(eid)
			if convertT:IsSelected(et[1]) and convertT:IsSelected(et[2]) then
				self:add(eid)
			end
		end
	end
	return self
end

function MeshEdgeSelection:GetEnumerator(): () -> number?
	local k = nil
	local sel = self.Selected
	return function()
		k = next(sel, k)
		return k
	end
end
MeshEdgeSelection.__iter = function(self)
	return self:GetEnumerator()
end

function MeshEdgeSelection:add(eid: number)
	if self.Selected[eid] == nil then
		self.Selected[eid] = true
		self.count += 1
	end
end
function MeshEdgeSelection:remove(eid: number)
	if self.Selected[eid] ~= nil then
		self.Selected[eid] = nil
		self.count -= 1
	end
end

function MeshEdgeSelection:Count(): number
	return self.count
end

function MeshEdgeSelection:IsSelected(eid: number): boolean
	return self.Selected[eid] == true
end

-- Select(int eid) / Select(int[]) / Select(List<int>) / Select(IEnumerable<int>)
function MeshEdgeSelection:Select(x: any)
	local Mesh = self.Mesh
	if type(x) == "number" then
		Util.gDevAssert(Mesh:IsEdge(x))
		if Mesh:IsEdge(x) then
			self:add(x)
		end
		return
	end
	for eid in toItr(x) do
		if Mesh:IsEdge(eid) then
			self:add(eid)
		end
	end
end

-- C# Select(Func<int,bool> selectF)
function MeshEdgeSelection:SelectF(selectF: (number) -> boolean)
	local temp = self.temp
	table.clear(temp)
	local Mesh = self.Mesh
	local NT = Mesh:MaxEdgeID()
	for eid = 0, NT - 1 do
		if Mesh:IsEdge(eid) and selectF(eid) then
			table.insert(temp, eid)
		end
	end
	self:Select(temp)
end

function MeshEdgeSelection:SelectVertexEdges(vertices: any)
	local Mesh = self.Mesh
	for vid in toItr(vertices) do
		for eid in Mesh:VtxEdgesItr(vid) do
			self:add(eid)
		end
	end
end

function MeshEdgeSelection:SelectTriangleEdges(triangles: any)
	local Mesh = self.Mesh
	for tid in toItr(triangles) do
		local et = Mesh:GetTriEdges(tid)
		self:add(et[1])
		self:add(et[2])
		self:add(et[3])
	end
end

function MeshEdgeSelection:SelectBoundaryTriEdges(triangles)
	local Mesh = self.Mesh
	for tid in toItr(triangles) do
		local te = Mesh:GetTriEdges(tid)
		for j = 1, 3 do
			local et = Mesh:GetEdgeT(te[j])
			local other_tid = (et[1] == tid) and et[2] or et[1]
			if triangles:IsSelected(other_tid) == false then
				self:add(te[j])
			end
		end
	end
end

-- Deselect(int) / Deselect(int[]) / Deselect(IEnumerable<int>)
function MeshEdgeSelection:Deselect(x: any)
	if type(x) == "number" then
		self:remove(x)
		return
	end
	if x == self then
		self:DeselectAll()
		return
	end
	for eid in toItr(x) do
		self:remove(eid)
	end
end

function MeshEdgeSelection:DeselectAll()
	table.clear(self.Selected)
	self.count = 0
end

function MeshEdgeSelection:ToArray(): { number }
	local edges = table.create(self.count)
	for eid in self.Selected do
		table.insert(edges, eid)
	end
	return edges
end

return MeshEdgeSelection
