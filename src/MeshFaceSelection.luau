-- Port of geometry3Sharp mesh_selection/MeshFaceSelection.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- A set of selected triangles of a mesh, with region-growing / cleanup operations.
--
-- Luau port notes (see PORTING.md):
--   * Constructors:
--       MeshFaceSelection.new(mesh)
--       MeshFaceSelection.FromCopy(copy)                                  C# MeshFaceSelection(MeshFaceSelection)
--       MeshFaceSelection.FromVertexSelection(mesh, convertV, minCount=3) C# MeshFaceSelection(mesh, MeshVertexSelection, int)
--       MeshFaceSelection.FromGroup(mesh, group_id)                       C# MeshFaceSelection(mesh, int group_id)
--   * IEnumerable<int>: sel:GetEnumerator() returns an iterator over selected tids (also `for tid in sel do`).
--     Iteration order is unspecified (C# HashSet).
--   * sel:Count() (property). The selected set is sel.Selected = {[tid] = true}.
--   * Select(x) / Deselect(x): x may be a tid, an array of tids, a set {[tid]=true}, an iterator function,
--     or another selection. The C# Select(Func<int,bool>) overload is SelectF(selectF).
--   * ExpandToFaceNeighbours(FilterF?) / ExpandToFaceNeighbours(rounds, FilterF?)
--     ExpandToOneRingNeighbours(FilterF?) / ExpandToOneRingNeighbours(nRings, FilterF?)  (dispatch on number arg)
--   * FloodFill(tSeed or Seeds array, TriFilterF?, EdgeFilterF?)
--   * LocalOptimize(bClipFins, bFillEars, bFillTinyHoles=true, bClipLoners=true, bRemoveBowties=false), or
--     LocalOptimize(bRemoveBowties=true) when called with at most one argument.
--   * FindNeighbourTris()/FindBorderTris()/ToArray() return 1-based arrays.

local Util = require(script.Parent.Util)
local MathUtil = require(script.Parent.MathUtil)
local BitArray = require(script.Parent.BitArray)

local InvalidID = -1

local MeshFaceSelection = {}
MeshFaceSelection.__index = MeshFaceSelection
MeshFaceSelection.ClassName = "MeshFaceSelection"

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
MeshFaceSelection.toItr = toItr

function MeshFaceSelection.new(mesh)
	local self = setmetatable({}, MeshFaceSelection)
	self.Mesh = mesh
	self.Selected = {}
	self.count = 0
	self.temp = {}
	self.temp2 = {}
	self.tempBits = nil
	return self
end

function MeshFaceSelection.FromCopy(copy)
	local self = MeshFaceSelection.new(copy.Mesh)
	self.Selected = table.clone(copy.Selected)
	self.count = copy.count
	return self
end

-- convert vertex selection to face selection. Require at least minCount verts of
-- tri to be selected (valid values are 1,2,3)
function MeshFaceSelection.FromVertexSelection(mesh, convertV, minCount: number?)
	local self = MeshFaceSelection.new(mesh)
	local mc = MathUtil.Clamp(minCount or 3, 1, 3)

	if mc == 1 then
		for vid in convertV:GetEnumerator() do
			for tid in mesh:VtxTrianglesItr(vid) do
				self:add(tid)
			end
		end
	else
		for tid in mesh:TriangleIndices() do
			local tri = mesh:GetTriangle(tid)
			if mc == 3 then
				if convertV:IsSelected(tri[1]) and convertV:IsSelected(tri[2]) and convertV:IsSelected(tri[3]) then
					self:add(tid)
				end
			else
				local n = (convertV:IsSelected(tri[1]) and 1 or 0)
					+ (convertV:IsSelected(tri[2]) and 1 or 0)
					+ (convertV:IsSelected(tri[3]) and 1 or 0)
				if n >= mc then
					self:add(tid)
				end
			end
		end
	end
	return self
end

-- select a group
function MeshFaceSelection.FromGroup(mesh, group_id: number)
	local self = MeshFaceSelection.new(mesh)
	self:SelectGroup(group_id)
	return self
end

-- protected Bitmap property
function MeshFaceSelection:Bitmap()
	if self.tempBits == nil then
		self.tempBits = BitArray.new(self.Mesh:MaxTriangleID())
	end
	return self.tempBits
end

function MeshFaceSelection:GetEnumerator(): () -> number?
	local k = nil
	local sel = self.Selected
	return function()
		k = next(sel, k)
		return k
	end
end
MeshFaceSelection.__iter = function(self)
	return self:GetEnumerator()
end

function MeshFaceSelection:add(tid: number)
	if self.Selected[tid] == nil then
		self.Selected[tid] = true
		self.count += 1
	end
end
function MeshFaceSelection:remove(tid: number)
	if self.Selected[tid] ~= nil then
		self.Selected[tid] = nil
		self.count -= 1
	end
end

function MeshFaceSelection:Count(): number
	return self.count
end

function MeshFaceSelection:IsSelected(tid: number): boolean
	return self.Selected[tid] == true
end

-- Select(int tid) / Select(int[]) / Select(List<int>) / Select(IEnumerable<int>)
function MeshFaceSelection:Select(x: any)
	local Mesh = self.Mesh
	if type(x) == "number" then
		Util.gDevAssert(Mesh:IsTriangle(x))
		if Mesh:IsTriangle(x) then
			self:add(x)
		end
		return
	end
	for tID in toItr(x) do
		if Mesh:IsTriangle(tID) then
			self:add(tID)
		end
	end
end

-- C# Select(Func<int,bool> selectF)
function MeshFaceSelection:SelectF(selectF: (number) -> boolean)
	local temp = self.temp
	table.clear(temp)
	local Mesh = self.Mesh
	local NT = Mesh:MaxTriangleID()
	for tID = 0, NT - 1 do
		if Mesh:IsTriangle(tID) and selectF(tID) then
			table.insert(temp, tID)
		end
	end
	self:Select(temp)
end

function MeshFaceSelection:SelectVertexOneRing(vid: number)
	for tid in self.Mesh:VtxTrianglesItr(vid) do
		self:add(tid)
	end
end

function MeshFaceSelection:SelectVertexOneRings(vertices: any)
	for vid in toItr(vertices) do
		for tid in self.Mesh:VtxTrianglesItr(vid) do
			self:add(tid)
		end
	end
end

function MeshFaceSelection:SelectEdgeTris(eid: number)
	local et = self.Mesh:GetEdgeT(eid)
	self:add(et[1])
	if et[2] ~= InvalidID then
		self:add(et[2])
	end
end

-- Deselect(int tid) / Deselect(int[]) / Deselect(IEnumerable<int>)
function MeshFaceSelection:Deselect(x: any)
	if type(x) == "number" then
		self:remove(x)
		return
	end
	if x == self then
		self:DeselectAll()
		return
	end
	for tid in toItr(x) do
		self:remove(tid)
	end
end

function MeshFaceSelection:DeselectAll()
	table.clear(self.Selected)
	self.count = 0
end

function MeshFaceSelection:SelectGroup(gid: number)
	local Mesh = self.Mesh
	local NT = Mesh:MaxTriangleID()
	for tid = 0, NT - 1 do
		if Mesh:IsTriangle(tid) and Mesh:GetTriangleGroup(tid) == gid then
			self:add(tid)
		end
	end
end

function MeshFaceSelection:SelectGroupInverse(gid: number)
	local Mesh = self.Mesh
	local NT = Mesh:MaxTriangleID()
	for tid = 0, NT - 1 do
		if Mesh:IsTriangle(tid) and Mesh:GetTriangleGroup(tid) ~= gid then
			self:add(tid)
		end
	end
end

function MeshFaceSelection:DeselectGroup(gid: number)
	-- cannot just iterate over selected tris because remove() will change them...
	local Mesh = self.Mesh
	local NT = Mesh:MaxTriangleID()
	for tid = 0, NT - 1 do
		if Mesh:IsTriangle(tid) and Mesh:GetTriangleGroup(tid) == gid then
			self:remove(tid)
		end
	end
end

function MeshFaceSelection:ToArray(): { number }
	local tris = table.create(self.count)
	for tid in self.Selected do
		table.insert(tris, tid)
	end
	return tris
end

-- find set of tris just outside border of selection
function MeshFaceSelection:FindNeighbourTris(): { number }
	local result = {}
	local Mesh = self.Mesh
	for tid in self.Selected do
		local nbr_tris = Mesh:GetTriNeighbourTris(tid)
		for j = 1, 3 do
			if nbr_tris[j] ~= InvalidID and self:IsSelected(nbr_tris[j]) == false then
				table.insert(result, nbr_tris[j])
			end
		end
	end
	return result
end

-- find set of tris just inside border of selection
function MeshFaceSelection:FindBorderTris(): { number }
	local result = {}
	local Mesh = self.Mesh
	for tid in self.Selected do
		local nbr_tris = Mesh:GetTriNeighbourTris(tid)
		if self:IsSelected(nbr_tris[1]) == false or self:IsSelected(nbr_tris[2]) == false or self:IsSelected(nbr_tris[3]) == false then
			table.insert(result, tid)
		end
	end
	return result
end

-- ExpandToFaceNeighbours(FilterF?) or ExpandToFaceNeighbours(rounds, FilterF?)
function MeshFaceSelection:ExpandToFaceNeighbours(a: any, b: any)
	if type(a) == "number" then
		for k = 1, a do
			self:ExpandToFaceNeighbours(b)
		end
		return
	end
	local FilterF = a
	local temp = self.temp
	table.clear(temp)
	local Mesh = self.Mesh

	for tid in self.Selected do
		local nbr_tris = Mesh:GetTriNeighbourTris(tid)
		for j = 1, 3 do
			if FilterF ~= nil and FilterF(nbr_tris[j]) == false then
				continue
			end
			if nbr_tris[j] ~= InvalidID and self:IsSelected(nbr_tris[j]) == false then
				table.insert(temp, nbr_tris[j])
			end
		end
	end

	for i = 1, #temp do
		self:add(temp[i])
	end
end

-- Add all triangles in vertex one-rings of current selection to set.
-- On a large mesh this is quite expensive as we don't know the boundary,
-- so we have to iterate over all triangles.
--
-- Return false from FilterF to prevent triangles from being included.
--
-- ExpandToOneRingNeighbours(nRings, FilterF?):
-- Expand selection by N vertex one-rings. This is *significantly* faster
-- than calling ExpandToOnering() multiple times, because we can track
-- the growing front and only check the new triangles.
function MeshFaceSelection:ExpandToOneRingNeighbours(a: any, b: any)
	local Mesh = self.Mesh
	if type(a) ~= "number" then
		local FilterF = a
		local temp = self.temp
		table.clear(temp)

		for tid in self.Selected do
			local tri_v = Mesh:GetTriangle(tid)
			for j = 1, 3 do
				local vid = tri_v[j]
				for nbr_t in Mesh:VtxTrianglesItr(vid) do
					if FilterF ~= nil and FilterF(nbr_t) == false then
						continue
					end
					if self:IsSelected(nbr_t) == false then
						table.insert(temp, nbr_t)
					end
				end
			end
		end

		for i = 1, #temp do
			self:add(temp[i])
		end
		return
	end

	local nRings, FilterF = a, b
	if nRings == 1 then
		self:ExpandToOneRingNeighbours(FilterF)
		return
	end

	local addTris = self.temp
	local checkTris = self.temp2
	table.clear(checkTris)
	for tid in self.Selected do
		table.insert(checkTris, tid)
	end

	local Bitmap = self:Bitmap()
	Bitmap:SetAll(false)
	for tid in self.Selected do
		Bitmap:Set(tid, true)
	end

	for ri = 1, nRings do
		table.clear(addTris)

		for _, tid in checkTris do
			local tri_v = Mesh:GetTriangle(tid)
			for j = 1, 3 do
				local vid = tri_v[j]
				for nbr_t in Mesh:VtxTrianglesItr(vid) do
					if FilterF ~= nil and FilterF(nbr_t) == false then
						continue
					end
					if not Bitmap:Get(nbr_t) then
						table.insert(addTris, nbr_t)
						Bitmap:Set(nbr_t, true)
					end
				end
			end
		end

		for i = 1, #addTris do
			self:add(addTris[i])
		end

		checkTris, addTris = addTris, checkTris -- swap
	end
end

-- remove all triangles in vertex one-rings of current selection to set.
-- On a large mesh this is quite expensive as we don't know the boundary,
-- so we have to iterate over all triangles.
function MeshFaceSelection:ContractBorderByOneRingNeighbours()
	local temp = self.temp
	table.clear(temp) -- border vertices
	local Mesh = self.Mesh

	-- [TODO] border vertices are pushed onto the temp list multiple times.
	-- minor inefficiency, but maybe we could improve it?

	-- find set of vertices on border
	for tid in self.Selected do
		local tri_v = Mesh:GetTriangle(tid)
		for j = 1, 3 do
			local vid = tri_v[j]
			for nbr_t in Mesh:VtxTrianglesItr(vid) do
				if self:IsSelected(nbr_t) == false then
					table.insert(temp, vid)
					break
				end
			end
		end
	end

	for _, vid in temp do
		for nbr_t in Mesh:VtxTrianglesItr(vid) do
			self:Deselect(nbr_t)
		end
	end
end

-- Grow selection outwards from seed triangle(s), until it hits boundaries defined by triangle and edge filters.
-- Edge filter is not effective unless it (possibly combined w/ triangle filter) defines closed loops.
-- Seeds: a tid or an array of tids
function MeshFaceSelection:FloodFill(Seeds: any, TriFilterF: ((number) -> boolean)?, EdgeFilterF: ((number) -> boolean)?)
	if type(Seeds) == "number" then
		Seeds = { Seeds }
	end
	local Mesh = self.Mesh
	local stack = table.clone(Seeds)
	for k = 1, #Seeds do
		self:add(Seeds[k])
	end
	while #stack > 0 do
		local tID = stack[#stack]
		stack[#stack] = nil

		local nbrs = Mesh:GetTriNeighbourTris(tID)
		for j = 1, 3 do
			local nbr_tid = nbrs[j]
			if nbr_tid == InvalidID or self:IsSelected(nbr_tid) then
				continue
			end
			if TriFilterF ~= nil and TriFilterF(nbr_tid) == false then
				continue
			end
			if EdgeFilterF ~= nil and EdgeFilterF(Mesh:GetTriEdge(tID, j - 1)) == false then
				continue
			end
			self:add(nbr_tid)

			table.insert(stack, nbr_tid)
		end
	end
end

-- return true if we clipped something
function MeshFaceSelection:ClipFins(bClipLoners: boolean): boolean
	local temp = self.temp
	table.clear(temp)
	for tid in self.Selected do
		if self:is_fin(tid, bClipLoners) then
			table.insert(temp, tid)
		end
	end
	if #temp == 0 then
		return false
	end
	for _, tid in temp do
		self:remove(tid)
	end
	return true
end

-- return true if we filled any ears.
function MeshFaceSelection:FillEars(bFillTinyHoles: boolean): boolean
	-- [TODO] not efficient! checks each nbr 3 times !! ugh!!
	local temp = self.temp
	table.clear(temp)
	local Mesh = self.Mesh
	for tid in self.Selected do
		local nbr_tris = Mesh:GetTriNeighbourTris(tid)
		for j = 1, 3 do
			local nbr_t = nbr_tris[j]
			if self:IsSelected(nbr_t) then
				continue
			end
			if self:is_ear(nbr_t, bFillTinyHoles) then
				table.insert(temp, nbr_t)
			end
		end
	end
	if #temp == 0 then
		return false
	end
	for _, tid in temp do
		self:add(tid)
	end
	return true
end

-- returns true if selection was modified
-- LocalOptimize(bClipFins, bFillEars, bFillTinyHoles=true, bClipLoners=true, bRemoveBowties=false)
-- LocalOptimize(bRemoveBowties=true)
function MeshFaceSelection:LocalOptimize(
	bClipFins: boolean?,
	bFillEars: boolean?,
	bFillTinyHoles: boolean?,
	bClipLoners: boolean?,
	bRemoveBowties: boolean?
): boolean
	if bFillEars == nil then
		-- C# LocalOptimize(bool bRemoveBowties = true)
		return self:LocalOptimize(true, true, true, true, bClipFins ~= false)
	end
	local fillTiny = bFillTinyHoles ~= false
	local clipLoners = bClipLoners ~= false
	local removeBowties = bRemoveBowties == true

	local bModified = false
	local done = false
	local count = 0
	local temp_hash = {}
	while not done do
		done = true
		if count == 25 then -- terminate in case we get stuck
			break
		end
		count += 1
		if bClipFins and self:ClipFins(clipLoners) then
			done = false
		end
		if bFillEars and self:FillEars(fillTiny) then
			done = false
		end
		if removeBowties and self:remove_bowties(temp_hash) then
			done = false
		end
		if done == false then
			bModified = true
		end
	end
	if removeBowties then
		self:remove_bowties(temp_hash) -- do a final pass of this because it is usually the most problematic...
	end
	return bModified
end

-- Find any "bowtie" vertices - ie vertex v such taht there is multiple spans of triangles
-- selected in v's triangle one-ring - and deselect those one-rings.
-- Returns true if selection was modified.
function MeshFaceSelection:RemoveBowties(): boolean
	return self:remove_bowties(nil)
end

function MeshFaceSelection:remove_bowties(tempHash: { [number]: boolean }?): boolean
	local Mesh = self.Mesh
	local bModified = false
	local done = false
	local vertices = tempHash or {}
	while not done do
		done = true
		table.clear(vertices)
		for tid in self.Selected do
			local tv = Mesh:GetTriangle(tid)
			vertices[tv[1]] = true
			vertices[tv[2]] = true
			vertices[tv[3]] = true
		end

		for vid in vertices do
			if self:is_bowtie_vtx(vid) then
				self:Deselect(Mesh:VtxTrianglesItr(vid))
				done = false
			end
		end
		if done == false then
			bModified = true
		end
	end
	return bModified
end

function MeshFaceSelection:is_bowtie_vtx(vid: number): boolean
	local Mesh = self.Mesh
	local border_edges = 0
	for eid in Mesh:VtxEdgesItr(vid) do
		local et = Mesh:GetEdgeT(eid)
		if et[2] ~= InvalidID then
			local in_a = self:IsSelected(et[1])
			local in_b = self:IsSelected(et[2])
			if in_a ~= in_b then
				border_edges += 1
			end
		else
			if self:IsSelected(et[1]) then
				border_edges += 1
			end
		end
	end
	return border_edges > 2
end

-- returns nbr_in, nbr_out, bdry_e
function MeshFaceSelection:count_nbrs(tid: number): (number, number, number)
	local nbr_tris = self.Mesh:GetTriNeighbourTris(tid)
	local nbr_in, nbr_out, bdry_e = 0, 0, 0
	for j = 1, 3 do
		local nbr_t = nbr_tris[j]
		if nbr_t == InvalidID then
			bdry_e += 1
		elseif self:IsSelected(nbr_t) == true then
			nbr_in += 1
		else
			nbr_out += 1
		end
	end
	return nbr_in, nbr_out, bdry_e
end

function MeshFaceSelection:is_ear(tid: number, include_tiny_holes: boolean): boolean
	if self:IsSelected(tid) == true then
		return false
	end
	local nbr_in, nbr_out, bdry_e = self:count_nbrs(tid)
	if bdry_e == 2 and nbr_in == 1 then
		return true -- unselected w/ 2 boundary edges, nbr is  in
	elseif nbr_in == 2 then
		if bdry_e == 1 or nbr_out == 1 then
			return true -- unselected w/ 2 selected nbrs
		end
	elseif include_tiny_holes and nbr_in == 3 then
		return true
	end
	return false
end

function MeshFaceSelection:is_fin(tid: number, include_loners: boolean): boolean
	if self:IsSelected(tid) == false then
		return false
	end
	local nbr_in, nbr_out, bdry_e = self:count_nbrs(tid)
	return (nbr_in == 1 and nbr_out == 2) or (include_loners == true and nbr_in == 0 and nbr_out == 3)
end

return MeshFaceSelection
