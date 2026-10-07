--!native
-- Port of geometry3Sharp mesh_selection/MeshConnectedComponents.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Find connected components of triangles (connected via edges).
--
-- Luau port notes (see PORTING.md):
--   * MeshConnectedComponents.new(mesh); fields FilterSet (IEnumerable<int>: array / set / iterator fn),
--     FilterF (tid -> bool), SeedFilterF (tid -> bool).
--   * Components is a 1-based array of Component tables { Indices = {tid, ...} } (C# struct Component).
--   * Properties/indexer -> methods: cc:Count(), cc:Get(index) (0-based like C# this[index]),
--     cc:LargestByCount() (returns 0-based index).
--   * IEnumerable<Component> -> cc:GetEnumerator() iterator (and `for comp in cc do` via __iter).
--   * Static overload FindConnectedT(DMesh3 mesh, int tSeed) -> MeshConnectedComponents.FindConnectedTSeed(mesh, tSeed),
--     returns a set {[tid] = true}.
--   * Static Separate(mesh) -> array of DMesh3, LargestT(mesh) -> DMesh3 (use DSubmesh3).

local Interval1i = require(script.Parent.Interval1i)

local InvalidID = -1

local MeshConnectedComponents = {}
MeshConnectedComponents.__index = MeshConnectedComponents
MeshConnectedComponents.ClassName = "MeshConnectedComponents"

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

function MeshConnectedComponents.new(mesh)
	local self = setmetatable({}, MeshConnectedComponents)
	self.Mesh = mesh
	-- if a Filter is set, only triangles contained in Filter are
	-- considered. Both filters will be applied if available.
	self.FilterSet = nil
	self.FilterF = nil
	-- filter on seed values for region-growing. This can be useful
	-- to restrict components to certain areas, when you don't want
	-- (or don't know) a full-triangle-set filter
	self.SeedFilterF = nil
	self.Components = {}
	return self
end

function MeshConnectedComponents:Count(): number
	return #self.Components
end

-- C# indexer this[index] (0-based)
function MeshConnectedComponents:Get(index: number)
	return self.Components[index + 1]
end

function MeshConnectedComponents:GetEnumerator(): () -> any
	local i = 0
	local comps = self.Components
	return function()
		i += 1
		return comps[i]
	end
end
MeshConnectedComponents.__iter = function(self)
	return self:GetEnumerator()
end

-- returns 0-based index of largest component
function MeshConnectedComponents:LargestByCount(): number
	local comps = self.Components
	local largest_i = 0
	local largest_count = #comps[1].Indices
	for i = 1, #comps - 1 do
		if #comps[i + 1].Indices > largest_count then
			largest_count = #comps[i + 1].Indices
			largest_i = i
		end
	end
	return largest_i
end

function MeshConnectedComponents:SortByCount(bIncreasing: boolean?)
	if bIncreasing ~= false then
		table.sort(self.Components, function(x, y)
			return #x.Indices < #y.Indices
		end)
	else
		table.sort(self.Components, function(x, y)
			return #x.Indices > #y.Indices
		end)
	end
end

-- Evaluate valueF for each component and sort by that
function MeshConnectedComponents:SortByValue(valueF: (any) -> number, bIncreasing: boolean?)
	local vals = {}
	for _, c in self.Components do
		vals[c] = valueF(c)
	end

	if bIncreasing ~= false then
		table.sort(self.Components, function(x, y)
			return vals[x] < vals[y]
		end)
	else
		table.sort(self.Components, function(x, y)
			return vals[x] > vals[y]
		end)
	end
end

function MeshConnectedComponents:FindConnectedT()
	self.Components = {}
	local Mesh = self.Mesh
	local FilterF = self.FilterF
	local SeedFilterF = self.SeedFilterF

	local NT = Mesh:MaxTriangleID()

	-- [TODO] could use Euler formula to determine if mesh is closed genus-0...

	local filter_func
	if FilterF ~= nil then
		filter_func = function(i)
			return Mesh:IsTriangle(i) and FilterF(i)
		end
	else
		filter_func = function(i)
			return Mesh:IsTriangle(i)
		end
	end

	-- initial active set contains all valid triangles  (active[i+1] for tid i)
	local active = table.create(NT, 255)
	local activeRange = Interval1i.Empty()
	local filterList = nil
	if self.FilterSet ~= nil then
		filterList = {}
		for tid in toItr(self.FilterSet) do
			table.insert(filterList, tid)
			local bValid = filter_func(tid)
			if bValid then
				active[tid + 1] = 0
				activeRange:Contain(tid)
			end
		end
	else
		for i = 0, NT - 1 do
			local bValid = filter_func(i)
			if bValid then
				active[i + 1] = 0
				activeRange:Contain(i)
			else
				active[i + 1] = 255
			end
		end
	end

	-- temporary buffers
	local queue = {}
	local cur_comp = {}

	local function process(i: number)
		if active[i + 1] == 255 then
			return
		end

		local seed_t = i
		if SeedFilterF ~= nil and SeedFilterF(seed_t) == false then
			return
		end

		table.insert(queue, seed_t)
		active[seed_t + 1] = 1 -- in queue

		while #queue > 0 do
			local cur_t = queue[#queue]
			queue[#queue] = nil

			active[cur_t + 1] = 2 -- tri has been processed
			table.insert(cur_comp, cur_t)

			local nbrs = Mesh:GetTriNeighbourTris(cur_t)
			for j = 1, 3 do
				local nbr_t = nbrs[j]
				if nbr_t ~= InvalidID and active[nbr_t + 1] == 0 then
					table.insert(queue, nbr_t)
					active[nbr_t + 1] = 1 -- in queue
				end
			end
		end

		local comp = { Indices = cur_comp }
		table.insert(self.Components, comp)

		-- remove tris in this component from active set
		for j = 1, #cur_comp do
			active[cur_comp[j] + 1] = 255
		end

		cur_comp = {}
		table.clear(queue)
	end

	-- keep finding valid seed triangles and growing connected components
	-- until we are done
	if filterList ~= nil then
		for _, i in filterList do
			process(i)
		end
	else
		for i = activeRange.a, activeRange.b do
			process(i)
		end
	end
end

-- Separate input mesh into disconnected shells.
-- Resulting array is sorted by decreasing triangle count.
function MeshConnectedComponents.Separate(meshIn): { any }
	local DSubmesh3 = require(script.Parent.DSubmesh3)
	local c = MeshConnectedComponents.new(meshIn)
	c:FindConnectedT()
	c:SortByCount(false)

	local result = table.create(#c.Components)
	for _, comp in c.Components do
		local submesh = DSubmesh3.new(meshIn, comp.Indices)
		table.insert(result, submesh.SubMesh)
	end
	return result
end

-- extract largest shell of meshIn
function MeshConnectedComponents.LargestT(meshIn)
	local DSubmesh3 = require(script.Parent.DSubmesh3)
	local c = MeshConnectedComponents.new(meshIn)
	c:FindConnectedT()
	c:SortByCount(false)
	local submesh = DSubmesh3.new(meshIn, c.Components[1].Indices)
	return submesh.SubMesh
end

-- Utility function that finds set of triangles connected to tSeed. Does not use MeshConnectedComponents class.
-- (C# static FindConnectedT(DMesh3 mesh, int tSeed)); returns set {[tid] = true}
function MeshConnectedComponents.FindConnectedTSeed(mesh, tSeed: number): { [number]: boolean }
	local found = {}
	found[tSeed] = true
	local queue = { tSeed }
	while #queue > 0 do
		local tid = queue[#queue]
		queue[#queue] = nil
		local nbr_t = mesh:GetTriNeighbourTris(tid)
		for j = 1, 3 do
			local nbrid = nbr_t[j]
			if nbrid == InvalidID or found[nbrid] then
				continue
			end
			found[nbrid] = true
			table.insert(queue, nbrid)
		end
	end
	return found
end

return MeshConnectedComponents
