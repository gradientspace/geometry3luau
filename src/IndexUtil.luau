--!native
-- Port of geometry3Sharp math/IndexUtil.cs (IndexUtil and gIndices)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Index2i/Index3i arguments are Luau arrays ({a,b} / {a,b,c}). C# int[] tri_verts arguments are the
-- same arrays. Functions returning a tri index return a 0-based position (0,1,2) like C#.
-- Functions with `ref int a, ref int b` parameters return the updated values as extra results.
-- find_tri_other_vtx(a, b, DVector<int> tri_array, int ti) is find_tri_other_vtx_dv(a, b, dvector, ti).

local InvalidID = -1

local IndexUtil = {}

local function same_pair_unordered(a0: number, a1: number, b0: number, b1: number): boolean
	if a0 == b0 then
		return a1 == b1
	end
	return a0 == b1 and a1 == b0
end
IndexUtil.same_pair_unordered = same_pair_unordered

-- find_shared_edge_v(ref Index2i ev0, ref Index2i ev1)
function IndexUtil.find_shared_edge_v(ev0: { number }, ev1: { number }): number
	if ev0[1] == ev1[1] then
		return ev0[1]
	elseif ev0[1] == ev1[2] then
		return ev0[1]
	elseif ev0[2] == ev1[1] then
		return ev0[2]
	elseif ev0[2] == ev1[2] then
		return ev0[2]
	end
	return InvalidID
end

function IndexUtil.find_edge_other_v(ev: { number }, v: number): number
	if ev[1] == v then
		return ev[2]
	elseif ev[2] == v then
		return ev[1]
	end
	return InvalidID
end

function IndexUtil.find_tri_index(a: number, tri_verts: { number }): number
	if tri_verts[1] == a then
		return 0
	end
	if tri_verts[2] == a then
		return 1
	end
	if tri_verts[3] == a then
		return 2
	end
	return InvalidID
end

function IndexUtil.find_edge_index_in_tri(a: number, b: number, tri_verts: { number }): number
	if same_pair_unordered(a, b, tri_verts[1], tri_verts[2]) then
		return 0
	end
	if same_pair_unordered(a, b, tri_verts[2], tri_verts[3]) then
		return 1
	end
	if same_pair_unordered(a, b, tri_verts[3], tri_verts[1]) then
		return 2
	end
	return InvalidID
end

function IndexUtil.find_tri_ordered_edge(a: number, b: number, tri_verts: { number }): number
	if tri_verts[1] == a and tri_verts[2] == b then
		return 0
	end
	if tri_verts[2] == a and tri_verts[3] == b then
		return 1
	end
	if tri_verts[3] == a and tri_verts[1] == b then
		return 2
	end
	return InvalidID
end

function IndexUtil.find_tri_other_vtx(a: number, b: number, tri_verts: { number }): number
	for j = 0, 2 do
		if same_pair_unordered(a, b, tri_verts[j + 1], tri_verts[((j + 1) % 3) + 1]) then
			return tri_verts[((j + 2) % 3) + 1]
		end
	end
	return InvalidID
end

-- find_tri_other_vtx(int a, int b, DVector<int> tri_array, int ti)
function IndexUtil.find_tri_other_vtx_dv(a: number, b: number, tri_array: any, ti: number): number
	local data = tri_array.data
	local i = 3 * ti + 1
	for j = 0, 2 do
		if same_pair_unordered(a, b, data[i + j], data[i + ((j + 1) % 3)]) then
			return data[i + ((j + 2) % 3)]
		end
	end
	return InvalidID
end

-- returns Index2i {b, c} of other two verts in cycle order, or Index2i.Max
function IndexUtil.find_tri_other_verts(a: number, tri_verts: { number }): { number }
	if tri_verts[1] == a then
		return { tri_verts[2], tri_verts[3] }
	elseif tri_verts[2] == a then
		return { tri_verts[3], tri_verts[1] }
	elseif tri_verts[3] == a then
		return { tri_verts[1], tri_verts[2] }
	end
	return { 2147483647, 2147483647 }
end

function IndexUtil.find_tri_other_index(a: number, b: number, tri_verts: { number }): number
	for j = 0, 2 do
		if same_pair_unordered(a, b, tri_verts[j + 1], tri_verts[((j + 1) % 3) + 1]) then
			return (j + 2) % 3
		end
	end
	return InvalidID
end

-- Set [a,b] to order found in tri_verts (detects if we need to swap).
-- C#: bool orient_tri_edge(ref int a, ref int b, ref Index3i tri_verts) -> returns swapped, a, b
function IndexUtil.orient_tri_edge(a: number, b: number, tri_verts: { number }): (boolean, number, number)
	if a == tri_verts[1] then
		if tri_verts[3] == b then
			return true, b, a
		end
	elseif a == tri_verts[2] then
		if tri_verts[1] == b then
			return true, b, a
		end
	elseif a == tri_verts[3] then
		if tri_verts[2] == b then
			return true, b, a
		end
	end
	return false, a, b
end

-- Set [a,b] to order found in tri_verts and return other vertex.
-- C#: int orient_tri_edge_and_find_other_vtx(ref int a, ref int b, int[] tri_verts) -> returns c, a, b
function IndexUtil.orient_tri_edge_and_find_other_vtx(a: number, b: number, tri_verts: { number }): (number, number, number)
	for j = 0, 2 do
		local v0, v1 = tri_verts[j + 1], tri_verts[((j + 1) % 3) + 1]
		if same_pair_unordered(a, b, v0, v1) then
			return tri_verts[((j + 2) % 3) + 1], v0, v1
		end
	end
	return InvalidID, a, b
end

function IndexUtil.is_ordered(a: number, b: number, tri_verts: { number }): boolean
	return (tri_verts[1] == a and tri_verts[2] == b)
		or (tri_verts[2] == a and tri_verts[3] == b)
		or (tri_verts[3] == a and tri_verts[1] == b)
end

function IndexUtil.is_same_triangle(a: number, b: number, c: number, tri: { number }): boolean
	if tri[1] == a then
		return same_pair_unordered(tri[2], tri[3], b, c)
	elseif tri[2] == a then
		return same_pair_unordered(tri[1], tri[3], b, c)
	elseif tri[3] == a then
		return same_pair_unordered(tri[1], tri[2], b, c)
	end
	return false
end

-- in-place (C# ref Index3i)
function IndexUtil.cycle_indices_minfirst(tri: { number })
	local a, b, c = tri[1], tri[2], tri[3]
	if b < a and b < c then
		tri[1], tri[2], tri[3] = b, c, a
	elseif c < a and c < b then
		tri[1], tri[2], tri[3] = c, a, b
	end
end

-- in-place (C# ref Index3i)
function IndexUtil.sort_indices(tri: { number })
	local a, b, c = tri[1], tri[2], tri[3]
	if a < b and a < c then
		if b > c then
			tri[2], tri[3] = c, b
		end
	elseif b < a and b < c then
		if a < c then
			tri[1], tri[2] = b, a
		else
			tri[1], tri[2], tri[3] = b, c, a
		end
	elseif c < a and c < b then
		if b < a then
			tri[1], tri[3] = c, a
		else
			tri[1], tri[2], tri[3] = c, a, b
		end
	end
end

function IndexUtil.ToGrid3Index(idx: number, nx: number, ny: number)
	local Vector3i = require(script.Parent.Vector3i)
	local x = idx % nx
	local y = (idx // nx) % ny
	local z = idx // (nx * ny)
	return Vector3i.new(x, y, z)
end

function IndexUtil.ToGrid3Linear(i: number, j: number, k: number, nx: number, ny: number): number
	return i + nx * (j + ny * k)
end

-- ToGrid3Linear(Vector3i ijk, int nx, int ny)
function IndexUtil.ToGrid3LinearV(ijk: any, nx: number, ny: number): number
	return ijk.x + nx * (ijk.y + ny * ijk.z)
end

function IndexUtil.FilterValid(indices: { number }, FilterF: (number) -> boolean, bForceCopy: boolean?): { number }
	local nValid = 0
	for i = 1, #indices do
		if FilterF(indices[i]) then
			nValid += 1
		end
	end
	if nValid == #indices and not bForceCopy then
		return indices
	end
	local valid = table.create(nValid)
	for i = 1, #indices do
		if FilterF(indices[i]) then
			table.insert(valid, indices[i])
		end
	end
	return valid
end

function IndexUtil.IndicesCheck(indices: { number }, CheckF: (number) -> boolean): boolean
	for i = 1, #indices do
		if not CheckF(indices[i]) then
			return false
		end
	end
	return true
end

-- Apply(indices, map): map is an IIndexMap (object with :Get(i)) or a 1-based array mapping
-- 0-based index i -> map[i+1]. Modifies indices in place.
function IndexUtil.Apply(indices: { number }, map: any)
	local N = #indices
	if type(map) == "table" and map.Get ~= nil then
		for i = 1, N do
			indices[i] = map:Get(indices[i])
		end
	else
		for i = 1, N do
			indices[i] = map[indices[i] + 1]
		end
	end
end

-- TrianglesToVertices(mesh, triangles (iterator or array), vertices set)
local function itr(src)
	if type(src) == "function" then
		return src
	end
	-- array or set
	if #src > 0 or next(src) == nil then
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

function IndexUtil.TrianglesToVertices(mesh: any, triangles: any, vertices: { [number]: boolean })
	for tid in itr(triangles) do
		local tv = mesh:GetTriangle(tid)
		vertices[tv[1]] = true
		vertices[tv[2]] = true
		vertices[tv[3]] = true
	end
end

function IndexUtil.TrianglesToEdges(mesh: any, triangles: any, edges: { [number]: boolean })
	for tid in itr(triangles) do
		local te = mesh:GetTriEdges(tid)
		edges[te[1]] = true
		edges[te[2]] = true
		edges[te[3]] = true
	end
end

function IndexUtil.EdgesToVertices(mesh: any, edges: any, vertices: { [number]: boolean })
	for eid in itr(edges) do
		local ev = mesh:GetEdgeV(eid)
		vertices[ev[1]] = true
		vertices[ev[2]] = true
	end
end

---------------------------------------------------------------------------------------------------
-- gIndices

local gIndices = {}
IndexUtil.gIndices = gIndices

do
	local Vector2i = require(script.Parent.Vector2i)
	local Vector3i = require(script.Parent.Vector3i)

	-- integer indices offsets in x/y directions (1-based arrays of vectors)
	gIndices.GridOffsets4 = table.freeze({
		Vector2i.new(-1, 0),
		Vector2i.new(1, 0),
		Vector2i.new(0, -1),
		Vector2i.new(0, 1),
	})

	-- integer indices offsets in x/y directions and diagonals
	gIndices.GridOffsets8 = table.freeze({
		Vector2i.new(-1, 0),
		Vector2i.new(1, 0),
		Vector2i.new(0, -1),
		Vector2i.new(0, 1),
		Vector2i.new(-1, 1),
		Vector2i.new(1, 1),
		Vector2i.new(-1, -1),
		Vector2i.new(1, -1),
	})

	-- Corners [0..7] of box faces. BoxFaces[face+1][k+1] in Luau == C# BoxFaces[face, k]
	gIndices.BoxFaces = table.freeze({
		table.freeze({ 1, 0, 3, 2 }), -- back, -z
		table.freeze({ 4, 5, 6, 7 }), -- front, +z
		table.freeze({ 0, 4, 7, 3 }), -- left, -x
		table.freeze({ 5, 1, 2, 6 }), -- right, +x,
		table.freeze({ 0, 1, 5, 4 }), -- bottom, -y
		table.freeze({ 7, 6, 2, 3 }), -- top, +y
	})

	-- what is this? (ported as-is)
	gIndices.BoxFaceNormals = table.freeze({ -3, 3, -1, 1, -2, 2 })

	-- 6 face-neighbour offsets
	gIndices.GridOffsets6 = table.freeze({
		Vector3i.new(0, 0, -1),
		Vector3i.new(0, 0, 1),
		Vector3i.new(-1, 0, 0),
		Vector3i.new(1, 0, 0),
		Vector3i.new(0, -1, 0),
		Vector3i.new(0, 1, 0),
	})

	-- 26 face/edge/vertex-neighbour offsets
	gIndices.GridOffsets26 = table.freeze({
		-- face-nbrs
		Vector3i.new(0, 0, -1),
		Vector3i.new(0, 0, 1),
		Vector3i.new(-1, 0, 0),
		Vector3i.new(1, 0, 0),
		Vector3i.new(0, -1, 0),
		Vector3i.new(0, 1, 0),
		-- edge-nbrs (+y, 0, -y)
		Vector3i.new(1, 1, 0),
		Vector3i.new(-1, 1, 0),
		Vector3i.new(0, 1, 1),
		Vector3i.new(0, 1, -1),
		Vector3i.new(1, 0, 1),
		Vector3i.new(-1, 0, 1),
		Vector3i.new(1, 0, -1),
		Vector3i.new(-1, 0, -1),
		Vector3i.new(1, -1, 0),
		Vector3i.new(-1, -1, 0),
		Vector3i.new(0, -1, 1),
		Vector3i.new(0, -1, -1),
		-- corner-nbrs (+y,-y)
		Vector3i.new(1, 1, 1),
		Vector3i.new(-1, 1, 1),
		Vector3i.new(1, 1, -1),
		Vector3i.new(-1, 1, -1),
		Vector3i.new(1, -1, 1),
		Vector3i.new(-1, -1, 1),
		Vector3i.new(1, -1, -1),
		Vector3i.new(-1, -1, -1),
	})

	-- iterate over all (x,y,z) grid indices, x fastest
	function gIndices.Grid3Indices(nx: number, ny: number, nz: number): () -> any
		local x, y, z = -1, 0, 0
		return function()
			x += 1
			if x >= nx then
				x = 0
				y += 1
				if y >= ny then
					y = 0
					z += 1
				end
			end
			if z >= nz or nx <= 0 or ny <= 0 then
				return nil
			end
			return Vector3i.new(x, y, z)
		end
	end

	-- iterate over (0,y,z) indices, y fastest
	function gIndices.Grid3IndicesYZ(ny: number, nz: number): () -> any
		local y, z = -1, 0
		return function()
			y += 1
			if y >= ny then
				y = 0
				z += 1
			end
			if z >= nz or ny <= 0 then
				return nil
			end
			return Vector3i.new(0, y, z)
		end
	end

	-- iterate over (x,0,z) indices, x fastest
	function gIndices.Grid3IndicesXZ(nx: number, nz: number): () -> any
		local x, z = -1, 0
		return function()
			x += 1
			if x >= nx then
				x = 0
				z += 1
			end
			if z >= nz or nx <= 0 then
				return nil
			end
			return Vector3i.new(x, 0, z)
		end
	end

	-- iterate over (x,y,0) indices, x fastest
	function gIndices.Grid3IndicesXY(nx: number, ny: number): () -> any
		local x, y = -1, 0
		return function()
			x += 1
			if x >= nx then
				x = 0
				y += 1
			end
			if y >= ny or nx <= 0 then
				return nil
			end
			return Vector3i.new(x, y, 0)
		end
	end
end

return IndexUtil
