-- Port of geometry3Sharp mesh/FaceGroupUtil.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Static utilities for triangle (face) group ids.
--
-- Luau port notes:
--   * SetGroupID(mesh, to) / SetGroupID(mesh, triangles, to): dispatch on argument count. triangles may be an
--     array, set {[tid]=true}, iterator function, or selection object.
--   * FindAllGroups(mesh) returns a set {[gid] = true}.
--   * CountAllGroups(mesh) returns a table {[gid] = count} (C# SparseList<int>; groups with count 0 are absent,
--     and reading a missing gid gives nil instead of 0).
--   * FindTriangleSetsByGroup(mesh, ignoreGID = int.MinValue) -> array of tid arrays (sorted by group id).
--   * FindTrianglesByGroup(mesh, gid) -> array of tids.
--   * SeparateMeshByGroups(mesh) -> array of DMesh3, groupIDs array (C# out int[] groupIDs).

local MathUtil = require(script.Parent.MathUtil)

local FaceGroupUtil = {}

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

-- Set group ID of all triangles in mesh: SetGroupID(mesh, to)
-- Set group id of subset of triangles in mesh: SetGroupID(mesh, triangles, to)
function FaceGroupUtil.SetGroupID(mesh, a: any, b: number?)
	if mesh:HasTriangleGroups() == false then
		return
	end
	if b == nil then
		local to = a :: number
		for tid in mesh:TriangleIndices() do
			mesh:SetTriangleGroup(tid, to)
		end
	else
		for tid in toItr(a) do
			mesh:SetTriangleGroup(tid, b)
		end
	end
end

-- replace group id in mesh
function FaceGroupUtil.SetGroupToGroup(mesh, from: number, to: number)
	if mesh:HasTriangleGroups() == false then
		return
	end

	local NT = mesh:MaxTriangleID()
	for tid = 0, NT - 1 do
		if mesh:IsTriangle(tid) then
			local gid = mesh:GetTriangleGroup(tid)
			if gid == from then
				mesh:SetTriangleGroup(tid, to)
			end
		end
	end
end

-- find the set of group ids used in mesh
function FaceGroupUtil.FindAllGroups(mesh): { [number]: boolean }
	local Groups = {}
	if mesh:HasTriangleGroups() then
		local NT = mesh:MaxTriangleID()
		for tid = 0, NT - 1 do
			if mesh:IsTriangle(tid) then
				Groups[mesh:GetTriangleGroup(tid)] = true
			end
		end
	end
	return Groups
end

-- count number of tris in each group in mesh. Returned table is [group_id] = tri_count
function FaceGroupUtil.CountAllGroups(mesh): { [number]: number }
	local GroupCounts = {}
	if mesh:HasTriangleGroups() then
		local NT = mesh:MaxTriangleID()
		for tid = 0, NT - 1 do
			if mesh:IsTriangle(tid) then
				local gid = mesh:GetTriangleGroup(tid)
				GroupCounts[gid] = (GroupCounts[gid] or 0) + 1
			end
		end
	end
	return GroupCounts
end

-- collect triangles by group id. Returns array of triangle lists (stored as arrays).
-- This requires 2 passes over mesh, but each pass is linear
function FaceGroupUtil.FindTriangleSetsByGroup(mesh, ignoreGID: number?): { { number } }
	local ignore = ignoreGID or MathUtil.MinInt
	if not mesh:HasTriangleGroups() then
		return {}
	end

	-- find # of groups and triangle count for each
	local counts = FaceGroupUtil.CountAllGroups(mesh)
	local GroupIDs = {}
	for gid, count in counts do
		if gid ~= ignore and count > 0 then
			table.insert(GroupIDs, gid)
		end
	end
	table.sort(GroupIDs) -- might as well sort ascending...
	local groupMap = {}

	-- allocate sets
	local sets = table.create(#GroupIDs)
	for i = 1, #GroupIDs do
		local gid = GroupIDs[i]
		sets[i] = table.create(counts[gid])
		groupMap[gid] = i
	end

	-- accumulate triangles
	local NT = mesh:MaxTriangleID()
	for tid = 0, NT - 1 do
		if mesh:IsTriangle(tid) then
			local gid = mesh:GetTriangleGroup(tid)
			local i = groupMap[gid]
			if i ~= nil then
				table.insert(sets[i], tid)
			end
		end
	end

	return sets
end

-- find list of triangles in mesh with specific group id
function FaceGroupUtil.FindTrianglesByGroup(mesh, findGroupID: number): { number }
	local tris = {}
	if mesh:HasTriangleGroups() == false then
		return tris
	end
	for tid in mesh:TriangleIndices() do
		if mesh:GetTriangleGroup(tid) == findGroupID then
			table.insert(tris, tid)
		end
	end
	return tris
end

-- split input mesh into submeshes based on group ID
-- **does not** separate disconnected components w/ same group ID
-- returns meshes, groupIDs
function FaceGroupUtil.SeparateMeshByGroups(mesh): ({ any }, { number })
	local DSubmesh3 = require(script.Parent.DSubmesh3)
	local meshes = {}
	local order = {}
	for tid in mesh:TriangleIndices() do
		local gid = mesh:GetTriangleGroup(tid)
		local tris = meshes[gid]
		if tris == nil then
			tris = {}
			meshes[gid] = tris
			table.insert(order, gid)
		end
		table.insert(tris, tid)
	end

	local result = table.create(#order)
	local groupIDs = table.create(#order)
	for k, gid in order do
		groupIDs[k] = gid
		result[k] = DSubmesh3.QuickSubmesh(mesh, meshes[gid])
	end

	return result, groupIDs
end

return FaceGroupUtil
