--!native
-- Conversion between DMesh3 and plain arrays, vertex welding, and spatial partitioning of a mesh into
-- pieces with bounded triangle/vertex counts. (Not part of geometry3Sharp; used by the Roblox conversion
-- layer, but has no Roblox dependencies.)
--
--   MeshArrays.ToArrays(mesh) -> { positions = {x0,y0,z0,...}, triangles = {a0,b0,c0,...} (0-based, compact),
--                                   vertexIds = {[k+1] = mesh vid of array vertex k}, triangleIds = {...} }
--   MeshArrays.FromArrays(positions, triangles, options?) -> DMesh3, info
--       positions: flat number array or array of {x,y,z}/{X,Y,Z} values; triangles: flat 0-based index array
--       or array of {a,b,c} (0-based). options:
--         weldTolerance: number?    merge vertices closer than this (nil/negative = no welding)
--         nonManifold: "duplicate" (default) | "discard"   behaviour for triangles that would create a
--                                   non-manifold edge (DMesh3Builder.AddTriangleFailBehaviors)
--       info = { vertexMap = {[k+1] = vid}, triangleMap = {[k+1] = tid or InvalidID}, failed = count }
--   MeshArrays.WeldVertices(positions(array of Vector3d), tolerance) -> remap ({[k+1] = welded index}), uniquePositions
--   MeshArrays.PartitionTriangles(mesh, maxTriangles, maxVertices?) -> { {tid,...}, ... }
--       spatially coherent groups (recursive median split on triangle centroids along the longest axis)
--   MeshArrays.ExtractSubmesh(mesh, triangles) -> DMesh3, vertexMap ({[subVid] = baseVid})
--       copies vertex normals/colors/uvs, triangle groups, and TriNormals/TriUVs attributes

local DMesh3 = require(script.Parent.DMesh3)
local DMesh3Builder = require(script.Parent.DMesh3Builder)
local Vector3d = require(script.Parent.Vector3d)

local floor = math.floor

local MeshArrays = {}

function MeshArrays.ToArrays(mesh)
	local positions, triangles = {}, {}
	local vertexIds, triangleIds = {}, {}
	local remap = {}
	local n = 0
	for vid in mesh:VertexIndices() do
		local x, y, z = mesh:GetVertexXYZ(vid)
		positions[3 * n + 1], positions[3 * n + 2], positions[3 * n + 3] = x, y, z
		remap[vid] = n
		n += 1
		vertexIds[n] = vid
	end
	local t = 0
	for tid in mesh:TriangleIndices() do
		local a, b, c = mesh:GetTriangleABC(tid)
		triangles[3 * t + 1], triangles[3 * t + 2], triangles[3 * t + 3] = remap[a], remap[b], remap[c]
		t += 1
		triangleIds[t] = tid
	end
	return { positions = positions, triangles = triangles, vertexIds = vertexIds, triangleIds = triangleIds }
end

-- spatial-hash welding: returns remap (1-based array: input k -> output index, 0-based) and unique positions
function MeshArrays.WeldVertices(points: { any }, tolerance: number)
	local remap = table.create(#points, 0)
	local unique = {}
	if tolerance == nil or tolerance < 0 then
		for k, p in points do
			remap[k] = k - 1
			unique[k] = p
		end
		return remap, unique
	end
	local cell = (tolerance > 0) and (tolerance * 2) or 1e-9
	local inv = 1 / cell
	local grid: { [string]: { number } } = {}
	local tolSqr = tolerance * tolerance
	for k, p in points do
		local ix, iy, iz = floor(p.x * inv), floor(p.y * inv), floor(p.z * inv)
		local found = -1
		for dz = -1, 1 do
			for dy = -1, 1 do
				for dx = -1, 1 do
					local bucket = grid[(ix + dx) .. "," .. (iy + dy) .. "," .. (iz + dz)]
					if bucket then
						for _, u in bucket do
							local q = unique[u + 1]
							local ex, ey, ez = q.x - p.x, q.y - p.y, q.z - p.z
							if ex * ex + ey * ey + ez * ez <= tolSqr then
								found = u
								break
							end
						end
					end
					if found >= 0 then
						break
					end
				end
				if found >= 0 then
					break
				end
			end
			if found >= 0 then
				break
			end
		end
		if found < 0 then
			found = #unique
			table.insert(unique, p)
			local key = ix .. "," .. iy .. "," .. iz
			local bucket = grid[key]
			if bucket == nil then
				bucket = {}
				grid[key] = bucket
			end
			table.insert(bucket, found)
		end
		remap[k] = found
	end
	return remap, unique
end

function MeshArrays.FromArrays(positions: { any }, triangles: { any }, options: any?)
	local opts = options or {}
	-- normalize inputs
	local points = {}
	if #positions > 0 and type(positions[1]) == "number" then
		for i = 0, (#positions // 3) - 1 do
			points[i + 1] = Vector3d.new(positions[3 * i + 1], positions[3 * i + 2], positions[3 * i + 3])
		end
	else
		for i, p in positions do
			points[i] = Vector3d.FromVector(p)
		end
	end
	local tris = {}
	if #triangles > 0 and type(triangles[1]) == "number" then
		for i = 0, (#triangles // 3) - 1 do
			tris[i + 1] = { triangles[3 * i + 1], triangles[3 * i + 2], triangles[3 * i + 3] }
		end
	else
		tris = triangles
	end

	local remap, unique = MeshArrays.WeldVertices(points, opts.weldTolerance or -1)

	local builder = DMesh3Builder.new()
	builder:AppendNewMesh(false, false, false, false)
	if opts.nonManifold == "discard" then
		builder.NonManifoldTriBehavior = DMesh3Builder.AddTriangleFailBehaviors.DiscardTriangle
	end
	local vids = table.create(#unique, 0)
	for i, p in unique do
		vids[i] = builder:AppendVertex(p)
	end
	local vertexMap = table.create(#points, 0)
	for k = 1, #points do
		vertexMap[k] = vids[remap[k] + 1]
	end
	local triangleMap = table.create(#tris, DMesh3.InvalidID)
	local failed = 0
	for k, t in tris do
		local a, b, c = vertexMap[t[1] + 1], vertexMap[t[2] + 1], vertexMap[t[3] + 1]
		local tid = DMesh3.InvalidID
		if a ~= nil and b ~= nil and c ~= nil and a ~= b and a ~= c and b ~= c then
			tid = builder:AppendTriangle(a, b, c)
		end
		if tid < 0 then
			failed += 1
		end
		triangleMap[k] = tid
	end
	return builder.Meshes[1], { vertexMap = vertexMap, triangleMap = triangleMap, failed = failed }
end

-- Recursive median split of triangle centroids along the longest axis, so every group has at most
-- maxTriangles triangles and (optionally) at most maxVertices distinct vertices.
function MeshArrays.PartitionTriangles(mesh, maxTriangles: number, maxVertices: number?)
	assert(maxTriangles >= 1, "PartitionTriangles: maxTriangles must be >= 1")
	local items = {}
	for tid in mesh:TriangleIndices() do
		local c = mesh:GetTriCentroid(tid)
		table.insert(items, { tid, c.x, c.y, c.z })
	end
	local groups = {}
	local function vertexCount(list)
		local seen, n = {}, 0
		for _, it in list do
			local a, b, c = mesh:GetTriangleABC(it[1])
			for _, v in { a, b, c } do
				if not seen[v] then
					seen[v] = true
					n += 1
				end
			end
		end
		return n
	end
	local function split(list)
		local tooMany = #list > maxTriangles
		if not tooMany and maxVertices ~= nil and #list > 1 then
			tooMany = vertexCount(list) > maxVertices
		end
		if not tooMany or #list <= 1 then
			local tids = table.create(#list)
			for i, it in list do
				tids[i] = it[1]
			end
			table.insert(groups, tids)
			return
		end
		local lo = { math.huge, math.huge, math.huge }
		local hi = { -math.huge, -math.huge, -math.huge }
		for _, it in list do
			for k = 1, 3 do
				local v = it[k + 1]
				if v < lo[k] then
					lo[k] = v
				end
				if v > hi[k] then
					hi[k] = v
				end
			end
		end
		local axis = 1
		for k = 2, 3 do
			if hi[k] - lo[k] > hi[axis] - lo[axis] then
				axis = k
			end
		end
		table.sort(list, function(p, q)
			return p[axis + 1] < q[axis + 1]
		end)
		-- split so that the first part is a multiple of maxTriangles when possible (fewer, fuller pieces)
		local half = #list // 2
		if #list > maxTriangles then
			local nPieces = math.ceil(#list / maxTriangles)
			half = math.ceil(nPieces / 2) * maxTriangles
			if half >= #list then
				half = #list // 2
			end
		end
		split(table.move(list, 1, half, 1, {}))
		split(table.move(list, half + 1, #list, 1, {}))
	end
	split(items)
	return groups
end

-- Copy the given triangles (array of tids) into a new mesh, with attributes. Returns mesh, vertexMap (sub -> base)
function MeshArrays.ExtractSubmesh(mesh, triangles: { number })
	local sub = DMesh3.new(mesh:HasVertexNormals(), mesh:HasVertexColors(), mesh:HasVertexUVs(), mesh:HasTriangleGroups())
	local baseToSub = {}
	local subToBase = {}
	local srcAttr = mesh:HasAttributes() and mesh:Attribs() or nil
	local triNormals = (srcAttr and srcAttr:HasTriNormals()) and srcAttr:TriNormals() or nil
	local triUVChannels = {}
	if srcAttr and srcAttr:HasTriUVs() then
		for ch = 0, srcAttr:NumTriUVChannels() - 1 do
			triUVChannels[ch + 1] = srcAttr:TriUVChannel(ch)
		end
	end
	local dstNormals, dstUVs = nil, {}
	if triNormals then
		sub:Attribs():EnableTriNormals()
		dstNormals = sub:Attribs():TriNormals()
	end
	if #triUVChannels > 0 then
		sub:Attribs():EnableTriUVs(#triUVChannels)
		for ch = 0, #triUVChannels - 1 do
			dstUVs[ch + 1] = sub:Attribs():TriUVChannel(ch)
		end
	end
	local function mapv(v)
		local s = baseToSub[v]
		if s == nil then
			s = sub:AppendVertexFromMesh(mesh, v)
			baseToSub[v] = s
			subToBase[s] = v
		end
		return s
	end
	for _, tid in triangles do
		local a, b, c = mesh:GetTriangleABC(tid)
		local g = mesh:HasTriangleGroups() and mesh:GetTriangleGroup(tid) or -1
		local ntid = sub:AppendTriangle(mapv(a), mapv(b), mapv(c), g)
		if ntid >= 0 then
			if dstNormals then
				dstNormals:SetValue(ntid, triNormals:GetValue(tid))
			end
			for ch, dst in dstUVs do
				dst:SetValue(ntid, triUVChannels[ch]:GetValue(tid))
			end
		end
	end
	return sub, subToBase
end

return MeshArrays
