-- Read/write a DMesh3 from/to an object with the Roblox EditableMesh API, without depending on Roblox
-- types. (Not part of geometry3Sharp.)
--
-- The editable object is duck-typed. Reading uses:
--   GetVertices() -> {vertexId}, GetPosition(vertexId) -> vector with X/Y/Z (or x/y/z),
--   GetFaces() -> {faceId}, GetFaceVertices(faceId) -> {v0, v1, v2}
--   optional: GetFaceNormals(faceId) -> {n0,n1,n2}, GetNormal(normalId) -> vector,
--             GetFaceUVs(faceId) -> {u0,u1,u2}, GetUV(uvId) -> vector with X/Y,
--             GetFaceColors(faceId) -> {c0,c1,c2}, GetColor(colorId) -> color with R/G/B, GetColorAlpha(colorId)
-- Writing uses:
--   AddVertex(vector3) -> vertexId, AddTriangle(v0, v1, v2) -> faceId
--   optional (only when normals/uvs/colors are written): AddNormal(vector3) -> id, SetFaceNormals(faceId, {ids}),
--   AddUV(vector2) -> id, SetFaceUVs(faceId, {ids}), AddColor(color3, alpha) -> id, SetFaceColors(faceId, {ids})
-- Vector/color values for writing are created with options.Vector3 / options.Vector2 / options.Color3
-- (tables with a .new constructor). They default to the Roblox globals Vector3/Vector2/Color3 when present.
--
-- Triangle orientation: DMesh3 triangles are counter-clockwise when viewed from the front, which is also
-- the EditableMesh front-face convention, so faces are written/read without reordering.
--
-- EditableMeshIO.Read(editable, options?) -> DMesh3, info
--   options:
--     weld: "ids" (default; one mesh vertex per editable vertex id) | "positions" (merge coincident
--           positions within weldTolerance; use for meshes whose vertices are split per corner) | "none"
--           (one mesh vertex per face corner, ie triangle soup)
--     weldTolerance: number (default 1e-5)
--     normals: "triangle" (default: store per-corner normals in the TriNormals attribute) | "vertex" (average
--              into vertex normals) | "none"
--     uvs: "triangle" (default: per-corner UVs in TriUVs channel 0) | "vertex" | "none"
--     colors: "none" (default) | "vertex" (per-vertex colors, from face-corner colors)
--     transform: optional function(x, y, z) -> x, y, z applied to positions
--     normalTransform: optional function(x, y, z) -> x, y, z applied to normals (then renormalized)
--   info = { vertexMap = {[editableVertexId] = vid} (weld="ids" only), faceMap = {[faceId] = tid}, failed = count }
--
-- EditableMeshIO.Write(mesh, editable, options?) -> info
--   options:
--     normals: "auto" (default: TriNormals attribute if present, otherwise none -> engine computes) |
--              "triangle" | "vertex" (mesh vertex normals) | "computed" (area-weighted vertex normals) |
--              "flat" (face normal per triangle) | "none"
--     uvs: "auto" (default: TriUVs channel 0 if present, else vertex UVs if present, else none) | "triangle" |
--          "vertex" | "none"
--     colors: "auto" (default: vertex colors if present) | "vertex" | "none"
--     offset: Vector3d-like {x,y,z} subtracted from every position (eg to center the mesh)
--     transform: optional function(x, y, z) -> x, y, z applied to positions (after offset)
--     Vector3, Vector2, Color3: constructor tables (default: Roblox globals)
--   info = { vertexIds = {[vid] = editableVertexId}, faceIds = {[tid] = faceId}, triangleCount, vertexCount }

local DMesh3 = require(script.Parent.DMesh3)
local DMesh3Builder = require(script.Parent.DMesh3Builder)
local Vector3d = require(script.Parent.Vector3d)
local Vector2d = require(script.Parent.Vector2d)
local GeoAttributes = require(script.Parent.GeoAttributes)
local MeshArrays = require(script.Parent.MeshArrays)

local EditableMeshIO = {}

local function vx(v)
	local x = v.X
	if x ~= nil then
		return x, v.Y, v.Z
	end
	return v.x, v.y, v.z
end

local function uvxy(v)
	local x = v.X
	if x ~= nil then
		return x, v.Y
	end
	return v.x, v.y
end

local function has(obj, name: string): boolean
	local ok, value = pcall(function()
		return obj[name]
	end)
	return ok and value ~= nil
end

function EditableMeshIO.Read(editable, options: any?)
	local opts = options or {}
	local weld = opts.weld or "ids"
	local normalsMode = opts.normals or "triangle"
	local uvsMode = opts.uvs or "triangle"
	local colorsMode = opts.colors or "none"
	local transform = opts.transform
	local normalTransform = opts.normalTransform

	local haveNormals = normalsMode ~= "none" and has(editable, "GetFaceNormals")
	local haveUVs = uvsMode ~= "none" and has(editable, "GetFaceUVs")
	local haveColors = colorsMode ~= "none" and has(editable, "GetFaceColors")

	local mesh = DMesh3.new(haveNormals and normalsMode == "vertex", haveColors, haveUVs and uvsMode == "vertex", false)
	local builder = DMesh3Builder.new()
	builder:AppendNewMesh(mesh)

	local function readPos(id)
		local x, y, z = vx(editable:GetPosition(id))
		if transform then
			x, y, z = transform(x, y, z)
		end
		return x, y, z
	end

	local vertexMap = {}
	local weldRemap = nil
	if weld == "ids" then
		for _, id in editable:GetVertices() do
			local x, y, z = readPos(id)
			vertexMap[id] = builder:AppendVertex(x, y, z)
		end
	elseif weld == "positions" then
		local ids = editable:GetVertices()
		local points = table.create(#ids)
		for k, id in ids do
			local x, y, z = readPos(id)
			points[k] = Vector3d.new(x, y, z)
		end
		local remap, unique = MeshArrays.WeldVertices(points, opts.weldTolerance or 1e-5)
		local vids = table.create(#unique)
		for i, p in unique do
			vids[i] = builder:AppendVertex(p)
		end
		weldRemap = {}
		for k, id in ids do
			weldRemap[id] = vids[remap[k] + 1]
		end
		vertexMap = weldRemap
	end

	local faces = editable:GetFaces()
	local faceMap = {}
	local failed = 0
	local corners = {} -- per-triangle data to apply after topology is built: { tid, normals?, uvs?, colors? }
	for _, fid in faces do
		local fv = editable:GetFaceVertices(fid)
		if #fv == 3 then
			local a, b, c
			if weld == "none" then
				local x, y, z = readPos(fv[1])
				a = builder:AppendVertex(x, y, z)
				x, y, z = readPos(fv[2])
				b = builder:AppendVertex(x, y, z)
				x, y, z = readPos(fv[3])
				c = builder:AppendVertex(x, y, z)
			else
				a, b, c = vertexMap[fv[1]], vertexMap[fv[2]], vertexMap[fv[3]]
			end
			local tid = DMesh3.InvalidID
			if a ~= nil and b ~= nil and c ~= nil and a ~= b and a ~= c and b ~= c then
				tid = builder:AppendTriangle(a, b, c)
			end
			if tid < 0 then
				failed += 1
			else
				faceMap[fid] = tid
				local entry = { tid }
				if haveNormals then
					local ids = editable:GetFaceNormals(fid)
					local ns = {}
					for k = 1, 3 do
						local id = ids[k]
						if id ~= nil and id ~= 0 then
							local x, y, z = vx(editable:GetNormal(id))
							if normalTransform then
								x, y, z = normalTransform(x, y, z)
							end
							ns[k] = Vector3d.new(x, y, z):Normalized()
						else
							ns[k] = Vector3d.Zero
						end
					end
					entry.n = ns
				end
				if haveUVs then
					local ids = editable:GetFaceUVs(fid)
					local us = {}
					for k = 1, 3 do
						local id = ids[k]
						if id ~= nil and id ~= 0 then
							local x, y = uvxy(editable:GetUV(id))
							us[k] = Vector2d.new(x, y)
						else
							us[k] = Vector2d.Zero
						end
					end
					entry.uv = us
				end
				if haveColors then
					local ids = editable:GetFaceColors(fid)
					local cs = {}
					for k = 1, 3 do
						local id = ids[k]
						if id ~= nil and id ~= 0 then
							local col = editable:GetColor(id)
							cs[k] = Vector3d.new(col.R, col.G, col.B)
						else
							cs[k] = Vector3d.One
						end
					end
					entry.c = cs
				end
				table.insert(corners, entry)
			end
		end
	end

	-- apply attribute data
	if haveNormals then
		if normalsMode == "triangle" then
			local attr = mesh:Attribs()
			attr:EnableTriNormals()
			local tn = attr:TriNormals()
			for _, e in corners do
				tn:SetValue(e[1], GeoAttributes.TriNormals.new(e.n[1], e.n[2], e.n[3]))
			end
		else
			local sums = {}
			for _, e in corners do
				local t = mesh:GetTriangle(e[1])
				for k = 1, 3 do
					sums[t[k]] = (sums[t[k]] or Vector3d.Zero) + e.n[k]
				end
			end
			for vid, s in sums do
				mesh:SetVertexNormal(vid, s:Normalized())
			end
		end
	end
	if haveUVs then
		if uvsMode == "triangle" then
			local attr = mesh:Attribs()
			attr:EnableTriUVs(1)
			local tu = attr:TriUVChannel(0)
			for _, e in corners do
				tu:SetValue(e[1], GeoAttributes.TriUVs.new(e.uv[1], e.uv[2], e.uv[3]))
			end
		else
			for _, e in corners do
				local t = mesh:GetTriangle(e[1])
				for k = 1, 3 do
					mesh:SetVertexUV(t[k], e.uv[k])
				end
			end
		end
	end
	if haveColors then
		for _, e in corners do
			local t = mesh:GetTriangle(e[1])
			for k = 1, 3 do
				mesh:SetVertexColor(t[k], e.c[k])
			end
		end
	end

	return mesh, { vertexMap = (weld ~= "none") and vertexMap or nil, faceMap = faceMap, failed = failed }
end

-- area-weighted vertex normals (used for normals = "computed")
local function computeVertexNormals(mesh)
	local sums = {}
	for tid in mesh:TriangleIndices() do
		local ax, ay, az, bx, by, bz, cx, cy, cz = mesh:GetTriVerticesXYZ(tid)
		local e1x, e1y, e1z = bx - ax, by - ay, bz - az
		local e2x, e2y, e2z = cx - ax, cy - ay, cz - az
		local nx, ny, nz = e1y * e2z - e1z * e2y, e1z * e2x - e1x * e2z, e1x * e2y - e1y * e2x
		local a, b, c = mesh:GetTriangleABC(tid)
		for _, v in { a, b, c } do
			local s = sums[v]
			if s == nil then
				sums[v] = { nx, ny, nz }
			else
				s[1] += nx
				s[2] += ny
				s[3] += nz
			end
		end
	end
	local result = {}
	for vid, s in sums do
		result[vid] = Vector3d.new(s[1], s[2], s[3]):Normalized()
	end
	return result
end

function EditableMeshIO.Write(mesh, editable, options: any?)
	local opts = options or {}
	local V3 = opts.Vector3 or (Vector3 :: any)
	local V2 = opts.Vector2 or (Vector2 :: any)
	local C3 = opts.Color3 or (Color3 :: any)
	assert(V3 ~= nil, "EditableMeshIO.Write: no Vector3 constructor (pass options.Vector3)")

	local attr = mesh:HasAttributes() and mesh:Attribs() or nil
	local hasTriNormals = attr ~= nil and attr:HasTriNormals()
	local hasTriUVs = attr ~= nil and attr:HasTriUVChannel(0)

	local normalsMode = opts.normals or "auto"
	if normalsMode == "auto" then
		normalsMode = hasTriNormals and "triangle" or "none"
	end
	local uvsMode = opts.uvs or "auto"
	if uvsMode == "auto" then
		uvsMode = hasTriUVs and "triangle" or (mesh:HasVertexUVs() and "vertex" or "none")
	end
	local colorsMode = opts.colors or "auto"
	if colorsMode == "auto" then
		colorsMode = mesh:HasVertexColors() and "vertex" or "none"
	end
	if normalsMode == "triangle" and not hasTriNormals then
		error("EditableMeshIO.Write: normals = 'triangle' but mesh has no TriNormals attribute")
	end
	if uvsMode == "triangle" and not hasTriUVs then
		error("EditableMeshIO.Write: uvs = 'triangle' but mesh has no TriUVs channel 0")
	end
	if uvsMode ~= "none" then
		assert(V2 ~= nil, "EditableMeshIO.Write: no Vector2 constructor (pass options.Vector2)")
	end
	if colorsMode ~= "none" then
		assert(C3 ~= nil, "EditableMeshIO.Write: no Color3 constructor (pass options.Color3)")
	end

	local ox, oy, oz = 0, 0, 0
	if opts.offset then
		ox, oy, oz = vx(opts.offset)
	end
	local transform = opts.transform

	local vertexIds = {}
	local vertexCount = 0
	for vid in mesh:VertexIndices() do
		local x, y, z = mesh:GetVertexXYZ(vid)
		x, y, z = x - ox, y - oy, z - oz
		if transform then
			x, y, z = transform(x, y, z)
		end
		vertexIds[vid] = editable:AddVertex(V3.new(x, y, z))
		vertexCount += 1
	end

	-- per-vertex attribute ids, shared between faces
	local vertexNormalIds, vertexUVIds, vertexColorIds = {}, {}, {}
	local computedNormals = (normalsMode == "computed") and computeVertexNormals(mesh) or nil
	local function vertexNormalId(vid)
		local id = vertexNormalIds[vid]
		if id == nil then
			local n = computedNormals and computedNormals[vid] or mesh:GetVertexNormal(vid)
			id = editable:AddNormal(V3.new(n.x, n.y, n.z))
			vertexNormalIds[vid] = id
		end
		return id
	end
	local function vertexUVId(vid)
		local id = vertexUVIds[vid]
		if id == nil then
			local u = mesh:GetVertexUV(vid)
			id = editable:AddUV(V2.new(u.x, u.y))
			vertexUVIds[vid] = id
		end
		return id
	end
	local function vertexColorId(vid)
		local id = vertexColorIds[vid]
		if id == nil then
			local c = mesh:GetVertexColor(vid)
			id = editable:AddColor(C3.new(c.x, c.y, c.z), 1)
			vertexColorIds[vid] = id
		end
		return id
	end

	local triNormals = hasTriNormals and attr:TriNormals() or nil
	local triUVs = hasTriUVs and attr:TriUVChannel(0) or nil

	local faceIds = {}
	local triangleCount = 0
	for tid in mesh:TriangleIndices() do
		local a, b, c = mesh:GetTriangleABC(tid)
		local fid = editable:AddTriangle(vertexIds[a], vertexIds[b], vertexIds[c])
		faceIds[tid] = fid
		triangleCount += 1

		if normalsMode == "triangle" then
			local tn = triNormals:GetValue(tid)
			local ids = {}
			for k = 0, 2 do
				local n = tn:Get(k)
				ids[k + 1] = editable:AddNormal(V3.new(n.x, n.y, n.z))
			end
			editable:SetFaceNormals(fid, ids)
		elseif normalsMode == "flat" then
			local n = mesh:GetTriNormal(tid)
			local id = editable:AddNormal(V3.new(n.x, n.y, n.z))
			editable:SetFaceNormals(fid, { id, id, id })
		elseif normalsMode == "vertex" or normalsMode == "computed" then
			editable:SetFaceNormals(fid, { vertexNormalId(a), vertexNormalId(b), vertexNormalId(c) })
		end

		if uvsMode == "triangle" then
			local tu = triUVs:GetValue(tid)
			local ids = {}
			for k = 0, 2 do
				local u = tu:Get(k)
				ids[k + 1] = editable:AddUV(V2.new(u.x, u.y))
			end
			editable:SetFaceUVs(fid, ids)
		elseif uvsMode == "vertex" then
			editable:SetFaceUVs(fid, { vertexUVId(a), vertexUVId(b), vertexUVId(c) })
		end

		if colorsMode == "vertex" then
			editable:SetFaceColors(fid, { vertexColorId(a), vertexColorId(b), vertexColorId(c) })
		end
	end

	return { vertexIds = vertexIds, faceIds = faceIds, triangleCount = triangleCount, vertexCount = vertexCount }
end

return EditableMeshIO
