-- Port of geometry3Sharp mesh/DMesh3Builder.cs (implements IMeshBuilder)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau notes:
--   * Meshes / Materials / MaterialAssignment / Metadata are 1-based Luau arrays, but mesh ids and material
--     ids returned by the API are 0-based like C# (mesh id i is Meshes[i+1]).
--   * AppendNewMesh(bHaveVtxNormals, bHaveVtxColors, bHaveVtxUVs, bHaveFaceGroups) or AppendNewMesh(existingMesh)
--   * AppendVertex(x, y, z) or AppendVertex(NewVertexInfo)
--   * Materials (GenericMaterial in C#) are opaque values here.
--   * DMesh3Builder.Build(Vertices, Triangles, Normals?, TriGroups?) accepts either flat number arrays
--     ({x0,y0,z0,x1,...} / {a0,b0,c0,a1,...}) or arrays of Vector3d / Index3i ({a,b,c}) values.

local DMesh3 = require(script.Parent.DMesh3)
local Vector3d = require(script.Parent.Vector3d)

local DMesh3Builder = {}
DMesh3Builder.__index = DMesh3Builder

DMesh3Builder.AddTriangleFailBehaviors = table.freeze({
	DiscardTriangle = 0,
	DuplicateAllVertices = 1,
})
local AddTriangleFailBehaviors = DMesh3Builder.AddTriangleFailBehaviors

function DMesh3Builder.new()
	return setmetatable({
		-- What should we do when AddTriangle() fails because triangle is non-manifold?
		NonManifoldTriBehavior = AddTriangleFailBehaviors.DuplicateAllVertices,
		-- What should we do when AddTriangle() fails because the triangle already exists?
		DuplicateTriBehavior = AddTriangleFailBehaviors.DiscardTriangle,
		Meshes = {},
		Materials = {},
		-- this is a map from index into Meshes to index into Materials (-1 if no material)
		--  (so, currently we can only have 1 material per mesh!)
		MaterialAssignment = {},
		Metadata = {},
		nActiveMesh = -1,
	}, DMesh3Builder)
end

function DMesh3Builder:AppendNewMesh(a, bHaveVtxColors: boolean?, bHaveVtxUVs: boolean?, bHaveFaceGroups: boolean?): number
	local index = #self.Meshes
	local m
	if type(a) == "table" then
		m = a
	else
		m = DMesh3.new(a == true, bHaveVtxColors == true, bHaveVtxUVs == true, bHaveFaceGroups == true)
	end
	table.insert(self.Meshes, m)
	table.insert(self.MaterialAssignment, -1) -- no material is known
	table.insert(self.Metadata, {})
	self.nActiveMesh = index
	return index
end

function DMesh3Builder:NumMeshes(): number
	return #self.Meshes
end

function DMesh3Builder:SetActiveMesh(id: number)
	if id >= 0 and id < #self.Meshes then
		self.nActiveMesh = id
	else
		error("DMesh3Builder.SetActiveMesh: active mesh id is out of range")
	end
end

function DMesh3Builder:ActiveMesh()
	return self.Meshes[self.nActiveMesh + 1]
end

local function append_duplicate_triangle(self, i: number, j: number, k: number, g: number): number
	local mesh = self:ActiveMesh()
	local _, vinfo = mesh:GetVertexInfo(i, true, true, true)
	local new_i = mesh:AppendVertex(vinfo)
	_, vinfo = mesh:GetVertexInfo(j, true, true, true)
	local new_j = mesh:AppendVertex(vinfo)
	_, vinfo = mesh:GetVertexInfo(k, true, true, true)
	local new_k = mesh:AppendVertex(vinfo)
	return mesh:AppendTriangle(new_i, new_j, new_k, g)
end

function DMesh3Builder:AppendTriangle(i: number, j: number, k: number, g: number?): number
	local gid = g or -1
	local mesh = self:ActiveMesh()
	-- [RMS] What to do here? We definitely do not want to add a duplicate triangle!!
	--   But is silently ignoring the right thing to do?
	local existing_tid = mesh:FindTriangle(i, j, k)
	if existing_tid ~= DMesh3.InvalidID then
		if self.DuplicateTriBehavior == AddTriangleFailBehaviors.DuplicateAllVertices then
			return append_duplicate_triangle(self, i, j, k, gid)
		end
		return existing_tid
	end

	local tid = mesh:AppendTriangle(i, j, k, gid)
	if tid == DMesh3.NonManifoldID then
		if self.NonManifoldTriBehavior == AddTriangleFailBehaviors.DuplicateAllVertices then
			return append_duplicate_triangle(self, i, j, k, gid)
		end
		return DMesh3.NonManifoldID
	end
	return tid
end

function DMesh3Builder:AppendVertex(x: any, y: number?, z: number?): number
	if type(x) == "number" then
		return self:ActiveMesh():AppendVertex(Vector3d.new(x, y :: number, z :: number))
	end
	return self:ActiveMesh():AppendVertex(x)
end

function DMesh3Builder:SupportsMetaData(): boolean
	return true
end

function DMesh3Builder:AppendMetaData(identifier: string, data: any)
	local md = self.Metadata[self.nActiveMesh + 1]
	if md[identifier] ~= nil then
		error("DMesh3Builder.AppendMetaData: duplicate key " .. identifier)
	end
	md[identifier] = data
end

-- just store material object, we can't use it here
function DMesh3Builder:BuildMaterial(m: any): number
	local id = #self.Materials
	table.insert(self.Materials, m)
	return id
end

-- do material assignment to mesh
function DMesh3Builder:AssignMaterial(materialID: number, meshID: number)
	if meshID >= #self.MaterialAssignment or materialID >= #self.Materials then
		error("[SimpleMeshBuilder::AssignMaterial] meshID or materialID are out-of-range")
	end
	self.MaterialAssignment[meshID + 1] = materialID
end

---------------------------------------------------------------------------------------------------
-- DMesh3 construction utilities

local function toVectors(src: { any }): { any }
	if #src > 0 and type(src[1]) == "number" then
		local out = table.create(#src // 3)
		for i = 0, (#src // 3) - 1 do
			out[i + 1] = Vector3d.new(src[3 * i + 1], src[3 * i + 2], src[3 * i + 3])
		end
		return out
	end
	local out = table.create(#src)
	for i, v in src do
		out[i] = Vector3d.FromVector(v)
	end
	return out
end

local function toTriangles(src: { any }): { { number } }
	if #src > 0 and type(src[1]) == "number" then
		local out = table.create(#src // 3)
		for i = 0, (#src // 3) - 1 do
			out[i + 1] = { src[3 * i + 1], src[3 * i + 2], src[3 * i + 3] }
		end
		return out
	end
	return src
end

-- ultimate generic mesh-builder, pass it arrays of numbers, or lists of Vector3d, or anything in-between.
-- Will figure out how to interpret
function DMesh3Builder.Build(Vertices: { any }, Triangles: { any }, Normals: { any }?, TriGroups: { number }?)
	local mesh = DMesh3.new(Normals ~= nil, false, false, TriGroups ~= nil)

	local v = toVectors(Vertices)
	for i = 1, #v do
		mesh:AppendVertex(v[i])
	end

	if Normals ~= nil then
		local n = toVectors(Normals)
		if #n ~= #v then
			error("DMesh3Builder.Build: incorrect number of normals provided")
		end
		for i = 1, #n do
			mesh:SetVertexNormal(i - 1, n[i])
		end
	end

	local t = toTriangles(Triangles)
	for i = 1, #t do
		mesh:AppendTriangle(t[i])
	end

	if TriGroups ~= nil then
		if #TriGroups ~= #t then
			error("DMesh3Builder.Build: incorect number of triangle groups")
		end
		for i = 1, #t do
			mesh:SetTriangleGroup(i - 1, TriGroups[i])
		end
	end

	return mesh
end

return DMesh3Builder
