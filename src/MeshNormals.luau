--!native
-- Port of geometry3Sharp mesh/MeshNormals.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau port notes (see PORTING.md):
--   * MeshNormals.new(mesh, eType?)           eType is a MeshNormals.NormalsTypes value
--   * normals:Get(vid)                        C# indexer this[vid]
--   * normals.Normals is a DVector of Vector3d (0-based get/set)
--   * normals.VertexF(vid) -> Vector3d        alternate vertex source (default mesh:GetVertex)
--   * C# static overloads QuickCompute(DMesh3) / QuickCompute(DMesh3, vid, type):
--       MeshNormals.QuickCompute(mesh)                 compute + copy normals into mesh
--       MeshNormals.QuickComputeVertex(mesh, vid, type?) -> Vector3d   (also QuickCompute(mesh, vid))

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local DVector = require(script.Parent.DVector)

local V3 = Vector3d.new
local sqrt = math.sqrt

local MeshNormals = {}
MeshNormals.__index = MeshNormals
MeshNormals.ClassName = "MeshNormals"

MeshNormals.NormalsTypes = table.freeze({
	Vertex_OneRingFaceAverage_AreaWeighted = 0,
})

function MeshNormals.new(mesh, eType: number?)
	local self = setmetatable({}, MeshNormals)
	self.Mesh = mesh
	self.NormalType = eType or MeshNormals.NormalsTypes.Vertex_OneRingFaceAverage_AreaWeighted
	self.Normals = DVector.new()
	-- By default this is Mesh.GetVertex(). Can override to provide alternate vertex source.
	self.VertexF = function(vid: number)
		return mesh:GetVertex(vid)
	end
	self._defaultVertexF = self.VertexF
	return self
end

function MeshNormals:Compute()
	self:Compute_FaceAvg_AreaWeighted()
end

-- C# indexer this[int vid]
function MeshNormals:Get(vid: number)
	return self.Normals.data[vid + 1]
end

function MeshNormals:CopyTo(SetMesh)
	if SetMesh:MaxVertexID() < self.Mesh:MaxVertexID() then
		error("MeshNormals.Set: SetMesh does not have enough vertices!")
	end
	if not SetMesh:HasVertexNormals() then
		SetMesh:EnableVertexNormals(Vector3d.AxisY)
	end
	local NV = self.Mesh:MaxVertexID()
	local nd = self.Normals.data
	for vi = 0, NV - 1 do
		if self.Mesh:IsVertex(vi) and SetMesh:IsVertex(vi) then
			SetMesh:SetVertexNormal(vi, nd[vi + 1])
		end
	end
end

-- TODO: parallel version, cache tri normals
function MeshNormals:Compute_FaceAvg_AreaWeighted()
	local mesh = self.Mesh
	local NV = mesh:MaxVertexID()
	local Normals = self.Normals
	if NV ~= Normals.size then
		Normals:resize(NV)
	end
	-- accumulate in flat scalar buffer, then build vectors
	local acc = table.create(3 * NV, 0)
	local useMeshVertices = (self.VertexF == self._defaultVertexF) and mesh.GetTriVerticesXYZ ~= nil
	for ti in mesh:TriangleIndices() do
		local tri = mesh:GetTriangle(ti)
		local ax, ay, az, bx, by, bz, cx, cy, cz
		if useMeshVertices then
			ax, ay, az, bx, by, bz, cx, cy, cz = mesh:GetTriVerticesXYZ(ti)
		else
			local va, vb, vc = self.VertexF(tri[1]), self.VertexF(tri[2]), self.VertexF(tri[3])
			ax, ay, az, bx, by, bz, cx, cy, cz = va.x, va.y, va.z, vb.x, vb.y, vb.z, vc.x, vc.y, vc.z
		end
		-- N = MathUtil.Normal(va, vb, vc)
		local e1x, e1y, e1z = bx - ax, by - ay, bz - az
		local e2x, e2y, e2z = cx - bx, cy - by, cz - bz
		local l1 = sqrt(e1x * e1x + e1y * e1y + e1z * e1z)
		if l1 > MathUtil.Epsilon then
			local s = 1.0 / l1
			e1x, e1y, e1z = e1x * s, e1y * s, e1z * s
		else
			e1x, e1y, e1z = 0, 0, 0
		end
		local l2 = sqrt(e2x * e2x + e2y * e2y + e2z * e2z)
		if l2 > MathUtil.Epsilon then
			local s = 1.0 / l2
			e2x, e2y, e2z = e2x * s, e2y * s, e2z * s
		else
			e2x, e2y, e2z = 0, 0, 0
		end
		local nx, ny, nz = e1y * e2z - e1z * e2y, e1z * e2x - e1x * e2z, e1x * e2y - e1y * e2x
		local ln = sqrt(nx * nx + ny * ny + nz * nz)
		if ln > MathUtil.Epsilon then
			local s = 1.0 / ln
			nx, ny, nz = nx * s, ny * s, nz * s
		else
			nx, ny, nz = 0, 0, 0
		end
		-- a = MathUtil.Area(va, vb, vc)
		local f1x, f1y, f1z = bx - ax, by - ay, bz - az
		local f2x, f2y, f2z = cx - ax, cy - ay, cz - az
		local qx, qy, qz = f1y * f2z - f1z * f2y, f1z * f2x - f1x * f2z, f1x * f2y - f1y * f2x
		local a = 0.5 * sqrt(qx * qx + qy * qy + qz * qz)
		nx, ny, nz = a * nx, a * ny, a * nz
		for j = 1, 3 do
			local k = 3 * tri[j]
			acc[k + 1] += nx
			acc[k + 2] += ny
			acc[k + 3] += nz
		end
	end

	local nd = Normals.data
	local tol = MathUtil.ZeroTolerancef
	for vi = 0, NV - 1 do
		local k = 3 * vi
		local x, y, z = acc[k + 1], acc[k + 2], acc[k + 3]
		local lsq = x * x + y * y + z * z
		if lsq > tol then
			local len = sqrt(lsq)
			if len > MathUtil.Epsilon then
				local s = 1.0 / len
				nd[vi + 1] = V3(x * s, y * s, z * s)
			else
				nd[vi + 1] = Vector3d.Zero
			end
		else
			nd[vi + 1] = V3(x, y, z)
		end
	end
end

-- C# static void QuickCompute(DMesh3 mesh), and QuickCompute(mesh, vid, type) when vid is given
function MeshNormals.QuickCompute(mesh, vid: number?, eType: number?)
	if vid ~= nil then
		return MeshNormals.QuickComputeVertex(mesh, vid, eType)
	end
	local normals = MeshNormals.new(mesh)
	normals:Compute()
	normals:CopyTo(mesh)
	return nil
end

-- C# static Vector3d QuickCompute(DMesh3 mesh, int vid, NormalsTypes type = ...)
function MeshNormals.QuickComputeVertex(mesh, vid: number, eType: number?)
	local sum = Vector3d.Zero
	for tid in mesh:VtxTrianglesItr(vid) do
		local n, a = mesh:GetTriInfo(tid)
		sum = sum + a * n
	end
	return sum:Normalized()
end

return MeshNormals
