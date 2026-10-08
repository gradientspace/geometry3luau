--!native
-- Port of geometry3Sharp mesh/MeshTransforms.cs (static class MeshTransforms; TransformWrapper is in TransformWrapper.luau)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- The C# IDeformableMesh interface is duck-typed: `mesh` needs MaxVertexID(), IsVertex(vid), GetVertex(vid),
-- SetVertex(vid, v), HasVertexNormals(), GetVertexNormal(vid), SetVertexNormal(vid, n) and VertexIndices().
-- DMesh3 implements it. (SimpleMesh, the other C# implementation, is not ported.)
--
-- C# overloads and their Luau forms (the Vector3f overloads are the same as the Vector3d ones):
--   Translate(mesh, Vector3d v) / Translate(mesh, tx, ty, tz)       MeshTransforms.Translate(mesh, v) / (mesh, tx, ty, tz)
--   Rotate(Frame3d f, origin, q) -> Frame3d                         MeshTransforms.Rotate(f, origin, q)   (f is not modified)
--   Rotate(Vector3d pos, origin, q) -> Vector3d                     MeshTransforms.Rotate(pos, origin, q)
--   Rotate(mesh, origin, q)                                       MeshTransforms.Rotate(mesh, origin, q)
--   Scale(mesh, Vector3d scale, Vector3d origin) / (mesh, sx, sy, sz) / (mesh, s)
--                                                                 MeshTransforms.Scale(mesh, ...) (dispatch on argument types)
--   ConvertZUpToYUp / ConvertYUpToZUp / FlipLeftRightCoordSystems (Vector3d | Frame3d | mesh): dispatch on argument type
--   PerVertexTransform overloads:
--     (mesh, Func<Vector3d,Vector3d>)                               PerVertexTransform(mesh, F)          F(v) -> v
--     (mesh, TransformSequence xform)                             PerVertexTransform(mesh, xform)
--     (mesh, IEnumerable<int> vertices, Func<Vector3d,int,Vector3d>) PerVertexTransform(mesh, vertices, F)   F(v, vid) -> v
--     (mesh, vertices, Func<int,int> MapV, Func<Vector3d,int,int,Vector3d>)
--                                                                 PerVertexTransform(mesh, vertices, MapV, F)   F(v, vid, map_vid)
--     (targetMesh, sourceMesh, int[] mapV, Func<Vector3d,int,int,Vector3d>)
--                                                                 PerVertexTransform(targetMesh, sourceMesh, mapV, F)
--                                                                 (mapV is a 1-based Luau array: C# mapV[vid] -> mapV[vid + 1])
--     (mesh, Func<Vector3d,Vector3f,Vector3d>)                      PerVertexTransformVN(mesh, F)        F(v, n) -> v
--     (mesh, Func<Vector3d,Vector3f,Vector3dTuple2>)               PerVertexTransformPN(mesh, F)        F(v, n) -> {p, n}
--                                                                 (a Vector3dTuple2, i.e. a 1-based array, see VectorTuple.luau)
--   `vertices` (C# IEnumerable<int>) may be a Luau array of vertex ids or an iterator function.
--   TransformMesh(DMesh3, Matrix4d)                                MeshTransforms.TransformMesh(mesh, transform)
-- FlipLeftRightCoordSystems(Frame3d) errors (it throws NotImplementedException in C#).

local Vector3d = require(script.Parent.Vector3d)
local Frame3d = require(script.Parent.Frame3d)
local Matrix3d = require(script.Parent.Matrix3d)
local TransformWrapper = require(script.Parent.TransformWrapper)

local V3 = Vector3d.new

local MeshTransforms = {}

MeshTransforms.TransformWrapper = TransformWrapper

-- iterate over an IEnumerable<int> given as a Luau array or an iterator function
local function itr(src: any): () -> number?
	if type(src) == "function" then
		return src
	end
	local i = 0
	return function()
		i += 1
		return src[i]
	end
end

-- Translate(mesh, Vector3d v) / Translate(mesh, tx, ty, tz)
function MeshTransforms.Translate(mesh: any, tx: any, ty: number?, tz: number?)
	if type(tx) ~= "number" then
		local v = tx
		tx, ty, tz = v.x, v.y, v.z
	end
	local dx, dy, dz = tx :: number, ty :: number, tz :: number
	local NV = mesh:MaxVertexID()
	for vid = 0, NV - 1 do
		if mesh:IsVertex(vid) then
			local v = mesh:GetVertex(vid)
			mesh:SetVertex(vid, V3(v.x + dx, v.y + dy, v.z + dz))
		end
	end
end

local function RotatePoint(pos: Vector3d.Vector3d, origin: Vector3d.Vector3d, rotation: any): Vector3d.Vector3d
	return rotation * (pos - origin) + origin
end

-- Rotate(Frame3d f, origin, rotation) -> Frame3d, Rotate(Vector3d pos, origin, rotation) -> Vector3d,
-- Rotate(mesh, origin, rotation)
function MeshTransforms.Rotate(x: any, origin: Vector3d.Vector3d, rotation: any): any
	local mt = getmetatable(x)
	if mt == Frame3d then
		-- C# Frame3d is passed by value: rotate a copy
		local f = x:Clone()
		f:Rotate(rotation)
		f.Origin = RotatePoint(f.Origin, origin, rotation)
		return f
	elseif mt == Vector3d then
		return RotatePoint(x, origin, rotation)
	end
	local mesh = x
	local bHasNormals = mesh:HasVertexNormals()
	local NV = mesh:MaxVertexID()
	for vid = 0, NV - 1 do
		if mesh:IsVertex(vid) then
			local v = rotation * (mesh:GetVertex(vid) - origin) + origin
			mesh:SetVertex(vid, v)
			if bHasNormals then
				mesh:SetVertexNormal(vid, rotation * mesh:GetVertexNormal(vid))
			end
		end
	end
	return nil
end

-- Scale(mesh, Vector3d scale, Vector3d origin) / Scale(mesh, sx, sy, sz) / Scale(mesh, s)
function MeshTransforms.Scale(mesh: any, scale: any, origin: any, sz: number?)
	if type(scale) == "number" then
		if origin == nil then
			-- Scale(mesh, s)
			scale = V3(scale, scale, scale)
		else
			-- Scale(mesh, sx, sy, sz)
			scale = V3(scale, origin, sz :: number)
		end
		origin = Vector3d.Zero
	end
	local ox, oy, oz = origin.x, origin.y, origin.z
	local sx, sy, szz = scale.x, scale.y, scale.z
	local NV = mesh:MaxVertexID()
	for vid = 0, NV - 1 do
		if mesh:IsVertex(vid) then
			local v = mesh:GetVertex(vid)
			-- v -= origin; v *= scale; v += origin
			mesh:SetVertex(vid, V3((v.x - ox) * sx + ox, (v.y - oy) * sy + oy, (v.z - oz) * szz + oz))
		end
	end
end

-- Map mesh *into* local coordinates of Frame
function MeshTransforms.ToFrame(mesh: any, f: Frame3d.Frame3d)
	local NV = mesh:MaxVertexID()
	local bHasNormals = mesh:HasVertexNormals()
	for vid = 0, NV - 1 do
		if mesh:IsVertex(vid) then
			local v = mesh:GetVertex(vid)
			local vf = f:ToFrameP(v)
			mesh:SetVertex(vid, vf)
			if bHasNormals then
				local n = mesh:GetVertexNormal(vid)
				local nf = f:ToFrameV(n)
				mesh:SetVertexNormal(vid, nf)
			end
		end
	end
end

-- Map mesh *from* local frame coordinates into "world" coordinates
function MeshTransforms.FromFrame(mesh: any, f: Frame3d.Frame3d)
	local NV = mesh:MaxVertexID()
	local bHasNormals = mesh:HasVertexNormals()
	for vid = 0, NV - 1 do
		if mesh:IsVertex(vid) then
			local vf = mesh:GetVertex(vid)
			local v = f:FromFrameP(vf)
			mesh:SetVertex(vid, v)
			if bHasNormals then
				local n = mesh:GetVertexNormal(vid)
				local nf = f:FromFrameV(n)
				mesh:SetVertexNormal(vid, nf)
			end
		end
	end
end

-- shared implementation of the ConvertZUpToYUp / ConvertYUpToZUp / FlipLeftRightCoordSystems overloads
local function convert_mesh(mesh: any, convertF: (number, number, number) -> Vector3d.Vector3d)
	local NV = mesh:MaxVertexID()
	local bHasNormals = mesh:HasVertexNormals()
	for vid = 0, NV - 1 do
		if mesh:IsVertex(vid) then
			local v = mesh:GetVertex(vid)
			mesh:SetVertex(vid, convertF(v.x, v.y, v.z))
			if bHasNormals then
				local n = mesh:GetVertexNormal(vid)
				mesh:SetVertexNormal(vid, convertF(n.x, n.y, n.z))
			end
		end
	end
end

local function zup_to_yup(x: number, y: number, z: number): Vector3d.Vector3d
	return V3(x, z, -y)
end
local function yup_to_zup(x: number, y: number, z: number): Vector3d.Vector3d
	return V3(x, -z, y)
end
local function flip_lr(x: number, y: number, z: number): Vector3d.Vector3d
	return V3(x, y, -z)
end

-- ConvertZUpToYUp(Vector3d) -> Vector3d, ConvertZUpToYUp(Frame3d) -> Frame3d, ConvertZUpToYUp(mesh)
function MeshTransforms.ConvertZUpToYUp(x: any): any
	local mt = getmetatable(x)
	if mt == Vector3d then
		return zup_to_yup(x.x, x.y, x.z)
	elseif mt == Frame3d then
		local f = x
		return Frame3d.FromXYZ(
			MeshTransforms.ConvertZUpToYUp(f.Origin),
			MeshTransforms.ConvertZUpToYUp(f:X()),
			MeshTransforms.ConvertZUpToYUp(f:Y()),
			MeshTransforms.ConvertZUpToYUp(f:Z())
		)
	end
	convert_mesh(x, zup_to_yup)
	return nil
end

-- ConvertYUpToZUp(Vector3d) -> Vector3d, ConvertYUpToZUp(Frame3d) -> Frame3d, ConvertYUpToZUp(mesh)
function MeshTransforms.ConvertYUpToZUp(x: any): any
	local mt = getmetatable(x)
	if mt == Vector3d then
		return yup_to_zup(x.x, x.y, x.z)
	elseif mt == Frame3d then
		local f = x
		return Frame3d.FromXYZ(
			MeshTransforms.ConvertYUpToZUp(f.Origin),
			MeshTransforms.ConvertYUpToZUp(f:X()),
			MeshTransforms.ConvertYUpToZUp(f:Y()),
			MeshTransforms.ConvertYUpToZUp(f:Z())
		)
	end
	convert_mesh(x, yup_to_zup)
	return nil
end

-- FlipLeftRightCoordSystems(Vector3d) -> Vector3d, FlipLeftRightCoordSystems(mesh).
-- The Frame3d overload throws in C# ("this doesn't work...frame becomes broken somehow?").
function MeshTransforms.FlipLeftRightCoordSystems(x: any): any
	local mt = getmetatable(x)
	if mt == Vector3d then
		return flip_lr(x.x, x.y, x.z)
	elseif mt == Frame3d then
		error("MeshTransforms.FlipLeftRightCoordSystems: not implemented for Frame3d (frame becomes broken somehow?)")
	end
	local mesh = x
	convert_mesh(mesh, flip_lr)

	-- C#: if (mesh is DMesh3) ... else throw
	if type(mesh.ReverseOrientation) == "function" then
		mesh:ReverseOrientation(false)
	else
		error("MeshTransforms.FlipLeftRightCoordSystems: argh don't want this in IDeformableMesh...but then for SimpleMesh??")
	end
	return nil
end

function MeshTransforms.VertexNormalOffset(mesh: any, offsetDistance: number)
	local NV = mesh:MaxVertexID()
	for vid = 0, NV - 1 do
		if mesh:IsVertex(vid) then
			local newPos = mesh:GetVertex(vid) + offsetDistance * mesh:GetVertexNormal(vid)
			mesh:SetVertex(vid, newPos)
		end
	end
end

-- Apply TransformF to vertices of mesh (see header for all the overloads)
function MeshTransforms.PerVertexTransform(mesh: any, a: any, b: any?, c: any?)
	if c ~= nil then
		if type(b) == "function" then
			-- (mesh, IEnumerable<int> vertices, Func<int,int> MapV, Func<Vector3d,int,int,Vector3d> TransformF)
			-- Apply TransformF to subset of mesh vertices defined by MapV[vertices]
			local MapV, TransformF = b, c
			for vid in itr(a) do
				local map_vid = MapV(vid)
				if mesh:IsVertex(map_vid) then
					local newPos = TransformF(mesh:GetVertex(map_vid), vid, map_vid)
					mesh:SetVertex(map_vid, newPos)
				end
			end
		else
			-- (targetMesh, sourceMesh, int[] mapV, Func<Vector3d,int,int,Vector3d> TransformF)
			-- Apply TransformF to subset of mesh vertices defined by MapV[vertices]
			local targetMesh, sourceMesh, mapV, TransformF = mesh, a, b, c
			for vid in sourceMesh:VertexIndices() do
				local map_vid = mapV[vid + 1]
				if targetMesh:IsVertex(map_vid) then
					local newPos = TransformF(targetMesh:GetVertex(map_vid), vid, map_vid)
					targetMesh:SetVertex(map_vid, newPos)
				end
			end
		end
		return
	end

	if b ~= nil then
		-- (mesh, IEnumerable<int> vertices, Func<Vector3d,int,Vector3d> TransformF)
		-- Apply TransformF to subset of vertices of mesh
		local TransformF = b
		for vid in itr(a) do
			if mesh:IsVertex(vid) then
				local newPos = TransformF(mesh:GetVertex(vid), vid)
				mesh:SetVertex(vid, newPos)
			end
		end
		return
	end

	local NV = mesh:MaxVertexID()
	if type(a) == "function" then
		-- (mesh, Func<Vector3d,Vector3d> TransformF)
		local TransformF = a
		for vid = 0, NV - 1 do
			if mesh:IsVertex(vid) then
				local newPos = TransformF(mesh:GetVertex(vid))
				mesh:SetVertex(vid, newPos)
			end
		end
		return
	end

	-- (mesh, TransformSequence xform)
	-- Apply Transform to vertices and normals of mesh
	local xform = a
	if mesh:HasVertexNormals() then
		for vid = 0, NV - 1 do
			if mesh:IsVertex(vid) then
				mesh:SetVertex(vid, xform:TransformP(mesh:GetVertex(vid)))
				mesh:SetVertexNormal(vid, xform:TransformV(mesh:GetVertexNormal(vid)))
			end
		end
	else
		for vid = 0, NV - 1 do
			if mesh:IsVertex(vid) then
				mesh:SetVertex(vid, xform:TransformP(mesh:GetVertex(vid)))
			end
		end
	end
end

-- C# PerVertexTransform(IDeformableMesh mesh, Func<Vector3d, Vector3f, Vector3d> TransformF)
function MeshTransforms.PerVertexTransformVN(mesh: any, TransformF: (Vector3d.Vector3d, Vector3d.Vector3d) -> Vector3d.Vector3d)
	local NV = mesh:MaxVertexID()
	for vid = 0, NV - 1 do
		if mesh:IsVertex(vid) then
			local newPos = TransformF(mesh:GetVertex(vid), mesh:GetVertexNormal(vid))
			mesh:SetVertex(vid, newPos)
		end
	end
end

-- C# PerVertexTransform(IDeformableMesh mesh, Func<Vector3d, Vector3f, Vector3dTuple2> TransformF)
-- Apply TransformF to vertices and normals of mesh. TransformF returns {newPos, newNormal}.
function MeshTransforms.PerVertexTransformPN(mesh: any, TransformF: (Vector3d.Vector3d, Vector3d.Vector3d) -> { Vector3d.Vector3d })
	local NV = mesh:MaxVertexID()
	for vid = 0, NV - 1 do
		if mesh:IsVertex(vid) then
			local newPN = TransformF(mesh:GetVertex(vid), mesh:GetVertexNormal(vid))
			mesh:SetVertex(vid, newPN[1])
			mesh:SetVertexNormal(vid, newPN[2])
		end
	end
end

-- Apply Matrix4d transform to a mesh (vertices and normals).
-- Assumes Transform is an Affine transform.
function MeshTransforms.TransformMesh(Mesh: any, Transform: any)
	local bHasNormals = Mesh:HasVertexNormals()
	local NormalXForm = Matrix3d.Identity
	if bHasNormals then
		local ok, nx = Transform:GetAffineNormalTransform()
		NormalXForm = nx
		if ok == false then
			NormalXForm = Transform:GetAffineTransform()
		end
	end

	for vid in Mesh:VertexIndices() do
		local v = Mesh:GetVertex(vid)
		v = Transform:TransformPointAffine(v)
		Mesh:SetVertex(vid, v)
		if bHasNormals then
			local n = Mesh:GetVertexNormal(vid)
			n = NormalXForm * n
			Mesh:SetVertexNormal(vid, n)
		end
	end
end

return MeshTransforms
