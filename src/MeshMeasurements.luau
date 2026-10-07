--!native
-- Port of geometry3Sharp mesh/MeshMeasurements.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau API notes (static functions; out parameters become return values; IEnumerable arguments accept
-- an iterator function or a 1-based array):
--   MassProperties(triangle_indices, getVertexF, bodyCoords?) -> mass, center, inertia3x3
--       triangle_indices yields Index3i arrays; inertia3x3 is a nested row-major table inertia[r+1][c+1]
--   MassPropertiesMesh(mesh, bodyCoords?) -> mass, center, inertia3x3   (C# MassProperties(DMesh3, ...);
--       like C# the bodyCoords argument is ignored and world coordinates are returned)
--   VolumeArea(mesh, triangles, getVertexF?) -> Vector2d(volume, area)   (getVertexF defaults to mesh:GetVertex)
--   VertexOneRingArea(mesh, vid, bDisjoint?)
--   Centroid(vertices) / CentroidValues(values, PositionF) / CentroidMesh(mesh, bOnlyTriVertices?)
--       (Centroid(mesh, bOnlyTriVertices?) also dispatches to CentroidMesh when given a mesh)
--   Bounds(mesh, TransformF?)   (DMesh3 or IMesh)
--   BoundsV(mesh, vertexIndices, TransformF?), BoundsT(mesh, triangleIndices, TransformF?)
--   AreaT(mesh, triangleIndices), BoundsInFrame(mesh, frame, TransformF?)
--   ExtentsOnAxis(mesh, axis, TransformF?) -> Interval1d
--   ExtremeVertices(mesh, axis, TransformF?) -> Interval1i
--   Genus(mesh) -> GenusResult { Valid, Genus, HasBoundary, MultipleConnectedComponents, HasBowtieVertices }

local Vector3d = require(script.Parent.Vector3d)
local Vector2d = require(script.Parent.Vector2d)
local AxisAlignedBox3d = require(script.Parent.AxisAlignedBox3d)
local Interval1d = require(script.Parent.Interval1d)
local Interval1i = require(script.Parent.Interval1i)

local V3 = Vector3d.new

local MeshMeasurements = {}

local function each(values: any, f: (any) -> ())
	if type(values) == "function" then
		for v in values do
			f(v)
		end
	else
		for _, v in ipairs(values) do
			f(v)
		end
	end
end

-- ported from WildMagic5 Wm5PolyhedralMassProperties
--
-- Computes mass/volume, center of mass, and 3x3 intertia tensor
-- for a closed mesh. You provide an enumerator over triangles and
-- a vertex accessor.
--
-- If bodyCoords is 'true', the inertia tensor will be relative to
-- body coordinates, if 'false' it is in world coordinates.
--
-- the inertia tensor is row-major
function MeshMeasurements.MassProperties(triangle_indices: any, getVertexF: (number) -> any, bodyCoords: boolean?)
	local oneDiv6 = (1.0 / 6.0)
	local oneDiv24 = (1.0 / 24.0)
	local oneDiv60 = (1.0 / 60.0)
	local oneDiv120 = (1.0 / 120.0)

	-- order:  1, x, y, z, x^2, y^2, z^2, xy, yz, zx
	local integral = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }

	each(triangle_indices, function(tri)
		-- Get vertices of triangle i.
		local v0 = getVertexF(tri[1])
		local v1 = getVertexF(tri[2])
		local v2 = getVertexF(tri[3])

		-- Get cross product of edges and normal vector.
		local V1mV0 = v1 - v0
		local V2mV0 = v2 - v0
		local N = V1mV0:Cross(V2mV0)

		-- Compute integral terms.
		local tmp0, tmp1, tmp2
		tmp0 = v0.x + v1.x
		local f1x = tmp0 + v2.x
		tmp1 = v0.x * v0.x
		tmp2 = tmp1 + v1.x * tmp0
		local f2x = tmp2 + v2.x * f1x
		local f3x = v0.x * tmp1 + v1.x * tmp2 + v2.x * f2x
		local g0x = f2x + v0.x * (f1x + v0.x)
		local g1x = f2x + v1.x * (f1x + v1.x)
		local g2x = f2x + v2.x * (f1x + v2.x)

		tmp0 = v0.y + v1.y
		local f1y = tmp0 + v2.y
		tmp1 = v0.y * v0.y
		tmp2 = tmp1 + v1.y * tmp0
		local f2y = tmp2 + v2.y * f1y
		local f3y = v0.y * tmp1 + v1.y * tmp2 + v2.y * f2y
		local g0y = f2y + v0.y * (f1y + v0.y)
		local g1y = f2y + v1.y * (f1y + v1.y)
		local g2y = f2y + v2.y * (f1y + v2.y)

		tmp0 = v0.z + v1.z
		local f1z = tmp0 + v2.z
		tmp1 = v0.z * v0.z
		tmp2 = tmp1 + v1.z * tmp0
		local f2z = tmp2 + v2.z * f1z
		local f3z = v0.z * tmp1 + v1.z * tmp2 + v2.z * f2z
		local g0z = f2z + v0.z * (f1z + v0.z)
		local g1z = f2z + v1.z * (f1z + v1.z)
		local g2z = f2z + v2.z * (f1z + v2.z)

		-- Update integrals.
		integral[1] += N.x * f1x
		integral[2] += N.x * f2x
		integral[3] += N.y * f2y
		integral[4] += N.z * f2z
		integral[5] += N.x * f3x
		integral[6] += N.y * f3y
		integral[7] += N.z * f3z
		integral[8] += N.x * (v0.y * g0x + v1.y * g1x + v2.y * g2x)
		integral[9] += N.y * (v0.z * g0y + v1.z * g1y + v2.z * g2y)
		integral[10] += N.z * (v0.x * g0z + v1.x * g1z + v2.x * g2z)
	end)

	integral[1] *= oneDiv6
	integral[2] *= oneDiv24
	integral[3] *= oneDiv24
	integral[4] *= oneDiv24
	integral[5] *= oneDiv60
	integral[6] *= oneDiv60
	integral[7] *= oneDiv60
	integral[8] *= oneDiv120
	integral[9] *= oneDiv120
	integral[10] *= oneDiv120

	-- mass
	local mass = integral[1]

	-- center of mass
	local center = V3(integral[2], integral[3], integral[4]) / mass

	-- inertia3x3 relative to world origin
	local I = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } }
	I[1][1] = integral[6] + integral[7]
	I[1][2] = -integral[8]
	I[1][3] = -integral[10]
	I[2][1] = I[1][2]
	I[2][2] = integral[5] + integral[7]
	I[2][3] = -integral[9]
	I[3][1] = I[1][3]
	I[3][2] = I[2][3]
	I[3][3] = integral[5] + integral[6]

	-- inertia3x3 relative to center of mass
	if bodyCoords then
		I[1][1] -= mass * (center.y * center.y + center.z * center.z)
		I[1][2] += mass * center.x * center.y
		I[1][3] += mass * center.z * center.x
		I[2][1] = I[1][2]
		I[2][2] -= mass * (center.z * center.z + center.x * center.x)
		I[2][3] += mass * center.y * center.z
		I[3][1] = I[1][3]
		I[3][2] = I[2][3]
		I[3][3] -= mass * (center.x * center.x + center.y * center.y)
	end
	return mass, center, I
end

function MeshMeasurements.MassPropertiesMesh(mesh, bodyCoords: boolean?)
	return MeshMeasurements.MassProperties(mesh:Triangles(), function(vID)
		return mesh:GetVertex(vID)
	end, false)
end

-- Compute volume and surface area of triangles of mesh.
-- Return value is (volume,area)
-- Note that if triangles don't define closed region, volume is probably nonsense...
function MeshMeasurements.VolumeArea(mesh, triangles: any, getVertexF: ((number) -> any)?)
	local getV = getVertexF or function(vid)
		return mesh:GetVertex(vid)
	end
	local mass_integral = 0.0
	local area_sum = 0
	each(triangles, function(tid)
		local tri = mesh:GetTriangle(tid)
		-- Get vertices of triangle i.
		local v0 = getV(tri[1])
		local v1 = getV(tri[2])
		local v2 = getV(tri[3])

		-- Get cross product of edges and (un-normalized) normal vector.
		local V1mV0 = v1 - v0
		local V2mV0 = v2 - v0
		local N = V1mV0:Cross(V2mV0)

		area_sum += 0.5 * N:Length()

		local tmp0 = v0.x + v1.x
		local f1x = tmp0 + v2.x
		mass_integral += N.x * f1x
	end)

	return Vector2d.new(mass_integral * (1.0 / 6.0), area_sum)
end

-- Compute area of one-ring of mesh vertex by summing triangle areas.
-- If bDisjoint = true, we multiple each triangle area by 1/3
function MeshMeasurements.VertexOneRingArea(mesh, vid: number, bDisjoint: boolean?): number
	local sum = 0
	local mul = (bDisjoint ~= false) and (1.0 / 3.0) or 1.0
	for tid in mesh:VtxTrianglesItr(vid) do
		sum += mesh:GetTriArea(tid) * mul
	end
	return sum
end

function MeshMeasurements.CentroidValues(values: any, PositionF: (any) -> any)
	local centroid = Vector3d.Zero
	local N = 0
	each(values, function(t)
		centroid = centroid + PositionF(t)
		N += 1
	end)
	return centroid / N
end

function MeshMeasurements.CentroidMesh(mesh, bOnlyTriVertices: boolean?)
	if bOnlyTriVertices ~= false then
		local centroid = Vector3d.Zero
		local N = 0
		for vid in mesh:VertexIndices() do
			if mesh:GetVtxEdgeCount(vid) > 0 then
				centroid = centroid + mesh:GetVertex(vid)
				N += 1
			end
		end
		return centroid / N
	else
		return MeshMeasurements.Centroid(mesh:Vertices())
	end
end

-- Centroid(IEnumerable<Vector3d> vertices), or Centroid(DMesh3 mesh, bOnlyTriVertices)
function MeshMeasurements.Centroid(vertices: any, bOnlyTriVertices: boolean?)
	if type(vertices) == "table" and type(vertices.VertexIndices) == "function" then
		return MeshMeasurements.CentroidMesh(vertices, bOnlyTriVertices)
	end
	local centroid = Vector3d.Zero
	local N = 0
	each(vertices, function(v)
		centroid = centroid + v
		N += 1
	end)
	return centroid / N
end

-- Bounds(DMesh3 mesh, TransformF) / Bounds(IMesh mesh, TransformF)
function MeshMeasurements.Bounds(mesh, TransformF: ((any) -> any)?)
	local bounds = AxisAlignedBox3d.Empty()
	if TransformF == nil then
		for vID in mesh:VertexIndices() do
			bounds:Contain(mesh:GetVertex(vID))
		end
	else
		for vID in mesh:VertexIndices() do
			local vT = TransformF(mesh:GetVertex(vID))
			bounds:Contain(vT)
		end
	end
	return bounds
end

function MeshMeasurements.BoundsV(mesh, vertexIndices: any, TransformF: ((any) -> any)?)
	local bounds = AxisAlignedBox3d.Empty()
	if TransformF == nil then
		each(vertexIndices, function(vid)
			bounds:Contain(mesh:GetVertex(vid))
		end)
	else
		each(vertexIndices, function(vid)
			bounds:Contain(TransformF(mesh:GetVertex(vid)))
		end)
	end
	return bounds
end

function MeshMeasurements.BoundsT(mesh, triangleIndices: any, TransformF: ((any) -> any)?)
	local bounds = AxisAlignedBox3d.Empty()
	if TransformF == nil then
		each(triangleIndices, function(tid)
			local tri = mesh:GetTriangle(tid)
			for j = 1, 3 do
				bounds:Contain(mesh:GetVertex(tri[j]))
			end
		end)
	else
		each(triangleIndices, function(tid)
			local tri = mesh:GetTriangle(tid)
			for j = 1, 3 do
				bounds:Contain(TransformF(mesh:GetVertex(tri[j])))
			end
		end)
	end
	return bounds
end

function MeshMeasurements.AreaT(mesh, triangleIndices: any): number
	local area = 0
	each(triangleIndices, function(tid)
		area += mesh:GetTriArea(tid)
	end)
	return area
end

-- calculate extents of mesh along axes of frame, with optional transform
function MeshMeasurements.BoundsInFrame(mesh, frame, TransformF: ((any) -> any)?)
	local bounds = AxisAlignedBox3d.Empty()
	if TransformF == nil then
		for v in mesh:Vertices() do
			bounds:Contain(frame:ToFrameP(v))
		end
	else
		for v in mesh:Vertices() do
			local vT = TransformF(v)
			bounds:Contain(frame:ToFrameP(vT))
		end
	end
	return bounds
end

-- Calculate extents of mesh along an axis, with optional transform (DMesh3 or IMesh)
function MeshMeasurements.ExtentsOnAxis(mesh, axis, TransformF: ((any) -> any)?)
	local extent = Interval1d.Empty()
	if TransformF == nil then
		for vid in mesh:VertexIndices() do
			extent:Contain(mesh:GetVertex(vid):Dot(axis))
		end
	else
		for vid in mesh:VertexIndices() do
			local vT = TransformF(mesh:GetVertex(vid))
			extent:Contain(vT:Dot(axis))
		end
	end
	return extent
end

-- Calculate the two most extreme vertices along an axis, with optional transform
function MeshMeasurements.ExtremeVertices(mesh, axis, TransformF: ((any) -> any)?)
	local extent = Interval1d.Empty()
	local extreme = Interval1i.new(-1, -1)
	for vid in mesh:VertexIndices() do
		local v = mesh:GetVertex(vid)
		if TransformF ~= nil then
			v = TransformF(v)
		end
		local t = v:Dot(axis)
		if t < extent.a then
			extent.a = t
			extreme.a = vid
		elseif t > extent.b then
			extent.b = t
			extreme.b = vid
		end
	end
	return extreme
end

-- returns GenusResult { Valid, Genus, HasBoundary, MultipleConnectedComponents, HasBowtieVertices }
function MeshMeasurements.Genus(mesh)
	local result = { Valid = false, Genus = -1, HasBoundary = false, MultipleConnectedComponents = false, HasBowtieVertices = false }

	if not mesh:CachedIsClosed() then
		result.HasBoundary = true
		return result
	end

	local MeshConnectedComponents = require(script.Parent.MeshConnectedComponents) :: any
	local compT = MeshConnectedComponents.new(mesh)
	compT:FindConnectedT()
	if compT:Count() > 1 then
		result.MultipleConnectedComponents = true
		return result
	end
	local isolated_verts = 0
	for vid in mesh:VertexIndices() do
		if mesh:IsBowtieVertex(vid) then
			result.HasBowtieVertices = true
			return result
		end
		if mesh:GetVtxTriangleCount(vid) == 0 then
			isolated_verts += 1
		end
	end

	local V = mesh:VertexCount() - isolated_verts
	local F = mesh:TriangleCount()
	local E = mesh:EdgeCount()

	-- C# integer division (2 - (V + F - E)) / 2 truncates toward zero
	local num = 2 - (V + F - E)
	result.Genus = (num >= 0) and (num // 2) or -(-num // 2)
	result.Valid = true
	return result
end

return MeshMeasurements
