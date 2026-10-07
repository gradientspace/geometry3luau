--!native
-- Port of geometry3Sharp math/BoundsUtil.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau API notes (static functions, C# overloads get distinct names):
--   BoundsMeshes(meshes)                     Bounds(IEnumerable<DMesh3>)   (array or iterator of meshes)
--   BoundsPointSet(source)                   Bounds(IPointSet)             (VertexIndices(), GetVertex(vid))
--   BoundsTriangle(tri)                      Bounds(ref Triangle3d)
--   BoundsVectors(v0, v1, v2)                Bounds(ref Vector3d v0, v1, v2)
--   BoundsTransformed(box, TransformF)       Bounds(ref AxisAlignedBox3d, Func<Vector3d,Vector3d>)  (returns a copy if TransformF is nil)
--   BoundsPositions(positions)               Bounds(IEnumerable<Vector3d>)  (array or iterator)
--   BoundsValues(values, PositionF)          Bounds<T>(IEnumerable<T>, Func<T,Vector3d>)
--   BoundsTransformSequence(values, xform)   Bounds(IEnumerable<Vector3d>, TransformSequence) (xform:TransformP(v))
--   BoundsInFrame(values, frame)
--   TrianglesContained(mesh, ContainF, AddF, nMode?)
--   Bounds(...) dispatches: Triangle3d -> BoundsTriangle, (v0, v1, v2) -> BoundsVectors,
--     (box, TransformF) -> BoundsTransformed, (values, PositionF) -> BoundsValues,
--     array/iterator of DMesh3 -> BoundsMeshes, point set -> BoundsPointSet, else BoundsPositions.
-- Not ported: the 2D overloads returning AxisAlignedBox2d/2f (AxisAlignedBox2d is not part of this port).

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local AxisAlignedBox3d = require(script.Parent.AxisAlignedBox3d)
local BitArray = require(script.Parent.BitArray)

local BoundsUtil = {}

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

function BoundsUtil.BoundsMeshes(meshes: any)
	local bounds = AxisAlignedBox3d.Empty()
	each(meshes, function(mesh)
		bounds:Contain(mesh:CachedBounds())
	end)
	return bounds
end

function BoundsUtil.BoundsPointSet(source)
	local bounds = AxisAlignedBox3d.Empty()
	for vid in source:VertexIndices() do
		bounds:Contain(source:GetVertex(vid))
	end
	return bounds
end

function BoundsUtil.BoundsVectors(v0, v1, v2)
	local minx, maxx = MathUtil.MinMax(v0.x, v1.x, v2.x)
	local miny, maxy = MathUtil.MinMax(v0.y, v1.y, v2.y)
	local minz, maxz = MathUtil.MinMax(v0.z, v1.z, v2.z)
	return AxisAlignedBox3d.new(minx, miny, minz, maxx, maxy, maxz)
end

function BoundsUtil.BoundsTriangle(tri)
	return BoundsUtil.BoundsVectors(tri.V0, tri.V1, tri.V2)
end

-- AABB of transformed AABB (corners)
function BoundsUtil.BoundsTransformed(boxIn, TransformF: ((any) -> any)?)
	if TransformF == nil then
		return boxIn:Clone()
	end
	local box = AxisAlignedBox3d.FromPoint(TransformF(boxIn:Corner(0)))
	for i = 1, 7 do
		box:Contain(TransformF(boxIn:Corner(i)))
	end
	return box
end

function BoundsUtil.BoundsPositions(positions: any)
	local box = AxisAlignedBox3d.Empty()
	each(positions, function(v)
		box:Contain(v)
	end)
	return box
end

function BoundsUtil.BoundsValues(values: any, PositionF: (any) -> any)
	local box = AxisAlignedBox3d.Empty()
	each(values, function(t)
		box:Contain(PositionF(t))
	end)
	return box
end

-- compute axis-aligned bounds of set of points after transforming
function BoundsUtil.BoundsTransformSequence(values: any, xform)
	local box = AxisAlignedBox3d.Empty()
	each(values, function(v)
		box:Contain(xform:TransformP(v))
	end)
	return box
end

-- compute axis-aligned bounds of set of points after transforming into frame f
function BoundsUtil.BoundsInFrame(values: any, f)
	local box = AxisAlignedBox3d.Empty()
	each(values, function(v)
		box:Contain(f:ToFrameP(v))
	end)
	return box
end

-- dispatching version of the C# Bounds() overloads (see header)
function BoundsUtil.Bounds(a: any, b: any?, c: any?)
	if type(a) == "table" and a.V0 ~= nil and a.V1 ~= nil and a.V2 ~= nil and b == nil then
		return BoundsUtil.BoundsTriangle(a)
	elseif c ~= nil then
		return BoundsUtil.BoundsVectors(a, b, c)
	elseif type(a) == "table" and a.Min ~= nil and a.Max ~= nil then
		return BoundsUtil.BoundsTransformed(a, b)
	elseif type(b) == "function" then
		return BoundsUtil.BoundsValues(a, b)
	elseif type(a) == "table" and type(a.VertexIndices) == "function" and type(a.GetVertex) == "function" then
		return BoundsUtil.BoundsPointSet(a)
	elseif type(a) == "table" and a[1] ~= nil and type(a[1]) == "table" and a[1].CachedBounds ~= nil then
		return BoundsUtil.BoundsMeshes(a)
	end
	return BoundsUtil.BoundsPositions(a)
end

-- Modes: 0: centroids, 1: any vertex, 2: 2 vertices, 3: all vertices
-- ContainF should return true if 3D position is in set (eg inside box, etc)
-- If mode = 0, will be called with (centroid, tri_idx)
-- If mode = 1/2/3, will be called with (vtx_pos, vtx_idx)
-- AddF is called with triangle IDs that are in set
function BoundsUtil.TrianglesContained(mesh, ContainF: (any, number) -> boolean, AddF: (number) -> (), nMode: number?)
	local mode = nMode or 0
	local inV = nil
	if mode ~= 0 then
		inV = BitArray.new(mesh:MaxVertexID())
		for vid in mesh:VertexIndices() do
			if ContainF(mesh:GetVertex(vid), vid) then
				inV:Set(vid, true)
			end
		end
	end

	for tid in mesh:TriangleIndices() do
		local tri = mesh:GetTriangle(tid)

		local bIn = false
		if mode == 0 then
			if ContainF(mesh:GetTriCentroid(tid), tid) then
				bIn = true
			end
		else
			local countIn = (inV:Get(tri[1]) and 1 or 0) + (inV:Get(tri[2]) and 1 or 0) + (inV:Get(tri[3]) and 1 or 0)
			bIn = (countIn >= mode)
		end

		if bIn then
			AddF(tid)
		end
	end
end

local _ = Vector3d
return BoundsUtil
