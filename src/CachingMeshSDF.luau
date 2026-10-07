--!native
-- Port of geometry3Sharp implicit/CachingMeshSDF.cs (class CachingMeshSDF)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- [RMS] this is variant of MeshSignedDistanceGrid that does lazy evaluation of actual distances,
-- using mesh spatial data structure. This is much faster if we are doing continuation-method
-- marching cubes as only values on surface will be computed!
--
-- Initialize() computes the grid dimensions and the inside/outside signs (by ray-intersection counting on
-- the grid, like MeshSignedDistanceGrid). Distances are then computed on demand by GetValue().
-- Use CachingMeshSDFImplicit to evaluate it as a (trilinear) implicit function.
--
-- This code is based on the C++ implementation found at https://github.com/christopherbatty/SDFGen
-- Original license was public domain. Permission granted by Christopher Batty to include C# port under Boost license.
--
-- Luau port notes:
--   * Constructor: CachingMeshSDF.new(mesh, cellSize, spatial). `spatial` (DMeshAABBTree3) must provide
--     FindNearestTriangleDistSqr(p, maxDist) -> tid, distSqr  (C# FindNearestTriangle(p, out dsqr, maxDist))
--   * Enum: CachingMeshSDF.InsideModes.{CrossingCount, ParityCount}
--   * Properties as methods: Dimensions(), Grid(), GridOrigin(), ClosestTriGrid(), IntersectionsGrid()
--   * Indexers: this[i,j,k] -> Get(i, j, k), this[Vector3i] -> GetV(idx)  (raw cached values, like C#)
--   * GetValue(Vector3i) is the C# method; GetValueIJK(i, j, k) is an allocation-free variant.
--   * C# float values are doubles here.

local Vector3d = require(script.Parent.Vector3d)
local Vector3i = require(script.Parent.Vector3i)
local MathUtil = require(script.Parent.MathUtil)
local DenseGrid3f = require(script.Parent.DenseGrid3f)
local DenseGrid3i = require(script.Parent.DenseGrid3i)
local MeshSignedDistanceGrid = require(script.Parent.MeshSignedDistanceGrid)

local V3 = Vector3d.new
local TruncInt = MathUtil.TruncInt

local CachingMeshSDF = {}
CachingMeshSDF.__index = CachingMeshSDF
CachingMeshSDF.ClassName = "CachingMeshSDF"

local InsideModes = MeshSignedDistanceGrid.InsideModes
CachingMeshSDF.InsideModes = InsideModes

function CachingMeshSDF.new(mesh, cellSize: number, spatial)
	return setmetatable({
		Mesh = mesh,
		Spatial = spatial,
		CellSize = cellSize,
		-- Bounds of grid will be expanded this much in positive and negative directions.
		ExpandBounds = Vector3d.Zero,
		-- max distance away from surface that we might need to evaluate
		MaxOffsetDistance = 0,
		-- (selects the C# parallel code paths; sequential in Luau)
		UseParallel = true,
		-- should we try to compute signs? if not, grid remains unsigned
		ComputeSigns = true,
		-- What counts as "inside" the mesh (see MeshSignedDistanceGrid)
		InsideMode = InsideModes.ParityCount,
		-- Implementation computes the triangle closest to each grid cell, can return this grid if desired
		WantClosestTriGrid = false,
		-- grid of per-cell crossing or parity counts
		WantIntersectionsGrid = false,
		-- if this function returns true, we should abort calculation
		CancelF = function()
			return false
		end,
		DebugPrint = false,

		-- computed results
		grid_origin = Vector3d.Zero,
		grid = nil :: any,
		closest_tri_grid = nil :: any,
		intersections_grid = nil :: any,
		UpperBoundDistance = 0,
		MaxDistQueryDist = 0,
	}, CachingMeshSDF)
end

function CachingMeshSDF:Initialize()
	-- figure out origin & dimensions
	local bounds = self.Mesh:CachedBounds()
	local CellSize = self.CellSize

	local fBufferWidth = math.max(4 * CellSize, 2 * self.MaxOffsetDistance + 2 * CellSize)
	local e = self.ExpandBounds
	local origin = V3(bounds.Min.x - fBufferWidth - e.x, bounds.Min.y - fBufferWidth - e.y, bounds.Min.z - fBufferWidth - e.z)
	local max = V3(bounds.Max.x + fBufferWidth + e.x, bounds.Max.y + fBufferWidth + e.y, bounds.Max.z + fBufferWidth + e.z)
	self.grid_origin = origin
	local ni = TruncInt((max.x - origin.x) / CellSize) + 1
	local nj = TruncInt((max.y - origin.y) / CellSize) + 1
	local nk = TruncInt((max.z - origin.z) / CellSize) + 1

	self.UpperBoundDistance = (ni + nj + nk) * CellSize
	self.grid = DenseGrid3f.new(ni, nj, nk, self.UpperBoundDistance)

	self.MaxDistQueryDist = self.MaxOffsetDistance + (2 * CellSize * MathUtil.SqrtTwo)

	-- closest triangle id for each grid cell
	if self.WantClosestTriGrid then
		self.closest_tri_grid = DenseGrid3i.new(ni, nj, nk, -1)
	end

	-- intersection_count(i,j,k) is # of tri intersections in (i-1,i]x{j}x{k}
	local intersection_count = DenseGrid3i.new(ni, nj, nk, 0)

	if self.ComputeSigns == true then
		self:compute_intersections(origin, CellSize, ni, nj, nk, intersection_count)
		if self.CancelF() then
			return
		end

		-- then figure out signs (inside/outside) from intersection counts
		self:compute_signs(ni, nj, nk, self.grid, intersection_count)
		if self.CancelF() then
			return
		end

		if self.WantIntersectionsGrid then
			self.intersections_grid = intersection_count
		end
	end
end

-- lazily-evaluated value at grid index (i, j, k)
function CachingMeshSDF:GetValueIJK(i: number, j: number, k: number): number
	local grid = self.grid
	local idx = i + grid.ni * (j + grid.nj * k) + 1
	local buf = grid.Buffer
	local f = buf[idx]
	local ub = self.UpperBoundDistance
	if f == ub or f == -ub then
		local o, c = self.grid_origin, self.CellSize
		local px, py, pz = i * c + o.x, j * c + o.y, k * c + o.z -- cell_center(idx)

		local sign = MathUtil.Sign(f)

		local spatial = self.Spatial
		local near_tid, dsqr
		if spatial.FindNearestTriangleXYZ ~= nil then
			near_tid, dsqr = spatial:FindNearestTriangleXYZ(px, py, pz, self.MaxDistQueryDist)
		else
			near_tid, dsqr = spatial:FindNearestTriangleDistSqr(V3(px, py, pz), self.MaxDistQueryDist)
		end
		if near_tid == -1 then
			f += 0.0001
		else
			f = sign * math.sqrt(dsqr)
		end

		buf[idx] = f
		if self.closest_tri_grid ~= nil then
			self.closest_tri_grid.Buffer[idx] = near_tid
		end
	end
	return f
end

function CachingMeshSDF:GetValue(idx): number
	return self:GetValueIJK(idx.x, idx.y, idx.z)
end

function CachingMeshSDF:Dimensions()
	return Vector3i.new(self.grid.ni, self.grid.nj, self.grid.nk)
end

-- SDF grid available after calling Initialize()
function CachingMeshSDF:Grid()
	return self.grid
end

-- Origin of the SDF grid, in same coordinates as mesh
function CachingMeshSDF:GridOrigin()
	return self.grid_origin
end

function CachingMeshSDF:ClosestTriGrid()
	if self.WantClosestTriGrid == false then
		error("Set WantClosestTriGrid=true to return this value")
	end
	return self.closest_tri_grid
end

function CachingMeshSDF:IntersectionsGrid()
	if self.WantIntersectionsGrid == false then
		error("Set WantIntersectionsGrid=true to return this value")
	end
	return self.intersections_grid
end

function CachingMeshSDF:Get(i: number, j: number, k: number): number
	return self.grid:Get(i, j, k)
end
function CachingMeshSDF:GetV(idx): number
	return self.grid:Get(idx.x, idx.y, idx.z)
end

function CachingMeshSDF:CellCenter(i: number, j: number, k: number)
	return self:cell_center(Vector3i.new(i, j, k))
end
function CachingMeshSDF:cell_center(ijk)
	local o, c = self.grid_origin, self.CellSize
	return V3(ijk.x * c + o.x, ijk.y * c + o.y, ijk.z * c + o.z)
end

-- fill the intersection grid w/ number of intersections in each cell
function CachingMeshSDF:compute_intersections(origin, dx: number, ni: number, nj: number, nk: number, intersection_count)
	MeshSignedDistanceGrid.compute_intersections_static(
		self.Mesh,
		self.InsideMode,
		self.CancelF,
		origin,
		dx,
		ni,
		nj,
		nk,
		intersection_count
	)
end

-- iterate through each x-row of grid and set unsigned distances to be negative
-- inside the mesh, based on the intersection_counts
function CachingMeshSDF:compute_signs(ni: number, nj: number, nk: number, distances, intersection_counts)
	MeshSignedDistanceGrid.compute_signs_static(self.InsideMode, self.UseParallel, self.CancelF, ni, nj, nk, distances, intersection_counts)
end

CachingMeshSDF.orientation = MeshSignedDistanceGrid.orientation
CachingMeshSDF.point_in_triangle_2d = MeshSignedDistanceGrid.point_in_triangle_2d

return CachingMeshSDF
