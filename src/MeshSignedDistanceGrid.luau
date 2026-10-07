--!native
-- Port of geometry3Sharp spatial/MeshSignedDistanceGrid.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Compute discretely-sampled (ie gridded) signed distance field for a mesh.
-- The basic approach is, first compute exact distances in a narrow band, and then extend out to rest of
-- grid using fast "sweeping" (ie like a distance transform). The resulting unsigned grid is then signed
-- using ray-intersection counting, which is also computed on the grid, so no BVH is necessary.
--
-- If you set ComputeMode to NarrowBandOnly, result is a narrow-band signed distance field.
-- This is quite a bit faster as the sweeping is the most computationally-intensive step.
--
-- Caveats:
--  - the "narrow band" is based on triangle bounding boxes, so it is not necessarily
--    that "narrow" if you have large triangles on a diagonal to grid axes
--
-- This code is based on the C++ implementation found at https://github.com/christopherbatty/SDFGen
-- Original license was public domain. Permission granted by Christopher Batty to include C# port under Boost license.
--
-- Luau port notes:
--   * Constructor: MeshSignedDistanceGrid.new(mesh, cellSize, spatial?)   (spatial = DMeshAABBTree3, optional;
--     needs FindNearestTriangle(p, maxDist?) -> tid; FindNearestTriangleXYZ(x, y, z, maxDist?) is used if present)
--   * Enums: MeshSignedDistanceGrid.ComputeModes.{FullGrid, NarrowBandOnly, NarrowBand_SpatialFloodFill},
--            MeshSignedDistanceGrid.InsideModes.{CrossingCount, ParityCount}
--   * Properties as methods: Dimensions(), Grid(), GridOrigin(), ClosestTriGrid(), IntersectionsGrid()
--   * Indexers: this[i,j,k] -> Get(i, j, k), this[Vector3i] -> GetV(idx)
--   * C# float fields/arithmetic (CellSize, grid origin, stored distances) are doubles here, so values can
--     differ from C# in the last float bits.
--   * UseParallel only selects between the C# "serial" and "parallel" code paths; both run sequentially.
--     (NarrowBand_SpatialFloodFill still requires UseParallel = true, like the C#.)
--   * C# Math.Min/Math.Max NaN semantics are preserved in the distance helpers (degenerate triangles).
--   * Static helpers: point_segment_distance(x0, x1, x2), point_triangle_distance(x0, x1, x2, x3) (Vector3d args),
--     scalar variants point_segment_distance_xyz / point_triangle_distance_xyz, orientation(x1, y1, x2, y2) ->
--     sign, twice_signed_area, point_in_triangle_2d(x0, y0, x1, y1, x2, y2, x3, y3) -> bool, a, b, c

local Vector3d = require(script.Parent.Vector3d)
local Vector3i = require(script.Parent.Vector3i)
local MathUtil = require(script.Parent.MathUtil)
local DenseGrid3f = require(script.Parent.DenseGrid3f)
local DenseGrid3i = require(script.Parent.DenseGrid3i)
local IndexUtil = require(script.Parent.IndexUtil)

local V3 = Vector3d.new
local sqrt, floor, ceil = math.sqrt, math.floor, math.ceil
local TruncInt = MathUtil.TruncInt

local MeshSignedDistanceGrid = {}
MeshSignedDistanceGrid.__index = MeshSignedDistanceGrid
MeshSignedDistanceGrid.ClassName = "MeshSignedDistanceGrid"

local ComputeModes = table.freeze({ FullGrid = 0, NarrowBandOnly = 1, NarrowBand_SpatialFloodFill = 2 })
local InsideModes = table.freeze({ CrossingCount = 0, ParityCount = 1 })
MeshSignedDistanceGrid.ComputeModes = ComputeModes
MeshSignedDistanceGrid.InsideModes = InsideModes

-- C# Math.Min / Math.Max (NaN if either argument is NaN)
local function cs_min(a: number, b: number): number
	if a < b then
		return a
	end
	if a ~= a then
		return a
	end
	return b
end
local function cs_max(a: number, b: number): number
	if a > b then
		return a
	end
	if a ~= a then
		return a
	end
	return b
end

-- spatial:FindNearestTriangle(Vector3d(x, y, z), maxDist), using the allocation-free XYZ query if available
local function find_nearest_tri(spatial: any, x: number, y: number, z: number, maxDist: number?): number
	if spatial.FindNearestTriangleXYZ ~= nil then
		local tid = spatial:FindNearestTriangleXYZ(x, y, z, maxDist)
		return tid
	end
	return spatial:FindNearestTriangle(V3(x, y, z), maxDist)
end
MeshSignedDistanceGrid.find_nearest_tri = find_nearest_tri

local function clampi(f: number, low: number, high: number): number
	if f < low then
		return low
	elseif f > high then
		return high
	end
	return f
end

local function min3(a: number, b: number, c: number): number
	return math.min(a, math.min(b, c))
end
local function max3(a: number, b: number, c: number): number
	return math.max(a, math.max(b, c))
end

----------------------------------------------------------------------------------------------------
-- static distance helpers
----------------------------------------------------------------------------------------------------

-- find distance x0 is from segment x1-x2
local function point_segment_distance_xyz(
	x0: number,
	y0: number,
	z0: number,
	x1: number,
	y1: number,
	z1: number,
	x2: number,
	y2: number,
	z2: number
): number
	local dx, dy, dz = x2 - x1, y2 - y1, z2 - z1
	local m2 = dx * dx + dy * dy + dz * dz
	-- find parameter value of closest point on segment
	local s12 = (dx * (x2 - x0) + dy * (y2 - y0) + dz * (z2 - z0)) / m2
	if s12 < 0 then
		s12 = 0
	elseif s12 > 1 then
		s12 = 1
	end
	-- and find the distance: x0.Distance(s12 * x1 + (1 - s12) * x2)
	local px = s12 * x1 + (1 - s12) * x2
	local py = s12 * y1 + (1 - s12) * y2
	local pz = s12 * z1 + (1 - s12) * z2
	local ex, ey, ez = px - x0, py - y0, pz - z0
	return sqrt(ex * ex + ey * ey + ez * ez)
end

-- find distance x0 is from triangle x1-x2-x3
local function point_triangle_distance_xyz(
	x0: number,
	y0: number,
	z0: number,
	x1: number,
	y1: number,
	z1: number,
	x2: number,
	y2: number,
	z2: number,
	x3: number,
	y3: number,
	z3: number
): number
	-- first find barycentric coordinates of closest point on infinite plane
	local x13x, x13y, x13z = x1 - x3, y1 - y3, z1 - z3
	local x23x, x23y, x23z = x2 - x3, y2 - y3, z2 - z3
	local x03x, x03y, x03z = x0 - x3, y0 - y3, z0 - z3
	local m13 = x13x * x13x + x13y * x13y + x13z * x13z
	local m23 = x23x * x23x + x23y * x23y + x23z * x23z
	local d = x13x * x23x + x13y * x23y + x13z * x23z
	local invdet = 1.0 / cs_max(m13 * m23 - d * d, 1e-30)
	local a = x13x * x03x + x13y * x03y + x13z * x03z
	local b = x23x * x03x + x23y * x03y + x23z * x03z
	-- the barycentric coordinates themselves
	local w23 = invdet * (m23 * a - d * b)
	local w31 = invdet * (m13 * b - d * a)
	local w12 = 1 - w23 - w31
	if w23 >= 0 and w31 >= 0 and w12 >= 0 then -- if we're inside the triangle
		local px = w23 * x1 + w31 * x2 + w12 * x3
		local py = w23 * y1 + w31 * y2 + w12 * y3
		local pz = w23 * z1 + w31 * z2 + w12 * z3
		local ex, ey, ez = px - x0, py - y0, pz - z0
		return sqrt(ex * ex + ey * ey + ez * ez)
	else -- we have to clamp to one of the edges
		if w23 > 0 then -- this rules out edge 2-3 for us
			return cs_min(
				point_segment_distance_xyz(x0, y0, z0, x1, y1, z1, x2, y2, z2),
				point_segment_distance_xyz(x0, y0, z0, x1, y1, z1, x3, y3, z3)
			)
		elseif w31 > 0 then -- this rules out edge 1-3
			return cs_min(
				point_segment_distance_xyz(x0, y0, z0, x1, y1, z1, x2, y2, z2),
				point_segment_distance_xyz(x0, y0, z0, x2, y2, z2, x3, y3, z3)
			)
		else -- w12 must be >0, ruling out edge 1-2
			return cs_min(
				point_segment_distance_xyz(x0, y0, z0, x1, y1, z1, x3, y3, z3),
				point_segment_distance_xyz(x0, y0, z0, x2, y2, z2, x3, y3, z3)
			)
		end
	end
end

MeshSignedDistanceGrid.point_segment_distance_xyz = point_segment_distance_xyz
MeshSignedDistanceGrid.point_triangle_distance_xyz = point_triangle_distance_xyz

function MeshSignedDistanceGrid.point_segment_distance(x0, x1, x2): number
	return point_segment_distance_xyz(x0.x, x0.y, x0.z, x1.x, x1.y, x1.z, x2.x, x2.y, x2.z)
end

function MeshSignedDistanceGrid.point_triangle_distance(x0, x1, x2, x3): number
	return point_triangle_distance_xyz(x0.x, x0.y, x0.z, x1.x, x1.y, x1.z, x2.x, x2.y, x2.z, x3.x, x3.y, x3.z)
end

-- calculate twice signed area of triangle (0,0)-(x1,y1)-(x2,y2)
-- return an SOS-determined sign (-1, +1, or 0 only if it's a truly degenerate triangle)
-- C# orientation(x1, y1, x2, y2, out twice_signed_area) -> sign, twice_signed_area
local function orientation(x1: number, y1: number, x2: number, y2: number): (number, number)
	local twice_signed_area = y1 * x2 - x1 * y2
	if twice_signed_area > 0 then
		return 1, twice_signed_area
	elseif twice_signed_area < 0 then
		return -1, twice_signed_area
	elseif y2 > y1 then
		return 1, twice_signed_area
	elseif y2 < y1 then
		return -1, twice_signed_area
	elseif x1 > x2 then
		return 1, twice_signed_area
	elseif x1 < x2 then
		return -1, twice_signed_area
	end
	return 0, twice_signed_area -- only true when x1==x2 and y1==y2
end
MeshSignedDistanceGrid.orientation = orientation

-- robust test of (x0,y0) in the triangle (x1,y1)-(x2,y2)-(x3,y3)
-- if true is returned, the barycentric coordinates are returned as a, b, c.
local function point_in_triangle_2d(
	x0: number,
	y0: number,
	x1: number,
	y1: number,
	x2: number,
	y2: number,
	x3: number,
	y3: number
): (boolean, number, number, number)
	x1 -= x0
	x2 -= x0
	x3 -= x0
	y1 -= y0
	y2 -= y0
	y3 -= y0
	local signa, a = orientation(x2, y2, x3, y3)
	if signa == 0 then
		return false, 0, 0, 0
	end
	local signb, b = orientation(x3, y3, x1, y1)
	if signb ~= signa then
		return false, a, 0, 0
	end
	local signc, c = orientation(x1, y1, x2, y2)
	if signc ~= signa then
		return false, a, b, 0
	end
	local sum = a + b + c
	-- if the SOS signs match and are nonzero, there's no way all of a, b, and c are zero.
	if sum == 0 then
		error("MakeNarrowBandLevelSet.point_in_triangle_2d: badness!")
	end
	return true, a / sum, b / sum, c / sum
end
MeshSignedDistanceGrid.point_in_triangle_2d = point_in_triangle_2d

----------------------------------------------------------------------------------------------------
-- class
----------------------------------------------------------------------------------------------------

function MeshSignedDistanceGrid.new(mesh, cellSize: number, spatial: any?)
	return setmetatable({
		Mesh = mesh,
		Spatial = spatial,
		CellSize = cellSize,
		-- Width of the band around triangles for which exact distances are computed
		-- (In fact this is conservative, the band is often larger locally)
		ExactBandWidth = 1,
		-- Bounds of grid will be expanded this much in positive and negative directions.
		ExpandBounds = Vector3d.Zero,
		-- Most of this parallelizes very well (selects the C# parallel code paths; sequential in Luau)
		UseParallel = true,
		ComputeMode = ComputeModes.NarrowBandOnly,
		-- how wide of narrow band should we compute. This value is currently only used if there is a
		-- spatial data structure, as we can efficiently explore the space (in that case ExactBandWidth is not used)
		NarrowBandMaxDistance = 0,
		-- should we try to compute signs? if not, grid remains unsigned
		ComputeSigns = true,
		-- What counts as "inside" the mesh. Crossing count does not use triangle orientation, so inverted faces
		-- are fine, but overlapping shells or self intersections will be filled using even/odd rules (as seen
		-- along X axis...). Parity count is basically mesh winding number, handles overlap shells and
		-- self-intersections, but inverted shells are 'subtracted', and inverted faces are a disaster.
		-- Both modes handle internal cavities, neither handles open sheets.
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
	}, MeshSignedDistanceGrid)
end

function MeshSignedDistanceGrid:debug(msg: string)
	if self.DebugPrint then
		print(msg)
	end
end

function MeshSignedDistanceGrid:Compute()
	-- figure out origin & dimensions
	local bounds = self.Mesh:CachedBounds()
	local CellSize = self.CellSize

	local fBufferWidth = 2 * self.ExactBandWidth * CellSize
	if self.ComputeMode == ComputeModes.NarrowBand_SpatialFloodFill then
		fBufferWidth = math.max(fBufferWidth, 2 * self.NarrowBandMaxDistance)
	end
	local e = self.ExpandBounds
	local origin = V3(bounds.Min.x - fBufferWidth - e.x, bounds.Min.y - fBufferWidth - e.y, bounds.Min.z - fBufferWidth - e.z)
	local max = V3(bounds.Max.x + fBufferWidth + e.x, bounds.Max.y + fBufferWidth + e.y, bounds.Max.z + fBufferWidth + e.z)
	self.grid_origin = origin
	local ni = TruncInt((max.x - origin.x) / CellSize) + 1
	local nj = TruncInt((max.y - origin.y) / CellSize) + 1
	local nk = TruncInt((max.z - origin.z) / CellSize) + 1

	self.grid = DenseGrid3f.new()
	if self.ComputeMode == ComputeModes.NarrowBand_SpatialFloodFill then
		if self.Spatial == nil or self.NarrowBandMaxDistance == 0 or self.UseParallel == false then
			error("MeshSignedDistanceGrid.Compute: must set Spatial data structure and band max distance, and UseParallel=true")
		end
		self:make_level_set3_parallel_floodfill(origin, CellSize, ni, nj, nk, self.grid, self.ExactBandWidth)
	else
		if self.UseParallel then
			if self.Spatial ~= nil then
				self:make_level_set3_parallel_spatial(origin, CellSize, ni, nj, nk, self.grid, self.ExactBandWidth)
			else
				self:make_level_set3_parallel(origin, CellSize, ni, nj, nk, self.grid, self.ExactBandWidth)
			end
		else
			self:make_level_set3(origin, CellSize, ni, nj, nk, self.grid, self.ExactBandWidth)
		end
	end
end

function MeshSignedDistanceGrid:Dimensions()
	return Vector3i.new(self.grid.ni, self.grid.nj, self.grid.nk)
end

-- SDF grid available after calling Compute()
function MeshSignedDistanceGrid:Grid()
	return self.grid
end

-- Origin of the SDF grid, in same coordinates as mesh
function MeshSignedDistanceGrid:GridOrigin()
	return self.grid_origin
end

function MeshSignedDistanceGrid:ClosestTriGrid()
	if self.WantClosestTriGrid == false then
		error("Set WantClosestTriGrid=true to return this value")
	end
	return self.closest_tri_grid
end

function MeshSignedDistanceGrid:IntersectionsGrid()
	if self.WantIntersectionsGrid == false then
		error("Set WantIntersectionsGrid=true to return this value")
	end
	return self.intersections_grid
end

-- this[int i, int j, int k]
function MeshSignedDistanceGrid:Get(i: number, j: number, k: number): number
	return self.grid:Get(i, j, k)
end
-- this[Vector3i idx]
function MeshSignedDistanceGrid:GetV(idx): number
	return self.grid:Get(idx.x, idx.y, idx.z)
end

function MeshSignedDistanceGrid:CellCenter(i: number, j: number, k: number)
	return self:cell_center(Vector3i.new(i, j, k))
end
function MeshSignedDistanceGrid:cell_center(ijk)
	local o, c = self.grid_origin, self.CellSize
	return V3(ijk.x * c + o.x, ijk.y * c + o.y, ijk.z * c + o.z)
end

function MeshSignedDistanceGrid:upper_bound(grid): number
	return (grid.ni + grid.nj + grid.nk) * self.CellSize
end

function MeshSignedDistanceGrid:cell_tri_dist(idx, tid: number): number
	local c = self:cell_center(idx)
	local ax, ay, az, bx, by, bz, cx, cy, cz = self.Mesh:GetTriVerticesXYZ(tid)
	return point_triangle_distance_xyz(c.x, c.y, c.z, ax, ay, az, bx, by, bz, cx, cy, cz)
end

-- Compute narrow-band distances. For each triangle, we find its grid-coord-bbox, and compute exact
-- distances within that box. (shared body of C# make_level_set3 and make_level_set3_parallel;
-- bUseInvDx selects the parallel version's multiply-by-inverse grid coordinates)
-- returns false if cancelled
function MeshSignedDistanceGrid:compute_exact_band(
	origin,
	dx: number,
	ni: number,
	nj: number,
	nk: number,
	distances,
	closest_tri,
	exact_band: number,
	bUseInvDx: boolean
): boolean
	local mesh = self.Mesh
	local CancelF = self.CancelF
	local ox, oy, oz = origin.x, origin.y, origin.z
	local invdx = 1.0 / dx
	local dbuf, cbuf = distances.Buffer, closest_tri.Buffer
	local abort = false
	for tid in mesh:TriangleIndices() do
		if tid % 100 == 0 then
			abort = CancelF()
		end
		if abort then
			break
		end
		local xpx, xpy, xpz, xqx, xqy, xqz, xrx, xry, xrz = mesh:GetTriVerticesXYZ(tid)

		-- real ijk coordinates of xp/xq/xr
		local fip, fjp, fkp, fiq, fjq, fkq, fir, fjr, fkr
		if bUseInvDx then
			fip, fjp, fkp = (xpx - ox) * invdx, (xpy - oy) * invdx, (xpz - oz) * invdx
			fiq, fjq, fkq = (xqx - ox) * invdx, (xqy - oy) * invdx, (xqz - oz) * invdx
			fir, fjr, fkr = (xrx - ox) * invdx, (xry - oy) * invdx, (xrz - oz) * invdx
		else
			fip, fjp, fkp = (xpx - ox) / dx, (xpy - oy) / dx, (xpz - oz) / dx
			fiq, fjq, fkq = (xqx - ox) / dx, (xqy - oy) / dx, (xqz - oz) / dx
			fir, fjr, fkr = (xrx - ox) / dx, (xry - oy) / dx, (xrz - oz) / dx
		end

		-- clamped integer bounding box of triangle plus exact-band
		local i0 = clampi(TruncInt(min3(fip, fiq, fir)) - exact_band, 0, ni - 1)
		local i1 = clampi(TruncInt(max3(fip, fiq, fir)) + exact_band + 1, 0, ni - 1)
		local j0 = clampi(TruncInt(min3(fjp, fjq, fjr)) - exact_band, 0, nj - 1)
		local j1 = clampi(TruncInt(max3(fjp, fjq, fjr)) + exact_band + 1, 0, nj - 1)
		local k0 = clampi(TruncInt(min3(fkp, fkq, fkr)) - exact_band, 0, nk - 1)
		local k1 = clampi(TruncInt(max3(fkp, fkq, fkr)) + exact_band + 1, 0, nk - 1)

		-- compute distance for each tri inside this bounding box
		-- note: this can be very conservative if the triangle is large and on diagonal to grid axes
		for k = k0, k1 do
			local gz = k * dx + oz
			for j = j0, j1 do
				local gy = j * dx + oy
				local rowbase = ni * (j + nj * k) + 1
				for i = i0, i1 do
					local d = point_triangle_distance_xyz(i * dx + ox, gy, gz, xpx, xpy, xpz, xqx, xqy, xqz, xrx, xry, xrz)
					local idx = rowbase + i
					if d < dbuf[idx] then
						dbuf[idx] = d
						cbuf[idx] = tid
					end
				end
			end
		end
	end
	return true
end

-- shared tail of all the make_level_set3 variants: intersections, optional sweeping, signs
function MeshSignedDistanceGrid:finish_level_set(
	origin,
	dx: number,
	ni: number,
	nj: number,
	nk: number,
	distances,
	closest_tri,
	intersection_count
)
	local CancelF = self.CancelF
	if self.ComputeSigns == true then
		self:debug("done narrow-band")

		self:compute_intersections(origin, dx, ni, nj, nk, intersection_count)
		if CancelF() then
			return
		end
		self:debug("done intersections")

		if self.ComputeMode == ComputeModes.FullGrid then
			-- and now we fill in the rest of the distances with fast sweeping
			for _ = 0, 1 do
				self:sweep_pass(origin, dx, distances, closest_tri)
				if CancelF() then
					return
				end
			end
			self:debug("done sweeping")
		else
			-- nothing!
			self:debug("skipped sweeping")
		end

		-- then figure out signs (inside/outside) from intersection counts
		self:compute_signs(ni, nj, nk, distances, intersection_count)
		if CancelF() then
			return
		end

		if self.WantIntersectionsGrid then
			self.intersections_grid = intersection_count
		end
		self:debug("done signs")
	end

	if self.WantClosestTriGrid then
		self.closest_tri_grid = closest_tri
	end
end

function MeshSignedDistanceGrid:make_level_set3(origin, dx: number, ni: number, nj: number, nk: number, distances, exact_band: number)
	distances:resize(ni, nj, nk)
	distances:assign(self:upper_bound(distances)) -- upper bound on distance

	-- closest triangle id for each grid cell
	local closest_tri = DenseGrid3i.new(ni, nj, nk, -1)

	-- intersection_count(i,j,k) is # of tri intersections in (i-1,i]x{j}x{k}
	local intersection_count = DenseGrid3i.new(ni, nj, nk, 0)

	self:debug("start")
	self:compute_exact_band(origin, dx, ni, nj, nk, distances, closest_tri, exact_band, false)
	if self.CancelF() then
		return
	end
	self:finish_level_set(origin, dx, ni, nj, nk, distances, closest_tri, intersection_count)
end

function MeshSignedDistanceGrid:make_level_set3_parallel(
	origin,
	dx: number,
	ni: number,
	nj: number,
	nk: number,
	distances,
	exact_band: number
)
	distances:resize(ni, nj, nk)
	distances:assign(self:upper_bound(self.grid)) -- upper bound on distance

	-- closest triangle id for each grid cell
	local closest_tri = DenseGrid3i.new(ni, nj, nk, -1)

	-- intersection_count(i,j,k) is # of tri intersections in (i-1,i]x{j}x{k}
	local intersection_count = DenseGrid3i.new(ni, nj, nk, 0)

	self:debug("start")
	-- (C# partitions the grid into 8 spinlock regions here; not needed sequentially)
	self:compute_exact_band(origin, dx, ni, nj, nk, distances, closest_tri, exact_band, true)
	self:debug("done narrow-band")
	if self.CancelF() then
		return
	end
	self:finish_level_set(origin, dx, ni, nj, nk, distances, closest_tri, intersection_count)
end

function MeshSignedDistanceGrid:make_level_set3_parallel_spatial(
	origin,
	dx: number,
	ni: number,
	nj: number,
	nk: number,
	distances,
	exact_band: number
)
	distances:resize(ni, nj, nk)
	local upper_bound = self:upper_bound(distances)
	distances:assign(upper_bound) -- upper bound on distance

	-- closest triangle id for each grid cell
	local closest_tri = DenseGrid3i.new(ni, nj, nk, -1)

	-- intersection_count(i,j,k) is # of tri intersections in (i-1,i]x{j}x{k}
	local intersection_count = DenseGrid3i.new(ni, nj, nk, 0)

	self:debug("start")

	local mesh, spatial, CancelF = self.Mesh, self.Spatial, self.CancelF
	local ox, oy, oz = origin.x, origin.y, origin.z
	local invdx = 1.0 / dx
	local dbuf, cbuf = distances.Buffer, closest_tri.Buffer

	-- tag the cells in the (conservative) narrow band
	local abort = false
	for tid in mesh:TriangleIndices() do
		if tid % 100 == 0 then
			abort = CancelF()
		end
		if abort then
			break
		end
		local xpx, xpy, xpz, xqx, xqy, xqz, xrx, xry, xrz = mesh:GetTriVerticesXYZ(tid)

		-- real ijk coordinates of xp/xq/xr
		local fip, fjp, fkp = (xpx - ox) * invdx, (xpy - oy) * invdx, (xpz - oz) * invdx
		local fiq, fjq, fkq = (xqx - ox) * invdx, (xqy - oy) * invdx, (xqz - oz) * invdx
		local fir, fjr, fkr = (xrx - ox) * invdx, (xry - oy) * invdx, (xrz - oz) * invdx

		-- clamped integer bounding box of triangle plus exact-band
		local i0 = clampi(TruncInt(min3(fip, fiq, fir)) - exact_band, 0, ni - 1)
		local i1 = clampi(TruncInt(max3(fip, fiq, fir)) + exact_band + 1, 0, ni - 1)
		local j0 = clampi(TruncInt(min3(fjp, fjq, fjr)) - exact_band, 0, nj - 1)
		local j1 = clampi(TruncInt(max3(fjp, fjq, fjr)) + exact_band + 1, 0, nj - 1)
		local k0 = clampi(TruncInt(min3(fkp, fkq, fkr)) - exact_band, 0, nk - 1)
		local k1 = clampi(TruncInt(max3(fkp, fkq, fkr)) + exact_band + 1, 0, nk - 1)

		for k = k0, k1 do
			for j = j0, j1 do
				local rowbase = ni * (j + nj * k) + 1
				for i = i0, i1 do
					dbuf[rowbase + i] = 1
				end
			end
		end
	end

	self:debug("done narrow-band tagging")

	local DistPoint3Triangle3 = require(script.Parent.DistPoint3Triangle3)
	local max_dist = exact_band * (dx * MathUtil.SqrtTwo)
	for k = 0, nk - 1 do
		for j = 0, nj - 1 do
			for i = 0, ni - 1 do
				local idx = i + ni * (j + nj * k) + 1
				if dbuf[idx] == 1 then
					local px, py, pz = i * dx + ox, j * dx + oy, k * dx + oz
					local near_tid = find_nearest_tri(spatial, px, py, pz, max_dist)
					if near_tid == -1 then
						dbuf[idx] = upper_bound
						continue
					end
					local ax, ay, az, bx, by, bz, cx, cy, cz = mesh:GetTriVerticesXYZ(near_tid)
					local dsqr = DistPoint3Triangle3.DistanceSqrXYZ(px, py, pz, ax, ay, az, bx, by, bz, cx, cy, cz)
					dbuf[idx] = sqrt(dsqr)
					cbuf[idx] = near_tid
				end
			end
		end
	end

	self:debug("done distances")

	if CancelF() then
		return
	end
	self:finish_level_set(origin, dx, ni, nj, nk, distances, closest_tri, intersection_count)
end

function MeshSignedDistanceGrid:make_level_set3_parallel_floodfill(
	origin,
	dx: number,
	ni: number,
	nj: number,
	nk: number,
	distances,
	exact_band: number
)
	distances:resize(ni, nj, nk)
	local upper_bound = self:upper_bound(distances)
	distances:assign(upper_bound) -- upper bound on distance

	-- closest triangle id for each grid cell
	local closest_tri = DenseGrid3i.new(ni, nj, nk, -1)

	-- intersection_count(i,j,k) is # of tri intersections in (i-1,i]x{j}x{k}
	local intersection_count = DenseGrid3i.new(ni, nj, nk, 0)

	self:debug("start")

	local DistPoint3Triangle3 = require(script.Parent.DistPoint3Triangle3)
	local mesh, spatial, CancelF = self.Mesh, self.Spatial, self.CancelF
	local ox, oy, oz = origin.x, origin.y, origin.z
	local invdx = 1.0 / dx
	local dbuf, cbuf = distances.Buffer, closest_tri.Buffer

	-- compute values at vertices
	local Q = {} -- linear (0-based) grid indices
	local done = table.create(distances:size(), false) -- done[linear + 1]

	local abort = false
	for vid in mesh:VertexIndices() do
		if vid % 100 == 0 then
			abort = CancelF()
		end
		if abort then
			break
		end
		local vx, vy, vz = mesh:GetVertexXYZ(vid)
		-- real ijk coordinates of v
		local fi, fj, fk = (vx - ox) * invdx, (vy - oy) * invdx, (vz - oz) * invdx
		local ii = clampi(TruncInt(fi), 0, ni - 1)
		local jj = clampi(TruncInt(fj), 0, nj - 1)
		local kk = clampi(TruncInt(fk), 0, nk - 1)
		local idx_linear = ii + ni * (jj + nj * kk)
		if dbuf[idx_linear + 1] < upper_bound then
			continue
		end

		local px, py, pz = ii * dx + ox, jj * dx + oy, kk * dx + oz -- cell_center(idx)
		local near_tid = find_nearest_tri(spatial, px, py, pz, nil)
		local ax, ay, az, bx, by, bz, cx, cy, cz = mesh:GetTriVerticesXYZ(near_tid)
		local dsqr = DistPoint3Triangle3.DistanceSqrXYZ(px, py, pz, ax, ay, az, bx, by, bz, cx, cy, cz)
		dbuf[idx_linear + 1] = sqrt(dsqr)
		cbuf[idx_linear + 1] = near_tid
		table.insert(Q, idx_linear)
		done[idx_linear + 1] = true
	end
	self:debug("done vertices")
	if CancelF() then
		return
	end

	-- we could do this parallel w/ some kind of producer-consumer...
	local next_Q = {}
	local offsets = IndexUtil.gIndices.GridOffsets26
	local max_dist = self.NarrowBandMaxDistance
	local max_query_dist = max_dist + (2 * dx * MathUtil.SqrtTwo)
	local slab = ni * nj
	local next_pass_count = #Q
	while next_pass_count > 0 do
		table.clear(next_Q)
		for _, cur_linear_index in Q do
			-- distances.to_index(cur_linear_index)
			local cx0 = cur_linear_index % ni
			local cy0 = (cur_linear_index // ni) % nj
			local cz0 = cur_linear_index // slab
			for oi = 1, 26 do
				local o = offsets[oi]
				local nx, ny, nz = cx0 + o.x, cy0 + o.y, cz0 + o.z
				-- bounds.Contains(nbr_idx), bounds = BoundsInclusive
				if nx < 0 or ny < 0 or nz < 0 or nx > ni - 1 or ny > nj - 1 or nz > nk - 1 then
					continue
				end
				local nbr_linear_idx = nx + ni * (ny + nj * nz)
				if done[nbr_linear_idx + 1] then
					continue
				end

				local px, py, pz = nx * dx + ox, ny * dx + oy, nz * dx + oz -- cell_center(nbr_idx)
				local near_tid = find_nearest_tri(spatial, px, py, pz, max_query_dist)
				if near_tid == -1 then
					done[nbr_linear_idx + 1] = true
					continue
				end

				local ax, ay, az, bx, by, bz, tcx, tcy, tcz = mesh:GetTriVerticesXYZ(near_tid)
				local dsqr = DistPoint3Triangle3.DistanceSqrXYZ(px, py, pz, ax, ay, az, bx, by, bz, tcx, tcy, tcz)
				local dist = sqrt(dsqr)

				if done[nbr_linear_idx + 1] == false then
					dbuf[nbr_linear_idx + 1] = dist
					cbuf[nbr_linear_idx + 1] = near_tid
					done[nbr_linear_idx + 1] = true
					if dist < max_dist then
						table.insert(next_Q, nbr_linear_idx)
					end
				end
			end
		end
		-- swap lists
		Q, next_Q = next_Q, Q
		next_pass_count = #Q
	end
	self:debug("done floodfill")
	if CancelF() then
		return
	end
	self:finish_level_set(origin, dx, ni, nj, nk, distances, closest_tri, intersection_count)
end

-- sweep through grid in different directions, distances and closest tris
function MeshSignedDistanceGrid:sweep_pass(origin, dx: number, distances, closest_tri)
	local CancelF = self.CancelF
	self:sweep(distances, closest_tri, origin, dx, 1, 1, 1)
	if CancelF() then
		return
	end
	self:sweep(distances, closest_tri, origin, dx, -1, -1, -1)
	if CancelF() then
		return
	end
	self:sweep(distances, closest_tri, origin, dx, 1, 1, -1)
	if CancelF() then
		return
	end
	self:sweep(distances, closest_tri, origin, dx, -1, -1, 1)
	if CancelF() then
		return
	end
	self:sweep(distances, closest_tri, origin, dx, 1, -1, 1)
	if CancelF() then
		return
	end
	self:sweep(distances, closest_tri, origin, dx, -1, 1, -1)
	if CancelF() then
		return
	end
	self:sweep(distances, closest_tri, origin, dx, 1, -1, -1)
	if CancelF() then
		return
	end
	self:sweep(distances, closest_tri, origin, dx, -1, 1, 1)
end

-- single sweep pass
function MeshSignedDistanceGrid:sweep(phi, closest_tri, origin, dx: number, di: number, dj: number, dk: number)
	local CancelF = self.CancelF
	local mesh = self.Mesh
	local tris, verts = mesh.triangles.data, mesh.vertices.data
	local ni, nj, nk = phi.ni, phi.nj, phi.nk
	local pbuf, cbuf = phi.Buffer, closest_tri.Buffer
	local ox, oy, oz = origin.x, origin.y, origin.z

	local i0, i1, j0, j1, k0, k1
	if di > 0 then
		i0, i1 = 1, ni
	else
		i0, i1 = ni - 2, -1
	end
	if dj > 0 then
		j0, j1 = 1, nj
	else
		j0, j1 = nj - 2, -1
	end
	if dk > 0 then
		k0, k1 = 1, nk
	else
		k0, k1 = nk - 2, -1
	end

	-- check_neighbour(phi, closest_tri, gx, i0, j0, k0, i1, j1, k1), on linear buffer indices
	local function check_neighbour(gx: number, gy: number, gz: number, idx0: number, idx1: number)
		local tid = cbuf[idx1]
		if tid >= 0 then
			local ti = 3 * tid
			local a, b, c = 3 * tris[ti + 1], 3 * tris[ti + 2], 3 * tris[ti + 3]
			local d = point_triangle_distance_xyz(
				gx,
				gy,
				gz,
				verts[a + 1],
				verts[a + 2],
				verts[a + 3],
				verts[b + 1],
				verts[b + 2],
				verts[b + 3],
				verts[c + 1],
				verts[c + 2],
				verts[c + 3]
			)
			if d < pbuf[idx0] then
				pbuf[idx0] = d
				cbuf[idx0] = tid
			end
		end
	end

	-- linear index offsets of the neighbours
	local oI, oJ, oK = di, ni * dj, ni * nj * dk
	local k = k0
	while k ~= k1 do
		if CancelF() then
			return
		end
		local gz = k * dx + oz
		local j = j0
		while j ~= j1 do
			local gy = j * dx + oy
			local rowbase = ni * (j + nj * k) + 1
			local i = i0
			while i ~= i1 do
				local gx = i * dx + ox
				local idx = rowbase + i
				check_neighbour(gx, gy, gz, idx, idx - oI)
				check_neighbour(gx, gy, gz, idx, idx - oJ)
				check_neighbour(gx, gy, gz, idx, idx - oI - oJ)
				check_neighbour(gx, gy, gz, idx, idx - oK)
				check_neighbour(gx, gy, gz, idx, idx - oI - oK)
				check_neighbour(gx, gy, gz, idx, idx - oJ - oK)
				check_neighbour(gx, gy, gz, idx, idx - oI - oJ - oK)
				i += di
			end
			j += dj
		end
		k += dk
	end
end

-- fill the intersection grid w/ number of intersections in each cell
function MeshSignedDistanceGrid:compute_intersections(origin, dx: number, ni: number, nj: number, nk: number, intersection_count)
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

-- shared with CachingMeshSDF
function MeshSignedDistanceGrid.compute_intersections_static(
	mesh,
	insideMode: number,
	CancelF: () -> boolean,
	origin,
	dx: number,
	ni: number,
	nj: number,
	nk: number,
	intersection_count
)
	local ox, oy, oz = origin.x, origin.y, origin.z
	local invdx = 1.0 / dx
	local ibuf = intersection_count.Buffer
	local bParity = (insideMode == InsideModes.ParityCount)

	local cancelled = false
	-- this is what we will do for each triangle. There are no grid-reads, only grid-writes
	for tid in mesh:TriangleIndices() do
		if tid % 100 == 0 and CancelF() == true then
			cancelled = true
		end
		if cancelled then
			return
		end

		local xpx, xpy, xpz, xqx, xqy, xqz, xrx, xry, xrz = mesh:GetTriVerticesXYZ(tid)

		local neg_x = false
		if bParity then
			-- n = MathUtil.FastNormalDirection(xp, xq, xr) = (xq - xp).Cross(xr - xp); neg_x = n.x > 0
			local e1y, e1z = xqy - xpy, xqz - xpz
			local e2y, e2z = xry - xpy, xrz - xpz
			neg_x = (e1y * e2z - e1z * e2y) > 0
		end
		local incr = neg_x and -1 or 1

		-- real ijk coordinates of xp/xq/xr
		local fip, fjp, fkp = (xpx - ox) * invdx, (xpy - oy) * invdx, (xpz - oz) * invdx
		local fiq, fjq, fkq = (xqx - ox) * invdx, (xqy - oy) * invdx, (xqz - oz) * invdx
		local fir, fjr, fkr = (xrx - ox) * invdx, (xry - oy) * invdx, (xrz - oz) * invdx

		-- recompute j/k integer bounds of triangle w/o exact band
		local j0 = clampi(ceil(min3(fjp, fjq, fjr)), 0, nj - 1)
		local j1 = clampi(floor(max3(fjp, fjq, fjr)), 0, nj - 1)
		local k0 = clampi(ceil(min3(fkp, fkq, fkr)), 0, nk - 1)
		local k1 = clampi(floor(max3(fkp, fkq, fkr)), 0, nk - 1)

		-- and do intersection counts
		for k = k0, k1 do
			for j = j0, j1 do
				local ok, a, b, c = point_in_triangle_2d(j, k, fjp, fkp, fjq, fkq, fjr, fkr)
				if ok then
					local fi = a * fip + b * fiq + c * fir -- intersection i coordinate
					local i_interval = ceil(fi) -- intersection is in (i_interval-1,i_interval]
					if i_interval < 0 then
						local idx = ni * (j + nj * k) + 1
						ibuf[idx] += incr
					elseif i_interval < ni then
						local idx = i_interval + ni * (j + nj * k) + 1
						ibuf[idx] += incr
					else
						-- we ignore intersections that are beyond the +x side of the grid
					end
				end
			end
		end
	end
end

-- iterate through each x-row of grid and set unsigned distances to be negative
-- inside the mesh, based on the intersection_counts
function MeshSignedDistanceGrid:compute_signs(ni: number, nj: number, nk: number, distances, intersection_counts)
	MeshSignedDistanceGrid.compute_signs_static(self.InsideMode, self.UseParallel, self.CancelF, ni, nj, nk, distances, intersection_counts)
end

-- shared with CachingMeshSDF
function MeshSignedDistanceGrid.compute_signs_static(
	insideMode: number,
	useParallel: boolean,
	CancelF: () -> boolean,
	ni: number,
	nj: number,
	nk: number,
	distances,
	intersection_counts
)
	local bParity = (insideMode == InsideModes.ParityCount)
	local dbuf, ibuf = distances.Buffer, intersection_counts.Buffer
	-- isInsideF: CrossingCount -> count % 2 == 1, ParityCount -> count > 0
	if useParallel then
		-- can process each x-row in parallel (C# iterates AxisAlignedBox2i(0,0,nj,nk).IndicesExclusive())
		for k = 0, nk - 1 do
			for j = 0, nj - 1 do
				if CancelF() then
					continue
				end
				local total_count = 0
				local rowbase = ni * (j + nj * k) + 1
				for i = 0, ni - 1 do
					local idx = rowbase + i
					total_count += ibuf[idx]
					local inside
					if bParity then
						inside = total_count > 0
					else
						inside = math.fmod(total_count, 2) == 1
					end
					if inside then -- if parity of intersections so far is odd,
						dbuf[idx] = -dbuf[idx] -- we are inside the mesh
					end
				end
			end
		end
	else
		for k = 0, nk - 1 do
			if CancelF() then
				return
			end
			for j = 0, nj - 1 do
				local total_count = 0
				local rowbase = ni * (j + nj * k) + 1
				for i = 0, ni - 1 do
					local idx = rowbase + i
					total_count += ibuf[idx]
					local inside
					if bParity then
						inside = total_count > 0
					else
						inside = math.fmod(total_count, 2) == 1
					end
					if inside then
						dbuf[idx] = -dbuf[idx]
					end
				end
			end
		end
	end
end

return MeshSignedDistanceGrid
