--!native
-- Port of geometry3Sharp spatial/MeshWindingNumberGrid.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Sample mesh winding number (MWN) on a discrete grid. Can sample full grid, or compute MWN values along
-- a specific iso-contour and then fill in rest of grid with correctly-signed values via fast sweeping
-- (this is the default).
--
-- TODO:
--   - I think we are over-exploring the grid most of the time. eg along an x-ray that intersects the
--     surface, we only need at most 2 cells, but we are computing at least 3, and possibly 5.
--   - it may be better to use something like bloomenthal polygonizer continuation? where we are keeping
--     track of active edges instead of active cells?
--
-- Typical "make solid" use:
--   local spatial = DMeshAABBTree3.new(mesh, true)
--   local mwn = MeshWindingNumberGrid.new(mesh, spatial, cellSize)
--   mwn:Compute()
--   local iso = DenseGridTrilinearImplicit.new(mwn:Grid(), mwn:GridOrigin(), mwn.CellSize)
--   iso.Outside = 0      -- winding number outside the grid is 0 (the default Outside is a huge positive value,
--                        -- which would be "inside" for a winding field and create a box-shaped shell)
--   local mc = MarchingCubes.new(); mc.Implicit = iso; mc.IsoValue = mwn.WindingIsoValue
--   mc.Bounds = iso:Bounds(); mc.CubeSize = cellSize; mc:Generate()
--   mc.Mesh:ReverseOrientation()
-- Note: grid values are winding numbers (~1 inside, ~0 outside), but MarchingCubes orients triangles for
-- fields that are negative inside (SDFs), so its output is inward-facing: reverse the orientation.
--
-- Luau port notes:
--   * Constructor: MeshWindingNumberGrid.new(mesh, spatial, cellSize). `spatial` must provide
--     WindingNumber(p) (and FastWindingNumber(p) if UseFastWindingNumber is set), e.g. DMeshAABBTree3.
--   * Enum: MeshWindingNumberGrid.ComputeModes.{FullGrid, NarrowBand}
--   * Properties as methods: Dimensions(), Grid(), GridOrigin(), SDFGrid(); indexer this[i,j,k] -> Get(i, j, k)
--   * Luau extension: UseFastWindingNumber (default false). When true, spatial:FastWindingNumber(p) is
--     used instead of spatial:WindingNumber(p) (much faster on large meshes, approximate far from the surface).
--   * The "unknown" sentinel is float.MaxValue (MathUtil.MaxFloat), as in C#. Values are doubles.
--   * The front / queue sets use linear grid indices (scalar keys) instead of Vector3i.
--   * The narrow-band seed SDF is computed with ExactBandWidth = 1, so (like the C#) BufferCells must be 1
--     in NarrowBand mode for the SDF and winding grids to have the same dimensions.

local Vector3d = require(script.Parent.Vector3d)
local Vector3i = require(script.Parent.Vector3i)
local MathUtil = require(script.Parent.MathUtil)
local DenseGrid3d = require(script.Parent.DenseGrid3d)
local Bitmap3 = require(script.Parent.Bitmap3)
local IndexUtil = require(script.Parent.IndexUtil)

local V3 = Vector3d.new
local TruncInt = MathUtil.TruncInt
local FLOAT_MAX = MathUtil.MaxFloat

local MeshWindingNumberGrid = {}
MeshWindingNumberGrid.__index = MeshWindingNumberGrid
MeshWindingNumberGrid.ClassName = "MeshWindingNumberGrid"

local ComputeModes = table.freeze({ FullGrid = 0, NarrowBand = 1 })
MeshWindingNumberGrid.ComputeModes = ComputeModes

function MeshWindingNumberGrid.new(mesh, spatial, cellSize: number)
	return setmetatable({
		Mesh = mesh,
		MeshSpatial = spatial,
		-- size of cubes in the grid
		CellSize = cellSize,
		-- how many cells around border should we keep
		BufferCells = 1,
		-- Should we compute MWN at all grid cells (expensive!!) or only in narrow band.
		-- In narrow-band mode, we guess rest of MWN values by propagating along x-rows
		ComputeMode = ComputeModes.NarrowBand,
		-- in narrow-band mode, if mesh is not closed, we will explore space around this MWN iso-value
		WindingIsoValue = 0.5,
		-- in NarrowBand mode, we compute mesh SDF grid, if true then it can be accessed via SDFGrid() after Compute()
		WantMeshSDFGrid = true,
		-- Luau extension: use spatial:FastWindingNumber instead of spatial:WindingNumber
		UseFastWindingNumber = false,
		-- if this function returns true, we should abort calculation
		CancelF = function()
			return false
		end,
		DebugPrint = false,

		-- computed results
		grid_origin = Vector3d.Zero,
		winding_grid = nil :: any,
		mesh_sdf = nil :: any,
	}, MeshWindingNumberGrid)
end

function MeshWindingNumberGrid:debug(msg: string)
	if self.DebugPrint then
		print(msg)
	end
end

function MeshWindingNumberGrid:Compute()
	-- figure out origin & dimensions
	local bounds = self.Mesh:CachedBounds()
	local CellSize = self.CellSize

	local fBufferWidth = 2 * self.BufferCells * CellSize
	local origin = V3(bounds.Min.x - fBufferWidth, bounds.Min.y - fBufferWidth, bounds.Min.z - fBufferWidth)
	local max = V3(bounds.Max.x + fBufferWidth, bounds.Max.y + fBufferWidth, bounds.Max.z + fBufferWidth)
	self.grid_origin = origin
	local ni = TruncInt((max.x - origin.x) / CellSize) + 1
	local nj = TruncInt((max.y - origin.y) / CellSize) + 1
	local nk = TruncInt((max.z - origin.z) / CellSize) + 1

	self.winding_grid = DenseGrid3d.new()
	if self.ComputeMode == ComputeModes.FullGrid then
		self:make_grid_dense(origin, CellSize, ni, nj, nk, self.winding_grid)
	else
		self:make_grid(origin, CellSize, ni, nj, nk, self.winding_grid)
	end
end

function MeshWindingNumberGrid:Dimensions()
	return Vector3i.new(self.winding_grid.ni, self.winding_grid.nj, self.winding_grid.nk)
end

-- winding-number grid available after calling Compute()
function MeshWindingNumberGrid:Grid()
	return self.winding_grid
end

-- Origin of the winding-number grid, in same coordinates as mesh
function MeshWindingNumberGrid:GridOrigin()
	return self.grid_origin
end

-- If ComputeMode==NarrowBand, then we internally compute a signed-distance grid, which will hang onto
function MeshWindingNumberGrid:SDFGrid()
	return self.mesh_sdf
end

-- this[int i, int j, int k]
function MeshWindingNumberGrid:Get(i: number, j: number, k: number): number
	return self.winding_grid:Get(i, j, k)
end

function MeshWindingNumberGrid:CellCenter(i: number, j: number, k: number)
	local o, c = self.grid_origin, self.CellSize
	return V3(i * c + o.x, j * c + o.y, k * c + o.z)
end

-- returns the winding-number query function (x, y, z) -> number for the spatial structure
function MeshWindingNumberGrid:winding_function(): (number, number, number) -> number
	local spatial = self.MeshSpatial
	if self.UseFastWindingNumber then
		if spatial.FastWindingNumberXYZ ~= nil then
			return function(x: number, y: number, z: number): number
				return spatial:FastWindingNumberXYZ(x, y, z)
			end
		end
		return function(x: number, y: number, z: number): number
			return spatial:FastWindingNumber(V3(x, y, z))
		end
	end
	return function(x: number, y: number, z: number): number
		return spatial:WindingNumber(V3(x, y, z))
	end
end

function MeshWindingNumberGrid:make_grid(origin, dx: number, ni: number, nj: number, nk: number, winding)
	winding:resize(ni, nj, nk)
	winding:assign(FLOAT_MAX) -- sentinel
	local CancelF = self.CancelF
	local WindingNumber = self:winding_function()

	-- seed MWN cache
	WindingNumber(0, 0, 0)

	self:debug("start")

	-- Ok, because the whole idea is that the surface might have holes, we are going to compute MWN along
	-- known triangles and then propagate the computed region outwards until any MWN iso-sign-change is
	-- surrounded. To seed propagation, we compute unsigned SDF and then compute MWN for any voxels
	-- containing surface (ie w/ distance smaller than cellsize)

	-- compute unsigned SDF
	local MeshSignedDistanceGrid = require(script.Parent.MeshSignedDistanceGrid)
	local sdf = MeshSignedDistanceGrid.new(self.Mesh, self.CellSize)
	sdf.ComputeSigns = false
	sdf.CancelF = CancelF
	sdf:Compute()
	if CancelF() then
		return
	end

	local distances = sdf:Grid()
	if self.WantMeshSDFGrid then
		self.mesh_sdf = sdf
	end
	self:debug("done initial sdf")

	local ox, oy, oz = origin.x, origin.y, origin.z
	local wbuf, dbuf = winding.Buffer, distances.Buffer
	local CellSize = self.CellSize

	-- compute MWN at surface voxels
	for k = 0, nk - 1 do
		for j = 0, nj - 1 do
			if CancelF() then
				continue
			end
			local rowbase = ni * (j + nj * k) + 1
			for i = 0, ni - 1 do
				local dist = dbuf[rowbase + i]
				-- this could be tighter? but I don't think it matters...
				if dist < CellSize then
					wbuf[rowbase + i] = WindingNumber(i * dx + ox, j * dx + oy, k * dx + oz)
				end
			end
		end
	end
	if CancelF() then
		return
	end

	self:debug("done narrow-band")

	-- Now propagate outwards from computed voxels.
	-- Current procedure is to check 26-neighbours around each 'front' voxel, and if there are any MWN
	-- sign changes, that neighbour is added to front. Front is initialized w/ all voxels we computed above

	-- bounds = winding.Bounds; bounds.Max -= Vector3i.One  (inclusive [0, n-1])
	local maxi, maxj, maxk = ni - 1, nj - 1, nk - 1

	-- since we will be computing new MWN values as necessary, we cannot use winding grid to track whether
	-- a voxel is 'new' or not. So, using 3D bitmap intead - is updated at end of each pass.
	local bits = Bitmap3.new(Vector3i.new(ni, nj, nk))
	local bbuf = bits.Bits.bits -- bbuf[linear + 1]
	local cur_front = {} -- linear (0-based) indices
	for idx = 0, ni * nj * nk - 1 do
		if wbuf[idx + 1] ~= FLOAT_MAX then
			table.insert(cur_front, idx)
			bbuf[idx + 1] = true
		end
	end
	if CancelF() then
		return
	end

	-- Unique set of 'new' voxels to compute in next iteration (insertion-ordered list + membership set)
	local queue, queue_set = {}, {}
	local offsets = IndexUtil.gIndices.GridOffsets26
	local ox26, oy26, oz26 = {}, {}, {}
	for n = 1, 26 do
		ox26[n], oy26[n], oz26[n] = offsets[n].x, offsets[n].y, offsets[n].z
	end
	local iso = self.WindingIsoValue
	local slab = ni * nj

	while true do
		if CancelF() then
			return
		end

		-- can process front voxels in parallel
		local abort, iter_count = false, 0
		for _, cur in cur_front do
			iter_count += 1
			if iter_count % 100 == 0 then
				abort = CancelF()
			end
			if abort then
				break
			end

			local val = wbuf[cur + 1]
			local ci = cur % ni
			local cj = (cur // ni) % nj
			local ck = cur // slab

			-- check 26-neighbours to see if we have a crossing in any direction
			for n = 1, 26 do
				local ni_, nj_, nk_ = ci + ox26[n], cj + oy26[n], ck + oz26[n]
				if ni_ < 0 or nj_ < 0 or nk_ < 0 or ni_ > maxi or nj_ > maxj or nk_ > maxk then
					continue
				end
				local nidx = ni_ + ni * (nj_ + nj * nk_)
				local val2 = wbuf[nidx + 1]
				if val2 == FLOAT_MAX then
					val2 = WindingNumber(ni_ * dx + ox, nj_ * dx + oy, nk_ * dx + oz)
					wbuf[nidx + 1] = val2
				end
				if bbuf[nidx + 1] == false then
					-- this is a 'new' voxel this round.
					-- If we have a MWN-iso-crossing, add it to the front next round
					local crossing = (val < iso and val2 > iso) or (val > iso and val2 < iso)
					if crossing and not queue_set[nidx] then
						queue_set[nidx] = true
						table.insert(queue, nidx)
					end
				end
			end
		end
		self:debug(string.format("front has %d voxels", #queue))
		if #queue == 0 then
			break
		end

		-- update known-voxels list and create front for next iteration
		for _, idx in queue do
			bbuf[idx + 1] = true
		end
		cur_front, queue = queue, cur_front
		table.clear(queue)
		table.clear(queue_set)
	end
	self:debug("done front-prop")

	if self.DebugPrint then
		local filled = 0
		for idx = 1, ni * nj * nk do
			if wbuf[idx] ~= FLOAT_MAX then
				filled += 1
			end
		end
		print(string.format("filled: %d / %d  -  %f%%", filled, ni * nj * nk, filled / (ni * nj * nk) * 100.0))
	end

	if CancelF() then
		return
	end

	-- fill in the rest of the grid by propagating know MWN values
	self:fill_spans(ni, nj, nk, winding)

	self:debug("done sweep")
end

function MeshWindingNumberGrid:make_grid_dense(origin, dx: number, ni: number, nj: number, nk: number, winding)
	winding:resize(ni, nj, nk)
	local CancelF = self.CancelF
	local WindingNumber = self:winding_function()
	WindingNumber(0, 0, 0)
	local wbuf = winding.Buffer
	local ox, oy, oz = origin.x, origin.y, origin.z
	local abort, count = false, 0
	for k = 0, nk - 1 do
		for j = 0, nj - 1 do
			for i = 0, ni - 1 do
				count += 1
				if count % 100 == 0 then
					abort = CancelF()
				end
				if abort then
					return
				end
				wbuf[i + ni * (j + nj * k) + 1] = WindingNumber(i * dx + ox, j * dx + oy, k * dx + oz)
			end
		end
	end
end

function MeshWindingNumberGrid:fill_spans(ni: number, nj: number, nk: number, winding)
	local wbuf = winding.Buffer
	local iso = self.WindingIsoValue
	for k = 0, nk - 1 do
		for j = 0, nj - 1 do
			local rowbase = ni * (j + nj * k) + 1
			local last = wbuf[rowbase]
			if last == FLOAT_MAX then
				last = 0
			end
			for i = 0, ni - 1 do
				local idx = rowbase + i
				if wbuf[idx] == FLOAT_MAX then
					wbuf[idx] = last
				else
					last = wbuf[idx]
					if last < iso then -- propagate zeros on outside
						last = 0
					end
				end
			end
		end
	end
end

return MeshWindingNumberGrid
