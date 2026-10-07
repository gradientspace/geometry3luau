-- "Make solid": convert an arbitrary (open, self-intersecting, multi-component, inconsistently oriented)
-- triangle mesh into a closed, manifold, consistently oriented solid, then optionally remesh and simplify.
-- (Not a geometry3Sharp class; this composes the ported classes the way geometry3Sharp-based tools do.)
--
-- Pipeline:
--   1. DMeshAABBTree3 on the input mesh
--   2. sample a volumetric field on a grid:
--        WindingNumber mode (default): MeshWindingNumberGrid (generalized/fast winding number, narrow band),
--          solid = { winding >= WindingIsoValue }. Robust to holes, overlaps and self-intersections.
--        SignedDistance mode: MeshSignedDistanceGrid, solid = { sdf <= SignedDistanceOffset }. Supports
--          offsetting (grow/shrink), but needs a reasonably closed input to sign correctly.
--   3. MarchingCubes on a trilinear interpolant of the grid (bisection root finding), orientation fixed
--      so the result has positive volume
--   4. optional isotropic remesh (Remesher, projected back onto the marching-cubes surface)
--   5. optional simplification (Reducer to a triangle count, projected onto the marching-cubes surface)
--
-- Usage:
--   local solid = MakeSolid.new(mesh)
--   solid.GridResolution = 96          -- cells along the longest bounding-box axis (or set CellSize)
--   solid.TargetTriangleCount = 5000   -- optional simplification
--   local result = solid:Compute()     -- DMesh3; also solid.Result, solid.SurfaceMesh (marching-cubes output)
-- or MakeSolid.Run(mesh, { GridResolution = 96, TargetTriangleCount = 5000 }) -> DMesh3
--
-- Fields (all optional):
--   SolidMode               MakeSolid.SolidModes.WindingNumber (default) | .SignedDistance
--   CellSize                grid cell size; if nil, computed as maxDim / GridResolution
--   GridResolution          default 64
--   WindingIsoValue         default 0.5
--   UseFastWindingNumber    default true (DMeshAABBTree3:FastWindingNumber; false = exact hierarchical WindingNumber)
--   SignedDistanceOffset    default 0 (SignedDistance mode: > 0 grows, < 0 shrinks the solid)
--   RootModeSteps           marching-cubes bisection steps, default 5
--   Remesh                  default true
--   TargetEdgeLength        remesh edge length, default = CellSize
--   RemeshPasses            default 10
--   SmoothSpeed             remesher SmoothSpeedT, default 0.5
--   TargetTriangleCount     if set, simplify to this many triangles after remeshing
--   MinComponentVolume      if set, discard output components whose |volume| is smaller (removes specks)
--   Progress                ProgressCancel (or a function returning true to cancel); polled throughout.
--                           In Roblox use ProgressCancel.Yielding() to avoid script timeouts on large inputs.
-- After Compute(): Result (DMesh3 or nil if cancelled), SurfaceMesh, CellSize (actual), Cancelled, Timings

local DMesh3 = require(script.Parent.DMesh3)
local DMeshAABBTree3 = require(script.Parent.DMeshAABBTree3)
local MeshWindingNumberGrid = require(script.Parent.MeshWindingNumberGrid)
local MeshSignedDistanceGrid = require(script.Parent.MeshSignedDistanceGrid)
local DenseGridTrilinearImplicit = require(script.Parent.DenseGridTrilinearImplicit)
local MarchingCubes = require(script.Parent.MarchingCubes)
local Remesher = require(script.Parent.Remesher)
local Reducer = require(script.Parent.Reducer)
local MeshProjectionTarget = require(script.Parent.MeshProjectionTarget)
local ProgressCancel = require(script.Parent.ProgressCancel)

local MakeSolid = {}
MakeSolid.__index = MakeSolid

MakeSolid.SolidModes = table.freeze({
	WindingNumber = 0,
	SignedDistance = 1,
})

function MakeSolid.new(mesh)
	return setmetatable({
		Mesh = mesh,
		SolidMode = MakeSolid.SolidModes.WindingNumber,
		CellSize = nil,
		GridResolution = 64,
		WindingIsoValue = 0.5,
		UseFastWindingNumber = true,
		SignedDistanceOffset = 0,
		RootModeSteps = 5,
		Remesh = true,
		TargetEdgeLength = nil,
		RemeshPasses = 10,
		SmoothSpeed = 0.5,
		TargetTriangleCount = nil,
		MinComponentVolume = nil,
		Progress = nil,
		Result = nil,
		SurfaceMesh = nil,
		Cancelled = false,
		Timings = {},
	}, MakeSolid)
end

function MakeSolid.Run(mesh, options: any?)
	local solid = MakeSolid.new(mesh)
	for k, v in options or {} do
		solid[k] = v
	end
	return solid:Compute()
end

local function signedVolume(mesh): number
	local vol = 0
	for tid in mesh:TriangleIndices() do
		local ax, ay, az, bx, by, bz, cx, cy, cz = mesh:GetTriVerticesXYZ(tid)
		vol += (ax * (by * cz - bz * cy) + ay * (bz * cx - bx * cz) + az * (bx * cy - by * cx)) / 6
	end
	return vol
end

-- remove connected components with small |volume|
local function removeSmallComponents(mesh, minVolume: number)
	local MeshConnectedComponents = require(script.Parent.MeshConnectedComponents)
	local MeshEditor = require(script.Parent.MeshEditor)
	local comps = MeshConnectedComponents.new(mesh)
	comps:FindConnectedT()
	for i = 0, comps:Count() - 1 do
		local c = comps:Get(i)
		local vol = 0
		for _, tid in c.Indices do
			local ax, ay, az, bx, by, bz, cx, cy, cz = mesh:GetTriVerticesXYZ(tid)
			vol += (ax * (by * cz - bz * cy) + ay * (bz * cx - bx * cz) + az * (bx * cy - by * cx)) / 6
		end
		if math.abs(vol) < minVolume then
			MeshEditor.RemoveTriangles(mesh, c.Indices, true)
		end
	end
end

function MakeSolid:Compute()
	local progress = self.Progress
	if type(progress) == "function" then
		progress = ProgressCancel.new(progress)
	end
	local function cancelled(): boolean
		if progress ~= nil and progress:Cancelled() then
			self.Cancelled = true
			return true
		end
		return false
	end
	local cancelF = function()
		return cancelled()
	end
	local timings = {}
	self.Timings = timings
	self.Cancelled = false
	self.Result = nil

	local mesh = self.Mesh
	assert(mesh ~= nil and mesh:TriangleCount() > 0, "MakeSolid: input mesh is empty")
	local bounds = mesh:GetBounds()
	local cellSize = self.CellSize or (bounds:MaxDim() / self.GridResolution)
	assert(cellSize > 0, "MakeSolid: invalid cell size")
	self.CellSize = cellSize

	-- 1. spatial data structure
	local t0 = os.clock()
	local spatial = DMeshAABBTree3.new(mesh, true)
	timings.tree = os.clock() - t0
	if cancelled() then
		return nil
	end

	-- 2. sample field on grid
	t0 = os.clock()
	local implicit, iso
	local fieldIsInsideNegative
	if self.SolidMode == MakeSolid.SolidModes.SignedDistance then
		local sdf = MeshSignedDistanceGrid.new(mesh, cellSize, spatial)
		local offset = self.SignedDistanceOffset or 0
		local band = math.max(2, math.ceil(math.abs(offset) / cellSize) + 2)
		sdf.ExactBandWidth = band
		sdf.CancelF = cancelF
		sdf:Compute()
		implicit = DenseGridTrilinearImplicit.FromSDFGrid(sdf)
		iso = offset
		fieldIsInsideNegative = true
	else
		local mwn = MeshWindingNumberGrid.new(mesh, spatial, cellSize)
		mwn.WindingIsoValue = self.WindingIsoValue
		mwn.UseFastWindingNumber = self.UseFastWindingNumber
		mwn.CancelF = cancelF
		mwn:Compute()
		implicit = DenseGridTrilinearImplicit.new(mwn:Grid(), mwn:GridOrigin(), mwn.CellSize)
		-- winding number outside the grid is 0 (default Outside is a huge positive value)
		implicit.Outside = 0
		iso = self.WindingIsoValue
		fieldIsInsideNegative = false
	end
	timings.grid = os.clock() - t0
	if cancelled() then
		return nil
	end

	-- 3. marching cubes
	t0 = os.clock()
	local mc = MarchingCubes.new()
	mc.Implicit = implicit
	mc.IsoValue = iso
	mc.Bounds = implicit:Bounds()
	mc.CubeSize = cellSize
	mc.RootMode = MarchingCubes.RootfindingModes.Bisection
	mc.RootModeSteps = self.RootModeSteps
	mc.CancelF = cancelF
	mc:Generate()
	local surface = mc.Mesh
	timings.marchingCubes = os.clock() - t0
	if cancelled() then
		return nil
	end
	-- MarchingCubes orients triangles for fields that are negative inside; a winding field is positive
	-- inside, so its output is inward-facing. Check the volume rather than relying on this.
	local _ = fieldIsInsideNegative
	if signedVolume(surface) < 0 then
		surface:ReverseOrientation()
	end
	if self.MinComponentVolume ~= nil then
		removeSmallComponents(surface, self.MinComponentVolume)
	end
	self.SurfaceMesh = surface

	if surface:TriangleCount() == 0 then
		self.Result = surface
		return surface
	end

	local result = DMesh3.FromCopy(surface, true, false, false, false)
	local target = nil
	local function projectionTarget()
		if target == nil then
			target = MeshProjectionTarget.new(DMesh3.FromCopy(surface, true, false, false, false))
		end
		return target
	end

	-- 4. remesh
	if self.Remesh and self.RemeshPasses > 0 then
		t0 = os.clock()
		local r = Remesher.new(result)
		r:SetTargetEdgeLength(self.TargetEdgeLength or cellSize)
		r.SmoothSpeedT = self.SmoothSpeed
		r:SetProjectionTarget(projectionTarget())
		r.Progress = progress
		for _ = 1, self.RemeshPasses do
			r:BasicRemeshPass()
			if cancelled() then
				return nil
			end
		end
		timings.remesh = os.clock() - t0
	end

	-- 5. simplify
	if self.TargetTriangleCount ~= nil and result:TriangleCount() > self.TargetTriangleCount then
		t0 = os.clock()
		local red = Reducer.new(result)
		red:SetProjectionTarget(projectionTarget())
		red.Progress = progress
		red:ReduceToTriangleCount(self.TargetTriangleCount)
		timings.reduce = os.clock() - t0
		if cancelled() then
			return nil
		end
	end

	if not result:IsCompact() then
		result = DMesh3.FromCopy(result, true)
	end
	self.Result = result
	return result
end

return MakeSolid
