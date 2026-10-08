--!native
-- Port of geometry3Sharp implicit/GridImplicits3d.cs (class DenseGridTrilinearImplicit)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Tri-linear interpolant for a 3D dense grid. Supports grid translation via GridOrigin, but does not
-- support scaling or rotation. If you need those, you can wrap this in something that does the xform.
-- Implements BoundedImplicitFunction3d (Value(pt), Bounds()) plus the ValueXYZ(x, y, z) fast path.
--
-- Luau port notes:
--   * Constructors: DenseGridTrilinearImplicit.new(grid: DenseGrid3d, gridOrigin: Vector3d, cellSize)
--                   DenseGridTrilinearImplicit.FromSDFGrid(sdf_grid)   (C# ctor taking a MeshSignedDistanceGrid;
--                   also works with any object exposing Grid(), GridOrigin() and CellSize, e.g. MeshWindingNumberGrid)
--   * Gradient(pt) -> Vector3d
--   * The grid Buffer is read directly (see DenseGrid3d for the layout). (int) casts truncate toward zero,
--     so points slightly below the grid origin (within one cell) are extrapolated exactly as in C#.

local Vector3d = require(script.Parent.Vector3d)
local AxisAlignedBox3d = require(script.Parent.AxisAlignedBox3d)
local MathUtil = require(script.Parent.MathUtil)

local floor, ceil = math.floor, math.ceil

local DenseGridTrilinearImplicit = {}
DenseGridTrilinearImplicit.__index = DenseGridTrilinearImplicit
DenseGridTrilinearImplicit.ClassName = "DenseGridTrilinearImplicit"

function DenseGridTrilinearImplicit.new(grid, gridOrigin, cellSize: number)
	return setmetatable({
		Grid = grid,
		GridOrigin = gridOrigin,
		CellSize = cellSize,
		-- value to return if query point is outside grid (in an SDF outside is usually positive).
		-- Need to do math with this value, so don't use double.MaxValue or square will overflow
		Outside = math.sqrt(math.sqrt(MathUtil.MaxDouble)),
	}, DenseGridTrilinearImplicit)
end

function DenseGridTrilinearImplicit.FromSDFGrid(sdf_grid)
	return DenseGridTrilinearImplicit.new(sdf_grid:Grid(), sdf_grid:GridOrigin(), sdf_grid.CellSize)
end

function DenseGridTrilinearImplicit:Bounds()
	local o, c, g = self.GridOrigin, self.CellSize, self.Grid
	return AxisAlignedBox3d.new(o.x, o.y, o.z, o.x + c * g.ni, o.y + c * g.nj, o.z + c * g.nk)
end

function DenseGridTrilinearImplicit:ValueXYZ(px: number, py: number, pz: number): number
	local o, cs, grid = self.GridOrigin, self.CellSize, self.Grid
	local gx = (px - o.x) / cs
	local gy = (py - o.y) / cs
	local gz = (pz - o.z) / cs

	-- compute integer coordinates  (C# (int) cast truncates toward zero)
	local x0 = gx >= 0 and floor(gx) or ceil(gx)
	local y0 = gy >= 0 and floor(gy) or ceil(gy)
	local z0 = gz >= 0 and floor(gz) or ceil(gz)
	local y1, z1 = y0 + 1, z0 + 1

	-- clamp to grid
	local ni, nj, nk = grid.ni, grid.nj, grid.nk
	if x0 < 0 or (x0 + 1) >= ni or y0 < 0 or y1 >= nj or z0 < 0 or z1 >= nk then
		return self.Outside
	end

	-- convert double coords to [0,1] range
	local fAx = gx - x0
	local fAy = gy - y0
	local fAz = gz - z0
	local OneMinusfAx = 1.0 - fAx

	-- compute trilinear interpolant (Grid.get_x_pair inlined; Buffer index = linear + 1)
	local buf = grid.Buffer
	local i000 = x0 + ni * (y0 + nj * z0) + 1
	local i001 = x0 + ni * (y0 + nj * z1) + 1
	local i010 = x0 + ni * (y1 + nj * z0) + 1
	local i011 = x0 + ni * (y1 + nj * z1) + 1

	local yz = (1 - fAy) * (1 - fAz)
	local sum = (OneMinusfAx * buf[i000] + fAx * buf[i000 + 1]) * yz

	yz = (1 - fAy) * fAz
	sum += (OneMinusfAx * buf[i001] + fAx * buf[i001 + 1]) * yz

	yz = fAy * (1 - fAz)
	sum += (OneMinusfAx * buf[i010] + fAx * buf[i010 + 1]) * yz

	yz = fAy * fAz
	sum += (OneMinusfAx * buf[i011] + fAx * buf[i011 + 1]) * yz

	return sum
end

function DenseGridTrilinearImplicit:Value(pt): number
	return self:ValueXYZ(pt.x, pt.y, pt.z)
end

function DenseGridTrilinearImplicit:Gradient(pt)
	local o, cs, grid = self.GridOrigin, self.CellSize, self.Grid
	local gx = (pt.x - o.x) / cs
	local gy = (pt.y - o.y) / cs
	local gz = (pt.z - o.z) / cs

	-- clamp to grid
	if gx < 0 or gx >= grid.ni - 1 or gy < 0 or gy >= grid.nj - 1 or gz < 0 or gz >= grid.nk - 1 then
		return Vector3d.Zero
	end

	-- compute integer coordinates
	local x0 = floor(gx)
	local y0 = floor(gy)
	local y1 = y0 + 1
	local z0 = floor(gz)
	local z1 = z0 + 1

	-- convert double coords to [0,1] range
	local fAx = gx - x0
	local fAy = gy - y0
	local fAz = gz - z0

	local fV000, fV100 = grid:get_x_pair(x0, y0, z0)
	local fV010, fV110 = grid:get_x_pair(x0, y1, z0)
	local fV001, fV101 = grid:get_x_pair(x0, y0, z1)
	local fV011, fV111 = grid:get_x_pair(x0, y1, z1)

	-- [TODO] can re-order this to vastly reduce number of ops!
	local gradX = -fV000 * (1 - fAy) * (1 - fAz)
		+ -fV001 * (1 - fAy) * fAz
		+ -fV010 * fAy * (1 - fAz)
		+ -fV011 * fAy * fAz
		+ fV100 * (1 - fAy) * (1 - fAz)
		+ fV101 * (1 - fAy) * fAz
		+ fV110 * fAy * (1 - fAz)
		+ fV111 * fAy * fAz

	local gradY = -fV000 * (1 - fAx) * (1 - fAz)
		+ -fV001 * (1 - fAx) * fAz
		+ fV010 * (1 - fAx) * (1 - fAz)
		+ fV011 * (1 - fAx) * fAz
		+ -fV100 * fAx * (1 - fAz)
		+ -fV101 * fAx * fAz
		+ fV110 * fAx * (1 - fAz)
		+ fV111 * fAx * fAz

	local gradZ = -fV000 * (1 - fAx) * (1 - fAy)
		+ fV001 * (1 - fAx) * (1 - fAy)
		+ -fV010 * (1 - fAx) * fAy
		+ fV011 * (1 - fAx) * fAy
		+ -fV100 * fAx * (1 - fAy)
		+ fV101 * fAx * (1 - fAy)
		+ -fV110 * fAx * fAy
		+ fV111 * fAx * fAy

	return Vector3d.new(gradX, gradY, gradZ)
end

return DenseGridTrilinearImplicit
