--!native
-- Port of geometry3Sharp implicit/CachingMeshSDF.cs (class CachingMeshSDFImplicit)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Tri-linear interpolant for a CachingMeshSDF (grid values are computed lazily on access).
-- Supports grid translation via GridOrigin, but does not support scaling or rotation.
-- Implements BoundedImplicitFunction3d (Value(pt), Bounds()) plus the ValueXYZ(x, y, z) fast path.
--
-- Luau port notes:
--   * Constructor: CachingMeshSDFImplicit.new(sdf: CachingMeshSDF)  (call sdf:Initialize() first)
--   * get_value_pair(i, j, k) -> a, b;  Gradient(pt) -> Vector3d
--   * (int) casts truncate toward zero, as in C#.

local Vector3d = require(script.Parent.Vector3d)
local AxisAlignedBox3d = require(script.Parent.AxisAlignedBox3d)
local MathUtil = require(script.Parent.MathUtil)

local floor, ceil = math.floor, math.ceil

local CachingMeshSDFImplicit = {}
CachingMeshSDFImplicit.__index = CachingMeshSDFImplicit
CachingMeshSDFImplicit.ClassName = "CachingMeshSDFImplicit"

function CachingMeshSDFImplicit.new(sdf)
	return setmetatable({
		SDF = sdf,
		GridOrigin = sdf:GridOrigin(),
		CellSize = sdf.CellSize,
		-- value to return if query point is outside grid (in an SDF outside is usually positive).
		-- Need to do math with this value, so don't use double.MaxValue or square will overflow
		Outside = math.sqrt(math.sqrt(MathUtil.MaxDouble)),
	}, CachingMeshSDFImplicit)
end

function CachingMeshSDFImplicit:Bounds()
	local o, c, g = self.GridOrigin, self.CellSize, self.SDF:Grid()
	return AxisAlignedBox3d.new(o.x, o.y, o.z, o.x + c * g.ni, o.y + c * g.nj, o.z + c * g.nk)
end

function CachingMeshSDFImplicit:get_value_pair(i: number, j: number, k: number): (number, number)
	local sdf = self.SDF
	return sdf:GetValueIJK(i, j, k), sdf:GetValueIJK(i + 1, j, k)
end

function CachingMeshSDFImplicit:ValueXYZ(px: number, py: number, pz: number): number
	local o, cs = self.GridOrigin, self.CellSize
	local grid = self.SDF:Grid()
	local gx = (px - o.x) / cs
	local gy = (py - o.y) / cs
	local gz = (pz - o.z) / cs

	-- compute integer coordinates
	local x0 = gx >= 0 and floor(gx) or ceil(gx)
	local y0 = gy >= 0 and floor(gy) or ceil(gy)
	local z0 = gz >= 0 and floor(gz) or ceil(gz)
	local y1, z1 = y0 + 1, z0 + 1

	-- clamp to grid
	if x0 < 0 or (x0 + 1) >= grid.ni or y0 < 0 or y1 >= grid.nj or z0 < 0 or z1 >= grid.nk then
		return self.Outside
	end

	-- convert double coords to [0,1] range
	local fAx = gx - x0
	local fAy = gy - y0
	local fAz = gz - z0
	local OneMinusfAx = 1.0 - fAx

	local xa, xb = self:get_value_pair(x0, y0, z0)
	local yz = (1 - fAy) * (1 - fAz)
	local sum = (OneMinusfAx * xa + fAx * xb) * yz

	xa, xb = self:get_value_pair(x0, y0, z1)
	yz = (1 - fAy) * fAz
	sum += (OneMinusfAx * xa + fAx * xb) * yz

	xa, xb = self:get_value_pair(x0, y1, z0)
	yz = fAy * (1 - fAz)
	sum += (OneMinusfAx * xa + fAx * xb) * yz

	xa, xb = self:get_value_pair(x0, y1, z1)
	yz = fAy * fAz
	sum += (OneMinusfAx * xa + fAx * xb) * yz

	return sum
end

function CachingMeshSDFImplicit:Value(pt): number
	return self:ValueXYZ(pt.x, pt.y, pt.z)
end

function CachingMeshSDFImplicit:Gradient(pt)
	local o, cs = self.GridOrigin, self.CellSize
	local grid = self.SDF:Grid()
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

	local fV000, fV100 = self:get_value_pair(x0, y0, z0)
	local fV010, fV110 = self:get_value_pair(x0, y1, z0)
	local fV001, fV101 = self:get_value_pair(x0, y0, z1)
	local fV011, fV111 = self:get_value_pair(x0, y1, z1)

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

return CachingMeshSDFImplicit
