--!native
-- Port of geometry3Sharp spatial/PointSetHashtable.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Hash table of points (a DSparseGrid3 of point-index lists) supporting fixed-radius ball queries.
--
-- Luau port notes:
--   * IPointSet is duck-typed: points:VertexIndices() (iterator), points:GetVertex(i) -> Vector3d.
--     (DMesh3, MeshEdgeMidpoints, MeshBoundaryEdgeMidpoints satisfy this.)
--   * Build overloads: Build(maxAxisSubdivs = 64) and BuildCell(cellSize, origin)
--     (C# Build(double cellSize, Vector3d origin)).
--   * FindInBall(pt, r, buffer, bufferCapacity?) -> (bool, buffer_count)
--       buffer is a Luau array that receives the found ids at buffer[1..count]; bufferCapacity
--       plays the role of the C# int[] length (default 1024). Returns false if the buffer was too small.
--   * indexF is a GridIndexing.ShiftGridIndexer3 (ToGrid truncates toward zero like the C# (int) cast).
--   * BoundsUtil.Bounds(IPointSet) is inlined (bounding box of points:GetVertex over VertexIndices).

local Vector3d = require(script.Parent.Vector3d)
local AxisAlignedBox3d = require(script.Parent.AxisAlignedBox3d)
local DSparseGrid3 = require(script.Parent.DSparseGrid3)
local IndexUtil = require(script.Parent.IndexUtil)
local GridIndexing = require(script.Parent.GridIndexing)

local abs = math.abs
local GridOffsets26 = IndexUtil.gIndices.GridOffsets26

-- PointList : List<int>, IGridElement3
local PointList = {}
PointList.__index = PointList
function PointList.new()
	return setmetatable({}, PointList)
end
function PointList:CreateNewGridElement(bCopy: boolean)
	return PointList.new()
end

local PointSetHashtable = {}
PointSetHashtable.__index = PointSetHashtable
PointSetHashtable.ClassName = "PointSetHashtable"
PointSetHashtable.PointList = PointList

function PointSetHashtable.new(points)
	local self = setmetatable({}, PointSetHashtable)
	self.Points = points
	self.Grid = nil
	self.indexF = nil
	self.Origin = Vector3d.Zero
	self.CellSize = 1
	return self
end

function PointSetHashtable:Build(maxAxisSubdivs: number?)
	local subdivs = maxAxisSubdivs or 64
	-- BoundsUtil.Bounds(Points)
	local bounds = AxisAlignedBox3d.Empty()
	local Points = self.Points
	for vid in Points:VertexIndices() do
		bounds:Contain(Points:GetVertex(vid))
	end
	local cellsize = bounds:MaxDim() / subdivs
	self:BuildCell(cellsize, bounds.Min)
end

-- C# Build(double cellSize, Vector3d origin)
function PointSetHashtable:BuildCell(cellSize: number, origin)
	self.Origin = origin
	self.CellSize = cellSize
	self.indexF = GridIndexing.ShiftGridIndexer3.new(origin, cellSize)

	self.Grid = DSparseGrid3.new(PointList.new())

	local Points = self.Points
	for vid in Points:VertexIndices() do
		local v = Points:GetVertex(vid)
		local idx = self.indexF:ToGrid(v)
		local cell = self.Grid:Get(idx)
		table.insert(cell, vid)
	end
end

function PointSetHashtable:FindInBall(pt, r: number, buffer: { number }, bufferCapacity: number?): (boolean, number)
	local capacity = bufferCapacity or 1024
	local buffer_count = 0

	local CellSize = self.CellSize
	local halfCell = CellSize * 0.5
	local indexF = self.indexF
	local idx = indexF:ToGrid(pt)
	local cc = indexF:FromGrid(idx)
	local ix, iy, iz = idx.x, idx.y, idx.z
	local cx, cy, cz = cc.x + halfCell, cc.y + halfCell, cc.z + halfCell

	if r > CellSize then
		error("PointSetHashtable.FindInBall: large radius unsupported")
	end

	local r2 = r * r
	local Points = self.Points
	local Grid = self.Grid
	local px, py, pz = pt.x, pt.y, pt.z

	-- check all in this cell
	local center_cell = Grid:Get(idx, false)
	if center_cell ~= nil then
		for _, vid in center_cell do
			if pt:DistanceSquared(Points:GetVertex(vid)) < r2 then
				if buffer_count == capacity then
					return false, buffer_count
				end
				buffer_count += 1
				buffer[buffer_count] = vid
			end
		end
	end

	-- if we are close enough to cell border we need to check nbrs
	-- [TODO] could iterate over fewer cells here, if r is bounded by CellSize,
	-- then we should only ever need to look at 3, depending on which octant we are in.
	-- C#: (pt-center).MaxAbs + r > halfCell
	if math.max(abs(px - cx), abs(py - cy), abs(pz - cz)) + r > halfCell then
		for ci = 1, 26 do
			local ioffset = GridOffsets26[ci]

			-- if we are within r from face, we need to look into it
			local dx = cx + halfCell * ioffset.x - px
			local dy = cy + halfCell * ioffset.y - py
			local dz = cz + halfCell * ioffset.z - pz
			-- C#: ptToFaceCenter.MinAbs > r
			if math.min(abs(dx), abs(dy), abs(dz)) > r then
				continue
			end

			local ncell = Grid:GetXYZ(ix + ioffset.x, iy + ioffset.y, iz + ioffset.z, false)
			if ncell ~= nil then
				for _, vid in ncell do
					if pt:DistanceSquared(Points:GetVertex(vid)) < r2 then
						if buffer_count == capacity then
							return false, buffer_count
						end
						buffer_count += 1
						buffer[buffer_count] = vid
					end
				end
			end
		end
	end

	return true, buffer_count
end

return PointSetHashtable
