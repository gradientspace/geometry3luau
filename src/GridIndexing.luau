--!native
-- Port of geometry3Sharp spatial/GridIndexing.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- This module returns a table holding all the types in GridIndexing.cs:
--   GridIndexing.ScaleGridIndexer3.new(cellSize)
--   GridIndexing.ShiftGridIndexer3.new(origin: Vector3d, cellSize)
--   GridIndexing.FrameGridIndexer3.new(frame: Frame3d, cellSize: Vector3d)
--   GridIndexing.MultigridIndexer3.new(blockSize: Vector3i)
--   GridIndexing.GridLevelIndex.new(block_index, local_index)    (plain table {block_index=, local_index=})
--
-- Interfaces (duck-typed):
--   IGridWorldIndexer3: ToGrid(Vector3d) -> Vector3i, ToGridf(Vector3d) -> Vector3d,
--                       FromGrid(Vector3i or Vector3d) -> Vector3d
--   IMultigridIndexer3: ToBlock, ToBlockIndex, ToBlockLocal, FromBlock
--
-- Luau port notes:
--   * The two C# FromGrid overloads (Vector3i / Vector3d argument) are one function: both just read x/y/z.
--   * (int) casts truncate toward zero (MathUtil.TruncInt), integer division truncates (MathUtil.IntDiv).
--   * These are structs in C#; the Luau objects are mutable tables (fields CellSize, Origin, ...).

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local Vector3i = require(script.Parent.Vector3i)

local TruncInt, IntDiv = MathUtil.TruncInt, MathUtil.IntDiv
local V3, V3i = Vector3d.new, Vector3i.new

local GridIndexing = {}

----------------------------------------------------------------------------------------------------
-- GridLevelIndex
----------------------------------------------------------------------------------------------------
local GridLevelIndex = {}
GridLevelIndex.__index = GridLevelIndex
function GridLevelIndex.new(block_index, local_index)
	return setmetatable({ block_index = block_index, local_index = local_index }, GridLevelIndex)
end
GridIndexing.GridLevelIndex = GridLevelIndex

----------------------------------------------------------------------------------------------------
-- ScaleGridIndexer3: Map to/from grid coords
----------------------------------------------------------------------------------------------------
local ScaleGridIndexer3 = {}
ScaleGridIndexer3.__index = ScaleGridIndexer3
ScaleGridIndexer3.ClassName = "ScaleGridIndexer3"

function ScaleGridIndexer3.new(cellSize: number)
	return setmetatable({ CellSize = cellSize }, ScaleGridIndexer3)
end
function ScaleGridIndexer3:Clone()
	return ScaleGridIndexer3.new(self.CellSize)
end
function ScaleGridIndexer3:ToGrid(point)
	local c = self.CellSize
	return V3i(TruncInt(point.x / c), TruncInt(point.y / c), TruncInt(point.z / c))
end
function ScaleGridIndexer3:ToGridf(point)
	local c = self.CellSize
	return V3(point.x / c, point.y / c, point.z / c)
end
-- FromGrid(Vector3i gridpoint) / FromGrid(Vector3d gridpointf)
function ScaleGridIndexer3:FromGrid(gridpoint)
	local c = self.CellSize
	return V3(gridpoint.x * c, gridpoint.y * c, gridpoint.z * c)
end
GridIndexing.ScaleGridIndexer3 = ScaleGridIndexer3

----------------------------------------------------------------------------------------------------
-- ShiftGridIndexer3: Map to/from grid coords, where grid is translated from origin
----------------------------------------------------------------------------------------------------
local ShiftGridIndexer3 = {}
ShiftGridIndexer3.__index = ShiftGridIndexer3
ShiftGridIndexer3.ClassName = "ShiftGridIndexer3"

function ShiftGridIndexer3.new(origin, cellSize: number)
	return setmetatable({ Origin = origin, CellSize = cellSize }, ShiftGridIndexer3)
end
function ShiftGridIndexer3:Clone()
	return ShiftGridIndexer3.new(self.Origin, self.CellSize)
end
function ShiftGridIndexer3:ToGrid(point)
	local c, o = self.CellSize, self.Origin
	return V3i(TruncInt((point.x - o.x) / c), TruncInt((point.y - o.y) / c), TruncInt((point.z - o.z) / c))
end
function ShiftGridIndexer3:ToGridf(point)
	local c, o = self.CellSize, self.Origin
	return V3((point.x - o.x) / c, (point.y - o.y) / c, (point.z - o.z) / c)
end
function ShiftGridIndexer3:FromGrid(gridpoint)
	local c, o = self.CellSize, self.Origin
	return V3(gridpoint.x * c + o.x, gridpoint.y * c + o.y, gridpoint.z * c + o.z)
end
GridIndexing.ShiftGridIndexer3 = ShiftGridIndexer3

----------------------------------------------------------------------------------------------------
-- FrameGridIndexer3: Map to/from grid coords, where grid is relative to frame coords/axes
----------------------------------------------------------------------------------------------------
local FrameGridIndexer3 = {}
FrameGridIndexer3.__index = FrameGridIndexer3
FrameGridIndexer3.ClassName = "FrameGridIndexer3"

function FrameGridIndexer3.new(frame, cellSize)
	return setmetatable({ GridFrame = frame, CellSize = cellSize }, FrameGridIndexer3)
end
function FrameGridIndexer3:Clone()
	return FrameGridIndexer3.new(self.GridFrame:Clone(), self.CellSize)
end
function FrameGridIndexer3:ToGrid(point)
	local pointf = self.GridFrame:ToFrameP(point)
	local c = self.CellSize
	return V3i(TruncInt(pointf.x / c.x), TruncInt(pointf.y / c.y), TruncInt(pointf.z / c.z))
end
function FrameGridIndexer3:ToGridf(point)
	point = self.GridFrame:ToFrameP(point)
	local c = self.CellSize
	return V3(point.x / c.x, point.y / c.y, point.z / c.z)
end
function FrameGridIndexer3:FromGrid(gridpoint)
	local c = self.CellSize
	local pointf = V3(c.x * gridpoint.x, c.y * gridpoint.y, c.z * gridpoint.z)
	return self.GridFrame:FromFrameP(pointf)
end
GridIndexing.FrameGridIndexer3 = FrameGridIndexer3

----------------------------------------------------------------------------------------------------
-- MultigridIndexer3: map between "outer" (ie higher-res) grid coordinates and "blocks" of those coordinates.
----------------------------------------------------------------------------------------------------
local MultigridIndexer3 = {}
MultigridIndexer3.__index = MultigridIndexer3
MultigridIndexer3.ClassName = "MultigridIndexer3"

function MultigridIndexer3.new(blockSize)
	return setmetatable({ BlockSize = blockSize, OuterShift = Vector3i.Zero, BlockShift = Vector3i.Zero }, MultigridIndexer3)
end
function MultigridIndexer3:Clone()
	local c = MultigridIndexer3.new(self.BlockSize)
	c.OuterShift, c.BlockShift = self.OuterShift, self.BlockShift
	return c
end

local function block_div(v: number, s: number): number
	-- (v >= 0) ? (int)(v / s) : ((int)(v / s) - 1)    [C# integer division]
	if v >= 0 then
		return IntDiv(v, s)
	end
	return IntDiv(v, s) - 1
end

function MultigridIndexer3:ToBlockIndex(outer_index)
	local bs, os_, bsh = self.BlockSize, self.OuterShift, self.BlockShift
	local bx = block_div(outer_index.x - os_.x, bs.x)
	local by = block_div(outer_index.y - os_.y, bs.y)
	local bz = block_div(outer_index.z - os_.z, bs.z)
	return V3i(bx - bsh.x, by - bsh.y, bz - bsh.z)
end

function MultigridIndexer3:ToBlockLocal(outer_index)
	local block_idx = self:ToBlockIndex(outer_index)
	local bs = self.BlockSize
	return V3i(outer_index.x - block_idx.x * bs.x, outer_index.y - block_idx.y * bs.y, outer_index.z - block_idx.z * bs.z)
end

function MultigridIndexer3:ToBlock(outer_index)
	local block_index = self:ToBlockIndex(outer_index)
	local bs = self.BlockSize
	return GridLevelIndex.new(
		block_index,
		V3i(outer_index.x - block_index.x * bs.x, outer_index.y - block_index.y * bs.y, outer_index.z - block_index.z * bs.z)
	)
end

function MultigridIndexer3:FromBlock(block_idx)
	local bs, os_, bsh = self.BlockSize, self.OuterShift, self.BlockShift
	return V3i((block_idx.x + bsh.x) * bs.x + os_.x, (block_idx.y + bsh.y) * bs.y + os_.y, (block_idx.z + bsh.z) * bs.z + os_.z)
end
GridIndexing.MultigridIndexer3 = MultigridIndexer3

return GridIndexing
