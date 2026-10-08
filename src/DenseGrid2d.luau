--!native
-- Port of geometry3Sharp spatial/DenseGrid2.cs (C# class DenseGrid2f; named DenseGrid2d here, see PORTING.md)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- 2D dense grid of floating-point scalar values.
--
-- Luau port notes:
--   * Buffer layout: flat 1-based Luau array; C# linear index i = x + ni*y is stored at Buffer[i + 1].
--   * Constructors: DenseGrid2d.new() (empty), DenseGrid2d.new(ni, nj, initialValue), DenseGrid2d.FromCopy(copy)
--   * Indexers: this[int i] -> GetLinear/SetLinear (0-based), this[i,j] -> Get(i, j)/Set(i, j, f),
--               this[Vector2i] -> GetV(ij)/SetV(ij, f)
--   * get_x_pair(i0, j) -> a, b;  size(), Bounds() are methods
--   * set_min(grid2)/set_max(grid2) take another grid (as in C#)

local Vector2i = require(script.Parent.Vector2i)
local Util = require(script.Parent.Util)

local DenseGrid2d = {}
DenseGrid2d.__index = DenseGrid2d
DenseGrid2d.ClassName = "DenseGrid2d"

function DenseGrid2d.new(ni: number?, nj: number?, initialValue: number?)
	local self = setmetatable({ Buffer = {}, ni = 0, nj = 0 }, DenseGrid2d)
	if ni ~= nil then
		self:resize(ni, nj :: number)
		self:assign(initialValue or 0)
	end
	return self
end

function DenseGrid2d.FromCopy(copy)
	return setmetatable({ Buffer = table.clone(copy.Buffer), ni = copy.ni, nj = copy.nj }, DenseGrid2d)
end

function DenseGrid2d:swap(g2)
	Util.gDevAssert(self.ni == g2.ni and self.nj == g2.nj)
	local tmp = g2.Buffer
	g2.Buffer = self.Buffer
	self.Buffer = tmp
end

function DenseGrid2d:size(): number
	return self.ni * self.nj
end

function DenseGrid2d:resize(ni: number, nj: number)
	self.Buffer = table.create(ni * nj, 0)
	self.ni, self.nj = ni, nj
end

function DenseGrid2d:assign(value: number)
	local buf = self.Buffer
	for i = 1, #buf do
		buf[i] = value
	end
end

function DenseGrid2d:assign_border(value: number, rings: number)
	local buf, ni, nj = self.Buffer, self.ni, self.nj
	for j = 0, rings - 1 do
		local jb = nj - 1 - j
		for i = 0, ni - 1 do
			buf[i + ni * j + 1] = value
			buf[i + ni * jb + 1] = value
		end
	end
	local stop = nj - 1 - rings
	for j = rings, stop - 1 do
		for i = 0, rings - 1 do
			buf[i + ni * j + 1] = value
			buf[(ni - 1 - i) + ni * j + 1] = value
		end
	end
end

function DenseGrid2d:clear()
	self:assign(0)
end

function DenseGrid2d:copy(copy)
	local buf, src = self.Buffer, copy.Buffer
	for i = 1, #buf do
		buf[i] = src[i]
	end
end

function DenseGrid2d:GetLinear(i: number): number
	return self.Buffer[i + 1]
end
function DenseGrid2d:SetLinear(i: number, f: number)
	self.Buffer[i + 1] = f
end
function DenseGrid2d:Get(i: number, j: number): number
	return self.Buffer[i + self.ni * j + 1]
end
function DenseGrid2d:Set(i: number, j: number, f: number)
	self.Buffer[i + self.ni * j + 1] = f
end
function DenseGrid2d:GetV(ij): number
	return self.Buffer[ij.x + self.ni * ij.y + 1]
end
function DenseGrid2d:SetV(ij, f: number)
	self.Buffer[ij.x + self.ni * ij.y + 1] = f
end

function DenseGrid2d:get_x_pair(i0: number, j: number): (number, number)
	local offset = self.ni * j + i0 + 1
	return self.Buffer[offset], self.Buffer[offset + 1]
end

function DenseGrid2d:apply(f: (number) -> number)
	local buf, ni, nj = self.Buffer, self.ni, self.nj
	for j = 0, nj - 1 do
		for i = 0, ni - 1 do
			local idx = i + ni * j + 1
			buf[idx] = f(buf[idx])
		end
	end
end

function DenseGrid2d:set_min(grid2)
	local buf, b2 = self.Buffer, grid2.Buffer
	for k = 1, #buf do
		buf[k] = math.min(buf[k], b2[k])
	end
end
function DenseGrid2d:set_max(grid2)
	local buf, b2 = self.Buffer, grid2.Buffer
	for k = 1, #buf do
		buf[k] = math.max(buf[k], b2[k])
	end
end

function DenseGrid2d:Bounds()
	return require(script.Parent.AxisAlignedBox2i).new(0, 0, self.ni, self.nj)
end

local function indices2(x0: number, y0: number, x1: number, y1: number)
	local x, y = x0 - 1, y0
	return function()
		x += 1
		if x >= x1 then
			x = x0
			y += 1
		end
		if y >= y1 or x0 >= x1 then
			return nil
		end
		return Vector2i.new(x, y)
	end
end
DenseGrid2d._indices2 = indices2

function DenseGrid2d:Indices()
	return indices2(0, 0, self.ni, self.nj)
end
function DenseGrid2d:InsetIndices(border_width: number)
	return indices2(border_width, border_width, self.ni - border_width, self.nj - border_width)
end

return DenseGrid2d
