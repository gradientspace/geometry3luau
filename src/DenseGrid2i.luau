--!native
-- Port of geometry3Sharp spatial/DenseGrid2.cs (class DenseGrid2i)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- 2D dense grid of integers.
--
-- Luau port notes:
--   * Buffer layout: flat 1-based Luau array; C# linear index i = x + ni*y is stored at Buffer[i + 1].
--   * Constructors: DenseGrid2i.new() (empty), DenseGrid2i.new(ni, nj, initialValue), DenseGrid2i.FromCopy(copy)
--   * Indexers: this[int i] -> GetLinear/SetLinear (0-based), this[i,j] -> Get(i, j)/Set(i, j, v),
--               this[Vector2i] -> GetV(ij)/SetV(ij, v)
--   * atomic_* functions are plain sequential updates; size() is a method

local DenseGrid2d = require(script.Parent.DenseGrid2d)

local DenseGrid2i = {}
DenseGrid2i.__index = DenseGrid2i
DenseGrid2i.ClassName = "DenseGrid2i"

function DenseGrid2i.new(ni: number?, nj: number?, initialValue: number?)
	local self = setmetatable({ Buffer = {}, ni = 0, nj = 0 }, DenseGrid2i)
	if ni ~= nil then
		self:resize(ni, nj :: number)
		self:assign(initialValue or 0)
	end
	return self
end

function DenseGrid2i.FromCopy(copy)
	return setmetatable({ Buffer = table.clone(copy.Buffer), ni = copy.ni, nj = copy.nj }, DenseGrid2i)
end

function DenseGrid2i:size(): number
	return self.ni * self.nj
end

function DenseGrid2i:resize(ni: number, nj: number)
	self.Buffer = table.create(ni * nj, 0)
	self.ni, self.nj = ni, nj
end

function DenseGrid2i:clear()
	self:assign(0)
end

function DenseGrid2i:copy(copy)
	local buf, src = self.Buffer, copy.Buffer
	for i = 1, #buf do
		buf[i] = src[i]
	end
end

function DenseGrid2i:assign(value: number)
	local buf = self.Buffer
	for i = 1, #buf do
		buf[i] = value
	end
end

function DenseGrid2i:GetLinear(i: number): number
	return self.Buffer[i + 1]
end
function DenseGrid2i:SetLinear(i: number, v: number)
	self.Buffer[i + 1] = v
end
function DenseGrid2i:Get(i: number, j: number): number
	return self.Buffer[i + self.ni * j + 1]
end
function DenseGrid2i:Set(i: number, j: number, v: number)
	self.Buffer[i + self.ni * j + 1] = v
end
function DenseGrid2i:GetV(ij): number
	return self.Buffer[ij.x + self.ni * ij.y + 1]
end
function DenseGrid2i:SetV(ij, v: number)
	self.Buffer[ij.x + self.ni * ij.y + 1] = v
end

function DenseGrid2i:increment(i: number, j: number)
	local idx = i + self.ni * j + 1
	self.Buffer[idx] += 1
end
function DenseGrid2i:decrement(i: number, j: number)
	local idx = i + self.ni * j + 1
	self.Buffer[idx] -= 1
end
DenseGrid2i.atomic_increment = DenseGrid2i.increment
DenseGrid2i.atomic_decrement = DenseGrid2i.decrement
function DenseGrid2i:atomic_incdec(i: number, j: number, decrement: boolean?)
	local idx = i + self.ni * j + 1
	if decrement then
		self.Buffer[idx] -= 1
	else
		self.Buffer[idx] += 1
	end
end

function DenseGrid2i:sum(): number
	local s = 0
	for _, v in self.Buffer do
		s += v
	end
	return s
end

function DenseGrid2i:Indices()
	return DenseGrid2d._indices2(0, 0, self.ni, self.nj)
end
function DenseGrid2i:InsetIndices(border_width: number)
	return DenseGrid2d._indices2(border_width, border_width, self.ni - border_width, self.nj - border_width)
end

return DenseGrid2i
