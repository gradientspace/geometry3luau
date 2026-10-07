--!native
-- Port of geometry3Sharp spatial/DenseGrid3.cs (class DenseGrid3i)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- 3D dense grid of integers.
--
-- Luau port notes:
--   * Buffer layout: `grid.Buffer` is a flat 1-based Luau array of numbers. The C# linear index
--     i = x + ni*(y + nj*z) (0-based) is stored at Buffer[i + 1]. resize() replaces the Buffer table.
--   * Constructors: DenseGrid3i.new() (empty), DenseGrid3i.new(ni, nj, nk, initialValue)
--   * Indexers:   this[int i]          -> grid:GetLinear(i) / grid:SetLinear(i, v)   (0-based i)
--                 this[int i,j,k]      -> grid:Get(i, j, k) / grid:Set(i, j, k, v)
--                 this[Vector3i ijk]   -> grid:GetV(ijk) / grid:SetV(ijk, v)
--   * atomic_increment / atomic_decrement / atomic_incdec are plain (sequential) updates.
--   * Properties as methods: size()

local Vector3i = require(script.Parent.Vector3i)
local DenseGrid3f = require(script.Parent.DenseGrid3f)

local DenseGrid3i = {}
DenseGrid3i.__index = DenseGrid3i
DenseGrid3i.ClassName = "DenseGrid3i"

function DenseGrid3i.new(ni: number?, nj: number?, nk: number?, initialValue: number?)
	local self = setmetatable({ Buffer = {}, ni = 0, nj = 0, nk = 0 }, DenseGrid3i)
	if ni ~= nil then
		self:resize(ni, nj :: number, nk :: number)
		self:assign(initialValue or 0)
	end
	return self
end

function DenseGrid3i:size(): number
	return self.ni * self.nj * self.nk
end

function DenseGrid3i:resize(ni: number, nj: number, nk: number)
	self.Buffer = table.create(ni * nj * nk, 0)
	self.ni, self.nj, self.nk = ni, nj, nk
end

function DenseGrid3i:assign(value: number)
	local buf = self.Buffer
	for i = 1, #buf do
		buf[i] = value
	end
end

function DenseGrid3i:GetLinear(i: number): number
	return self.Buffer[i + 1]
end
function DenseGrid3i:SetLinear(i: number, v: number)
	self.Buffer[i + 1] = v
end
function DenseGrid3i:Get(i: number, j: number, k: number): number
	return self.Buffer[i + self.ni * (j + self.nj * k) + 1]
end
function DenseGrid3i:Set(i: number, j: number, k: number, v: number)
	self.Buffer[i + self.ni * (j + self.nj * k) + 1] = v
end
function DenseGrid3i:GetV(ijk): number
	return self.Buffer[ijk.x + self.ni * (ijk.y + self.nj * ijk.z) + 1]
end
function DenseGrid3i:SetV(ijk, v: number)
	self.Buffer[ijk.x + self.ni * (ijk.y + self.nj * ijk.z) + 1] = v
end

function DenseGrid3i:increment(i: number, j: number, k: number)
	local idx = i + self.ni * (j + self.nj * k) + 1
	self.Buffer[idx] += 1
end
function DenseGrid3i:decrement(i: number, j: number, k: number)
	local idx = i + self.ni * (j + self.nj * k) + 1
	self.Buffer[idx] -= 1
end
DenseGrid3i.atomic_increment = DenseGrid3i.increment
DenseGrid3i.atomic_decrement = DenseGrid3i.decrement
function DenseGrid3i:atomic_incdec(i: number, j: number, k: number, decrement: boolean?)
	local idx = i + self.ni * (j + self.nj * k) + 1
	if decrement then
		self.Buffer[idx] -= 1
	else
		self.Buffer[idx] += 1
	end
end

function DenseGrid3i:get_slice(slice_i: number, dimension: number)
	local DenseGrid2i = require(script.Parent.DenseGrid2i)
	local ni, nj, nk, buf = self.ni, self.nj, self.nk, self.Buffer
	local slice
	if dimension == 0 then
		slice = DenseGrid2i.new(nj, nk, 0)
		local sb = slice.Buffer
		for k = 0, nk - 1 do
			for j = 0, nj - 1 do
				sb[j + nj * k + 1] = buf[slice_i + ni * (j + nj * k) + 1]
			end
		end
	elseif dimension == 1 then
		slice = DenseGrid2i.new(ni, nk, 0)
		local sb = slice.Buffer
		for k = 0, nk - 1 do
			for i = 0, ni - 1 do
				sb[i + ni * k + 1] = buf[i + ni * (slice_i + nj * k) + 1]
			end
		end
	else
		slice = DenseGrid2i.new(ni, nj, 0)
		local sb = slice.Buffer
		for j = 0, nj - 1 do
			for i = 0, ni - 1 do
				sb[i + ni * j + 1] = buf[i + ni * (j + nj * slice_i) + 1]
			end
		end
	end
	return slice
end

-- convert to binary bitmap
function DenseGrid3i:get_bitmap(thresh: number?)
	local t = thresh or 0
	local Bitmap3 = require(script.Parent.Bitmap3)
	local bmp = Bitmap3.new(Vector3i.new(self.ni, self.nj, self.nk))
	local bits, buf = bmp.Bits.bits, self.Buffer
	for i = 1, #buf do
		bits[i] = buf[i] > t
	end
	return bmp
end

function DenseGrid3i:Indices()
	return DenseGrid3f._gridIndices(0, 0, 0, self.ni, self.nj, self.nk)
end

function DenseGrid3i:InsetIndices(border_width: number)
	return DenseGrid3f._gridIndices(
		border_width,
		border_width,
		border_width,
		self.ni - border_width,
		self.nj - border_width,
		self.nk - border_width
	)
end

return DenseGrid3i
