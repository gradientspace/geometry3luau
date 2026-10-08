--!native
-- Port of geometry3Sharp spatial/DenseGrid3.cs (C# class DenseGrid3f; named DenseGrid3d here, see PORTING.md)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- 3D dense grid of floating-point scalar values.
--
-- Luau port notes:
--   * Buffer layout: `grid.Buffer` is a flat 1-based Luau array of numbers. The C# linear index
--     i = x + ni*(y + nj*z) (0-based) is stored at Buffer[i + 1]. Hot code may read/write Buffer
--     directly. Note that resize() and swap() replace the Buffer table, so re-read it after those.
--   * Values are doubles (C# stored floats).
--   * Constructors: DenseGrid3d.new() (empty), DenseGrid3d.new(ni, nj, nk, initialValue),
--     DenseGrid3d.FromCopy(copy)  (C# copy constructor)
--   * Indexers:   this[int i]          -> grid:GetLinear(i) / grid:SetLinear(i, f)   (0-based i)
--                 this[int i,j,k]      -> grid:Get(i, j, k) / grid:Set(i, j, k, f)
--                 this[Vector3i ijk]   -> grid:GetV(ijk) / grid:SetV(ijk, f)
--   * get_x_pair(i0, j, k) -> a, b   (C# out parameters)
--   * Properties as methods: size(), Bounds(), BoundsInclusive()
--   * to_linear(i, j, k) or to_linear(Vector3i)
--   * Indices() / InsetIndices(border) return iterators yielding new Vector3i (x fastest)

local Vector3i = require(script.Parent.Vector3i)
local Util = require(script.Parent.Util)

local DenseGrid3d = {}
DenseGrid3d.__index = DenseGrid3d
DenseGrid3d.ClassName = "DenseGrid3d"

function DenseGrid3d.new(ni: number?, nj: number?, nk: number?, initialValue: number?)
	local self = setmetatable({ Buffer = {}, ni = 0, nj = 0, nk = 0 }, DenseGrid3d)
	if ni ~= nil then
		self:resize(ni, nj :: number, nk :: number)
		self:assign(initialValue or 0)
	end
	return self
end

function DenseGrid3d.FromCopy(copy)
	local self = setmetatable({ Buffer = table.clone(copy.Buffer), ni = copy.ni, nj = copy.nj, nk = copy.nk }, DenseGrid3d)
	return self
end

function DenseGrid3d:swap(g2)
	Util.gDevAssert(self.ni == g2.ni and self.nj == g2.nj and self.nk == g2.nk)
	local tmp = g2.Buffer
	g2.Buffer = self.Buffer
	self.Buffer = tmp
end

function DenseGrid3d:size(): number
	return self.ni * self.nj * self.nk
end

function DenseGrid3d:resize(ni: number, nj: number, nk: number)
	self.Buffer = table.create(ni * nj * nk, 0)
	self.ni, self.nj, self.nk = ni, nj, nk
end

function DenseGrid3d:assign(value: number)
	local buf = self.Buffer
	for i = 1, #buf do
		buf[i] = value
	end
end

function DenseGrid3d:set_min(ijk, f: number)
	local idx = ijk.x + self.ni * (ijk.y + self.nj * ijk.z) + 1
	if f < self.Buffer[idx] then
		self.Buffer[idx] = f
	end
end
function DenseGrid3d:set_max(ijk, f: number)
	local idx = ijk.x + self.ni * (ijk.y + self.nj * ijk.z) + 1
	if f > self.Buffer[idx] then
		self.Buffer[idx] = f
	end
end

-- this[int i]
function DenseGrid3d:GetLinear(i: number): number
	return self.Buffer[i + 1]
end
function DenseGrid3d:SetLinear(i: number, f: number)
	self.Buffer[i + 1] = f
end

-- this[int i, int j, int k]
function DenseGrid3d:Get(i: number, j: number, k: number): number
	return self.Buffer[i + self.ni * (j + self.nj * k) + 1]
end
function DenseGrid3d:Set(i: number, j: number, k: number, f: number)
	self.Buffer[i + self.ni * (j + self.nj * k) + 1] = f
end

-- this[Vector3i ijk]
function DenseGrid3d:GetV(ijk): number
	return self.Buffer[ijk.x + self.ni * (ijk.y + self.nj * ijk.z) + 1]
end
function DenseGrid3d:SetV(ijk, f: number)
	self.Buffer[ijk.x + self.ni * (ijk.y + self.nj * ijk.z) + 1] = f
end

-- get_x_pair(i0, j, k, out a, out b) -> a, b
function DenseGrid3d:get_x_pair(i0: number, j: number, k: number): (number, number)
	local offset = self.ni * (j + self.nj * k) + i0 + 1
	local buf = self.Buffer
	return buf[offset], buf[offset + 1]
end

function DenseGrid3d:apply(f: (number) -> number)
	local buf = self.Buffer
	local ni, nj, nk = self.ni, self.nj, self.nk
	for k = 0, nk - 1 do
		for j = 0, nj - 1 do
			for i = 0, ni - 1 do
				local idx = i + ni * (j + nj * k) + 1
				buf[idx] = f(buf[idx])
			end
		end
	end
end

function DenseGrid3d:get_slice(slice_i: number, dimension: number)
	local DenseGrid2d = require(script.Parent.DenseGrid2d)
	local ni, nj, nk, buf = self.ni, self.nj, self.nk, self.Buffer
	local slice
	if dimension == 0 then
		slice = DenseGrid2d.new(nj, nk, 0)
		local sb = slice.Buffer
		for k = 0, nk - 1 do
			for j = 0, nj - 1 do
				sb[j + nj * k + 1] = buf[slice_i + ni * (j + nj * k) + 1]
			end
		end
	elseif dimension == 1 then
		slice = DenseGrid2d.new(ni, nk, 0)
		local sb = slice.Buffer
		for k = 0, nk - 1 do
			for i = 0, ni - 1 do
				sb[i + ni * k + 1] = buf[i + ni * (slice_i + nj * k) + 1]
			end
		end
	else
		slice = DenseGrid2d.new(ni, nj, 0)
		local sb = slice.Buffer
		for j = 0, nj - 1 do
			for i = 0, ni - 1 do
				sb[i + ni * j + 1] = buf[i + ni * (j + nj * slice_i) + 1]
			end
		end
	end
	return slice
end

function DenseGrid3d:set_slice(slice, slice_i: number, dimension: number)
	local ni, nj, nk, buf = self.ni, self.nj, self.nk, self.Buffer
	local sb, sni = slice.Buffer, slice.ni
	if dimension == 0 then
		for k = 0, nk - 1 do
			for j = 0, nj - 1 do
				buf[slice_i + ni * (j + nj * k) + 1] = sb[j + sni * k + 1]
			end
		end
	elseif dimension == 1 then
		for k = 0, nk - 1 do
			for i = 0, ni - 1 do
				buf[i + ni * (slice_i + nj * k) + 1] = sb[i + sni * k + 1]
			end
		end
	else
		for j = 0, nj - 1 do
			for i = 0, ni - 1 do
				buf[i + ni * (j + nj * slice_i) + 1] = sb[i + sni * j + 1]
			end
		end
	end
end

function DenseGrid3d:Bounds()
	return require(script.Parent.AxisAlignedBox3i).new(0, 0, 0, self.ni, self.nj, self.nk)
end
function DenseGrid3d:BoundsInclusive()
	return require(script.Parent.AxisAlignedBox3i).new(0, 0, 0, self.ni - 1, self.nj - 1, self.nk - 1)
end

local function gridIndices(x0: number, y0: number, z0: number, x1: number, y1: number, z1: number)
	-- iterate [x0,x1) x [y0,y1) x [z0,z1), x fastest
	local x, y, z = x0 - 1, y0, z0
	return function()
		x += 1
		if x >= x1 then
			x = x0
			y += 1
			if y >= y1 then
				y = y0
				z += 1
			end
		end
		if z >= z1 or x0 >= x1 or y0 >= y1 then
			return nil
		end
		return Vector3i.new(x, y, z)
	end
end
DenseGrid3d._gridIndices = gridIndices

function DenseGrid3d:Indices()
	return gridIndices(0, 0, 0, self.ni, self.nj, self.nk)
end

function DenseGrid3d:InsetIndices(border_width: number)
	return gridIndices(border_width, border_width, border_width, self.ni - border_width, self.nj - border_width, self.nk - border_width)
end

function DenseGrid3d:to_index(idx: number)
	local ni, nj = self.ni, self.nj
	local x = idx % ni
	local y = (idx // ni) % nj
	local z = idx // (ni * nj)
	return Vector3i.new(x, y, z)
end

-- to_linear(i, j, k) or to_linear(Vector3i ijk)
function DenseGrid3d:to_linear(i: any, j: number?, k: number?): number
	if type(i) == "table" then
		return i.x + self.ni * (i.y + self.nj * i.z)
	end
	return i + self.ni * ((j :: number) + self.nj * (k :: number))
end

return DenseGrid3d
