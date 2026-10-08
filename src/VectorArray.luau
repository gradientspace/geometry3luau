-- Port of geometry3Sharp core/VectorArray.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- These classes are just wrappers around a flat array that provide convenient 2/3/4-element set/get
-- access. Useful for things like treating a number array as a list of vectors.
--
-- Luau port notes (see PORTING.md):
--   * Same class names as C#; only the double/int versions are ported:
--       VectorArray3 (C# generic VectorArray3<T>), VectorArray3d, VectorArray3i, IndexArray3i
--       VectorArray2 (C# generic VectorArray2<T>), VectorArray2d, IndexArray2i
--       VectorArray4 (C# generic VectorArray4<T>), IndexArray4i
--     Not ported (float storage): VectorArray3f, VectorArray2f. Use VectorArray3d / VectorArray2d.
--     All classes live in this module: G3.VectorArray.VectorArray3d, ...
--   * .array is a flat 1-based Luau array: C# array[k] -> array[k + 1]. Element indices i used by the
--     API (Get, Set, Count) are 0-based like C#. new(nCount) fills the array with zeros (C# default(T)).
--   * Constructors:
--       X.new(nCount?)          new X(int nCount = 0)          (VectorArray3d.new(nCount, debug?) as C#)
--       X.FromArray(data)       new X(T[] data)                (wraps data, no copy)
--   * Count() is a method. GetEnumerator() iterates the raw array values (C# IEnumerable<T>).
--   * indexer this[i] -> Get(i) / Set(i, v). Set dispatches on its arguments:
--       Set(i, v)                      indexer setter (v is a vector or IndexNi array)
--       Set(i, a, b, c)                C# Set(int i, T a, T b, T c)   (3i types: Set(i, a, b, c, bCycle))
--       Set(iStart, iCount, source)    C# Set(int iStart, int iCount, VectorArray3d<T> source)
--     The explicit forms are also available as SetComponents(i, ...) and SetRange(iStart, iCount, source).
--   * AsVector3() / AsVector3i() / AsIndex3i() / AsVector2() / AsIndex2i() / AsIndex4i() return iterators.
--     AsArray() returns a 1-based Luau array.
--   * IndexArray3i:Get(i) returns an Index3i array { a, b, c } (IndexArray2i: Index2i, IndexArray4i: Index4i).
--   * VectorArray3d debug mode (C# #if DEBUG only) is active when debug is true and Util.DebugChecks is on.
--   * Fixed C# bug: the IndexArray4i indexer setter used value[4] for the 4th element (out of range).

local Vector3d = require(script.Parent.Vector3d)
local Vector2d = require(script.Parent.Vector2d)
local Vector3i = require(script.Parent.Vector3i)
local Util = require(script.Parent.Util)

local VectorArray = {}

local function valueIter(arr: { number })
	local i = 0
	return function()
		i += 1
		return arr[i]
	end
end

-- copy iCount*N values from source.array (from its start) to array at element iStart
local function copyRange(dst: { number }, N: number, iStart: number, iCount: number, source)
	table.move(source.array, 1, N * iCount, N * iStart + 1, dst)
end

--------------------------------------------------------------------------------------------------
-- VectorArray3d<T>
--------------------------------------------------------------------------------------------------

local VectorArray3 = {}
VectorArray3.__index = VectorArray3
VectorArray3.ClassName = "VectorArray3"
VectorArray.VectorArray3 = VectorArray3

function VectorArray3.new(nCount: number?)
	return setmetatable({ array = table.create((nCount or 0) * 3, 0) }, VectorArray3)
end

function VectorArray3.FromArray(data: { number })
	return setmetatable({ array = data }, VectorArray3)
end

function VectorArray3:Count(): number
	return #self.array // 3
end

function VectorArray3:GetEnumerator()
	return valueIter(self.array)
end

function VectorArray3:Resize(Count: number)
	self.array = table.create(3 * Count, 0)
end

function VectorArray3:SetComponents(i: number, a: number, b: number, c: number)
	local array = self.array
	array[3 * i + 1] = a
	array[3 * i + 2] = b
	array[3 * i + 3] = c
end

function VectorArray3:SetRange(iStart: number, iCount: number, source)
	copyRange(self.array, 3, iStart, iCount, source)
end

-- Set(i, a, b, c) / Set(iStart, iCount, source)
function VectorArray3:Set(i: number, a: any, b: any, c: any)
	if type(b) == "table" then
		self:SetRange(i, a, b)
	else
		self:SetComponents(i, a, b, c)
	end
end

--------------------------------------------------------------------------------------------------
-- VectorArray3d -> VectorArray3d
--------------------------------------------------------------------------------------------------

local VectorArray3d = setmetatable({}, { __index = VectorArray3 })
VectorArray3d.__index = VectorArray3d
VectorArray3d.ClassName = "VectorArray3d"
VectorArray.VectorArray3d = VectorArray3d

local invalid_value = -99999999.0

function VectorArray3d.new(nCount: number, debug: boolean?)
	local self = setmetatable({ array = table.create(nCount * 3, 0), __debug = false }, VectorArray3d)
	if Util.DebugChecks then
		self.__debug = debug == true
		if self.__debug then
			for i = 0, self:Count() - 1 do
				self:SetComponents(i, invalid_value, invalid_value, invalid_value)
			end
		end
	end
	return self
end

function VectorArray3d.FromArray(data: { number })
	return setmetatable({ array = data, __debug = false }, VectorArray3d)
end

function VectorArray3d:Get(i: number)
	local array = self.array
	return Vector3d.new(array[3 * i + 1], array[3 * i + 2], array[3 * i + 3])
end

-- Set(i, Vector3d) (indexer setter) / Set(i, a, b, c) / Set(iStart, iCount, source)
function VectorArray3d:Set(i: number, a: any, b: any, c: any)
	if type(a) == "table" then
		if self.__debug and self.array[3 * i + 1] ~= invalid_value then
			error(string.format("VectorArray3d.set - value %d is already set!", i))
		end
		self:SetComponents(i, a.x, a.y, a.z)
	else
		VectorArray3.Set(self, i, a, b, c)
	end
end

function VectorArray3d:AsVector3()
	local i, N = -1, self:Count()
	return function()
		i += 1
		if i < N then
			return self:Get(i)
		end
		return nil
	end
end

function VectorArray3d:AsArray()
	local N = self:Count()
	local array = self.array
	local values = table.create(N)
	for i = 0, N - 1 do
		values[i + 1] = Vector3d.new(array[3 * i + 1], array[3 * i + 2], array[3 * i + 3])
	end
	return values
end

--------------------------------------------------------------------------------------------------
-- VectorArray3i, IndexArray3i
--------------------------------------------------------------------------------------------------

-- [RMS] for CW/CCW codes
local function setCycle(self, i: number, a: number, b: number, c: number, bCycle: boolean?)
	local array = self.array
	array[3 * i + 1] = a
	if bCycle then
		array[3 * i + 2] = c
		array[3 * i + 3] = b
	else
		array[3 * i + 2] = b
		array[3 * i + 3] = c
	end
end

local function makeInt3(name: string)
	local class = setmetatable({}, { __index = VectorArray3 })
	class.__index = class
	class.ClassName = name
	function class.new(nCount: number?)
		return setmetatable({ array = table.create((nCount or 0) * 3, 0) }, class)
	end
	function class.FromArray(data: { number })
		return setmetatable({ array = data }, class)
	end
	class.SetComponents = setCycle
	return class
end

local VectorArray3i = makeInt3("VectorArray3i")
VectorArray.VectorArray3i = VectorArray3i

function VectorArray3i:Get(i: number)
	local array = self.array
	return Vector3i.new(array[3 * i + 1], array[3 * i + 2], array[3 * i + 3])
end

-- Set(i, Vector3i) / Set(i, a, b, c, bCycle?) / Set(iStart, iCount, source)
function VectorArray3i:Set(i: number, a: any, b: any, c: any, bCycle: boolean?)
	if type(a) == "table" then
		setCycle(self, i, a.x, a.y, a.z)
	elseif type(b) == "table" then
		self:SetRange(i, a, b)
	else
		setCycle(self, i, a, b, c, bCycle)
	end
end

function VectorArray3i:AsVector3i()
	local i, N = -1, self:Count()
	return function()
		i += 1
		if i < N then
			return self:Get(i)
		end
		return nil
	end
end

local IndexArray3i = makeInt3("IndexArray3i")
VectorArray.IndexArray3i = IndexArray3i

function IndexArray3i:Get(i: number): { number }
	local array = self.array
	return { array[3 * i + 1], array[3 * i + 2], array[3 * i + 3] }
end

-- Set(i, Index3i) / Set(i, a, b, c, bCycle?) / Set(iStart, iCount, source)
function IndexArray3i:Set(i: number, a: any, b: any, c: any, bCycle: boolean?)
	if type(a) == "table" then
		setCycle(self, i, a[1], a[2], a[3])
	elseif type(b) == "table" then
		self:SetRange(i, a, b)
	else
		setCycle(self, i, a, b, c, bCycle)
	end
end

function IndexArray3i:AsIndex3i()
	local i, N = -1, self:Count()
	local array = self.array
	return function()
		i += 1
		if i < N then
			return { array[3 * i + 1], array[3 * i + 2], array[3 * i + 3] }
		end
		return nil
	end
end

--------------------------------------------------------------------------------------------------
-- VectorArray2d<T>: same as VectorArray3d, but for 2D vectors/etc
--------------------------------------------------------------------------------------------------

local VectorArray2 = {}
VectorArray2.__index = VectorArray2
VectorArray2.ClassName = "VectorArray2"
VectorArray.VectorArray2 = VectorArray2

function VectorArray2.new(nCount: number?)
	return setmetatable({ array = table.create((nCount or 0) * 2, 0) }, VectorArray2)
end

function VectorArray2.FromArray(data: { number })
	return setmetatable({ array = data }, VectorArray2)
end

function VectorArray2:Count(): number
	return #self.array // 2
end

function VectorArray2:GetEnumerator()
	return valueIter(self.array)
end

function VectorArray2:Resize(Count: number)
	self.array = table.create(2 * Count, 0)
end

function VectorArray2:SetComponents(i: number, a: number, b: number)
	local array = self.array
	array[2 * i + 1] = a
	array[2 * i + 2] = b
end

function VectorArray2:SetRange(iStart: number, iCount: number, source)
	copyRange(self.array, 2, iStart, iCount, source)
end

-- Set(i, a, b) / Set(iStart, iCount, source)
function VectorArray2:Set(i: number, a: any, b: any)
	if type(b) == "table" then
		self:SetRange(i, a, b)
	else
		self:SetComponents(i, a, b)
	end
end

local function makeSub2(name: string)
	local class = setmetatable({}, { __index = VectorArray2 })
	class.__index = class
	class.ClassName = name
	function class.new(nCount: number?)
		return setmetatable({ array = table.create((nCount or 0) * 2, 0) }, class)
	end
	function class.FromArray(data: { number })
		return setmetatable({ array = data }, class)
	end
	return class
end

-- VectorArray2d -> VectorArray2d
local VectorArray2d = makeSub2("VectorArray2d")
VectorArray.VectorArray2d = VectorArray2d

function VectorArray2d:Get(i: number)
	local array = self.array
	return Vector2d.new(array[2 * i + 1], array[2 * i + 2])
end

-- Set(i, Vector2d) / Set(i, a, b) / Set(iStart, iCount, source)
function VectorArray2d:Set(i: number, a: any, b: any)
	if type(a) == "table" then
		self:SetComponents(i, a.x, a.y)
	else
		VectorArray2.Set(self, i, a, b)
	end
end

function VectorArray2d:AsVector2()
	local i, N = -1, self:Count()
	return function()
		i += 1
		if i < N then
			return self:Get(i)
		end
		return nil
	end
end

function VectorArray2d:AsArray()
	local N = self:Count()
	local array = self.array
	local values = table.create(N)
	for i = 0, N - 1 do
		values[i + 1] = Vector2d.new(array[2 * i + 1], array[2 * i + 2])
	end
	return values
end

local IndexArray2i = makeSub2("IndexArray2i")
VectorArray.IndexArray2i = IndexArray2i

function IndexArray2i:Get(i: number): { number }
	local array = self.array
	return { array[2 * i + 1], array[2 * i + 2] }
end

-- Set(i, Index2i) / Set(i, a, b) / Set(iStart, iCount, source)
function IndexArray2i:Set(i: number, a: any, b: any)
	if type(a) == "table" then
		self:SetComponents(i, a[1], a[2])
	else
		VectorArray2.Set(self, i, a, b)
	end
end

function IndexArray2i:AsIndex2i()
	local i, N = -1, self:Count()
	local array = self.array
	return function()
		i += 1
		if i < N then
			return { array[2 * i + 1], array[2 * i + 2] }
		end
		return nil
	end
end

--------------------------------------------------------------------------------------------------
-- VectorArray4<T>
--------------------------------------------------------------------------------------------------

local VectorArray4 = {}
VectorArray4.__index = VectorArray4
VectorArray4.ClassName = "VectorArray4"
VectorArray.VectorArray4 = VectorArray4

function VectorArray4.new(nCount: number?)
	return setmetatable({ array = table.create((nCount or 0) * 4, 0) }, VectorArray4)
end

function VectorArray4.FromArray(data: { number })
	return setmetatable({ array = data }, VectorArray4)
end

function VectorArray4:Count(): number
	return #self.array // 4
end

function VectorArray4:GetEnumerator()
	return valueIter(self.array)
end

function VectorArray4:Resize(Count: number)
	self.array = table.create(4 * Count, 0)
end

function VectorArray4:SetComponents(i: number, a: number, b: number, c: number, d: number)
	local j = 4 * i
	local array = self.array
	array[j + 1] = a
	array[j + 2] = b
	array[j + 3] = c
	array[j + 4] = d
end

function VectorArray4:SetRange(iStart: number, iCount: number, source)
	copyRange(self.array, 4, iStart, iCount, source)
end

-- Set(i, a, b, c, d) / Set(iStart, iCount, source)
function VectorArray4:Set(i: number, a: any, b: any, c: any, d: any)
	if type(b) == "table" then
		self:SetRange(i, a, b)
	else
		self:SetComponents(i, a, b, c, d)
	end
end

local IndexArray4i = setmetatable({}, { __index = VectorArray4 })
IndexArray4i.__index = IndexArray4i
IndexArray4i.ClassName = "IndexArray4i"
VectorArray.IndexArray4i = IndexArray4i

function IndexArray4i.new(nCount: number?)
	return setmetatable({ array = table.create((nCount or 0) * 4, 0) }, IndexArray4i)
end

function IndexArray4i.FromArray(data: { number })
	return setmetatable({ array = data }, IndexArray4i)
end

function IndexArray4i:Get(i: number): { number }
	local j = 4 * i
	local array = self.array
	return { array[j + 1], array[j + 2], array[j + 3], array[j + 4] }
end

-- Set(i, Index4i) / Set(i, a, b, c, d) / Set(iStart, iCount, source)
function IndexArray4i:Set(i: number, a: any, b: any, c: any, d: any)
	if type(a) == "table" then
		self:SetComponents(i, a[1], a[2], a[3], a[4])
	else
		VectorArray4.Set(self, i, a, b, c, d)
	end
end

function IndexArray4i:AsIndex4i()
	local i, N = -1, self:Count()
	return function()
		i += 1
		if i < N then
			return self:Get(i)
		end
		return nil
	end
end

return VectorArray
