--!native
-- Port of geometry3Sharp core/DVector.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- DVector is a dynamic (growable) array. The C# version stores fixed-size blocks to avoid large
-- reallocations; a Luau table already grows efficiently, so here the storage is one Luau array.
--
-- All index arguments are 0-based like C#. Element i is stored at data[i + 1].
-- `data` is never replaced during the lifetime of the DVector, so hot code may cache
-- `local d = dvector.data` and read/write d[i+1] directly (only for i < size).
--
-- Constructors: DVector.new(), DVector.Copy(other), DVector.FromArray(t) (t is a 1-based array)

local DVector = {}
DVector.__index = DVector
DVector.ClassName = "DVector"

export type DVector = typeof(setmetatable({} :: { data: { any }, size: number }, DVector))

function DVector.new(): DVector
	return setmetatable({ data = {}, size = 0 }, DVector) :: any
end

-- new DVector<T>(DVector<T> copy)
function DVector.Copy(copy: DVector): DVector
	local n = copy.size
	local data = table.move(copy.data, 1, n, 1, table.create(n))
	return setmetatable({ data = data, size = n }, DVector) :: any
end

-- new DVector<T>(T[] data) / new DVector<T>(IEnumerable<T>)
function DVector.FromArray(t: { any }): DVector
	local n = #t
	local data = table.move(t, 1, n, 1, table.create(n))
	return setmetatable({ data = data, size = n }, DVector) :: any
end

function DVector.Length(self: DVector): number
	return self.size
end

function DVector.empty(self: DVector): boolean
	return self.size == 0
end

function DVector.get(self: DVector, i: number): any
	return self.data[i + 1]
end

function DVector.set(self: DVector, i: number, value: any)
	self.data[i + 1] = value
end

function DVector.Add(self: DVector, value: any)
	local n = self.size + 1
	self.data[n] = value
	self.size = n
end

-- Add(value, nRepeat)
function DVector.AddRepeat(self: DVector, value: any, nRepeat: number)
	local data, n = self.data, self.size
	for i = 1, nRepeat do
		data[n + i] = value
	end
	self.size = n + nRepeat
end

-- Add(T[] values) where values is a 1-based array
function DVector.AddArray(self: DVector, values: { any })
	local data, n = self.data, self.size
	for i = 1, #values do
		data[n + i] = values[i]
	end
	self.size = n + #values
end

DVector.push_back = DVector.Add

function DVector.pop_back(self: DVector)
	if self.size > 0 then
		self.size -= 1
	end
end

-- insert(value, index): sets element at index, growing the vector if index >= size
function DVector.insert(self: DVector, value: any, index: number)
	local s = self.size
	if index == s then
		self.data[s + 1] = value
		self.size = s + 1
	elseif index > s then
		self:resize(index)
		self.data[index + 1] = value
		self.size = index + 1
	else
		self.data[index + 1] = value
	end
end
DVector.insertAt = DVector.insert

-- resize to count elements. New elements are default-initialized to 0 (C# default(T) for numbers).
-- Elements beyond the new size are discarded.
function DVector.resize(self: DVector, count: number)
	local s = self.size
	if s == count then
		return
	end
	local data = self.data
	if count > s then
		for i = s + 1, count do
			data[i] = 0
		end
	else
		for i = #data, count + 1, -1 do
			data[i] = nil
		end
	end
	self.size = count
end

-- copy(DVector copyIn): copy contents of other vector into this one
function DVector.copy(self: DVector, copyIn: DVector)
	local n = copyIn.size
	self:resize(0)
	table.move(copyIn.data, 1, n, 1, self.data)
	self.size = n
end

function DVector.back(self: DVector): any
	return self.data[self.size]
end

function DVector.setBack(self: DVector, value: any)
	self.data[self.size] = value
end

function DVector.front(self: DVector): any
	return self.data[1]
end

-- returns a new 1-based Luau array with the contents (C# GetBuffer/ToArray)
function DVector.ToArray(self: DVector): { any }
	return table.move(self.data, 1, self.size, 1, table.create(self.size))
end
DVector.GetBuffer = DVector.ToArray

-- C# Clear() sets all elements to default (0), keeping size
function DVector.Clear(self: DVector)
	local data = self.data
	for i = 1, self.size do
		data[i] = 0
	end
end

-- Apply(Action<T,int> applyF) - note that like C# the index passed is NOT the global index
-- (C# passes the in-block index); here we pass the 0-based global index.
function DVector.Apply(self: DVector, applyF: (any, number) -> ())
	local data = self.data
	for i = 1, self.size do
		applyF(data[i], i - 1)
	end
end

-- set vec[i] = applyF(vec[i], i) for each element of vector
function DVector.ApplyReplace(self: DVector, applyF: (any, number) -> any)
	local data = self.data
	for i = 1, self.size do
		data[i] = applyF(data[i], i - 1)
	end
end

-- iterate over values (C# IEnumerable<T>)
function DVector.Values(self: DVector): () -> any
	local i, data, n = 0, self.data, self.size
	return function()
		i += 1
		if i <= n then
			return data[i]
		end
		return nil
	end
end

return DVector
