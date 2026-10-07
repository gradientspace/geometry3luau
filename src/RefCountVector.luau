--!native
-- Port of geometry3Sharp core/RefCountVector.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- RefCountedVector is used to keep track of which indices in a linear index list are in use/referenced.
-- A free list is tracked so that unreferenced indices can be re-used.
-- Indices are 0-based. The iterator (Indices()) iterates over valid indices (ie where refcount > 0).
-- C# property names: count -> count(), max_index -> max_index(), is_dense -> is_dense(), empty -> empty()

local DVector = require(script.Parent.DVector)

local RefCountVector = {}
RefCountVector.__index = RefCountVector
RefCountVector.ClassName = "RefCountVector"

RefCountVector.invalid = -1
local invalid = -1

export type RefCountVector = typeof(setmetatable({} :: { ref_counts: DVector.DVector, free_indices: DVector.DVector, used_count: number }, RefCountVector))

function RefCountVector.new(): RefCountVector
	return setmetatable({
		ref_counts = DVector.new(),
		free_indices = DVector.new(),
		used_count = 0,
	}, RefCountVector) :: any
end

-- new RefCountVector(RefCountVector copy)
function RefCountVector.Copy(copy: RefCountVector): RefCountVector
	return setmetatable({
		ref_counts = DVector.Copy(copy.ref_counts),
		free_indices = DVector.Copy(copy.free_indices),
		used_count = copy.used_count,
	}, RefCountVector) :: any
end

-- new RefCountVector(short[] raw_ref_counts, bool build_free_list = false)
function RefCountVector.FromRawRefCounts(raw_ref_counts: { number }, build_free_list: boolean?): RefCountVector
	local r = setmetatable({
		ref_counts = DVector.FromArray(raw_ref_counts),
		free_indices = DVector.new(),
		used_count = 0,
	}, RefCountVector) :: any
	if build_free_list then
		r:rebuild_free_list()
	end
	return r
end

function RefCountVector.RawRefCounts(self: RefCountVector): DVector.DVector
	return self.ref_counts
end

function RefCountVector.empty(self: RefCountVector): boolean
	return self.used_count == 0
end
function RefCountVector.count(self: RefCountVector): number
	return self.used_count
end
function RefCountVector.max_index(self: RefCountVector): number
	return self.ref_counts.size
end
function RefCountVector.is_dense(self: RefCountVector): boolean
	return self.free_indices.size == 0
end

function RefCountVector.isValid(self: RefCountVector, index: number): boolean
	local rc = self.ref_counts
	return index >= 0 and index < rc.size and rc.data[index + 1] > 0
end

function RefCountVector.isValidUnsafe(self: RefCountVector, index: number): boolean
	return self.ref_counts.data[index + 1] > 0
end

function RefCountVector.refCount(self: RefCountVector, index: number): number
	local n = self.ref_counts.data[index + 1]
	return (n == invalid) and 0 or n
end

function RefCountVector.rawRefCount(self: RefCountVector, index: number): number
	return self.ref_counts.data[index + 1]
end

function RefCountVector.allocate(self: RefCountVector): number
	self.used_count += 1
	local free_indices = self.free_indices
	local ref_counts = self.ref_counts
	if free_indices.size == 0 then
		-- [RMS] do we need this branch anymore?
		ref_counts:push_back(1)
		return ref_counts.size - 1
	else
		local iFree = invalid
		while iFree == invalid and free_indices.size > 0 do
			iFree = free_indices.data[free_indices.size]
			free_indices.size -= 1
		end
		if iFree ~= invalid then
			ref_counts.data[iFree + 1] = 1
			return iFree
		else
			ref_counts:push_back(1)
			return ref_counts.size - 1
		end
	end
end

function RefCountVector.increment(self: RefCountVector, index: number, increment: number?): number
	local data = self.ref_counts.data
	local n = data[index + 1] + (increment or 1)
	data[index + 1] = n
	return n
end

function RefCountVector.decrement(self: RefCountVector, index: number, decrement: number?)
	local data = self.ref_counts.data
	local n = data[index + 1] - (decrement or 1)
	data[index + 1] = n
	if n == 0 then
		self.free_indices:push_back(index)
		data[index + 1] = invalid
		self.used_count -= 1
	end
end

-- allocate at specific index, which must either be larger than current max index,
-- or on the free list. If larger, all elements up to this one will be pushed onto
-- free list. otherwise we have to do a linear search through free list.
-- If you are doing many of these, it is likely faster to use
-- allocate_at_unsafe(), and then rebuild_free_list() after you are done.
function RefCountVector.allocate_at(self: RefCountVector, index: number): boolean
	local ref_counts = self.ref_counts
	if index >= ref_counts.size then
		local j = ref_counts.size
		while j < index do
			ref_counts:push_back(invalid)
			self.free_indices:push_back(j)
			j += 1
		end
		ref_counts:push_back(1)
		self.used_count += 1
		return true
	else
		if ref_counts.data[index + 1] > 0 then
			return false
		end
		local free_indices = self.free_indices
		local N = free_indices.size
		local fdata = free_indices.data
		for i = 1, N do
			if fdata[i] == index then
				fdata[i] = invalid
				ref_counts.data[index + 1] = 1
				self.used_count += 1
				return true
			end
		end
		return false
	end
end

-- allocate at specific index, which must be free or larger than current max index.
-- However, we do not update free list. So, you probably need to do
-- rebuild_free_list() after calling this.
function RefCountVector.allocate_at_unsafe(self: RefCountVector, index: number): boolean
	local ref_counts = self.ref_counts
	if index >= ref_counts.size then
		local j = ref_counts.size
		while j < index do
			ref_counts:push_back(invalid)
			j += 1
		end
		ref_counts:push_back(1)
		self.used_count += 1
		return true
	else
		if ref_counts.data[index + 1] > 0 then
			return false
		end
		ref_counts.data[index + 1] = 1
		self.used_count += 1
		return true
	end
end

-- [RMS] really should not use this!!
function RefCountVector.set_Unsafe(self: RefCountVector, index: number, count: number)
	self.ref_counts.data[index + 1] = count
end

function RefCountVector.rebuild_free_list(self: RefCountVector)
	self.free_indices = DVector.new()
	self.used_count = 0
	local data = self.ref_counts.data
	local N = self.ref_counts.size
	for i = 0, N - 1 do
		if data[i + 1] > 0 then
			self.used_count += 1
		else
			self.free_indices:Add(i)
		end
	end
end

function RefCountVector.trim(self: RefCountVector, maxIndex: number)
	self.free_indices = DVector.new()
	self.ref_counts:resize(maxIndex)
	self.used_count = maxIndex
end

-- C# GetEnumerator: iterate over valid indices. `for i in rc:Indices() do`
function RefCountVector.Indices(self: RefCountVector): () -> number?
	local data = self.ref_counts.data
	local nLast = self.ref_counts.size
	local nIndex = 0
	return function()
		while nIndex < nLast do
			local i = nIndex
			nIndex += 1
			if data[i + 1] > 0 then
				return i
			end
		end
		return nil
	end
end

function RefCountVector.UsageStats(self: RefCountVector): string
	return string.format("RefCountSize %d  FreeSize %d", self.ref_counts.size, self.free_indices.size)
end

function RefCountVector.debug_print(self: RefCountVector): string
	local parts = { string.format("size %d used %d free_size %d\n", self.ref_counts.size, self.used_count, self.free_indices.size) }
	for i = 0, self.ref_counts.size - 1 do
		table.insert(parts, string.format("%d:%d ", i, self.ref_counts.data[i + 1]))
	end
	table.insert(parts, "\nfree:\n")
	for i = 0, self.free_indices.size - 1 do
		table.insert(parts, tostring(self.free_indices.data[i + 1]) .. " ")
	end
	return table.concat(parts)
end

return RefCountVector
