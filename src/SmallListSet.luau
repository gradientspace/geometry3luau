--!native
-- Port of geometry3Sharp core/SmallListSet.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- SmallListSet stores a set of short integer-valued variable-size lists.
-- The lists are encoded into a few large flat buffers, with internal pooling,
-- so adding/removing lists usually does not involve any allocation.
--
-- The lists are stored in two parts. The first N elements are stored in a linear
-- subset of a buffer. If the list spills past these N elements, the extra elements
-- are stored in a linked list (which is also stored in a flat array).
--
-- Each list stores its count, so list-size operations are constant time.
--
-- All list indices and internal "pointers" are 0-based like C#; a buffer element p is stored at
-- buf[p + 1]. ValueItr(list_index) returns an iterator: `for v in sls:ValueItr(i) do`.

local Null = -1
local BLOCKSIZE = 8
local BLOCK_LIST_OFFSET = BLOCKSIZE + 1

local SmallListSet = {}
SmallListSet.__index = SmallListSet
SmallListSet.ClassName = "SmallListSet"
SmallListSet.BLOCKSIZE = BLOCKSIZE
SmallListSet.BLOCK_LIST_OFFSET = BLOCK_LIST_OFFSET

export type SmallListSet = typeof(setmetatable(
	{} :: {
		list_heads: { number },
		list_heads_size: number,
		block_store: { number },
		block_store_size: number,
		free_blocks: { number },
		allocated_count: number,
		linked_store: { number },
		linked_store_size: number,
		free_head_ptr: number,
	},
	SmallListSet
))

function SmallListSet.new(): SmallListSet
	return setmetatable({
		list_heads = {}, -- each "list" is stored as index of first element in block-store (like a pointer)
		list_heads_size = 0,
		block_store = {}, -- flat buffer used to store per-list initial block
		-- blocks are BLOCKSIZE+2 long, elements are [CurrentCount, item0...itemN, LinkedListPtr]
		block_store_size = 0,
		free_blocks = {}, -- list of free blocks, indices into block_store (1-based stack of 0-based ptrs)
		allocated_count = 0,
		linked_store = {}, -- flat buffer used for linked-list elements, each element is [value, next_ptr]
		linked_store_size = 0,
		free_head_ptr = Null, -- index of first free element in linked_store
	}, SmallListSet) :: any
end

-- new SmallListSet(SmallListSet copy)
function SmallListSet.Copy(copy: SmallListSet): SmallListSet
	return setmetatable({
		list_heads = table.clone(copy.list_heads),
		list_heads_size = copy.list_heads_size,
		block_store = table.clone(copy.block_store),
		block_store_size = copy.block_store_size,
		free_blocks = table.clone(copy.free_blocks),
		allocated_count = copy.allocated_count,
		linked_store = table.clone(copy.linked_store),
		linked_store_size = copy.linked_store_size,
		free_head_ptr = copy.free_head_ptr,
	}, SmallListSet) :: any
end

-- returns largest current list_index
function SmallListSet.Size(self: SmallListSet): number
	return self.list_heads_size
end

-- resize the list-of-lists
function SmallListSet.Resize(self: SmallListSet, new_size: number)
	local cur_size = self.list_heads_size
	if new_size > cur_size then
		local heads = self.list_heads
		for k = cur_size + 1, new_size do
			heads[k] = Null
		end
		self.list_heads_size = new_size
	end
end

-- create a new list at list_index
function SmallListSet.AllocateAt(self: SmallListSet, list_index: number)
	local heads = self.list_heads
	if list_index >= self.list_heads_size then
		-- need to set intermediate values to null!
		for k = self.list_heads_size + 1, list_index + 1 do
			heads[k] = Null
		end
		self.list_heads_size = list_index + 1
	else
		if heads[list_index + 1] ~= Null then
			error("SmallListSet: list at " .. list_index .. " is not empty!")
		end
	end
end

-- grab a block from the free list, or allocate a new one
local function allocate_block(self: SmallListSet): number
	local free_blocks = self.free_blocks
	local nfree = #free_blocks
	if nfree > 0 then
		local ptr = free_blocks[nfree]
		free_blocks[nfree] = nil
		return ptr
	end
	local nsize = self.block_store_size
	local bs = self.block_store
	-- block_store.insert(Null, nsize + BLOCK_LIST_OFFSET): grows store by BLOCKSIZE+2 elements
	for k = nsize + 1, nsize + BLOCK_LIST_OFFSET do
		bs[k] = 0
	end
	bs[nsize + BLOCK_LIST_OFFSET + 1] = Null
	self.block_store_size = nsize + BLOCK_LIST_OFFSET + 1
	bs[nsize + 1] = 0
	self.allocated_count += 1
	return nsize
end

-- push a link-node onto the free list
local function add_free_link(self: SmallListSet, ptr: number)
	self.linked_store[ptr + 2] = self.free_head_ptr
	self.free_head_ptr = ptr
end

-- insert val into list at list_index.
function SmallListSet.Insert(self: SmallListSet, list_index: number, val: number)
	local block_ptr = self.list_heads[list_index + 1]
	local bs = self.block_store
	if block_ptr == Null then
		block_ptr = allocate_block(self)
		bs[block_ptr + 1] = 0
		self.list_heads[list_index + 1] = block_ptr
	end

	local N = bs[block_ptr + 1]
	if N < BLOCKSIZE then
		bs[block_ptr + N + 2] = val
	else
		-- spill to linked list
		local cur_head = bs[block_ptr + BLOCK_LIST_OFFSET + 1]
		local ls = self.linked_store
		if self.free_head_ptr == Null then
			-- allocate new linkedlist node
			local new_ptr = self.linked_store_size
			ls[new_ptr + 1] = val
			ls[new_ptr + 2] = cur_head
			self.linked_store_size = new_ptr + 2
			bs[block_ptr + BLOCK_LIST_OFFSET + 1] = new_ptr
		else
			-- pull from free list
			local free_ptr = self.free_head_ptr
			self.free_head_ptr = ls[free_ptr + 2]
			ls[free_ptr + 1] = val
			ls[free_ptr + 2] = cur_head
			bs[block_ptr + BLOCK_LIST_OFFSET + 1] = free_ptr
		end
	end

	-- count new element
	bs[block_ptr + 1] += 1
end

-- remove val from the linked-list attached to block_ptr
local function remove_from_linked_list(self: SmallListSet, block_ptr: number, val: number): boolean
	local bs, ls = self.block_store, self.linked_store
	local cur_ptr = bs[block_ptr + BLOCK_LIST_OFFSET + 1]
	local prev_ptr = Null
	while cur_ptr ~= Null do
		if ls[cur_ptr + 1] == val then
			local next_ptr = ls[cur_ptr + 2]
			if prev_ptr == Null then
				bs[block_ptr + BLOCK_LIST_OFFSET + 1] = next_ptr
			else
				ls[prev_ptr + 2] = next_ptr
			end
			add_free_link(self, cur_ptr)
			return true
		end
		prev_ptr = cur_ptr
		cur_ptr = ls[cur_ptr + 2]
	end
	return false
end

-- remove val from the list at list_index. return false if val was not in list.
function SmallListSet.Remove(self: SmallListSet, list_index: number, val: number): boolean
	local block_ptr = self.list_heads[list_index + 1]
	local bs = self.block_store
	local N = bs[block_ptr + 1]

	local iEnd = block_ptr + math.min(N, BLOCKSIZE)
	for i = block_ptr + 1, iEnd do
		if bs[i + 1] == val then
			for j = i + 1, iEnd do -- shift left
				bs[j] = bs[j + 1]
			end
			--block_store[iEnd] = -2;     // OPTIONAL

			if N > BLOCKSIZE then
				local ls = self.linked_store
				local cur_ptr = bs[block_ptr + BLOCK_LIST_OFFSET + 1]
				bs[block_ptr + BLOCK_LIST_OFFSET + 1] = ls[cur_ptr + 2] -- point to cur->next
				bs[iEnd + 1] = ls[cur_ptr + 1]
				add_free_link(self, cur_ptr)
			end

			bs[block_ptr + 1] -= 1
			return true
		end
	end

	-- search list
	if N > BLOCKSIZE then
		if remove_from_linked_list(self, block_ptr, val) then
			bs[block_ptr + 1] -= 1
			return true
		end
	end

	return false
end

-- move list at from_index to to_index
function SmallListSet.Move(self: SmallListSet, from_index: number, to_index: number)
	local heads = self.list_heads
	if heads[to_index + 1] ~= Null then
		error("SmallListSet.MoveTo: list at " .. to_index .. " is not empty!")
	end
	if heads[from_index + 1] == Null then
		error("SmallListSet.MoveTo: list at " .. from_index .. " is empty!")
	end
	heads[to_index + 1] = heads[from_index + 1]
	heads[from_index + 1] = Null
end

-- remove all elements from list at list_index
function SmallListSet.Clear(self: SmallListSet, list_index: number)
	local heads = self.list_heads
	local block_ptr = heads[list_index + 1]
	if block_ptr ~= Null then
		local bs = self.block_store
		local N = bs[block_ptr + 1]

		-- if we have spilled to linked-list, free nodes
		if N > BLOCKSIZE then
			local ls = self.linked_store
			local cur_ptr = bs[block_ptr + BLOCK_LIST_OFFSET + 1]
			while cur_ptr ~= Null do
				local free_ptr = cur_ptr
				cur_ptr = ls[cur_ptr + 2]
				add_free_link(self, free_ptr)
			end
			bs[block_ptr + BLOCK_LIST_OFFSET + 1] = Null
		end

		-- free our block
		bs[block_ptr + 1] = 0
		table.insert(self.free_blocks, block_ptr)
		heads[list_index + 1] = Null
	end
end

-- return size of list at list_index
function SmallListSet.Count(self: SmallListSet, list_index: number): number
	local block_ptr = self.list_heads[list_index + 1]
	return (block_ptr == Null) and 0 or self.block_store[block_ptr + 1]
end

-- search for val in list at list_index
function SmallListSet.Contains(self: SmallListSet, list_index: number, val: number): boolean
	local block_ptr = self.list_heads[list_index + 1]
	if block_ptr ~= Null then
		local bs = self.block_store
		local N = bs[block_ptr + 1]
		if N < BLOCKSIZE then
			local iEnd = block_ptr + N
			for i = block_ptr + 1, iEnd do
				if bs[i + 1] == val then
					return true
				end
			end
		else
			-- we spilled to linked list, have to iterate through it as well
			local iEnd = block_ptr + BLOCKSIZE
			for i = block_ptr + 1, iEnd do
				if bs[i + 1] == val then
					return true
				end
			end
			local ls = self.linked_store
			local cur_ptr = bs[block_ptr + BLOCK_LIST_OFFSET + 1]
			while cur_ptr ~= Null do
				if ls[cur_ptr + 1] == val then
					return true
				end
				cur_ptr = ls[cur_ptr + 2]
			end
		end
	end
	return false
end

-- return the first item in the list at list_index (no zero-size-list checking)
function SmallListSet.First(self: SmallListSet, list_index: number): number
	local block_ptr = self.list_heads[list_index + 1]
	return self.block_store[block_ptr + 2]
end

-- iterate over the values of list at list_index
function SmallListSet.ValueItr(self: SmallListSet, list_index: number): () -> number?
	local block_ptr = self.list_heads[list_index + 1]
	if block_ptr == nil or block_ptr == Null then
		return function()
			return nil
		end
	end
	local bs, ls = self.block_store, self.linked_store
	local N = bs[block_ptr + 1]
	local i = block_ptr + 1 -- 0-based position of next block item
	local iEnd = block_ptr + math.min(N, BLOCKSIZE)
	local cur_ptr = (N > BLOCKSIZE) and bs[block_ptr + BLOCK_LIST_OFFSET + 1] or Null
	return function()
		if i <= iEnd then
			local v = bs[i + 1]
			i += 1
			return v
		end
		if cur_ptr ~= Null then
			local v = ls[cur_ptr + 1]
			cur_ptr = ls[cur_ptr + 2]
			return v
		end
		return nil
	end
end

-- Copy the values of list at list_index into a new 1-based Luau array (useful when the
-- list is modified during iteration).
function SmallListSet.ValuesArray(self: SmallListSet, list_index: number): { number }
	local result = {}
	local block_ptr = self.list_heads[list_index + 1]
	if block_ptr ~= nil and block_ptr ~= Null then
		local bs, ls = self.block_store, self.linked_store
		local N = bs[block_ptr + 1]
		local iEnd = block_ptr + math.min(N, BLOCKSIZE)
		for i = block_ptr + 1, iEnd do
			table.insert(result, bs[i + 1])
		end
		if N > BLOCKSIZE then
			local cur_ptr = bs[block_ptr + BLOCK_LIST_OFFSET + 1]
			while cur_ptr ~= Null do
				table.insert(result, ls[cur_ptr + 1])
				cur_ptr = ls[cur_ptr + 2]
			end
		end
	end
	return result
end

-- search for findF(list_value) == true, of list at list_index, and return list_value
function SmallListSet.Find(self: SmallListSet, list_index: number, findF: (number) -> boolean, invalidValue: number?): number
	for val in self:ValueItr(list_index) do
		if findF(val) then
			return val
		end
	end
	return invalidValue or -1
end

-- search for findF(list_value) == true, of list at list_index, and replace with new_value.
-- returns false if not found
function SmallListSet.Replace(self: SmallListSet, list_index: number, findF: (number) -> boolean, new_value: number): boolean
	local block_ptr = self.list_heads[list_index + 1]
	if block_ptr ~= Null then
		local bs = self.block_store
		local N = bs[block_ptr + 1]
		local iEnd = block_ptr + math.min(N, BLOCKSIZE)
		for i = block_ptr + 1, iEnd do
			if findF(bs[i + 1]) then
				bs[i + 1] = new_value
				return true
			end
		end
		if N > BLOCKSIZE then
			local ls = self.linked_store
			local cur_ptr = bs[block_ptr + BLOCK_LIST_OFFSET + 1]
			while cur_ptr ~= Null do
				if findF(ls[cur_ptr + 1]) then
					ls[cur_ptr + 1] = new_value
					return true
				end
				cur_ptr = ls[cur_ptr + 2]
			end
		end
	end
	return false
end

-- Replace the first occurrence of old_value with new_value (allocation-free variant of Replace)
function SmallListSet.ReplaceValue(self: SmallListSet, list_index: number, old_value: number, new_value: number): boolean
	local block_ptr = self.list_heads[list_index + 1]
	if block_ptr ~= Null then
		local bs = self.block_store
		local N = bs[block_ptr + 1]
		local iEnd = block_ptr + math.min(N, BLOCKSIZE)
		for i = block_ptr + 1, iEnd do
			if bs[i + 1] == old_value then
				bs[i + 1] = new_value
				return true
			end
		end
		if N > BLOCKSIZE then
			local ls = self.linked_store
			local cur_ptr = bs[block_ptr + BLOCK_LIST_OFFSET + 1]
			while cur_ptr ~= Null do
				if ls[cur_ptr + 1] == old_value then
					ls[cur_ptr + 1] = new_value
					return true
				end
				cur_ptr = ls[cur_ptr + 2]
			end
		end
	end
	return false
end

function SmallListSet.MemoryUsage(self: SmallListSet): string
	return string.format(
		"ListSize %d  Blocks Count %d Free %d Mem %d  Linked Mem %d",
		self.list_heads_size,
		self.allocated_count,
		#self.free_blocks,
		self.block_store_size,
		self.linked_store_size
	)
end

return SmallListSet
