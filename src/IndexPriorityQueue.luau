--!native
-- Port of geometry3Sharp core/IndexPriorityQueue.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- This is a min-heap priority queue class that does not use an object for each queue node.
-- Integer IDs must be provided by the user to identify unique nodes.
-- Internally an array is used to keep track of the mapping from ids to internal indices,
-- so the max ID must also be provided.
--
-- conceptually based on https://github.com/BlueRaja/High-Speed-Priority-Queue-for-C-Sharp
--
-- Luau port notes (see PORTING.md):
--   * ids are 0-based like C#, in range [0, maxID).
--   * Count(), First(), FirstPriority() are methods. Iterate ids with `for id in q:Ids() do` (C# GetEnumerator).
--   * The C# QueueNode structs are stored in flat arrays: node_ids[i], node_priority[i] for heap
--     node i (1-based, like the C# tree, which leaves slot 0 unused). The C# QueueNode.index field is
--     always equal to the node's position for live nodes, so it is not stored.
--   * Priorities are doubles (C# float). Ordering of near-equal priorities may therefore differ in
--     the last bits from the C# version.
--   * Contains(id) additionally checks that the node at the stored index really holds id; the C#
--     version can return true for a removed id whose old slot was reused by another node.

local IndexPriorityQueue = {}
IndexPriorityQueue.__index = IndexPriorityQueue
IndexPriorityQueue.ClassName = "IndexPriorityQueue"

function IndexPriorityQueue.new(maxID: number)
	local self = setmetatable({}, IndexPriorityQueue)
	-- set this to true during development to catch issues
	self.EnableDebugChecks = false
	self.node_ids = {} -- tree of allocated nodes, stored linearly. active up to num_nodes
	self.node_priority = {}
	self.num_nodes = 0 -- count of active nodes
	-- mapping from external ids to internal node indices: id_to_index[id + 1]
	self.id_to_index = table.create(maxID, -1)
	self.maxID = maxID
	return self
end

function IndexPriorityQueue:Count(): number
	return self.num_nodes
end

-- reset the queue to empty state.
-- if bFreeMemory is false, we don't discard internal data structures, so there will be less allocation next time
function IndexPriorityQueue:Clear(bFreeMemory: boolean?)
	if bFreeMemory ~= false then
		self.node_ids = {}
		self.node_priority = {}
	end
	local idx = self.id_to_index
	for i = 1, self.maxID do
		idx[i] = 0
	end
	self.num_nodes = 0
end

-- id of node at head of queue
function IndexPriorityQueue:First(): number
	return self.node_ids[1]
end

-- Priority of node at head of queue
function IndexPriorityQueue:FirstPriority(): number
	return self.node_priority[1]
end

-- constant-time check to see if id is already in queue
function IndexPriorityQueue:Contains(id: number): boolean
	local iNode = self.id_to_index[id + 1]
	if iNode == nil or iNode <= 0 or iNode > self.num_nodes then
		return false
	end
	return self.node_ids[iNode] == id
end

-- move iNode up tree to correct position by iteratively swapping w/ parent
local function move_up(self, iNode: number)
	local ids, pri, idx = self.node_ids, self.node_priority, self.id_to_index
	-- save start node, we will move this one to correct position in tree
	local iStart = iNode
	local startId, startPriority = ids[iStart], pri[iStart]

	-- while our priority is lower than parent, we swap upwards, ie move parent down
	local iParent = iNode // 2
	while iParent >= 1 do
		if pri[iParent] < startPriority then
			break
		end
		-- move(iParent, iNode)
		local pid = ids[iParent]
		ids[iNode] = pid
		pri[iNode] = pri[iParent]
		idx[pid + 1] = iNode
		iNode = iParent
		iParent = iNode // 2
	end

	-- write input node into final position, if we moved it
	if iNode ~= iStart then
		ids[iNode] = startId
		pri[iNode] = startPriority
		idx[startId + 1] = iNode
	end
end

-- move iNode down tree branches to correct position, by iteratively swapping w/ children
local function move_down(self, iNode: number)
	local ids, pri, idx = self.node_ids, self.node_priority, self.id_to_index
	local num_nodes = self.num_nodes
	-- save start node, we will move this one to correct position in tree
	local iStart = iNode
	local startId, startPriority = ids[iStart], pri[iStart]

	-- keep moving down until lower nodes have higher priority
	while true do
		local iMoveTo = iNode
		local iLeftChild = 2 * iNode

		-- past end of tree, must be in the right spot
		if iLeftChild > num_nodes then
			break
		end

		-- check if priority is larger than either child - if so we want to swap
		local min_priority = startPriority
		local left_child_priority = pri[iLeftChild]
		if left_child_priority < min_priority then
			iMoveTo = iLeftChild
			min_priority = left_child_priority
		end
		local iRightChild = iLeftChild + 1
		if iRightChild <= num_nodes then
			if pri[iRightChild] < min_priority then
				iMoveTo = iRightChild
			end
		end

		-- if we found node with higher priority, swap with it (ie move it up) and take its place
		-- (but we only write start node to final position, not intermediary slots)
		if iMoveTo ~= iNode then
			-- move(iMoveTo, iNode)
			local mid = ids[iMoveTo]
			ids[iNode] = mid
			pri[iNode] = pri[iMoveTo]
			idx[mid + 1] = iNode
			iNode = iMoveTo
		else
			break
		end
	end

	-- if we moved node, write it to its new position
	if iNode ~= iStart then
		ids[iNode] = startId
		pri[iNode] = startPriority
		idx[startId + 1] = iNode
	end
end

-- call after node is modified, to move it to correct position in queue
local function on_node_updated(self, iNode: number)
	local iParent = iNode // 2
	local pri = self.node_priority
	if iParent > 0 and pri[iNode] < pri[iParent] then
		move_up(self, iNode)
	else
		move_down(self, iNode)
	end
end

local function remove_at_index(self, iNode: number)
	local ids, pri, idx = self.node_ids, self.node_priority, self.id_to_index
	local num_nodes = self.num_nodes
	-- node-is-last-node case
	if iNode == num_nodes then
		ids[iNode] = nil
		pri[iNode] = nil
		self.num_nodes = num_nodes - 1
		return
	end

	-- [RMS] is there a better way to do this? seems random to move the last node to
	-- top of tree? But I guess otherwise we might have to shift entire branches??

	-- Swap the node with the last node
	local id1, id2 = ids[iNode], ids[num_nodes]
	local p1, p2 = pri[iNode], pri[num_nodes]
	ids[iNode], pri[iNode] = id2, p2
	ids[num_nodes], pri[num_nodes] = id1, p1
	idx[id2 + 1] = iNode
	idx[id1 + 1] = num_nodes
	-- after swap, inode is the one we want to keep, and numNodes is the one we discard
	ids[num_nodes] = nil
	pri[num_nodes] = nil
	self.num_nodes = num_nodes - 1

	-- Now shift iNode (ie the former last node) up or down as appropriate
	on_node_updated(self, iNode)
end

-- Add id to list w/ given priority
-- Behavior is undefined if you call w/ same id twice
function IndexPriorityQueue:Insert(id: number, priority: number)
	if self.EnableDebugChecks and self:Contains(id) == true then
		error("IndexPriorityQueue.Insert: tried to add node that is already in queue!")
	end
	local n = self.num_nodes + 1
	self.num_nodes = n
	self.node_ids[n] = id
	self.node_priority[n] = priority
	self.id_to_index[id + 1] = n
	move_up(self, n)
end
IndexPriorityQueue.Enqueue = IndexPriorityQueue.Insert

-- remove node at head of queue, update queue, and return id for that node
function IndexPriorityQueue:Dequeue(): number
	if self.EnableDebugChecks and self.num_nodes == 0 then
		error("IndexPriorityQueue.Dequeue: queue is empty!")
	end
	local id = self.node_ids[1]
	remove_at_index(self, 1)
	return id
end

-- remove this node from queue. Undefined behavior if called w/ same id twice!
-- Behavior is undefined if you call w/ id that is not in queue
function IndexPriorityQueue:Remove(id: number)
	if self.EnableDebugChecks and self:Contains(id) == false then
		error("IndexPriorityQueue.Remove: tried to remove node that does not exist in queue!")
	end
	local iNode = self.id_to_index[id + 1]
	remove_at_index(self, iNode)
end

-- update priority at node id, and then move it to correct position in queue
-- Behavior is undefined if you call w/ id that is not in queue
function IndexPriorityQueue:Update(id: number, priority: number)
	if self.EnableDebugChecks and self:Contains(id) == false then
		error("IndexPriorityQueue.Update: tried to update node that does not exist in queue!")
	end
	local iNode = self.id_to_index[id + 1]
	self.node_priority[iNode] = priority
	on_node_updated(self, iNode)
end

-- Query the priority at node id, assuming it exists in queue
function IndexPriorityQueue:GetPriority(id: number): number
	if self.EnableDebugChecks and self:Contains(id) == false then
		error("IndexPriorityQueue.Update: tried to get priorty of node that does not exist in queue!")
	end
	local iNode = self.id_to_index[id + 1]
	return self.node_priority[iNode]
end

-- C# GetEnumerator: ids in heap order
function IndexPriorityQueue:Ids(): () -> number?
	local i = 0
	local ids = self.node_ids
	local n = self.num_nodes
	return function()
		i += 1
		if i <= n then
			return ids[i]
		end
		return nil
	end
end

-- Check if queue has been corrupted
function IndexPriorityQueue:IsValidQueue(): boolean
	local pri = self.node_priority
	local num_nodes = self.num_nodes
	for i = 1, num_nodes - 1 do
		local childLeftIndex = 2 * i
		if childLeftIndex < num_nodes and pri[childLeftIndex] < pri[i] then
			return false
		end
		local childRightIndex = childLeftIndex + 1
		if childRightIndex < num_nodes and pri[childRightIndex] < pri[i] then
			return false
		end
	end
	return true
end

function IndexPriorityQueue:DebugPrint()
	for i = 1, self.num_nodes do
		print(string.format("%d : p %s  index %d  id %d", i, tostring(self.node_priority[i]), i, self.node_ids[i]))
	end
end

return IndexPriorityQueue
