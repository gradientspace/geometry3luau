--!native
-- Port of geometry3Sharp core/DynamicPriorityQueue.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- This is a min-heap priority queue class that does not use any fixed-size internal data structures.
-- It is meant mainly for use on subsets of larger graphs.
-- If you need a PQ for a larger portion of a graph, consider IndexPriorityQueue instead.
--
-- Luau port notes:
--   * C# DynamicPriorityQueueNode subclasses become plain tables. Any table can be a node; the queue
--     stores its bookkeeping in the node fields `priority` and `index` (do not modify them yourself).
--     DynamicPriorityQueue.NewNode(fields?) is a convenience that returns a fresh node table.
--   * Properties -> methods: q:Count(), q:First(), q:FirstPriority().
--   * C# IEnumerable -> q:GetEnumerator() returns an iterator over the nodes in heap order
--     (also usable as `for node in q do` via __iter).
--   * Priorities are C# floats; here they are plain numbers (doubles).

local DynamicPriorityQueue = {}
DynamicPriorityQueue.__index = DynamicPriorityQueue
DynamicPriorityQueue.ClassName = "DynamicPriorityQueue"

function DynamicPriorityQueue.new()
	local self = setmetatable({}, DynamicPriorityQueue)
	-- set this to true during development to catch issues
	self.EnableDebugChecks = false
	-- tree of allocated nodes, stored linearly (nodes[1] is the head, like the C# code which does not use slot 0)
	self.nodes = {}
	-- count of active nodes
	self.num_nodes = 0
	return self
end

-- convenience: create a node table (C# `new MyNode() { ... }` where MyNode : DynamicPriorityQueueNode)
function DynamicPriorityQueue.NewNode(fields: { [string]: any }?)
	local node = fields or {}
	node.priority = node.priority or 0
	node.index = node.index or 0
	return node
end

-- number of nodes currently in queue
function DynamicPriorityQueue:Count(): number
	return self.num_nodes
end

-- reset the queue to empty state.
-- if bFreeMemory is false, we don't discard internal data structures
function DynamicPriorityQueue:Clear(bFreeMemory: boolean?)
	if bFreeMemory ~= false then
		self.nodes = {}
	else
		table.clear(self.nodes)
	end
	self.num_nodes = 0
end

-- node at head of queue
function DynamicPriorityQueue:First()
	return self.nodes[1]
end

-- Priority of node at head of queue
function DynamicPriorityQueue:FirstPriority(): number
	return self.nodes[1].priority
end

-- constant-time check to see if node is already in queue
function DynamicPriorityQueue:Contains(node): boolean
	local idx = node.index
	return idx ~= nil and self.nodes[idx] == node
end

-- Add node to list w/ given priority
-- Behavior is undefined if you call w/ same node twice
function DynamicPriorityQueue:Enqueue(node, priority: number)
	if self.EnableDebugChecks and self:Contains(node) == true then
		error("DynamicPriorityQueue.Enqueue: tried to add node that is already in queue!")
	end

	node.priority = priority
	self.num_nodes += 1
	self.nodes[self.num_nodes] = node
	node.index = self.num_nodes
	self:move_up(self.nodes[self.num_nodes])
end

-- remove node at head of queue, update queue, and return that node
function DynamicPriorityQueue:Dequeue()
	if self.EnableDebugChecks and self.num_nodes == 0 then
		error("DynamicPriorityQueue.Dequeue: queue is empty!")
	end

	local returnMe = self.nodes[1]
	self:Remove(returnMe)
	return returnMe
end

-- remove this node from queue. Undefined behavior if called w/ same node twice!
-- Behavior is undefined if you call w/ node that is not in queue
function DynamicPriorityQueue:Remove(node)
	if self.EnableDebugChecks and self:Contains(node) == false then
		error("DynamicPriorityQueue.Remove: tried to remove node that does not exist in queue!")
	end

	local nodes = self.nodes
	-- If the node is already the last node, we can remove it immediately
	if node.index == self.num_nodes then
		nodes[self.num_nodes] = nil
		self.num_nodes -= 1
		return
	end

	-- Swap the node with the last node
	local formerLastNode = nodes[self.num_nodes]
	self:swap_nodes(node, formerLastNode)
	nodes[self.num_nodes] = nil
	self.num_nodes -= 1

	-- Now bubble formerLastNode (which is no longer the last node) up or down as appropriate
	self:on_node_updated(formerLastNode)
end

-- update priority at node, and then move it to correct position in queue
-- Behavior is undefined if you call w/ node that is not in queue
function DynamicPriorityQueue:Update(node, priority: number)
	if self.EnableDebugChecks and self:Contains(node) == false then
		error("DynamicPriorityQueue.Update: tried to update node that does not exist in queue!")
	end

	node.priority = priority
	self:on_node_updated(node)
end

-- iterate over nodes (in heap order)
function DynamicPriorityQueue:GetEnumerator(): () -> any
	local i = 0
	local nodes = self.nodes
	return function()
		i += 1
		if i <= self.num_nodes then
			return nodes[i]
		end
		return nil
	end
end
DynamicPriorityQueue.__iter = function(self)
	return self:GetEnumerator()
end

--
-- Internals
--

-- swap node locations and indices
function DynamicPriorityQueue:swap_nodes(node1, node2)
	local nodes = self.nodes
	nodes[node1.index] = node2
	nodes[node2.index] = node1
	local temp = node1.index
	node1.index = node2.index
	node2.index = temp
end

-- move node up tree to correct position by iteratively swapping w/ parent
function DynamicPriorityQueue:move_up(node)
	local nodes = self.nodes
	local parent = node.index // 2
	while parent >= 1 do
		local parentNode = nodes[parent]
		if parentNode.priority < node.priority then -- has_higher_priority(parentNode, node)
			break
		end
		self:swap_nodes(node, parentNode)
		parent = node.index // 2
	end
end

-- move node down tree branches to correct position, by iteratively swapping w/ children
function DynamicPriorityQueue:move_down(node)
	local nodes = self.nodes
	local num_nodes = self.num_nodes
	-- we will put input node at this position after we are done swaps (ie actually moves, not swaps)
	local cur_node_index = node.index

	while true do
		local min_node = node
		local iLeftChild = 2 * cur_node_index

		-- past end of tree, must be in the right spot
		if iLeftChild > num_nodes then
			-- store input node in final position
			node.index = cur_node_index
			nodes[cur_node_index] = node
			break
		end

		-- check if priority is larger than either child - if so we want to swap
		local left_child_node = nodes[iLeftChild]
		if left_child_node.priority < min_node.priority then
			min_node = left_child_node
		end
		local iRightChild = iLeftChild + 1
		if iRightChild <= num_nodes then
			local right_child_node = nodes[iRightChild]
			if right_child_node.priority < min_node.priority then
				min_node = right_child_node
			end
		end

		-- if we found node with higher priority, swap with it (ie move it up) and take its place
		-- (but we only write start node to final position, not intermediary slots)
		if min_node ~= node then
			nodes[cur_node_index] = min_node

			local temp = min_node.index
			min_node.index = cur_node_index
			cur_node_index = temp
		else
			-- store input node in final position
			node.index = cur_node_index
			nodes[cur_node_index] = node
			break
		end
	end
end

-- call after node is modified, to move it to correct position in queue
function DynamicPriorityQueue:on_node_updated(node)
	local parentIndex = node.index // 2
	local parentNode = self.nodes[parentIndex]
	if parentIndex > 0 and self:has_higher_priority(node, parentNode) then
		self:move_up(node)
	else
		self:move_down(node)
	end
end

-- returns true if priority at higher is less than at lower
function DynamicPriorityQueue:has_higher_priority(higher, lower): boolean
	return higher.priority < lower.priority
end

-- Check if queue has been corrupted
function DynamicPriorityQueue:IsValidQueue(): boolean
	local nodes = self.nodes
	local num_nodes = self.num_nodes
	for i = 1, num_nodes - 1 do
		if nodes[i] ~= nil then
			local childLeftIndex = 2 * i
			if
				childLeftIndex < num_nodes
				and nodes[childLeftIndex] ~= nil
				and self:has_higher_priority(nodes[childLeftIndex], nodes[i])
			then
				return false
			end

			local childRightIndex = childLeftIndex + 1
			if
				childRightIndex < num_nodes
				and nodes[childRightIndex] ~= nil
				and self:has_higher_priority(nodes[childRightIndex], nodes[i])
			then
				return false
			end
		end
	end
	return true
end

function DynamicPriorityQueue:DebugPrint()
	for i = 1, self.num_nodes do
		local n = self.nodes[i]
		print(string.format("%d : p %s  idx %d", i, tostring(n.priority), n.index))
	end
end

return DynamicPriorityQueue
