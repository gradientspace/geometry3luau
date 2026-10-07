-- Port of geometry3Sharp mesh_selection/MeshBoundaryLoops.cs (MeshBoundaryLoops, MeshBoundaryLoopsException)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Extract boundary EdgeLoops from Mesh. Can also extract EdgeSpans for open areas,
-- however default behavior is to ignore these. Set .SpanBehavior to configure.
--
-- Luau port notes (see PORTING.md):
--   * MeshBoundaryLoops.new(mesh, bAutoCompute = true)
--   * Loops / Spans are 1-based Luau arrays of EdgeLoop / EdgeSpan.
--     Count(), SpanCount(), MaxVerticesLoopIndex() are methods; C# indexer this[i] is Get(i) (0-based).
--     IEnumerable<EdgeLoop>: `for _, loop in loops.Loops do`, `for loop in loops:Itr() do`, or
--     `for _, loop in loops do` (the class defines __iter).
--   * FindVertexIndex(vid) returns an Index2i array {loop_index, in_loop_index} (0-based), or
--     Index2i.Max ({MaxInt, MaxInt}) if not found.
--   * EdgeFilterF(eid) -> boolean, FailureBowties is a Luau array.
--   * MeshBoundaryLoopsException is thrown with error(exceptionObject). Catch it with pcall; the
--     error value is a MeshBoundaryLoopsException table with fields Message, UnclosedLoop,
--     BowtieFailure, RepeatedEdge (tostring() gives the message). MeshBoundaryLoops.IsException(e) tests it.

local Vector3d = require(script.Parent.Vector3d)
local MathUtil = require(script.Parent.MathUtil)
local BitArray = require(script.Parent.BitArray)
local Util = require(script.Parent.Util)
local EdgeLoop = require(script.Parent.EdgeLoop)
local EdgeSpan = require(script.Parent.EdgeSpan)

local MaxInt = MathUtil.MaxInt

-- exception type ------------------------------------------------------------------------------------
local MeshBoundaryLoopsException = {}
MeshBoundaryLoopsException.__index = MeshBoundaryLoopsException
MeshBoundaryLoopsException.ClassName = "MeshBoundaryLoopsException"
MeshBoundaryLoopsException.__tostring = function(e)
	return e.Message
end

function MeshBoundaryLoopsException.new(message: string)
	return setmetatable(
		{ Message = message, UnclosedLoop = false, BowtieFailure = false, RepeatedEdge = false },
		MeshBoundaryLoopsException
	)
end

local function throw(message: string, field: string?)
	local e = MeshBoundaryLoopsException.new(message)
	if field then
		e[field] = true
	end
	error(e, 0)
end

-- MeshBoundaryLoops ---------------------------------------------------------------------------------
local MeshBoundaryLoops = {}
MeshBoundaryLoops.__index = MeshBoundaryLoops
MeshBoundaryLoops.ClassName = "MeshBoundaryLoops"
MeshBoundaryLoops.MeshBoundaryLoopsException = MeshBoundaryLoopsException

function MeshBoundaryLoops.IsException(e: any): boolean
	return type(e) == "table" and getmetatable(e) == MeshBoundaryLoopsException
end

-- What should we do if we encounter open spans. Mainly a result of EdgeFilter, but can also
-- happen on meshes w/ crazy bowties
MeshBoundaryLoops.SpanBehaviors = table.freeze({
	Ignore = 0,
	ThrowException = 1,
	Compute = 2,
})
local SpanBehaviors = MeshBoundaryLoops.SpanBehaviors

-- What should we do if we encounter an unrecoverable failure while walking a loop
MeshBoundaryLoops.FailureBehaviors = table.freeze({
	ThrowException = 0, -- die, and you clean up
	ConvertToOpenSpan = 1, -- keep un-closed loop as a span
})
local FailureBehaviors = MeshBoundaryLoops.FailureBehaviors

MeshBoundaryLoops.__iter = function(self)
	return next, self.Loops
end

function MeshBoundaryLoops.new(mesh, bAutoCompute: boolean?)
	local self = setmetatable({}, MeshBoundaryLoops)
	self.Mesh = mesh
	self.Loops = {}
	self.Spans = {} -- spans are unclosed loops
	self.SawOpenSpans = false -- will be set to true if we find any open spans
	self.FellBackToSpansOnFailure = false -- set to true if we had to add spans to recover from failure
	self.SpanBehavior = SpanBehaviors.Compute
	self.FailureBehavior = FailureBehaviors.ConvertToOpenSpan
	-- if enabled, only edges where this returns true are considered
	self.EdgeFilterF = nil
	-- if we throw an exception, we will try to set FailureBowties, so that client
	-- can try repairing these vertices
	self.FailureBowties = nil
	if bAutoCompute ~= false then
		self:Compute()
	end
	return self
end

function MeshBoundaryLoops:Count(): number
	return #self.Loops
end
function MeshBoundaryLoops:SpanCount(): number
	return #self.Spans
end

-- C# indexer this[int index] (0-based)
function MeshBoundaryLoops:Get(index: number)
	return self.Loops[index + 1]
end

function MeshBoundaryLoops:Itr(): () -> any
	local i = 0
	local loops = self.Loops
	return function()
		i += 1
		return loops[i]
	end
end

-- Index of Loop with largest vertex count
function MeshBoundaryLoops:MaxVerticesLoopIndex(): number
	local j = 1
	local Loops = self.Loops
	for i = 2, #Loops do
		if #Loops[i].Vertices > #Loops[j].Vertices then
			j = i
		end
	end
	return j - 1
end

-- find pair (loop_index,in_loop_index) of vertex vID in EdgeLoops, or Index2i.Max if not found
function MeshBoundaryLoops:FindVertexIndex(vID: number): { number }
	local Loops = self.Loops
	for li = 1, #Loops do
		local idx = Loops[li]:FindVertexIndex(vID)
		if idx >= 0 then
			return { li - 1, idx }
		end
	end
	return { MaxInt, MaxInt }
end

-- find the loop index that contains a vertex, or return -1
function MeshBoundaryLoops:FindLoopContainingVertex(vid: number): number
	local Loops = self.Loops
	for li = 1, #Loops do
		if table.find(Loops[li].Vertices, vid) ~= nil then
			return li - 1
		end
	end
	return -1
end

-- find the loop index that contains an edge, or return -1
function MeshBoundaryLoops:FindLoopContainingEdge(eid: number): number
	local Loops = self.Loops
	for li = 1, #Loops do
		if table.find(Loops[li].Edges, eid) ~= nil then
			return li - 1
		end
	end
	return -1
end

local function reverse(a: { number })
	local n = #a
	for i = 1, n // 2 do
		a[i], a[n - i + 1] = a[n - i + 1], a[i]
	end
end

-- BufferUtil.CountValid(all_e, FilterF, num)
local function count_valid(data: { number }, filterF: (number) -> boolean, max_i: number): number
	local n = 0
	for i = 1, max_i do
		if filterF(data[i]) then
			n += 1
		end
	end
	return n
end

-- BufferUtil.FilterInPlace(all_e, FilterF, num)
local function filter_in_place(data: { number }, filterF: (number) -> boolean, max_i: number): number
	local iUpdate = 0
	for i = 1, max_i do
		if filterF(data[i]) then
			iUpdate += 1
			data[iUpdate] = data[i]
		end
	end
	return iUpdate
end

-- Find the set of boundary EdgeLoops. Note that if we encounter topological
-- issues, we will throw MeshBoundaryLoopsException w/ more info (if possible)
function MeshBoundaryLoops:Compute(): boolean
	-- This algorithm assumes that triangles are oriented consistently,
	-- so closed boundary-loop can be followed by walking edges in-order

	self.Loops = {}
	self.Spans = {}
	local Mesh = self.Mesh
	local EdgeFilterF = self.EdgeFilterF

	-- early-out if we don't actually have boundaries
	if Mesh:CachedIsClosed() then
		return true
	end

	local NE = Mesh:MaxEdgeID()

	-- Temporary memory used to indicate when we have "used" an edge.
	local used_edge = BitArray.new(NE)
	used_edge:SetAll(false)
	local used = used_edge.bits -- direct access: used[eid + 1]

	-- current loop is stored here, cleared after each loop extracted
	local loop_edges = {}
	local loop_verts = {}
	local bowties = {}

	-- Temp buffer for reading back all boundary edges of a vertex.
	local all_e = {}

	-- process all edges of mesh
	for eid = 0, NE - 1 do
		if not Mesh:IsEdge(eid) then
			continue
		end
		if used[eid + 1] == true then
			continue
		end
		if Mesh:IsBoundaryEdge(eid) == false then
			continue
		end

		if EdgeFilterF ~= nil and EdgeFilterF(eid) == false then
			used[eid + 1] = true
			continue
		end

		-- ok this is start of a boundary chain
		local eStart = eid
		used[eStart + 1] = true
		table.insert(loop_edges, eStart)

		local eCur = eid

		-- follow the chain in order of oriented edges
		local bClosed = false
		local bIsOpenSpan = false
		while not bClosed do
			local ev = Mesh:GetOrientedBoundaryEdgeV(eCur)
			local cure_a, cure_b = ev[1], ev[2]
			if bIsOpenSpan then
				cure_a, cure_b = ev[2], ev[1]
			else
				table.insert(loop_verts, cure_a)
			end

			local bdry_nbrs, e0, e1 = Mesh:VtxBoundaryEdges(cure_b, -1, 1)

			-- have to filter this list, if we are filtering. this is ugly.
			if EdgeFilterF ~= nil then
				if bdry_nbrs > 2 then
					-- we may repreat this below...irritating...
					local num_be = Mesh:VtxAllBoundaryEdges(cure_b, all_e)
					num_be = count_valid(all_e, EdgeFilterF, num_be)
				else
					if EdgeFilterF(e0) == false then
						bdry_nbrs -= 1
					end
					if EdgeFilterF(e1) == false then
						bdry_nbrs -= 1
					end
				end
			end

			if bdry_nbrs < 2 then -- hit an 'endpoint' vertex (should only happen when Filter is on...)
				if self.SpanBehavior == SpanBehaviors.ThrowException then
					throw("MeshBoundaryLoops.Compute: found open span at vertex " .. cure_b, "UnclosedLoop")
				end
				if bIsOpenSpan then
					bClosed = true
					continue
				else
					bIsOpenSpan = true -- begin open span
					eCur = loop_edges[1] -- restart at other end of loop
					reverse(loop_edges) -- do this so we can push to front
					continue
				end
			end

			local eNext = -1

			if bdry_nbrs > 2 then
				-- found "bowtie" vertex...things just got complicated!

				if cure_b == loop_verts[1] then
					-- The "end" of the current edge is the same as the start vertex.
					-- This means we can close the loop here. Might as well!
					eNext = -2 -- sentinel value used below
				else
					-- try to find an unused outgoing edge that is oriented properly.
					-- This could create sub-loops, we will handle those later
					local num_be = Mesh:VtxAllBoundaryEdges(cure_b, all_e)
					Util.gDevAssert(num_be == bdry_nbrs)

					if EdgeFilterF ~= nil then
						num_be = filter_in_place(all_e, EdgeFilterF, num_be)
					end

					-- Try to pick the best "turn left" vertex.
					eNext = self:find_left_turn_edge(eCur, cure_b, all_e, num_be, used)
					if eNext == -1 then
						if self.FailureBehavior == FailureBehaviors.ThrowException or self.SpanBehavior == SpanBehaviors.ThrowException then
							throw("MeshBoundaryLoops.Compute: cannot find valid outgoing edge at bowtie vertex " .. cure_b, "BowtieFailure")
						end

						-- ok, we are stuck. all we can do now is terminate this loop and keep it as a span
						if bIsOpenSpan then
							bClosed = true
						else
							bIsOpenSpan = true
							bClosed = true
						end
						continue
					end
				end

				if table.find(bowties, cure_b) == nil then
					table.insert(bowties, cure_b)
				end
			else
				-- walk forward to next available edge
				Util.gDevAssert(e0 == eCur or e1 == eCur)
				eNext = (e0 == eCur) and e1 or e0
			end

			if eNext == -2 then
				-- found a bowtie vert that is the same as start-of-loop, so we
				-- are just closing it off explicitly
				bClosed = true
			elseif eNext == eStart then
				-- found edge at start of loop, so loop is done.
				bClosed = true
			elseif used[eNext + 1] ~= false then
				-- disaster case - the next edge is already used, but it is not the start of our loop
				-- All we can do is convert to open span and terminate
				if self.FailureBehavior == FailureBehaviors.ThrowException or self.SpanBehavior == SpanBehaviors.ThrowException then
					throw("MeshBoundaryLoops.Compute: encountered repeated edge " .. eNext, "RepeatedEdge")
				end
				bIsOpenSpan = true
				bClosed = true
			else
				-- push onto accumulated list
				Util.gDevAssert(used[eNext + 1] == false)
				table.insert(loop_edges, eNext)
				used[eNext + 1] = true
				eCur = eNext
			end
		end

		if bIsOpenSpan then
			self.SawOpenSpans = true
			if self.SpanBehavior == SpanBehaviors.Compute then
				reverse(loop_edges) -- orient properly
				local span = EdgeSpan.FromEdges(Mesh, loop_edges)
				table.insert(self.Spans, span)
			end
		elseif #bowties > 0 then
			-- if we saw a bowtie vertex, we might need to break up this loop,
			-- so call extract_subloops
			local subloops = self:extract_subloops(loop_verts, loop_edges, bowties)
			for _, loop in subloops.Loops do
				table.insert(self.Loops, loop)
			end
			if #subloops.Spans > 0 then
				self.FellBackToSpansOnFailure = true
				for _, span in subloops.Spans do
					table.insert(self.Spans, span)
				end
			end
		else
			-- clean simple loop, convert to EdgeLoop instance
			local loop = EdgeLoop.new(Mesh)
			loop.Vertices = table.clone(loop_verts)
			loop.Edges = table.clone(loop_edges)
			table.insert(self.Loops, loop)
		end

		-- reset these lists
		table.clear(loop_edges)
		table.clear(loop_verts)
		table.clear(bowties)
	end

	return true
end

-- [TODO] cache this in a dictionary? we will not need very many, but we will
--   need each multiple times!
function MeshBoundaryLoops:get_vtx_normal(vid: number)
	local n = Vector3d.Zero
	for ti in self.Mesh:VtxTrianglesItr(vid) do
		n = n + self.Mesh:GetTriNormal(ti)
	end
	return n:Normalized()
end

-- ok, bdry_edges[0...bdry_edges_count] contains the boundary edges coming out of bowtie_v.
-- We want to pick the best one to continue the loop that came in to bowtie_v on incoming_e.
-- If the loops are all sane, then we will get the smallest loops by "turning left" at bowtie_v.
-- So, we compute the tangent plane at bowtie_v, and then the signed angle for each
-- viable edge in this plane.
-- (bdry_edges is a 1-based array; used_edges is the raw boolean array of the BitArray, used_edges[eid+1])
function MeshBoundaryLoops:find_left_turn_edge(
	incoming_e: number,
	bowtie_v: number,
	bdry_edges: { number },
	bdry_edges_count: number,
	used_edges: { boolean }
): number
	local Mesh = self.Mesh
	-- compute normal and edge [a,bowtie]
	local n = self:get_vtx_normal(bowtie_v)
	local other_v = Mesh:edge_other_v(incoming_e, bowtie_v)
	local ab = Mesh:GetVertex(bowtie_v) - Mesh:GetVertex(other_v)

	-- our winner
	local best_e = -1
	local best_angle = MathUtil.MaxDouble

	for i = 1, bdry_edges_count do
		local bdry_eid = bdry_edges[i]
		if used_edges[bdry_eid + 1] == true then
			continue -- this edge is already used
		end
		local bdry_ev = Mesh:GetOrientedBoundaryEdgeV(bdry_eid)
		if bdry_ev[1] ~= bowtie_v then
			continue -- have to be able to chain to end of current edge, orientation-wise
		end

		-- compute projected angle
		local bc = Mesh:GetVertex(bdry_ev[2]) - Mesh:GetVertex(bowtie_v)
		local fAngleS = MathUtil.PlaneAngleSignedD(ab, bc, n)

		-- turn left!
		if best_angle == MathUtil.MaxDouble or fAngleS < best_angle then
			best_angle = fAngleS
			best_e = bdry_eid
		end
	end

	-- [RMS] w/ bowtie vertices and open spans, this does happen
	return best_e
end

-- count number of valid vertices in l between loop[i0] and loop[i1-1]  (0-based i0, i1)
local function count_span(l: { number }, i0: number, i1: number): number
	local c = 0
	local N = #l
	local i = i0
	while i ~= i1 do
		if l[i + 1] ~= -1 then
			c += 1
		end
		i = (i + 1) % N
	end
	return c
end

-- find the index of item in loop, starting at start index (0-based)
local function find_index(loop: { number }, start: number, item: number): number
	for i = start, #loop - 1 do
		if loop[i + 1] == item then
			return i
		end
	end
	return -1
end

-- count number of times item appears in loop
local function count_in_list(loop: { number }, item: number): number
	local c = 0
	for i = 1, #loop do
		if loop[i] == item then
			c += 1
		end
	end
	return c
end

-- Read out the span from loop[i0] to loop [i1-1] into an array.
-- If bMarkInvalid, then these values are set to -1 in loop
local function extract_span(loop: { number }, i0: number, i1: number, bMarkInvalid: boolean): { number }
	local num = count_span(loop, i0, i1)
	local a = table.create(num, 0)
	local ai = 0
	local N = #loop
	local i = i0
	while i ~= i1 do
		if loop[i + 1] ~= -1 then
			ai += 1
			a[ai] = loop[i + 1]
			if bMarkInvalid then
				loop[i + 1] = -1
			end
		end
		i = (i + 1) % N
	end
	return a
end

-- check if forward path from loopV[i1] to loopV[i2] contains any bowtie verts other than bowtieV
local function is_simple_path(loopV: { number }, bowties: { number }, bowtieV: number, i1: number, i2: number): boolean
	local N = #loopV
	local i = i1
	while i ~= i2 do
		local vi = loopV[i + 1]
		if vi ~= -1 then -- skip removed vertices
			if vi ~= bowtieV and table.find(bowties, vi) ~= nil then
				return false
			end
		end
		i = (i + 1) % N
	end
	return true
end

-- Check if the loop from bowtieV to bowtieV inside loopV contains any other bowtie verts.
-- Also returns start and end indices in loopV of "clean" loop
-- Note that start may be < end, if the "clean" loop wraps around the end
-- C# is_simple_bowtie_loop(loopV, bowties, bowtieV, out start_i, out end_i) -> ok, start_i, end_i
local function is_simple_bowtie_loop(loopV: { number }, bowties: { number }, bowtieV: number): (boolean, number, number)
	-- find two indices of bowtie vert
	local start_i = find_index(loopV, 0, bowtieV)
	local end_i = find_index(loopV, start_i + 1, bowtieV)

	if is_simple_path(loopV, bowties, bowtieV, start_i, end_i) then
		return true, start_i, end_i
	elseif is_simple_path(loopV, bowties, bowtieV, end_i, start_i) then
		return true, end_i, start_i
	end
	return false, start_i, end_i -- not a simple bowtie loop!
end

-- This is called when loopV contains one or more "bowtie" vertices.
-- These vertices *might* be duplicated in loopV (but not necessarily)
-- If they are, we have to break loopV into subloops that don't contain duplicates.
--
-- The list bowties contains all the possible duplicates
-- (all v in bowties occur in loopV at least once)
--
-- Currently loopE is not used, and the returned EdgeLoop objects do not have their Edges
-- arrays initialized. Perhaps to improve in future.
--
-- An unhandled case to think about is where we have a sequence [..A..B..A..B..] where
-- A and B are bowties. In this case there are no A->A or B->B subloops. What should
-- we do here??
-- Returns { Loops = {EdgeLoop...}, Spans = {EdgeSpan...} }
function MeshBoundaryLoops:extract_subloops(loopV: { number }, loopE: { number }, bowties: { number })
	local Mesh = self.Mesh
	local subs = { Loops = {}, Spans = {} }

	-- figure out which bowties we saw are actually duplicated in loopV
	local dupes = {}
	for _, bv in bowties do
		if count_in_list(loopV, bv) > 1 then
			table.insert(dupes, bv)
		end
	end

	-- we might not actually have any duplicates, if we got luck. Early out in that case
	if #dupes == 0 then
		local loop = EdgeLoop.new(Mesh)
		loop.Vertices = table.clone(loopV)
		loop.Edges = table.clone(loopE)
		loop.BowtieVertices = table.clone(bowties)
		table.insert(subs.Loops, loop)
		return subs
	end

	-- This loop extracts subloops until we have dealt with all the
	-- duplicate vertices in loopV
	while #dupes > 0 do
		-- Find shortest "simple" loop, ie a loop from a bowtie to itself that
		-- does not contain any other bowties. This is an independent loop.
		-- We're doing a lot of extra work here if we only have one element in dupes...
		local bv = 0
		local start_i, end_i = -1, -1
		local bv_shortest = -1
		local shortest = MaxInt
		for bi = 1, #dupes do
			bv = dupes[bi]
			local ok
			ok, start_i, end_i = is_simple_bowtie_loop(loopV, dupes, bv)
			if ok then
				local len = count_span(loopV, start_i, end_i)
				if len < shortest then
					bv_shortest = bv
					shortest = len
				end
			end
		end

		-- failed to find a simple loop. Not sure what to do in this situation.
		-- If we don't want to throw, all we can do is convert the remaining
		-- loop to a span and return.
		-- (Or should we keep it as a loop and set flag??)
		if bv_shortest == -1 then
			if self.FailureBehavior == FailureBehaviors.ThrowException then
				self.FailureBowties = dupes
				throw("MeshBoundaryLoops.Compute: Cannot find a valid simple loop")
			end
			local span = EdgeSpan.new(Mesh)
			local verts = {}
			for i = 1, #loopV do
				if loopV[i] ~= -1 then
					table.insert(verts, loopV[i])
				end
			end
			span.Vertices = verts
			span.Edges = EdgeSpan.VerticesToEdges(Mesh, span.Vertices)
			span.BowtieVertices = table.clone(bowties)
			table.insert(subs.Spans, span)
			return subs
		end

		if bv ~= bv_shortest then
			bv = bv_shortest
			-- running again just to get start_i and end_i...
			local _
			_, start_i, end_i = is_simple_bowtie_loop(loopV, dupes, bv)
		end

		Util.gDevAssert(loopV[start_i + 1] == bv and loopV[end_i + 1] == bv)

		local loop = EdgeLoop.new(Mesh)
		loop.Vertices = extract_span(loopV, start_i, end_i, true)
		loop.Edges = EdgeLoop.VertexLoopToEdgeLoop(Mesh, loop.Vertices)
		loop.BowtieVertices = table.clone(bowties)
		table.insert(subs.Loops, loop)

		-- If there are no more duplicates of this bowtie, we can treat
		-- it like a regular vertex now
		if count_in_list(loopV, bv) < 2 then
			local idx = table.find(dupes, bv)
			if idx then
				table.remove(dupes, idx)
			end
		end
	end

	-- Should have one loop left that contains duplicates.
	-- Extract this as a separate loop
	local nLeft = 0
	for i = 1, #loopV do
		if loopV[i] ~= -1 then
			nLeft += 1
		end
	end
	if nLeft > 0 then
		local loop = EdgeLoop.new(Mesh)
		local verts = table.create(nLeft, 0)
		local vi = 0
		for i = 1, #loopV do
			if loopV[i] ~= -1 then
				vi += 1
				verts[vi] = loopV[i]
			end
		end
		loop.Vertices = verts
		loop.Edges = EdgeLoop.VertexLoopToEdgeLoop(Mesh, loop.Vertices)
		loop.BowtieVertices = table.clone(bowties)
		table.insert(subs.Loops, loop)
	end

	return subs
end

return MeshBoundaryLoops
