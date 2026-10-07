--!native
-- Port of geometry3Sharp mesh_ops/MinimalHoleFill.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Construct a "minimal" fill surface for the hole. This surface
-- is often quasi-developable, reconstructs sharp edges, etc.
-- There are various options.
--
-- Luau port notes:
--   * MinimalHoleFill.new(mesh, fillLoop) where fillLoop is an EdgeLoop (Vertices, Edges arrays).
--   * Options: IgnoreBoundaryTriangles, OptimizeDevelopability, OptimizeTriangles, DevelopabilityTolerance.
--   * Outputs: FillVertices, FillTriangles (1-based arrays of base-mesh ids).
--   * Apply() -> bool.
--   * The C# HashSet<int> edge work-lists are iterated in insertion order (which is what the C# HashSet
--     does when no elements are removed), so the flip passes visit edges in the same order as C#.
--   * Util.gBreakToDebugger() (debugger hook when interior vertices remain) is a no-op.

local MathUtil = require(script.Parent.MathUtil)
local IndexUtil = require(script.Parent.IndexUtil)
local MeshResult = require(script.Parent.MeshResult)
local MeshIterators = require(script.Parent.MeshIterators)
local MeshUtil = require(script.Parent.MeshUtil)
local MeshQueries = require(script.Parent.MeshQueries)
local MeshConstraintUtil = require(script.Parent.MeshConstraintUtil)
local SimpleHoleFiller = require(script.Parent.SimpleHoleFiller)
local RegionOperator = require(script.Parent.RegionOperator)

local abs = math.abs
local InvalidID = -1
local Epsilonf = MathUtil.Epsilonf
local ZeroTolerancef = MathUtil.ZeroTolerancef
local TwoPI = MathUtil.TwoPI

-- insertion-ordered integer set (stand-in for C# HashSet<int> enumeration order)
local function newOrderedSet()
	return { list = {}, has = {} }
end
local function os_add(s, v: number)
	if s.has[v] == nil then
		s.has[v] = true
		table.insert(s.list, v)
	end
end
local function os_clear(s)
	table.clear(s.list)
	table.clear(s.has)
end
local function os_from_itr(itr: () -> number?)
	local s = newOrderedSet()
	for v in itr do
		os_add(s, v)
	end
	return s
end

local MinimalHoleFill = {}
MinimalHoleFill.__index = MinimalHoleFill
MinimalHoleFill.ClassName = "MinimalHoleFill"

function MinimalHoleFill.new(mesh, fillLoop)
	local self = setmetatable({}, MinimalHoleFill)
	self.Mesh = mesh
	self.FillLoop = fillLoop

	self.IgnoreBoundaryTriangles = false
	self.OptimizeDevelopability = true
	self.OptimizeTriangles = true
	self.DevelopabilityTolerance = 0.0001

	-- Outputs
	self.FillVertices = nil -- Final fill vertices (should be empty?)
	self.FillTriangles = nil -- Final fill triangles

	self.regionop = nil
	self.fillmesh = nil
	self.boundaryv = nil
	self.exterior_angle_sums = nil
	self.curvatures = nil
	return self
end

function MinimalHoleFill:Apply(): boolean
	local Mesh = self.Mesh
	local FillLoop = self.FillLoop

	-- do a simple fill
	local simplefill = SimpleHoleFiller.new(Mesh, FillLoop)
	local fill_gid = Mesh:AllocateTriangleGroup()
	local bOK = simplefill:Fill(fill_gid)
	if bOK == false then
		return false
	end

	if #FillLoop.Vertices <= 3 then
		self.FillTriangles = simplefill.NewTriangles
		self.FillVertices = {}
		return true
	end

	-- extract the simple fill mesh as a submesh, via RegionOperator, so we can backsub later
	local intial_fill_tris = {}
	for _, tid in simplefill.NewTriangles do
		intial_fill_tris[tid] = true
	end
	local regionop = RegionOperator.new(Mesh, simplefill.NewTriangles, function(submesh)
		submesh.ComputeTriMaps = true
	end)
	self.regionop = regionop
	local fillmesh = regionop.Region.SubMesh
	self.fillmesh = fillmesh

	-- for each boundary vertex, compute the exterior angle sum
	-- we will use this to compute gaussian curvature later
	local boundaryv = {}
	for vid in MeshIterators.BoundaryEdgeVertices(fillmesh) do
		boundaryv[vid] = true
	end
	self.boundaryv = boundaryv
	local exterior_angle_sums = {}
	self.exterior_angle_sums = exterior_angle_sums
	if self.IgnoreBoundaryTriangles == false then
		local BaseMesh = regionop.BaseMesh
		for sub_vid in boundaryv do
			local angle_sum = 0
			local base_vid = regionop.Region:MapVertexToBaseMesh(sub_vid)
			for tid in BaseMesh:VtxTrianglesItr(base_vid) do
				if intial_fill_tris[tid] == nil then
					local et = BaseMesh:GetTriangle(tid)
					local idx = IndexUtil.find_tri_index(base_vid, et)
					angle_sum += BaseMesh:GetTriInternalAngleR(tid, idx)
				end
			end
			exterior_angle_sums[sub_vid] = angle_sum
		end
	end

	-- try to guess a reasonable edge length that will give us enough geometry to work with in simplify pass
	local _loop_mine, _loop_maxe, loop_avge = MeshQueries.EdgeLengthStatsFromEdges(Mesh, FillLoop.Edges)
	local _fill_mine, fill_maxe, _fill_avge = MeshQueries.EdgeLengthStats(fillmesh)
	local remesh_target_len = loop_avge
	if fill_maxe / remesh_target_len > 10 then
		remesh_target_len = fill_maxe / 10
	end
	--double remesh_target_len = Math.Min(loop_avge, fill_avge / 4);

	-- remesh up to target edge length, ideally gives us some triangles to work with
	local RemesherPro = require(script.Parent.RemesherPro)
	local remesh1 = RemesherPro.new(fillmesh)
	remesh1.SmoothSpeedT = 1.0
	MeshConstraintUtil.FixAllBoundaryEdges(remesh1)
	--remesh1.SetTargetEdgeLength(remesh_target_len / 2);       // would this speed things up? on large regions?
	--remesh1.FastestRemesh();
	remesh1:SetTargetEdgeLength(remesh_target_len)
	remesh1:FastestRemesh()

	--[[
	 * first round: collapse to minimal mesh, while flipping to try to
	 * get to ballpark minimal mesh. We stop these passes as soon as
	 * we have done two rounds where we couldn't do another collapse
	 *
	 * This is the most unstable part of the algorithm because there
	 * are strong ordering effects. maybe we could sort the edges somehow??
	--]]

	local zero_collapse_passes = 0
	local collapse_passes = 0
	while true do
		local cont = collapse_passes < 20 and zero_collapse_passes < 2
		collapse_passes += 1
		if not cont then
			break
		end

		-- collapse pass
		local NE = fillmesh:MaxEdgeID()
		local collapses = 0
		for ei = 0, NE - 1 do
			if fillmesh:IsEdge(ei) == false or fillmesh:IsBoundaryEdge(ei) then
				continue
			end
			local ev = fillmesh:GetEdgeV(ei)
			local a_bdry, b_bdry = boundaryv[ev[1]] == true, boundaryv[ev[2]] == true
			if a_bdry and b_bdry then
				continue
			end
			local keepv = a_bdry and ev[1] or ev[2]
			local otherv = (keepv == ev[1]) and ev[2] or ev[1]
			local newv = fillmesh:GetVertex(keepv)
			if MeshUtil.CheckIfCollapseCreatesFlip(fillmesh, ei, newv) then
				continue
			end
			local result = fillmesh:CollapseEdge(keepv, otherv)
			if result == MeshResult.Ok then
				collapses += 1
			end
		end
		if collapses == 0 then
			zero_collapse_passes += 1
		else
			zero_collapse_passes = 0
		end

		-- flip pass. we flip in these cases:
		--  1) if angle between current triangles is too small (slightly more than 90 degrees, currently)
		--  2) if angle between flipped triangles is smaller than between current triangles
		--  3) if flipped edge length is shorter *and* such a flip won't flip the normal
		NE = fillmesh:MaxEdgeID()
		for ei = 0, NE - 1 do
			if fillmesh:IsEdge(ei) == false or fillmesh:IsBoundaryEdge(ei) then
				continue
			end
			local do_flip = false

			local ev = fillmesh:GetEdgeV(ei)
			local n1, n2, on1, on2 = MeshUtil.GetEdgeFlipNormals(fillmesh, ei)
			local dot_cur = n1:Dot(n2)
			local dot_flip = on1:Dot(on2)
			if n1:Dot(n2) < 0.1 or dot_flip > dot_cur + Epsilonf then
				do_flip = true
			end

			if do_flip == false then
				local otherv = fillmesh:GetEdgeOpposingV(ei)
				local len_e = fillmesh:GetVertex(ev[1]):Distance(fillmesh:GetVertex(ev[2]))
				local len_flip = fillmesh:GetVertex(otherv[1]):Distance(fillmesh:GetVertex(otherv[2]))
				if len_flip < len_e then
					if MeshUtil.CheckIfEdgeFlipCreatesFlip(fillmesh, ei) == false then
						do_flip = true
					end
				end
			end

			if do_flip then
				fillmesh:FlipEdge(ei)
			end
		end
	end

	-- Sometimes, for some reason, we have a remaining interior vertex (have only ever seen one?)
	-- Try to force removal of such vertices, even if it makes ugly mesh
	self:remove_remaining_interior_verts()

	-- enable/disable passes.
	local DO_FLATTER_PASS = true
	local DO_CURVATURE_PASS = self.OptimizeDevelopability and true
	local DO_AREA_PASS = self.OptimizeDevelopability and self.OptimizeTriangles and true

	--[[
	 * In this pass we repeat the flipping iterations from the previous pass.
	 *
	 * Note that because of the always-flip-if-dot-is-small case (commented),
	 * this pass will frequently not converge, as some number of edges will
	 * be able to flip back and forth (because neither has large enough dot).
	 * This is not ideal, but also, if we remove this behavior, then we
	 * generally get worse fills. This case basically introduces a sort of
	 * randomization factor that lets us escape local minima...
	--]]

	local remaining_edges = os_from_itr(fillmesh:EdgeIndices())
	local updated_edges = newOrderedSet()

	local flatter_passes = 0
	local zero_flips_passes = 0
	while true do
		local cont = flatter_passes < 40 and zero_flips_passes < 2 and #remaining_edges.list > 0 and DO_FLATTER_PASS
		flatter_passes += 1
		if not cont then
			break
		end
		zero_flips_passes += 1
		for _, ei in remaining_edges.list do
			if fillmesh:IsBoundaryEdge(ei) then
				continue
			end

			local do_flip = false

			local n1, n2, on1, on2 = MeshUtil.GetEdgeFlipNormals(fillmesh, ei)
			local dot_cur = n1:Dot(n2)
			local dot_flip = on1:Dot(on2)
			if flatter_passes < 20 and dot_cur < 0.1 then -- this check causes oscillatory behavior
				do_flip = true
			end
			if dot_flip > dot_cur + Epsilonf then
				do_flip = true
			end

			if do_flip then
				local result = fillmesh:FlipEdge(ei)
				if result == MeshResult.Ok then
					zero_flips_passes = 0
					self:add_all_edges(ei, updated_edges)
				end
			end
		end

		local tmp = remaining_edges
		remaining_edges = updated_edges
		updated_edges = tmp
		os_clear(updated_edges)
	end

	local curvature_passes = 0
	if DO_CURVATURE_PASS then
		self.curvatures = {}
		for vid in fillmesh:VertexIndices() do
			self:update_curvature(vid)
		end

		remaining_edges = os_from_itr(fillmesh:EdgeIndices())
		updated_edges = newOrderedSet()

		--[[
		 *  In this pass we try to minimize gaussian curvature at all the vertices.
		 *  This will recover sharp edges, etc, and do lots of good stuff.
		 *  However, this pass will not make much progress if we are not already
		 *  relatively close to a minimal mesh, so it really relies on the previous
		 *  passes getting us in the ballpark.
		--]]
		while true do
			local cont = curvature_passes < 40 and #remaining_edges.list > 0 and DO_CURVATURE_PASS
			curvature_passes += 1
			if not cont then
				break
			end
			for _, ei in remaining_edges.list do
				if fillmesh:IsBoundaryEdge(ei) then
					continue
				end

				local ev = fillmesh:GetEdgeV(ei)
				local ov = fillmesh:GetEdgeOpposingV(ei)

				local find_other = fillmesh:FindEdge(ov[1], ov[2])
				if find_other ~= InvalidID then
					continue
				end

				local total_curv_cur = self:curvature_metric_cached(ev[1], ev[2], ov[1], ov[2])
				if total_curv_cur < ZeroTolerancef then
					continue
				end

				local result = fillmesh:FlipEdge(ei)
				if result ~= MeshResult.Ok then
					continue
				end

				local total_curv_flip = self:curvature_metric_eval(ev[1], ev[2], ov[1], ov[2])

				local keep_flip = total_curv_flip < total_curv_cur - ZeroTolerancef
				if keep_flip == false then
					fillmesh:FlipEdge(ei)
				else
					self:update_curvature(ev[1])
					self:update_curvature(ev[2])
					self:update_curvature(ov[1])
					self:update_curvature(ov[2])
					self:add_all_edges(ei, updated_edges)
				end
			end
			local tmp = remaining_edges
			remaining_edges = updated_edges
			updated_edges = tmp
			os_clear(updated_edges)
		end
	end
	--System.Console.WriteLine("collapse {0}   flatter {1}   curvature {2}", collapse_passes, flatter_passes, curvature_passes);

	--[[
	 * In this final pass, we try to improve triangle quality. We flip if
	 * the flipped triangles have better total aspect ratio, and the
	 * curvature doesn't change **too** much. The .DevelopabilityTolerance
	 * parameter determines what is "too much" curvature change.
	--]]
	if DO_AREA_PASS then
		remaining_edges = os_from_itr(fillmesh:EdgeIndices())
		updated_edges = newOrderedSet()
		local area_passes = 0
		while #remaining_edges.list > 0 and area_passes < 20 do
			area_passes += 1
			for _, ei in remaining_edges.list do
				if fillmesh:IsBoundaryEdge(ei) then
					continue
				end

				local ev = fillmesh:GetEdgeV(ei)
				local ov = fillmesh:GetEdgeOpposingV(ei)

				local find_other = fillmesh:FindEdge(ov[1], ov[2])
				if find_other ~= InvalidID then
					continue
				end

				local total_curv_cur = self:curvature_metric_cached(ev[1], ev[2], ov[1], ov[2])

				local a = self:aspect_metric(ei)
				if a > 1 then
					continue
				end

				local result = fillmesh:FlipEdge(ei)
				if result ~= MeshResult.Ok then
					continue
				end

				local total_curv_flip = self:curvature_metric_eval(ev[1], ev[2], ov[1], ov[2])

				local keep_flip = abs(total_curv_cur - total_curv_flip) < self.DevelopabilityTolerance
				if keep_flip == false then
					fillmesh:FlipEdge(ei)
				else
					self:update_curvature(ev[1])
					self:update_curvature(ev[2])
					self:update_curvature(ov[1])
					self:update_curvature(ov[2])
					self:add_all_edges(ei, updated_edges)
				end
			end
			local tmp = remaining_edges
			remaining_edges = updated_edges
			updated_edges = tmp
			os_clear(updated_edges)
		end
	end

	regionop:BackPropropagate()
	self.FillTriangles = regionop:CurrentBaseTriangles()
	local fillv = {}
	for vid in regionop:CurrentBaseInteriorVertices() do
		table.insert(fillv, vid)
	end
	self.FillVertices = fillv

	return true
end

function MinimalHoleFill:remove_remaining_interior_verts()
	local fillmesh = self.fillmesh
	local interiorv = os_from_itr(MeshIterators.InteriorVertices(fillmesh))
	local count = #interiorv.list
	local prev_count = 0
	while count > 0 and count ~= prev_count do
		prev_count = count
		local curv = table.clone(interiorv.list)
		for _, vid in curv do
			for e in fillmesh:VtxEdgesItr(vid) do
				local ev = fillmesh:GetEdgeV(e)
				local otherv = (ev[1] == vid) and ev[2] or ev[1]
				local result = fillmesh:CollapseEdge(otherv, vid)
				if result == MeshResult.Ok then
					break
				end
			end
			if fillmesh:IsVertex(vid) == false and interiorv.has[vid] then
				interiorv.has[vid] = nil
				local k = table.find(interiorv.list, vid)
				if k ~= nil then
					table.remove(interiorv.list, k)
				end
				count -= 1
			end
		end
	end
	-- if (interiorv.Count > 0) Util.gBreakToDebugger();
end

function MinimalHoleFill:add_all_edges(ei: number, edge_set)
	local fillmesh = self.fillmesh
	local et = fillmesh:GetEdgeT(ei)
	local te = fillmesh:GetTriEdges(et[1])
	os_add(edge_set, te[1])
	os_add(edge_set, te[2])
	os_add(edge_set, te[3])
	te = fillmesh:GetTriEdges(et[2])
	os_add(edge_set, te[1])
	os_add(edge_set, te[2])
	os_add(edge_set, te[3])
end

local function get_tri_area(mesh, tri: { number }): number
	return MathUtil.Area(mesh:GetVertex(tri[1]), mesh:GetVertex(tri[2]), mesh:GetVertex(tri[3]))
end
local function get_tri_aspect(mesh, tri: { number }): number
	return MathUtil.AspectRatio(mesh:GetVertex(tri[1]), mesh:GetVertex(tri[2]), mesh:GetVertex(tri[3]))
end
local function get_tri_normal(mesh, tri: { number })
	return MathUtil.Normal(mesh:GetVertex(tri[1]), mesh:GetVertex(tri[2]), mesh:GetVertex(tri[3]))
end
MinimalHoleFill.get_tri_normal = get_tri_normal

function MinimalHoleFill:area_metric(eid: number): number
	local fillmesh = self.fillmesh
	local ta, tb, ota, otb = MeshUtil.GetEdgeFlipTris(fillmesh, eid)
	local area_a = get_tri_area(fillmesh, ta)
	local area_b = get_tri_area(fillmesh, tb)
	local area_c = get_tri_area(fillmesh, ota)
	local area_d = get_tri_area(fillmesh, otb)
	local avg_ab = (area_a + area_b) * 0.5
	local avg_cd = (area_c + area_d) * 0.5
	local metric_ab = abs(area_a - avg_ab) + abs(area_b - avg_ab)
	local metric_cd = abs(area_c - avg_cd) + abs(area_d - avg_cd)
	return metric_cd / metric_ab
end

function MinimalHoleFill:aspect_metric(eid: number): number
	local fillmesh = self.fillmesh
	local ta, tb, ota, otb = MeshUtil.GetEdgeFlipTris(fillmesh, eid)
	local aspect_a = get_tri_aspect(fillmesh, ta)
	local aspect_b = get_tri_aspect(fillmesh, tb)
	local aspect_c = get_tri_aspect(fillmesh, ota)
	local aspect_d = get_tri_aspect(fillmesh, otb)
	local metric_ab = abs(aspect_a - 1.0) + abs(aspect_b - 1.0)
	local metric_cd = abs(aspect_c - 1.0) + abs(aspect_d - 1.0)
	return metric_cd / metric_ab
end

function MinimalHoleFill:update_curvature(vid: number)
	self.curvatures[vid] = self:compute_gauss_curvature(vid)
end

function MinimalHoleFill:curvature_metric_cached(a: number, b: number, c: number, d: number): number
	local curvatures = self.curvatures
	local defect_a = curvatures[a]
	local defect_b = curvatures[b]
	local defect_c = curvatures[c]
	local defect_d = curvatures[d]
	return abs(defect_a) + abs(defect_b) + abs(defect_c) + abs(defect_d)
end

function MinimalHoleFill:curvature_metric_eval(a: number, b: number, c: number, d: number): number
	local defect_a = self:compute_gauss_curvature(a)
	local defect_b = self:compute_gauss_curvature(b)
	local defect_c = self:compute_gauss_curvature(c)
	local defect_d = self:compute_gauss_curvature(d)
	return abs(defect_a) + abs(defect_b) + abs(defect_c) + abs(defect_d)
end

function MinimalHoleFill:compute_gauss_curvature(vid: number): number
	local fillmesh = self.fillmesh
	local angle_sum = self.exterior_angle_sums[vid] or 0
	for tid in fillmesh:VtxTrianglesItr(vid) do
		local et = fillmesh:GetTriangle(tid)
		local idx = IndexUtil.find_tri_index(vid, et)
		angle_sum += fillmesh:GetTriInternalAngleR(tid, idx)
	end
	return angle_sum - TwoPI
end

return MinimalHoleFill
