--!native
-- Port of geometry3Sharp spatial/DMeshAABBTree.cs, part 2: intersection tests (triangle / mesh / tree),
-- tree-vs-tree queries, generic traversal and debugging functions of DMeshAABBTree3.
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Returns a function that installs these methods into the DMeshAABBTree3 class table
-- (see DMeshAABBTree3.luau header for the Luau API summary).

local MathUtil = require(script.Parent.MathUtil)
local Triangle3d = require(script.Parent.Triangle3d)
local Index = require(script.Parent.Index)

local MaxDouble = MathUtil.MaxDouble

local function IntrTriangle3Triangle3()
	return require(script.Parent.IntrTriangle3Triangle3) :: any
end
local function DistTriangle3Triangle3()
	return require(script.Parent.DistTriangle3Triangle3) :: any
end
local function BoundsUtil()
	return require(script.Parent.BoundsUtil) :: any
end
local function MeshQueries()
	return require(script.Parent.MeshQueries) :: any
end

return function(DMeshAABBTree3)
	local check_valid = DMeshAABBTree3.check_valid

	-------------------------------------------------------------------------------------------------
	-- box helpers

	-- box_box_intersect(iBox, ref testBox): AxisAlignedBox3d.Intersects with the eps-expanded box iBox
	function DMeshAABBTree3:box_box_intersect(iBox: number, testBox): boolean
		local bb = self.box_bounds
		local o = 6 * iBox
		local tmin, tmax = testBox.Min, testBox.Max
		return not (
			(tmax.x <= bb[o + 1])
			or (tmin.x >= bb[o + 4])
			or (tmax.y <= bb[o + 2])
			or (tmin.y >= bb[o + 5])
			or (tmax.z <= bb[o + 3])
			or (tmin.z >= bb[o + 6])
		)
	end

	function DMeshAABBTree3:box_box_distsqr(iBox: number, testBox): number
		return self:get_box(iBox):DistanceSquared(testBox)
	end

	-------------------------------------------------------------------------------------------------
	-- TestIntersection overloads

	-- TestIntersection(IMesh testMesh, TransformF?, bBoundsCheck?) / TestIntersection(Triangle3d) /
	-- TestIntersection(DMeshAABBTree3 otherTree, TransformF?)
	function DMeshAABBTree3:TestIntersection(x: any, TransformF: ((any) -> any)?, bBoundsCheck: boolean?): boolean
		if getmetatable(x) == DMeshAABBTree3 then
			return self:TestIntersectionTree(x, TransformF)
		elseif x.V0 ~= nil and x.V1 ~= nil and x.V2 ~= nil then
			return self:TestIntersectionTriangle(x)
		end
		return self:TestIntersectionMesh(x, TransformF, bBoundsCheck)
	end

	-- return true if *any* triangle of testMesh intersects with our tree.
	-- Use TransformF to transform vertices of testMesh into space of this tree.
	-- if boundsCheck is false, we skip bbox/bbox early-out
	function DMeshAABBTree3:TestIntersectionMesh(testMesh, TransformF: ((any) -> any)?, bBoundsCheck: boolean?): boolean
		check_valid(self, "TestIntersection")

		if bBoundsCheck ~= false then
			local meshBox = require(script.Parent.MeshMeasurements).Bounds(testMesh, TransformF)
			if self:box_box_intersect(self.root_index, meshBox) == false then
				return false
			end
		end

		local xform = TransformF or function(v)
			return v
		end

		for tid in testMesh:TriangleIndices() do
			local tri = testMesh:GetTriangle(tid)
			local test_tri =
				Triangle3d.new(xform(testMesh:GetVertex(tri[1])), xform(testMesh:GetVertex(tri[2])), xform(testMesh:GetVertex(tri[3])))
			if self:TestIntersectionTriangle(test_tri) then
				return true
			end
		end
		return false
	end

	-- Returns true if triangle intersects any triangle of our mesh
	function DMeshAABBTree3:TestIntersectionTriangle(triangle): boolean
		check_valid(self, "TestIntersection")
		local triBounds = BoundsUtil().Bounds(triangle)
		local interTri = self:find_any_intersection(self.root_index, triangle, triBounds)
		return interTri >= 0
	end

	function DMeshAABBTree3:find_any_intersection(iBox: number, triangle, triBounds): number
		local ilist = self.index_list
		local idx = self.box_to_index[iBox + 1]
		if idx < self.triangles_end then -- triange-list case, array is [N t1 t2 ... tN]
			local Intersects = IntrTriangle3Triangle3().Intersects
			local filter = self.TriangleFilterF
			local mesh = self.mesh
			local num_tris = ilist[idx + 1]
			for i = 1, num_tris do
				local ti = ilist[idx + i + 1]
				if filter ~= nil and not filter(ti) then
					continue
				end
				local box_tri = mesh:GetTriVerticesTriangle(ti)
				if Intersects(triangle, box_tri) then
					return ti
				end
			end
		else -- internal node, either 1 or 2 child boxes
			local iChild1 = ilist[idx + 1]
			if iChild1 < 0 then -- 1 child, descend if nearer than cur min-dist
				iChild1 = -iChild1 - 1
				if self:box_box_intersect(iChild1, triBounds) then
					return self:find_any_intersection(iChild1, triangle, triBounds)
				end
			else -- 2 children, descend closest first
				iChild1 = iChild1 - 1
				local iChild2 = ilist[idx + 2] - 1

				local interTri = -1
				if self:box_box_intersect(iChild1, triBounds) then
					interTri = self:find_any_intersection(iChild1, triangle, triBounds)
				end
				if interTri == -1 and self:box_box_intersect(iChild2, triBounds) then
					interTri = self:find_any_intersection(iChild2, triangle, triBounds)
				end
				return interTri
			end
		end
		return -1
	end

	-- Returns true if there is *any* intersection between our mesh and 'other' mesh.
	-- TransformF takes vertices of otherTree into our tree - can be null if in same coord space
	function DMeshAABBTree3:TestIntersectionTree(otherTree, TransformF: ((any) -> any)?): boolean
		check_valid(self, "TestIntersection")
		if self:find_any_intersection_tree(self.root_index, otherTree, TransformF, otherTree.root_index, 0) then
			return true
		end
		return false
	end

	local function get_tri_xform(mesh, tid: number, TransformF)
		local tri = mesh:GetTriVerticesTriangle(tid)
		if TransformF ~= nil then
			tri.V0 = TransformF(tri.V0)
			tri.V1 = TransformF(tri.V1)
			tri.V2 = TransformF(tri.V2)
		end
		return tri
	end

	-- C# find_any_intersection(int iBox, DMeshAABBTree3 otherTree, TransformF, int oBox, int depth)
	function DMeshAABBTree3:find_any_intersection_tree(iBox: number, otherTree, TransformF, oBox: number, depth: number): boolean
		local idx = self.box_to_index[iBox + 1]
		local odx = otherTree.box_to_index[oBox + 1]
		local ilist, olist = self.index_list, otherTree.index_list

		if idx < self.triangles_end and odx < otherTree.triangles_end then
			-- ok we are at triangles for both trees, do triangle-level testing
			local Intersects = IntrTriangle3Triangle3().Intersects
			local num_tris, onum_tris = ilist[idx + 1], olist[odx + 1]
			local filter, ofilter = self.TriangleFilterF, otherTree.TriangleFilterF

			-- outer iteration is "other" tris that need to be transformed (more expensive)
			for j = 1, onum_tris do
				local tj = olist[odx + j + 1]
				if ofilter ~= nil and not ofilter(tj) then
					continue
				end
				local otri = get_tri_xform(otherTree.mesh, tj, TransformF)

				-- inner iteration over "our" triangles
				for i = 1, num_tris do
					local ti = ilist[idx + i + 1]
					if filter ~= nil and not filter(ti) then
						continue
					end
					local tri = self.mesh:GetTriVerticesTriangle(ti)
					if Intersects(otri, tri) then
						return true
					end
				end
			end
			return false
		end

		-- we either descend "our" tree or the other tree
		--   - if we have hit triangles on "our" tree, we have to descend other
		--   - if we hit triangles on "other", we have to descend ours
		--   - otherwise, we alternate at each depth. This produces wider
		--     branching but is significantly faster (~10x) for both hits and misses
		local bDescendOther = (idx < self.triangles_end or depth % 2 == 0)
		if bDescendOther and odx < otherTree.triangles_end then
			bDescendOther = false -- can't
		end

		if bDescendOther then
			-- ok we hit triangles on our side but we need to still reach triangles on
			-- the other side, so we descend "their" children
			local bounds = self:get_boxd(iBox)

			local oChild1 = olist[odx + 1]
			if oChild1 < 0 then -- 1 child, descend if nearer than cur min-dist
				oChild1 = -oChild1 - 1
				local oChild1Box = otherTree:get_boxd(oChild1, TransformF)
				if oChild1Box:Intersects(bounds) then
					-- [note] C# passes oBox here (which recurses forever on single-child nodes); fixed to oChild1
					return self:find_any_intersection_tree(iBox, otherTree, TransformF, oChild1, depth + 1)
				end
			else -- 2 children
				oChild1 = oChild1 - 1 -- [TODO] could descend one w/ larger overlap volume first??
				local oChild2 = olist[odx + 2] - 1

				local intersects = false
				local oChild1Box = otherTree:get_boxd(oChild1, TransformF)
				if oChild1Box:Intersects(bounds) then
					intersects = self:find_any_intersection_tree(iBox, otherTree, TransformF, oChild1, depth + 1)
				end

				if intersects == false then
					local oChild2Box = otherTree:get_boxd(oChild2, TransformF)
					if oChild2Box:Intersects(bounds) then
						intersects = self:find_any_intersection_tree(iBox, otherTree, TransformF, oChild2, depth + 1)
					end
				end
				return intersects
			end
		else
			-- descend our tree nodes if they intersect w/ current bounds of other tree
			local oBounds = otherTree:get_boxd(oBox, TransformF)

			local iChild1 = ilist[idx + 1]
			if iChild1 < 0 then -- 1 child, descend if nearer than cur min-dist
				iChild1 = -iChild1 - 1
				if self:box_box_intersect(iChild1, oBounds) then
					return self:find_any_intersection_tree(iChild1, otherTree, TransformF, oBox, depth + 1)
				end
			else -- 2 children
				iChild1 = iChild1 - 1 -- [TODO] could descend one w/ larger overlap volume first??
				local iChild2 = ilist[idx + 2] - 1

				local intersects = false
				if self:box_box_intersect(iChild1, oBounds) then
					intersects = self:find_any_intersection_tree(iChild1, otherTree, TransformF, oBox, depth + 1)
				end
				if intersects == false and self:box_box_intersect(iChild2, oBounds) then
					intersects = self:find_any_intersection_tree(iChild2, otherTree, TransformF, oBox, depth + 1)
				end
				return intersects
			end
		end
		return false
	end

	-------------------------------------------------------------------------------------------------
	-- FindAllIntersections

	-- Compute all intersections between two Meshes.
	-- TransformF argument transforms vertices of otherTree to our tree (can be null if in same coord space)
	-- Returns pairs of intersecting triangles, which could intersect in either point or segment
	-- Currently *does not* return coplanar intersections.
	-- returns IntersectionsQueryResult { Points = { {t0, t1, point} }, Segments = { {t0, t1, point0, point1} } }
	function DMeshAABBTree3:FindAllIntersections(otherTree, TransformF: ((any) -> any)?)
		check_valid(self, "FindIntersections")

		local result = { Points = {}, Segments = {} }
		local intr = IntrTriangle3Triangle3().new(Triangle3d.new(), Triangle3d.new())
		self:find_intersections(self.root_index, otherTree, TransformF, otherTree.root_index, 0, intr, result)
		return result
	end

	function DMeshAABBTree3:find_intersections(iBox: number, otherTree, TransformF, oBox: number, depth: number, intr, result)
		local idx = self.box_to_index[iBox + 1]
		local odx = otherTree.box_to_index[oBox + 1]
		local ilist, olist = self.index_list, otherTree.index_list

		if idx < self.triangles_end and odx < otherTree.triangles_end then
			-- ok we are at triangles for both trees, do triangle-level testing
			local num_tris, onum_tris = ilist[idx + 1], olist[odx + 1]
			local filter, ofilter = self.TriangleFilterF, otherTree.TriangleFilterF

			-- outer iteration is "other" tris that need to be transformed (more expensive)
			for j = 1, onum_tris do
				local tj = olist[odx + j + 1]
				if ofilter ~= nil and not ofilter(tj) then
					continue
				end
				local otri = get_tri_xform(otherTree.mesh, tj, TransformF)
				intr:SetTriangle0(otri)

				-- inner iteration over "our" triangles
				for i = 1, num_tris do
					local ti = ilist[idx + i + 1]
					if filter ~= nil and not filter(ti) then
						continue
					end
					local tri = self.mesh:GetTriVerticesTriangle(ti)
					intr:SetTriangle1(tri)

					-- [RMS] Test() is much faster than Find() so it makes sense to call it first, as most
					-- triangles will not intersect (right?)
					if intr:Test() then
						if intr:Find() then
							if intr.Quantity == 1 then
								table.insert(result.Points, { t0 = ti, t1 = tj, point = intr.Points[1] })
							elseif intr.Quantity == 2 then
								table.insert(result.Segments, { t0 = ti, t1 = tj, point0 = intr.Points[1], point1 = intr.Points[2] })
							else
								error("DMeshAABBTree.find_intersections: found quantity " .. tostring(intr.Quantity))
							end
						end
					end
				end
			end

			-- done these nodes
			return
		end

		-- we either descend "our" tree or the other tree
		--   - if we have hit triangles on "our" tree, we have to descend other
		--   - if we hit triangles on "other", we have to descend ours
		--   - otherwise, we alternate at each depth. This produces wider
		--     branching but is significantly faster (~10x) for both hits and misses
		local bDescendOther = (idx < self.triangles_end or depth % 2 == 0)
		if bDescendOther and odx < otherTree.triangles_end then
			bDescendOther = false -- can't
		end

		if bDescendOther then
			-- ok we hit triangles on our side but we need to still reach triangles on
			-- the other side, so we descend "their" children
			local bounds = self:get_boxd(iBox)

			local oChild1 = olist[odx + 1]
			if oChild1 < 0 then -- 1 child, descend if nearer than cur min-dist
				oChild1 = -oChild1 - 1
				local oChild1Box = otherTree:get_boxd(oChild1, TransformF)
				if oChild1Box:Intersects(bounds) then
					self:find_intersections(iBox, otherTree, TransformF, oChild1, depth + 1, intr, result)
				end
			else -- 2 children
				oChild1 = oChild1 - 1

				local oChild1Box = otherTree:get_boxd(oChild1, TransformF)
				if oChild1Box:Intersects(bounds) then
					self:find_intersections(iBox, otherTree, TransformF, oChild1, depth + 1, intr, result)
				end

				local oChild2 = olist[odx + 2] - 1
				local oChild2Box = otherTree:get_boxd(oChild2, TransformF)
				if oChild2Box:Intersects(bounds) then
					self:find_intersections(iBox, otherTree, TransformF, oChild2, depth + 1, intr, result)
				end
			end
		else
			-- descend our tree nodes if they intersect w/ current bounds of other tree
			local oBounds = otherTree:get_boxd(oBox, TransformF)

			local iChild1 = ilist[idx + 1]
			if iChild1 < 0 then -- 1 child, descend if nearer than cur min-dist
				iChild1 = -iChild1 - 1
				if self:box_box_intersect(iChild1, oBounds) then
					self:find_intersections(iChild1, otherTree, TransformF, oBox, depth + 1, intr, result)
				end
			else -- 2 children
				iChild1 = iChild1 - 1
				if self:box_box_intersect(iChild1, oBounds) then
					self:find_intersections(iChild1, otherTree, TransformF, oBox, depth + 1, intr, result)
				end

				local iChild2 = ilist[idx + 2] - 1
				if self:box_box_intersect(iChild2, oBounds) then
					self:find_intersections(iChild2, otherTree, TransformF, oBox, depth + 1, intr, result)
				end
			end
		end
	end

	-------------------------------------------------------------------------------------------------
	-- FindNearestTriangles

	-- Find nearest pair of triangles on this tree with otherTree, within max_dist.
	-- TransformF transforms vertices of otherTree into our coordinates. can be null.
	-- returns triangle-id pair (my_tri,other_tri), or Index2i.Max if not found within max_dist
	-- Use MeshQueries.TrianglesDistance() to get more information
	-- C# FindNearestTriangles(otherTree, TransformF, out distance, max_dist) -> returns Index2i, distance
	function DMeshAABBTree3:FindNearestTriangles(otherTree, TransformF: ((any) -> any)?, max_dist: number?): ({ number }, number)
		check_valid(self, "TestIntersection")

		local maxd = max_dist or MaxDouble
		local state = { nearest_sqr = MaxDouble, nearest_pair = Index.Index2i.Max() }
		if maxd < MaxDouble then
			state.nearest_sqr = maxd * maxd
		end

		self:find_nearest_triangles(self.root_index, otherTree, TransformF, otherTree.root_index, 0, state)
		local distance = (state.nearest_sqr < MaxDouble) and math.sqrt(state.nearest_sqr) or MaxDouble
		return state.nearest_pair, distance
	end

	-- C# find_nearest_triangles(..., ref double nearest_sqr, ref Index2i nearest_pair): refs live in `state`
	function DMeshAABBTree3:find_nearest_triangles(iBox: number, otherTree, TransformF, oBox: number, depth: number, state)
		local idx = self.box_to_index[iBox + 1]
		local odx = otherTree.box_to_index[oBox + 1]
		local ilist, olist = self.index_list, otherTree.index_list

		if idx < self.triangles_end and odx < otherTree.triangles_end then
			-- ok we are at triangles for both trees, do triangle-level testing
			local num_tris, onum_tris = ilist[idx + 1], olist[odx + 1]
			local filter, ofilter = self.TriangleFilterF, otherTree.TriangleFilterF

			local dist = DistTriangle3Triangle3().new(Triangle3d.new(), Triangle3d.new())

			-- outer iteration is "other" tris that need to be transformed (more expensive)
			for j = 1, onum_tris do
				local tj = olist[odx + j + 1]
				if ofilter ~= nil and not ofilter(tj) then
					continue
				end
				local otri = get_tri_xform(otherTree.mesh, tj, TransformF)
				dist:SetTriangle0(otri)

				-- inner iteration over "our" triangles
				for i = 1, num_tris do
					local ti = ilist[idx + i + 1]
					if filter ~= nil and not filter(ti) then
						continue
					end
					local tri = self.mesh:GetTriVerticesTriangle(ti)
					dist:SetTriangle1(tri)
					local dist_sqr = dist:GetSquared()
					if dist_sqr < state.nearest_sqr then
						state.nearest_sqr = dist_sqr
						state.nearest_pair = { ti, tj }
					end
				end
			end
			return
		end

		-- we either descend "our" tree or the other tree
		--   - if we have hit triangles on "our" tree, we have to descend other
		--   - if we hit triangles on "other", we have to descend ours
		--   - otherwise, we alternate at each depth. This produces wider
		--     branching but is significantly faster (~10x) for both hits and misses
		local bDescendOther = (idx < self.triangles_end or depth % 2 == 0)
		if bDescendOther and odx < otherTree.triangles_end then
			bDescendOther = false -- can't
		end

		if bDescendOther then
			-- ok we reached triangles on our side but we need to still reach triangles on
			-- the other side, so we descend "their" children
			local bounds = self:get_boxd(iBox)

			local oChild1 = olist[odx + 1]
			if oChild1 < 0 then -- 1 child, descend if nearer than cur min-dist
				oChild1 = -oChild1 - 1
				local oChild1Box = otherTree:get_boxd(oChild1, TransformF)
				if oChild1Box:DistanceSquared(bounds) < state.nearest_sqr then
					self:find_nearest_triangles(iBox, otherTree, TransformF, oChild1, depth + 1, state)
				end
			else -- 2 children
				oChild1 = oChild1 - 1
				local oChild2 = olist[odx + 2] - 1

				local oChild1Box = otherTree:get_boxd(oChild1, TransformF)
				local oChild2Box = otherTree:get_boxd(oChild2, TransformF)

				-- descend closer box first
				local d1Sqr = oChild1Box:DistanceSquared(bounds)
				local d2Sqr = oChild2Box:DistanceSquared(bounds)
				if d2Sqr < d1Sqr then
					if d2Sqr < state.nearest_sqr then
						self:find_nearest_triangles(iBox, otherTree, TransformF, oChild2, depth + 1, state)
					end
					if d1Sqr < state.nearest_sqr then
						self:find_nearest_triangles(iBox, otherTree, TransformF, oChild1, depth + 1, state)
					end
				else
					if d1Sqr < state.nearest_sqr then
						self:find_nearest_triangles(iBox, otherTree, TransformF, oChild1, depth + 1, state)
					end
					if d2Sqr < state.nearest_sqr then
						self:find_nearest_triangles(iBox, otherTree, TransformF, oChild2, depth + 1, state)
					end
				end
			end
		else
			-- descend our tree nodes if they intersect w/ current bounds of other tree
			local oBounds = otherTree:get_boxd(oBox, TransformF)

			local iChild1 = ilist[idx + 1]
			if iChild1 < 0 then -- 1 child, descend if nearer than cur min-dist
				iChild1 = -iChild1 - 1
				if self:box_box_distsqr(iChild1, oBounds) < state.nearest_sqr then
					self:find_nearest_triangles(iChild1, otherTree, TransformF, oBox, depth + 1, state)
				end
			else -- 2 children
				iChild1 = iChild1 - 1
				local iChild2 = ilist[idx + 2] - 1

				-- descend closer box first
				local d1Sqr = self:box_box_distsqr(iChild1, oBounds)
				local d2Sqr = self:box_box_distsqr(iChild2, oBounds)
				if d2Sqr < d1Sqr then
					if d2Sqr < state.nearest_sqr then
						self:find_nearest_triangles(iChild2, otherTree, TransformF, oBox, depth + 1, state)
					end
					if d1Sqr < state.nearest_sqr then
						self:find_nearest_triangles(iChild1, otherTree, TransformF, oBox, depth + 1, state)
					end
				else
					if d1Sqr < state.nearest_sqr then
						self:find_nearest_triangles(iChild1, otherTree, TransformF, oBox, depth + 1, state)
					end
					if d2Sqr < state.nearest_sqr then
						self:find_nearest_triangles(iChild2, otherTree, TransformF, oBox, depth + 1, state)
					end
				end
			end
		end
	end

	-------------------------------------------------------------------------------------------------
	-- generic traversal

	-- Instances of this class can be passed in to the DoTraversal() function to implement your
	-- own tree-traversal queries.
	-- NextBoxF(box, depth) is called for each box node. Return false from this function to halt terminate
	-- that branch of the traversal, or true to descend into that box's children (boxes or triangles).
	-- NextTriangleF(tID) is called for each triangle.
	local TreeTraversal = {}
	TreeTraversal.__index = TreeTraversal
	TreeTraversal.ClassName = "DMeshAABBTree3.TreeTraversal"
	function TreeTraversal.new(nextBoxF: ((any, number) -> boolean)?, nextTriangleF: ((number) -> ())?)
		return setmetatable({
			-- return false to terminate this branch
			-- arguments are box and depth in tree
			NextBoxF = nextBoxF or function(box, depth)
				return true
			end,
			NextTriangleF = nextTriangleF or function(tID) end,
		}, TreeTraversal)
	end
	DMeshAABBTree3.TreeTraversal = TreeTraversal

	-- Hierarchically descend through the tree nodes, calling the TreeTrversal functions at each level
	function DMeshAABBTree3:DoTraversal(traversal)
		check_valid(self, "DoTraversal")
		self:tree_traversal(self.root_index, 0, traversal)
	end

	-- traversal implementation. you can override to customize this if necessary.
	function DMeshAABBTree3:tree_traversal(iBox: number, depth: number, traversal)
		local ilist = self.index_list
		local idx = self.box_to_index[iBox + 1]

		if idx < self.triangles_end then
			-- triange-list case, array is [N t1 t2 ... tN]
			local filter = self.TriangleFilterF
			local n = ilist[idx + 1]
			for i = 1, n do
				local ti = ilist[idx + i + 1]
				if filter ~= nil and not filter(ti) then
					continue
				end
				traversal.NextTriangleF(ti)
			end
		else
			local i0 = ilist[idx + 1]
			if i0 < 0 then
				-- negative index means we only have one 'child' box to descend into
				i0 = -i0 - 1
				if traversal.NextBoxF(self:get_box(i0), depth + 1) then
					self:tree_traversal(i0, depth + 1, traversal)
				end
			else
				-- positive index, two sequential child box indices to descend into
				i0 = i0 - 1
				if traversal.NextBoxF(self:get_box(i0), depth + 1) then
					self:tree_traversal(i0, depth + 1, traversal)
				end
				local i1 = ilist[idx + 2] - 1
				if traversal.NextBoxF(self:get_box(i1), depth + 1) then
					self:tree_traversal(i1, depth + 1, traversal)
				end
			end
		end
	end

	-- Total sum of volumes of all boxes in the tree. Mainly useful to evaluate tree quality.
	function DMeshAABBTree3:TotalVolume(): number
		local volSum = 0
		local t = TreeTraversal.new(function(box, depth)
			volSum += box:Volume()
			return true
		end)
		self:DoTraversal(t)
		return volSum
	end

	-- Total sum-of-extents over all boxes in the tree. Mainly useful to evaluate tree quality.
	function DMeshAABBTree3:TotalExtentSum(): number
		local extSum = 0
		local t = TreeTraversal.new(function(box, depth)
			extSum += box:Extents():LengthL1()
			return true
		end)
		self:DoTraversal(t)
		return extSum
	end

	-------------------------------------------------------------------------------------------------
	-- debugging

	local function gBreakToDebugger(msg: string)
		error("DMeshAABBTree3: " .. msg, 2)
	end

	-- 1) make sure we can reach every tri in mesh through tree (also demo of how to traverse tree...)
	-- 2) make sure that triangles are contained in parent boxes
	-- (C# calls Util.gBreakToDebugger on failure; here an error is raised)
	function DMeshAABBTree3:TestCoverage()
		local mesh = self.mesh
		local tri_counts = table.create(mesh:MaxTriangleID(), 0)
		local parent_indices = table.create(#self.box_to_index, 0)

		self:test_coverage(tri_counts, parent_indices, self.root_index)

		for ti in mesh:TriangleIndices() do
			if tri_counts[ti + 1] ~= 1 then
				gBreakToDebugger("TestCoverage: triangle " .. ti .. " reached " .. tri_counts[ti + 1] .. " times")
			end
		end
	end

	-- accumulate triangle counts and track each box-parent index.
	-- also checks that triangles are contained in boxes
	function DMeshAABBTree3:test_coverage(tri_counts: { number }, parent_indices: { number }, iBox: number)
		local ilist = self.index_list
		local idx = self.box_to_index[iBox + 1]

		self:debug_check_child_tris_in_box(iBox)

		if idx < self.triangles_end then
			-- triange-list case, array is [N t1 t2 ... tN]
			local n = ilist[idx + 1]
			local box = self:get_box(iBox)
			for i = 1, n do
				local ti = ilist[idx + i + 1]
				tri_counts[ti + 1] += 1

				local tv = self.mesh:GetTriangle(ti)
				for j = 1, 3 do
					local v = self.mesh:GetVertex(tv[j])
					if not box:Contains(v) then
						gBreakToDebugger("test_coverage: vertex not contained in leaf box")
					end
				end
			end
		else
			local i0 = ilist[idx + 1]
			if i0 < 0 then
				-- negative index means we only have one 'child' box to descend into
				i0 = -i0 - 1
				parent_indices[i0 + 1] = iBox
				self:test_coverage(tri_counts, parent_indices, i0)
			else
				-- positive index, two sequential child box indices to descend into
				i0 = i0 - 1
				parent_indices[i0 + 1] = iBox
				self:test_coverage(tri_counts, parent_indices, i0)
				local i1 = ilist[idx + 2]
				i1 = i1 - 1
				parent_indices[i1 + 1] = iBox
				self:test_coverage(tri_counts, parent_indices, i1)
			end
		end
	end

	-- do full tree traversal below iBox and make sure that all triangles are further
	-- than box-distance-sqr
	function DMeshAABBTree3:debug_check_child_tri_distances(iBox: number, p)
		local fBoxDistSqr = self:box_distance_sqr(iBox, p.x, p.y, p.z)
		local mesh = self.mesh
		local t = TreeTraversal.new(nil, function(tID)
			local fTriDistSqr = MeshQueries().TriDistanceSqr(mesh, tID, p)
			if fTriDistSqr < fBoxDistSqr then
				if math.abs(fTriDistSqr - fBoxDistSqr) > MathUtil.ZeroTolerance * 100 then
					gBreakToDebugger("debug_check_child_tri_distances: triangle closer than box")
				end
			end
		end)
		self:tree_traversal(iBox, 0, t)
	end

	-- do full tree traversal below iBox to make sure that all child triangles are contained
	function DMeshAABBTree3:debug_check_child_tris_in_box(iBox: number)
		local box = self:get_box(iBox)
		local mesh = self.mesh
		local t = TreeTraversal.new(nil, function(tID)
			local tv = mesh:GetTriangle(tID)
			for j = 1, 3 do
				local v = mesh:GetVertex(tv[j])
				if box:Contains(v) == false then
					gBreakToDebugger("debug_check_child_tris_in_box: vertex not contained in box")
				end
			end
		end)
		self:tree_traversal(iBox, 0, t)
	end
end
