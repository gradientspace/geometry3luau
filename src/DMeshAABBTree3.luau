--!native
-- Port of geometry3Sharp spatial/DMeshAABBTree.cs (DMeshAABBTree3)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Hierarchical Axis-Aligned-Bounding-Box tree for a DMesh3 mesh.
-- This class supports a variety of spatial queries, listed below.
--
-- Various construction strategies are also available, the default is the
-- fastest to build but if you are doing a *lot* of queries, you might experiment
-- with the others (eg TopDownMedian)
--
-- Available queries:
--   - FindNearestTriangle(point, maxdist)
--   - FindNearestHitTriangle(ray, maxdist)
--   - FindAllHitTriangles(ray, maxdist)
--   - TestIntersection(triangle)
--   - TestIntersection(mesh)
--   - TestIntersection(otherAABBTree)
--   - FindAllIntersections(otherAABBTree)
--   - FindNearestTriangles(otherAABBTree, maxdist)
--   - IsInside(point)
--   - WindingNumber(point)
--   - FastWindingNumber(point)
--   - DoTraversal(generic_traversal_object)
--
-- ===================================================================================================
-- Luau port notes (see PORTING.md):
--   * DMeshAABBTree3.new(mesh, autoBuild?)    tree:Build(eStrategy?, ePolicy?)
--     enums DMeshAABBTree3.BuildStrategy / DMeshAABBTree3.ClusterPolicy (C# nested enums, same values)
--   * implements ISpatial (see SpatialInterfaces.luau)
--   * C# properties -> methods: tree:Mesh(), tree:IsValid(), tree:Bounds(), tree:SupportsNearestTriangle(),
--     tree:SupportsTriangleRayIntersection(), tree:SupportsPointContainment()
--   * public fields keep their names: TriangleFilterF (function(tid) -> bool; a falsy return rejects the
--     triangle), TopDownLeafMaxTriCount, BottomUpClusterLookahead, FWNBeta, FWNApproxOrder
--   * overloads / out parameters:
--       FindNearestTriangle(p, fMaxDist?) -> tid
--       FindNearestTriangleDistSqr(p, fMaxDist?) -> tid, fNearestDistSqr   (C# FindNearestTriangle(p, out dist, max))
--       FindNearestTriangleXYZ(x, y, z, fMaxDist?) -> tid, fNearestDistSqr  (allocation-free extra)
--       FindNearestVertex(p, fMaxDist?) -> vid
--       FindNearestHitTriangle(ray, fMaxDist?) -> tid
--       FindAllHitTriangles(ray, hitTriangles?, fMaxDist?) -> count   (hitTriangles: Luau array, appended to)
--       TestIntersection(x, ...) dispatches on x: Triangle3d -> TestIntersectionTriangle(tri),
--         DMeshAABBTree3 -> TestIntersectionTree(otherTree, TransformF?),
--         otherwise mesh -> TestIntersectionMesh(testMesh, TransformF?, bBoundsCheck?)
--       FindAllIntersections(otherTree, TransformF?) -> { Points = {PointIntersection}, Segments = {SegmentIntersection} }
--         PointIntersection = { t0, t1, point }, SegmentIntersection = { t0, t1, point0, point1 }
--       FindNearestTriangles(otherTree, TransformF?, max_dist?) -> Index2i {my_tri, other_tri}, distance
--       WindingNumber(p), FastWindingNumber(p), FastWindingNumberXYZ(x, y, z) (allocation-free extra), IsInside(p)
--       DoTraversal(traversal), DMeshAABBTree3.TreeTraversal.new(nextBoxF?, nextTriangleF?)
--       TotalVolume(), TotalExtentSum(), TestCoverage() (errors instead of gBreakToDebugger)
--   * internal storage (flat Luau arrays, 0-based ids):
--       box_to_index[iBox + 1], index_list[i + 1] (same record layout as C#),
--       box_centers / box_extents: 3 numbers per box at [3*iBox + 1 .. 3*iBox + 3] (C# Vector3f -> doubles)
--       box_bounds: 6 numbers per box (min xyz, max xyz) of the box_eps-expanded box used by all queries
--   * Build() also discards the winding-number caches (C# keeps them if the mesh timestamp did not change,
--     which leaves stale caches after a rebuild with a different strategy).
--   * the tree must be built before queries; queries on an unbuilt or out-of-date tree raise an error.

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local AxisAlignedBox3d = require(script.Parent.AxisAlignedBox3d)
local Ray3d = require(script.Parent.Ray3d)
local IntrRay3Triangle3 = require(script.Parent.IntrRay3Triangle3)
local IntrLine3AxisAlignedBox3 = require(script.Parent.IntrLine3AxisAlignedBox3)
local DistPoint3Triangle3 = require(script.Parent.DistPoint3Triangle3)
local FastWindingMath = require(script.Parent.FastWindingMath)
local MeshTriInfoCache = require(script.Parent.MeshTriInfoCache)

local V3 = Vector3d.new
local sqrt, atan2, min, max, abs = math.sqrt, math.atan2, math.min, math.max, math.abs
local MaxDouble = MathUtil.MaxDouble
local MaxFloat = MathUtil.MaxFloat
local FourPI = MathUtil.FourPI
local ZeroTolerance = MathUtil.ZeroTolerance
local ZeroTolerancef = MathUtil.ZeroTolerancef
local NEG_INF = -math.huge
local InvalidID = -1

local distance_sqr_xyz = DistPoint3Triangle3.distance_sqr_xyz
local ray_tri_intersects_xyz = IntrRay3Triangle3.IntersectsXYZ
local DoClippingXYZ = IntrLine3AxisAlignedBox3.DoClippingXYZ
local ComputeCoeffsFlat = FastWindingMath.FastTriWinding.ComputeCoeffsFlat

-- const float box_eps = 50.0f * MathUtil.Epsilonf   (exactly 50 * 2^-23 in float)
local box_eps = 50.0 * 1.1920928955078125e-07

local DMeshAABBTree3 = {}
DMeshAABBTree3.__index = DMeshAABBTree3
DMeshAABBTree3.ClassName = "DMeshAABBTree3"

-- how should we build the tree?
DMeshAABBTree3.BuildStrategy = table.freeze({
	Default = 0, -- currently TopDownMidpoint
	TopDownMidpoint = 1, -- Recursively split triangle set by midpoint of axis interval.
	--   This is the fastest and usually produces lower total-volume trees than bottom-up.
	--   Resulting trees are unbalanced, though.
	BottomUpFromOneRings = 2, -- Construct leaf layer based on triangle one-rings, and then
	--   cluster boxes to build tree upward. Various cluster policies (below).
	--   About 2.5x slower than TopDownMidpoint. Trees are basically balanced, although
	--   current approach to clustering odd-count layers is to duplicate the extra box.
	TopDownMedian = 3, -- Like TopDownMidpoint except we sort the triangle lists at each step and split on the median.
	--   2-4x slower than TopDownMidpoint, but trees are generally more efficient and balanced.
})
local BuildStrategy = DMeshAABBTree3.BuildStrategy

DMeshAABBTree3.ClusterPolicy = table.freeze({
	Default = 0, -- currently FastVolumeMetric
	Fastest = 1, -- sort list and then just cluster sequential boxes.
	--   Tree efficiency suffers, but fast.
	FastVolumeMetric = 2, -- sort list and then check next N boxes for best cluster. Only slightly slower than
	--   sequential clustering but trees are often quite a bit more efficient.
	MinimalVolume = 3, -- compute full pair matrix at each step (N^2), and sequentially pick out best pairs.
	--   this usually does quite a bit better job, but it is unusable for large tri counts.
})
local ClusterPolicy = DMeshAABBTree3.ClusterPolicy

function DMeshAABBTree3.new(m, autoBuild: boolean?)
	local self = setmetatable({
		mesh = m,
		mesh_timestamp = -1,

		-- If non-nil, only triangle IDs that pass this filter (ie filter is true) are considered
		TriangleFilterF = nil,

		-- Top-down build strategies will put at most this many triangles into a box.
		-- Larger value == shallower trees, but leaves cost more to test
		TopDownLeafMaxTriCount = 4,

		-- bottom-up FastVolumeMetric cluster policy sorts available boxes along an axis
		-- and then proceeds from min to max, greedily grouping best-pairs. This value determines
		-- the range of the greedy search. Larger == slower but better bounding
		-- (at least in theory...)
		BottomUpClusterLookahead = 10,

		-- FWN beta parameter - is 2.0 in paper
		FWNBeta = 2.0,
		-- FWN approximation order. can be 1 or 2. 2 is more accurate, obviously.
		FWNApproxOrder = 2,

		-- storage for box nodes (see header)
		box_to_index = {},
		box_centers = {},
		box_extents = {},
		index_list = {},
		box_bounds = {},
		triangles_end = -1,
		root_index = -1,

		WindingCache = nil,
		winding_cache_timestamp = -1,
		FastWindingCache = nil,
		fast_winding_cache_timestamp = -1,
		fwn_branch = nil,

		_stackB = {},
		_stackD = {},
		_stack_busy = false,
	}, DMeshAABBTree3)
	if autoBuild then
		self:Build()
	end
	return self
end

function DMeshAABBTree3:Mesh()
	return self.mesh
end

-- Build the tree. Policy only matters for bottom-up strategies
function DMeshAABBTree3:Build(eStrategy: number?, ePolicy: number?)
	local strategy = eStrategy or BuildStrategy.TopDownMidpoint
	local policy = ePolicy or ClusterPolicy.Default
	if strategy == BuildStrategy.BottomUpFromOneRings then
		self:build_by_one_rings(policy)
	elseif strategy == BuildStrategy.TopDownMedian then
		self:build_top_down(true)
	elseif strategy == BuildStrategy.TopDownMidpoint then
		self:build_top_down(false)
	elseif strategy == BuildStrategy.Default then
		self:build_top_down(false)
	end
	self.mesh_timestamp = self.mesh:ShapeTimestamp()
	self:finalize_boxes()
	self.WindingCache = nil
	self.winding_cache_timestamp = -1
	self.FastWindingCache = nil
	self.fast_winding_cache_timestamp = -1
	self.fwn_branch = nil
end

function DMeshAABBTree3:IsValid(): boolean
	return self.mesh_timestamp == self.mesh:ShapeTimestamp()
end

-- Does this ISpatial implementation support nearest-point query? (yes)
function DMeshAABBTree3:SupportsNearestTriangle(): boolean
	return true
end
-- Does this ISpatial implementation support ray-triangle intersection? (yes)
function DMeshAABBTree3:SupportsTriangleRayIntersection(): boolean
	return true
end
-- Does this ISpatial support IsInside() test (yes!)
function DMeshAABBTree3:SupportsPointContainment(): boolean
	return true
end

local function check_valid(self, fname: string)
	if self.mesh_timestamp ~= self.mesh:ShapeTimestamp() then
		error("DMeshAABBTree3." .. fname .. ": mesh has been modified since tree construction", 3)
	end
end
DMeshAABBTree3.check_valid = check_valid

-- traversal stacks are re-used across queries, unless a query is re-entered (eg from TriangleFilterF)
local function acquire_stack(self): ({ number }, { number }, boolean)
	if self._stack_busy then
		return {}, {}, false
	end
	self._stack_busy = true
	return self._stackB, self._stackD, true
end

---------------------------------------------------------------------------------------------------
-- nearest triangle

local function find_nearest_tri(self, px: number, py: number, pz: number, fNearestSqr: number): (number, number)
	local btoi, ilist, bb, tend = self.box_to_index, self.index_list, self.box_bounds, self.triangles_end
	local mtris, mverts = self.mesh.triangles.data, self.mesh.vertices.data
	local filter = self.TriangleFilterF
	local stackB, stackD, shared = acquire_stack(self)
	local tID = InvalidID

	local top = 1
	stackB[1] = self.root_index
	stackD[1] = NEG_INF
	while top > 0 do
		local iBox = stackB[top]
		local boxDist = stackD[top]
		top -= 1
		if boxDist < fNearestSqr then
			local idx = btoi[iBox + 1]
			if idx < tend then -- triange-list case, array is [N t1 t2 ... tN]
				local num_tris = ilist[idx + 1]
				for i = 1, num_tris do
					local ti = ilist[idx + i + 1]
					if filter ~= nil and not filter(ti) then
						continue
					end
					-- fTriDistSqr = MeshQueries.TriDistanceSqr(mesh, ti, p)
					local t3 = 3 * ti
					local a, b, c = 3 * mtris[t3 + 1], 3 * mtris[t3 + 2], 3 * mtris[t3 + 3]
					local fTriDistSqr = distance_sqr_xyz(
						px,
						py,
						pz,
						mverts[a + 1],
						mverts[a + 2],
						mverts[a + 3],
						mverts[b + 1],
						mverts[b + 2],
						mverts[b + 3],
						mverts[c + 1],
						mverts[c + 2],
						mverts[c + 3]
					)
					if fTriDistSqr < 0 then
						fTriDistSqr = 0
					end
					if fTriDistSqr < fNearestSqr then
						fNearestSqr = fTriDistSqr
						tID = ti
					end
				end
			else -- internal node, either 1 or 2 child boxes
				local iChild1 = ilist[idx + 1]
				if iChild1 < 0 then -- 1 child, descend if nearer than cur min-dist
					iChild1 = -iChild1 - 1
					-- fChild1DistSqr = box_distance_sqr(iChild1, p)
					local o = 6 * iChild1
					local mnx, mny, mnz, mxx, mxy, mxz = bb[o + 1], bb[o + 2], bb[o + 3], bb[o + 4], bb[o + 5], bb[o + 6]
					local dx = (px < mnx) and (mnx - px) or ((px > mxx) and (px - mxx) or 0)
					local dy = (py < mny) and (mny - py) or ((py > mxy) and (py - mxy) or 0)
					local dz = (pz < mnz) and (mnz - pz) or ((pz > mxz) and (pz - mxz) or 0)
					local fChild1DistSqr = dx * dx + dy * dy + dz * dz
					if fChild1DistSqr <= fNearestSqr then
						top += 1
						stackB[top] = iChild1
						stackD[top] = NEG_INF
					end
				else -- 2 children, descend closest first
					iChild1 = iChild1 - 1
					local iChild2 = ilist[idx + 2] - 1

					local o = 6 * iChild1
					local mnx, mny, mnz, mxx, mxy, mxz = bb[o + 1], bb[o + 2], bb[o + 3], bb[o + 4], bb[o + 5], bb[o + 6]
					local dx = (px < mnx) and (mnx - px) or ((px > mxx) and (px - mxx) or 0)
					local dy = (py < mny) and (mny - py) or ((py > mxy) and (py - mxy) or 0)
					local dz = (pz < mnz) and (mnz - pz) or ((pz > mxz) and (pz - mxz) or 0)
					local fChild1DistSqr = dx * dx + dy * dy + dz * dz

					o = 6 * iChild2
					mnx, mny, mnz, mxx, mxy, mxz = bb[o + 1], bb[o + 2], bb[o + 3], bb[o + 4], bb[o + 5], bb[o + 6]
					dx = (px < mnx) and (mnx - px) or ((px > mxx) and (px - mxx) or 0)
					dy = (py < mny) and (mny - py) or ((py > mxy) and (py - mxy) or 0)
					dz = (pz < mnz) and (mnz - pz) or ((pz > mxz) and (pz - mxz) or 0)
					local fChild2DistSqr = dx * dx + dy * dy + dz * dz

					-- the farther child is pushed first with its distance, and re-tested against the
					-- (possibly updated) nearest distance when popped, exactly like the C# recursion
					if fChild1DistSqr < fChild2DistSqr then
						if fChild1DistSqr < fNearestSqr then
							stackB[top + 1] = iChild2
							stackD[top + 1] = fChild2DistSqr
							stackB[top + 2] = iChild1
							stackD[top + 2] = NEG_INF
							top += 2
						end
					else
						if fChild2DistSqr < fNearestSqr then
							stackB[top + 1] = iChild1
							stackD[top + 1] = fChild1DistSqr
							stackB[top + 2] = iChild2
							stackD[top + 2] = NEG_INF
							top += 2
						end
					end
				end
			end
		end
	end
	if shared then
		self._stack_busy = false
	end
	return tID, fNearestSqr
end

-- Find the triangle closest to p, within distance fMaxDist, or return InvalidID
-- Use MeshQueries.TriangleDistance() to get more information
function DMeshAABBTree3:FindNearestTriangle(p, fMaxDist: number?): number
	check_valid(self, "FindNearestTriangle")
	local maxDist = fMaxDist or MaxDouble
	local fNearestSqr = (maxDist < MaxDouble) and maxDist * maxDist or MaxDouble
	local tNearID = find_nearest_tri(self, p.x, p.y, p.z, fNearestSqr)
	return tNearID
end

-- Find the triangle closest to p, and distance to it, within distance fMaxDist, or return InvalidID
-- C# FindNearestTriangle(Vector3d p, out double fNearestDistSqr, double fMaxDist) -> returns tid, fNearestDistSqr
function DMeshAABBTree3:FindNearestTriangleDistSqr(p, fMaxDist: number?): (number, number)
	check_valid(self, "FindNearestTriangle")
	local maxDist = fMaxDist or MaxDouble
	local fNearestDistSqr = (maxDist < MaxDouble) and maxDist * maxDist or MaxDouble
	return find_nearest_tri(self, p.x, p.y, p.z, fNearestDistSqr)
end

-- allocation-free variant of FindNearestTriangleDistSqr
function DMeshAABBTree3:FindNearestTriangleXYZ(x: number, y: number, z: number, fMaxDist: number?): (number, number)
	check_valid(self, "FindNearestTriangle")
	local maxDist = fMaxDist or MaxDouble
	local fNearestDistSqr = (maxDist < MaxDouble) and maxDist * maxDist or MaxDouble
	return find_nearest_tri(self, x, y, z, fNearestDistSqr)
end

---------------------------------------------------------------------------------------------------
-- box helpers (all use the box_eps-expanded box, like C# get_box/box_distance_sqr/...)

local function box_distance_sqr(self, iBox: number, px: number, py: number, pz: number): number
	local bb = self.box_bounds
	local o = 6 * iBox
	local mnx, mny, mnz, mxx, mxy, mxz = bb[o + 1], bb[o + 2], bb[o + 3], bb[o + 4], bb[o + 5], bb[o + 6]
	local dx = (px < mnx) and (mnx - px) or ((px > mxx) and (px - mxx) or 0)
	local dy = (py < mny) and (mny - py) or ((py > mxy) and (py - mxy) or 0)
	local dz = (pz < mnz) and (mnz - pz) or ((pz > mxz) and (pz - mxz) or 0)
	return dx * dx + dy * dy + dz * dz
end
DMeshAABBTree3.box_distance_sqr = box_distance_sqr

local function box_contains(self, iBox: number, px: number, py: number, pz: number): boolean
	local bb = self.box_bounds
	local o = 6 * iBox
	return (bb[o + 1] <= px) and (bb[o + 2] <= py) and (bb[o + 3] <= pz) and (bb[o + 4] >= px) and (bb[o + 5] >= py) and (bb[o + 6] >= pz)
end
DMeshAABBTree3.box_contains = box_contains

-- ray-param of first intersection with box, or double.MaxValue on miss
local function box_ray_intersect_t(bb: { number }, iBox: number, ox, oy, oz, dx, dy, dz): number
	local o = 6 * iBox
	local ok, t0 = DoClippingXYZ(0.0, MaxDouble, ox, oy, oz, dx, dy, dz, bb[o + 1], bb[o + 2], bb[o + 3], bb[o + 4], bb[o + 5], bb[o + 6])
	if ok then
		return t0
	end
	return MaxDouble
end
DMeshAABBTree3.box_ray_intersect_t = function(self, iBox: number, ray): number
	local o, d = ray.Origin, ray.Direction
	return box_ray_intersect_t(self.box_bounds, iBox, o.x, o.y, o.z, d.x, d.y, d.z)
end

-- C# get_box(iBox) / get_boxd(iBox): box_eps-expanded box as a new AxisAlignedBox3d
function DMeshAABBTree3:get_box(iBox: number)
	local bb = self.box_bounds
	local o = 6 * iBox
	return AxisAlignedBox3d.new(bb[o + 1], bb[o + 2], bb[o + 3], bb[o + 4], bb[o + 5], bb[o + 6])
end

-- C# get_boxd(iBox, TransformF)
function DMeshAABBTree3:get_boxd(iBox: number, TransformF: ((any) -> any)?)
	local box = self:get_box(iBox)
	if TransformF ~= nil then
		return require(script.Parent.BoundsUtil).BoundsTransformed(box, TransformF)
	end
	return box
end

-- center of box iBox (C# box_centers[iBox])
function DMeshAABBTree3:get_box_center(iBox: number)
	local bc = self.box_centers
	return V3(bc[3 * iBox + 1], bc[3 * iBox + 2], bc[3 * iBox + 3])
end

-- Root bounding box of tree
function DMeshAABBTree3:Bounds()
	return self:get_box(self.root_index)
end

---------------------------------------------------------------------------------------------------
-- nearest vertex

-- Find the vertex closest to p, within distance fMaxDist, or return InvalidID
function DMeshAABBTree3:FindNearestVertex(p, fMaxDist: number?): number
	check_valid(self, "FindNearestVertex")
	local maxDist = fMaxDist or MaxDouble
	local fNearestSqr = (maxDist < MaxDouble) and maxDist * maxDist or MaxDouble
	local px, py, pz = p.x, p.y, p.z

	local btoi, ilist, tend = self.box_to_index, self.index_list, self.triangles_end
	local mtris, mverts = self.mesh.triangles.data, self.mesh.vertices.data
	local filter = self.TriangleFilterF
	local stackB, stackD, shared = acquire_stack(self)
	local vid = InvalidID

	local top = 1
	stackB[1] = self.root_index
	stackD[1] = NEG_INF
	while top > 0 do
		local iBox = stackB[top]
		local boxDist = stackD[top]
		top -= 1
		if boxDist < fNearestSqr then
			local idx = btoi[iBox + 1]
			if idx < tend then
				local num_tris = ilist[idx + 1]
				for i = 1, num_tris do
					local ti = ilist[idx + i + 1]
					if filter ~= nil and not filter(ti) then
						continue
					end
					for j = 1, 3 do
						local tv = mtris[3 * ti + j]
						local vx, vy, vz = mverts[3 * tv + 1], mverts[3 * tv + 2], mverts[3 * tv + 3]
						local dx, dy, dz = px - vx, py - vy, pz - vz
						local dsqr = dx * dx + dy * dy + dz * dz
						if dsqr < fNearestSqr then
							fNearestSqr = dsqr
							vid = tv
						end
					end
				end
			else
				local iChild1 = ilist[idx + 1]
				if iChild1 < 0 then
					iChild1 = -iChild1 - 1
					local fChild1DistSqr = box_distance_sqr(self, iChild1, px, py, pz)
					if fChild1DistSqr <= fNearestSqr then
						top += 1
						stackB[top] = iChild1
						stackD[top] = NEG_INF
					end
				else
					iChild1 = iChild1 - 1
					local iChild2 = ilist[idx + 2] - 1
					local fChild1DistSqr = box_distance_sqr(self, iChild1, px, py, pz)
					local fChild2DistSqr = box_distance_sqr(self, iChild2, px, py, pz)
					if fChild1DistSqr < fChild2DistSqr then
						if fChild1DistSqr < fNearestSqr then
							stackB[top + 1] = iChild2
							stackD[top + 1] = fChild2DistSqr
							stackB[top + 2] = iChild1
							stackD[top + 2] = NEG_INF
							top += 2
						end
					else
						if fChild2DistSqr < fNearestSqr then
							stackB[top + 1] = iChild1
							stackD[top + 1] = fChild1DistSqr
							stackB[top + 2] = iChild2
							stackD[top + 2] = NEG_INF
							top += 2
						end
					end
				end
			end
		end
	end
	if shared then
		self._stack_busy = false
	end
	return vid
end

---------------------------------------------------------------------------------------------------
-- ray queries

-- find id of first triangle that ray hits, within distance fMaxDist, or return DMesh3.InvalidID
-- Use MeshQueries.TriangleIntersection() to get more information
function DMeshAABBTree3:FindNearestHitTriangle(ray, fMaxDist: number?): number
	check_valid(self, "FindNearestHitTriangle")
	if ray.Direction:IsNormalized() == false then
		error("DMeshAABBTree3.FindNearestHitTriangle: ray direction is not normalized", 2)
	end
	local maxDist = fMaxDist or MaxDouble

	-- [RMS] note: using float.MaxValue here because we need to use <= to compare box hit
	--   to fNearestT, and box hit returns double.MaxValue on no-hit. So, if we set
	--   nearestT to double.MaxValue, then we will test all boxes (!)
	local fNearestT = (maxDist < MaxDouble) and maxDist or MaxFloat
	local tID = InvalidID

	local o, d = ray.Origin, ray.Direction
	local ox, oy, oz, dx, dy, dz = o.x, o.y, o.z, d.x, d.y, d.z
	local btoi, ilist, bb, tend = self.box_to_index, self.index_list, self.box_bounds, self.triangles_end
	local mtris, mverts = self.mesh.triangles.data, self.mesh.vertices.data
	local filter = self.TriangleFilterF
	local e = ZeroTolerancef
	local stackB, stackD, shared = acquire_stack(self)

	local top = 1
	stackB[1] = self.root_index
	stackD[1] = NEG_INF
	while top > 0 do
		local iBox = stackB[top]
		local boxT = stackD[top]
		top -= 1
		if boxT <= fNearestT + e then
			local idx = btoi[iBox + 1]
			if idx < tend then -- triange-list case, array is [N t1 t2 ... tN]
				local num_tris = ilist[idx + 1]
				for i = 1, num_tris do
					local ti = ilist[idx + i + 1]
					if filter ~= nil and not filter(ti) then
						continue
					end
					local t3 = 3 * ti
					local a, b, c = 3 * mtris[t3 + 1], 3 * mtris[t3 + 2], 3 * mtris[t3 + 3]
					local hit, rayt = ray_tri_intersects_xyz(
						ox,
						oy,
						oz,
						dx,
						dy,
						dz,
						mverts[a + 1],
						mverts[a + 2],
						mverts[a + 3],
						mverts[b + 1],
						mverts[b + 2],
						mverts[b + 3],
						mverts[c + 1],
						mverts[c + 2],
						mverts[c + 3]
					)
					if hit then
						if rayt < fNearestT then
							fNearestT = rayt
							tID = ti
						end
					end
				end
			else -- internal node, either 1 or 2 child boxes
				local iChild1 = ilist[idx + 1]
				if iChild1 < 0 then -- 1 child, descend if nearer than cur min-dist
					iChild1 = -iChild1 - 1
					local fChild1T = box_ray_intersect_t(bb, iChild1, ox, oy, oz, dx, dy, dz)
					if fChild1T <= fNearestT + e then
						top += 1
						stackB[top] = iChild1
						stackD[top] = NEG_INF
					end
				else -- 2 children, descend closest first
					iChild1 = iChild1 - 1
					local iChild2 = ilist[idx + 2] - 1
					local fChild1T = box_ray_intersect_t(bb, iChild1, ox, oy, oz, dx, dy, dz)
					local fChild2T = box_ray_intersect_t(bb, iChild2, ox, oy, oz, dx, dy, dz)
					if fChild1T < fChild2T then
						if fChild1T <= fNearestT + e then
							stackB[top + 1] = iChild2
							stackD[top + 1] = fChild2T
							stackB[top + 2] = iChild1
							stackD[top + 2] = NEG_INF
							top += 2
						end
					else
						if fChild2T <= fNearestT + e then
							stackB[top + 1] = iChild1
							stackD[top + 1] = fChild1T
							stackB[top + 2] = iChild2
							stackD[top + 2] = NEG_INF
							top += 2
						end
					end
				end
			end
		end
	end
	if shared then
		self._stack_busy = false
	end
	return tID
end

-- Find the ids of all the triangles that they ray intersects, within distance fMaxDist from ray origin
-- Returns count of triangles. hitTriangles (optional Luau array) is appended to.
function DMeshAABBTree3:FindAllHitTriangles(ray, hitTriangles: { number }?, fMaxDist: number?): number
	check_valid(self, "FindNearestHitTriangle")
	if ray.Direction:IsNormalized() == false then
		error("DMeshAABBTree3.FindNearestHitTriangle: ray direction is not normalized", 2)
	end
	local maxDist = fMaxDist or MaxDouble
	-- [RMS] note: using float.MaxValue here because we need to use <= to compare box hit
	--   to fNearestT, and box hit returns double.MaxValue on no-hit. So, if we set
	--   nearestT to double.MaxValue, then we will test all boxes (!)
	local fUseMaxDist = (maxDist < MaxDouble) and maxDist or MaxFloat

	local o, d = ray.Origin, ray.Direction
	local ox, oy, oz, dx, dy, dz = o.x, o.y, o.z, d.x, d.y, d.z
	local btoi, ilist, bb, tend = self.box_to_index, self.index_list, self.box_bounds, self.triangles_end
	local mtris, mverts = self.mesh.triangles.data, self.mesh.vertices.data
	local filter = self.TriangleFilterF
	local e = ZeroTolerancef
	local limit = fUseMaxDist + e
	local stackB, _, shared = acquire_stack(self)
	local hit_count = 0

	-- depth-first, child1 before child2 (same visiting order as the C# recursion)
	local top = 1
	stackB[1] = self.root_index
	while top > 0 do
		local iBox = stackB[top]
		top -= 1
		local idx = btoi[iBox + 1]
		if idx < tend then
			local num_tris = ilist[idx + 1]
			for i = 1, num_tris do
				local ti = ilist[idx + i + 1]
				if filter ~= nil and not filter(ti) then
					continue
				end
				local t3 = 3 * ti
				local a, b, c = 3 * mtris[t3 + 1], 3 * mtris[t3 + 2], 3 * mtris[t3 + 3]
				local hit, rayt = ray_tri_intersects_xyz(
					ox,
					oy,
					oz,
					dx,
					dy,
					dz,
					mverts[a + 1],
					mverts[a + 2],
					mverts[a + 3],
					mverts[b + 1],
					mverts[b + 2],
					mverts[b + 3],
					mverts[c + 1],
					mverts[c + 2],
					mverts[c + 3]
				)
				if hit then
					if rayt < fUseMaxDist then
						if hitTriangles ~= nil then
							table.insert(hitTriangles, ti)
						end
						hit_count += 1
					end
				end
			end
		else
			local iChild1 = ilist[idx + 1]
			if iChild1 < 0 then
				iChild1 = -iChild1 - 1
				if box_ray_intersect_t(bb, iChild1, ox, oy, oz, dx, dy, dz) <= limit then
					top += 1
					stackB[top] = iChild1
				end
			else
				iChild1 = iChild1 - 1
				local iChild2 = ilist[idx + 2] - 1
				if box_ray_intersect_t(bb, iChild2, ox, oy, oz, dx, dy, dz) <= limit then
					top += 1
					stackB[top] = iChild2
				end
				if box_ray_intersect_t(bb, iChild1, ox, oy, oz, dx, dy, dz) <= limit then
					top += 1
					stackB[top] = iChild1
				end
			end
		end
	end
	if shared then
		self._stack_busy = false
	end
	return hit_count
end

-- Returns true if point p is inside this mesh.
function DMeshAABBTree3:IsInside(p): boolean
	-- This is a raycast crossing-count test, which is not ideal!
	-- Only works for closed meshes.

	-- [RMS] this is just a random direction I picked...
	local rayDir = V3(0.331960519038825, 0.462531727525156, 0.822111072077288)
	local rayOrigin = p
	local ray = Ray3d.new(rayOrigin, rayDir)
	local nHits = self:FindAllHitTriangles(ray, nil)
	return (nHits % 2) ~= 0
end

---------------------------------------------------------------------------------------------------
-- Hierarchical Mesh Winding Number computation

-- MathUtil.TriSolidAngle(a, b, c, ref p) on scalars
local function tri_solid_angle(
	ax: number,
	ay: number,
	az: number,
	bx: number,
	by: number,
	bz: number,
	cx: number,
	cy: number,
	cz: number,
	px: number,
	py: number,
	pz: number
): number
	ax, ay, az = ax - px, ay - py, az - pz
	bx, by, bz = bx - px, by - py, bz - pz
	cx, cy, cz = cx - px, cy - py, cz - pz
	local la = sqrt(ax * ax + ay * ay + az * az)
	local lb = sqrt(bx * bx + by * by + bz * bz)
	local lc = sqrt(cx * cx + cy * cy + cz * cz)
	local bottom = (la * lb * lc)
		+ (ax * bx + ay * by + az * bz) * lc
		+ (bx * cx + by * cy + bz * cz) * la
		+ (cx * ax + cy * ay + cz * az) * lb
	local top = ax * (by * cz - cy * bz) - ay * (bx * cz - cx * bz) + az * (bx * cy - cx * by)
	return 2.0 * atan2(top, bottom)
end
DMeshAABBTree3.tri_solid_angle = tri_solid_angle

-- Evaluate the mesh winding number at point. To do this, we must construct additional
-- information to short-circuit tree branches. This happens on the first evaluation.
-- This does consume some additional memory, mainly temporary memory during construction.
-- If you don't want this, just use Mesh.WindingNumber() directly. Also note that if you
-- are only evaluating a few times, it is not sensible - assume you need at least
-- hundreds of evaluations to see speed improvements.
function DMeshAABBTree3:WindingNumber(p): number
	check_valid(self, "WindingNumber")
	if self.WindingCache == nil or self.winding_cache_timestamp ~= self.mesh:ShapeTimestamp() then
		self:build_winding_cache()
		self.winding_cache_timestamp = self.mesh:ShapeTimestamp()
	end
	local sum = self:branch_winding_num(self.root_index, p.x, p.y, p.z)
	return sum / (4.0 * math.pi)
end

-- evaluate winding number contribution for all triangles below iBox
function DMeshAABBTree3:branch_winding_num(iBox: number, px: number, py: number, pz: number): number
	local branch_sum = 0
	local ilist = self.index_list
	local idx = self.box_to_index[iBox + 1]
	if idx < self.triangles_end then -- triange-list case, array is [N t1 t2 ... tN]
		local mtris, mverts = self.mesh.triangles.data, self.mesh.vertices.data
		local num_tris = ilist[idx + 1]
		for i = 1, num_tris do
			local ti = ilist[idx + i + 1]
			local t3 = 3 * ti
			local a, b, c = 3 * mtris[t3 + 1], 3 * mtris[t3 + 2], 3 * mtris[t3 + 3]
			branch_sum += tri_solid_angle(
				mverts[a + 1],
				mverts[a + 2],
				mverts[a + 3],
				mverts[b + 1],
				mverts[b + 2],
				mverts[b + 3],
				mverts[c + 1],
				mverts[c + 2],
				mverts[c + 3],
				px,
				py,
				pz
			)
		end
	else -- internal node, either 1 or 2 child boxes
		local cache = self.WindingCache
		local iChild1 = ilist[idx + 1]
		if iChild1 < 0 then -- 1 child
			iChild1 = -iChild1 - 1
			-- if we have winding cache, we can more efficiently compute contribution of all triangles
			-- below this box. Otherwise, recursively descend tree.
			local contained = box_contains(self, iChild1, px, py, pz)
			if contained == false and cache[iChild1 + 1] then
				branch_sum += self:evaluate_box_winding_cache(iChild1, px, py, pz)
			else
				branch_sum += self:branch_winding_num(iChild1, px, py, pz)
			end
		else -- 2 children
			iChild1 = iChild1 - 1
			local iChild2 = ilist[idx + 2] - 1

			local contained1 = box_contains(self, iChild1, px, py, pz)
			if contained1 == false and cache[iChild1 + 1] then
				branch_sum += self:evaluate_box_winding_cache(iChild1, px, py, pz)
			else
				branch_sum += self:branch_winding_num(iChild1, px, py, pz)
			end

			local contained2 = box_contains(self, iChild2, px, py, pz)
			if contained2 == false and cache[iChild2 + 1] then
				branch_sum += self:evaluate_box_winding_cache(iChild2, px, py, pz)
			else
				branch_sum += self:branch_winding_num(iChild2, px, py, pz)
			end
		end
	end
	return branch_sum
end

-- insertion-ordered integer set (C# HashSet<int>; .NET enumerates a HashSet without removals in
-- insertion order, so iterating `list` reproduces the C# summation order)
local function oset_new()
	return { list = {}, set = {}, n = 0 }
end
local function oset_add(s, v: number)
	if not s.set[v] then
		s.set[v] = true
		local n = s.n + 1
		s.n = n
		s.list[n] = v
	end
end
local function oset_union(s, other)
	local list = other.list
	for i = 1, other.n do
		oset_add(s, list[i])
	end
end
DMeshAABBTree3.oset_new = oset_new
DMeshAABBTree3.oset_add = oset_add

function DMeshAABBTree3:build_winding_cache()
	-- The basic strategy to build the winding cache is to descend the tree until we hit a node with N
	-- triangles below it, then build a cache for those triangles. We also (currently) build all caches
	-- above such a node, because it makes a big speed difference. Changing this threshold does not appear
	-- to make a big difference in query speed, but it does affect the build time and memory usage.
	-- If the mesh is large, we can use larger caches, but on a small mesh it may result in
	-- not actually getting that many caches, which is les compute-efficient.
	-- So, we step up as the threshold the mesh gets larger.
	local WINDING_CACHE_THRESH = 100
	if self.mesh:TriangleCount() > 250000 then
		WINDING_CACHE_THRESH = 500
	end
	if self.mesh:TriangleCount() > 1000000 then
		WINDING_CACHE_THRESH = 1000
	end

	-- WindingCache[iBox + 1] = edge list, or false (pre-sized array, see build_fast_winding_cache)
	self.WindingCache = table.create(#self.box_to_index, false)
	self:build_winding_cache_rec(self.root_index, 0, WINDING_CACHE_THRESH)
end

-- C# build_winding_cache(int iBox, int depth, int tri_count_thresh, out HashSet<int> tri_hash) -> returns count, tri_hash
function DMeshAABBTree3:build_winding_cache_rec(iBox: number, depth: number, tri_count_thresh: number): (number, any)
	local tri_hash = nil
	local ilist = self.index_list
	local idx = self.box_to_index[iBox + 1]
	if idx < self.triangles_end then -- triange-list case, array is [N t1 t2 ... tN]
		local num_tris = ilist[idx + 1]
		return num_tris, nil
	else -- internal node, either 1 or 2 child boxes
		local iChild1 = ilist[idx + 1]
		if iChild1 < 0 then -- 1 child
			iChild1 = -iChild1 - 1
			local num_child_tris
			num_child_tris, tri_hash = self:build_winding_cache_rec(iChild1, depth + 1, tri_count_thresh)
			-- if count in child is large enough, we already built a cache at lower node
			return num_child_tris, tri_hash
		else -- 2 children
			iChild1 = iChild1 - 1
			local iChild2 = ilist[idx + 2] - 1

			-- let each child build its own cache if it wants. If so, it will return the
			-- list of its child tris
			local num_tris_1, num_tris_2, child2_hash
			num_tris_1, tri_hash = self:build_winding_cache_rec(iChild1, depth + 1, tri_count_thresh)
			num_tris_2, child2_hash = self:build_winding_cache_rec(iChild2, depth + 1, tri_count_thresh)
			local build_cache = (num_tris_1 + num_tris_2 > tri_count_thresh)

			if depth == 0 then
				return num_tris_1 + num_tris_2, tri_hash -- cannot build cache at level 0...
			end

			-- collect up the triangles we need. there are various cases depending on what children already did
			if tri_hash ~= nil or child2_hash ~= nil or build_cache then
				if tri_hash == nil and child2_hash ~= nil then
					self:collect_triangles(iChild1, child2_hash)
					tri_hash = child2_hash
				else
					if tri_hash == nil then
						tri_hash = oset_new()
						self:collect_triangles(iChild1, tri_hash)
					end
					if child2_hash == nil then
						self:collect_triangles(iChild2, tri_hash)
					else
						oset_union(tri_hash, child2_hash)
					end
				end
			end
			if build_cache then
				self:make_box_winding_cache(iBox, tri_hash)
			end

			return num_tris_1 + num_tris_2, tri_hash
		end
	end
end

-- collect all triangles under iBox, find open edges [a,b],
-- and add them all to a list associated with iBox
function DMeshAABBTree3:make_box_winding_cache(iBox: number, triangles)
	local mesh = self.mesh
	local edges = {}
	local set, list = triangles.set, triangles.list
	for k = 1, triangles.n do
		local tid = list[k]
		local tri = mesh:GetTriangle(tid)
		local nbr_tris = mesh:GetTriNeighbourTris(tid)
		for j = 0, 2 do
			local nbr = nbr_tris[j + 1]
			if nbr == InvalidID or not set[nbr] then
				table.insert(edges, tri[((j + 1) % 3) + 1])
				table.insert(edges, tri[j + 1])
			end
		end
	end
	self.WindingCache[iBox + 1] = edges
end

-- evaluate the winding cache for iBox
function DMeshAABBTree3:evaluate_box_winding_cache(iBox: number, px: number, py: number, pz: number): number
	local boxcache = self.WindingCache[iBox + 1]
	local N = #boxcache // 2
	-- evaluate winding calc over arbitrary triangle fan that "closes"
	-- the open mesh below this box.
	local bc = self.box_centers
	local cx, cy, cz = bc[3 * iBox + 1], bc[3 * iBox + 2], bc[3 * iBox + 3]
	local mverts = self.mesh.vertices.data
	local cluster_sum = 0
	for i = 0, N - 1 do
		local a = 3 * boxcache[2 * i + 1]
		local b = 3 * boxcache[2 * i + 2]
		cluster_sum += tri_solid_angle(
			mverts[a + 1],
			mverts[a + 2],
			mverts[a + 3],
			mverts[b + 1],
			mverts[b + 2],
			mverts[b + 3],
			cx,
			cy,
			cz,
			px,
			py,
			pz
		)
	end
	-- contribution of open mesh is -sum over fan
	return -cluster_sum
end

-- collect all the triangles below iBox in a hash
function DMeshAABBTree3:collect_triangles(iBox: number, triangles)
	local ilist = self.index_list
	local idx = self.box_to_index[iBox + 1]
	if idx < self.triangles_end then -- triange-list case, array is [N t1 t2 ... tN]
		local num_tris = ilist[idx + 1]
		for i = 1, num_tris do
			oset_add(triangles, ilist[idx + i + 1])
		end
	else
		local iChild1 = ilist[idx + 1]
		if iChild1 < 0 then -- 1 child
			self:collect_triangles(-iChild1 - 1, triangles)
		else -- 2 children
			self:collect_triangles(iChild1 - 1, triangles)
			self:collect_triangles(ilist[idx + 2] - 1, triangles)
		end
	end
end

---------------------------------------------------------------------------------------------------
-- Fast Mesh Winding Number computation

-- builds the recursive evaluator closure for the current tree + fast winding cache.
-- C# branch_fast_winding_num / can_use_fast_winding_cache / evaluate_box_fast_winding_cache, inlined on scalars.
local function make_fwn_branch(self)
	local btoi, ilist, bb, tend = self.box_to_index, self.index_list, self.box_bounds, self.triangles_end
	local mtris, mverts = self.mesh.triangles.data, self.mesh.vertices.data
	local cache = self.FastWindingCache
	local fwn, has = cache.coeffs, cache.has

	-- query parameters (FWNBeta, FWNApproxOrder == 2) are upvalues set by the returned evaluate function,
	-- the evaluation does not call out to user code so it cannot be re-entered
	local beta, order2 = 2.0, true
	local branch
	branch = function(iBox: number, px: number, py: number, pz: number): number
		local branch_sum = 0
		local idx = btoi[iBox + 1]
		if idx < tend then -- triange-list case, array is [N t1 t2 ... tN]
			local num_tris = ilist[idx + 1]
			for i = 1, num_tris do
				local ti = ilist[idx + i + 1]
				local t3 = 3 * ti
				local a, b, c = 3 * mtris[t3 + 1], 3 * mtris[t3 + 2], 3 * mtris[t3 + 3]
				-- MathUtil.TriSolidAngle(a, b, c, ref p) / MathUtil.FourPI
				local ax, ay, az = mverts[a + 1] - px, mverts[a + 2] - py, mverts[a + 3] - pz
				local bx, by, bz = mverts[b + 1] - px, mverts[b + 2] - py, mverts[b + 3] - pz
				local cx, cy, cz = mverts[c + 1] - px, mverts[c + 2] - py, mverts[c + 3] - pz
				local la = sqrt(ax * ax + ay * ay + az * az)
				local lb = sqrt(bx * bx + by * by + bz * bz)
				local lc = sqrt(cx * cx + cy * cy + cz * cz)
				local bottom = (la * lb * lc)
					+ (ax * bx + ay * by + az * bz) * lc
					+ (bx * cx + by * cy + bz * cz) * la
					+ (cx * ax + cy * ay + cz * az) * lb
				local top = ax * (by * cz - cy * bz) - ay * (bx * cz - cx * bz) + az * (bx * cy - cx * by)
				branch_sum += (2.0 * atan2(top, bottom)) / FourPI
			end
			return branch_sum
		end

		-- internal node, either 1 or 2 child boxes (negative index == single child)
		local iChild1 = ilist[idx + 1]
		local nChildren = 2
		if iChild1 < 0 then
			iChild1 = -iChild1 - 1
			nChildren = 1
		else
			iChild1 = iChild1 - 1
		end
		for k = 1, nChildren do
			local ci = (k == 1) and iChild1 or (ilist[idx + 2] - 1)
			-- if we have winding cache, we can more efficiently compute contribution of all triangles
			-- below this box. Otherwise, recursively descend tree.
			local o = 6 * ci
			-- contained = box_contains(ci, p)
			local contained = (bb[o + 1] <= px)
				and (bb[o + 2] <= py)
				and (bb[o + 3] <= pz)
				and (bb[o + 4] >= px)
				and (bb[o + 5] >= py)
				and (bb[o + 6] >= pz)
			local used_cache = false
			if contained == false and has[ci + 1] then
				-- can_use_fast_winding_cache: cacheInfo.Center.Distance(q) > FWNBeta * cacheInfo.R
				local fo = 16 * ci
				local dx, dy, dz = fwn[fo + 1] - px, fwn[fo + 2] - py, fwn[fo + 3] - pz -- dpq = center - q
				local len = sqrt(dx * dx + dy * dy + dz * dz)
				if len > beta * fwn[fo + 4] then
					used_cache = true
					local len3 = len * len * len
					if order2 then
						-- FastTriWinding.EvaluateOrder2Approx(center, order1, order2, q)
						local fourPi_len3 = 1.0 / (FourPI * len3)
						local order1v = fourPi_len3 * (fwn[fo + 5] * dx + fwn[fo + 6] * dy + fwn[fo + 7] * dz)
						local c = -3.0 / (FourPI * len3 * len * len)
						local order2v = (
							fwn[fo + 8] * (fourPi_len3 + c * dx * dx)
							+ fwn[fo + 9] * (c * dx * dy)
							+ fwn[fo + 10] * (c * dx * dz)
						)
							+ (fwn[fo + 11] * (c * dy * dx) + fwn[fo + 12] * (fourPi_len3 + c * dy * dy) + fwn[fo + 13] * (c * dy * dz))
							+ (fwn[fo + 14] * (c * dz * dx) + fwn[fo + 15] * (c * dz * dy) + fwn[fo + 16] * (fourPi_len3 + c * dz * dz))
						branch_sum += order1v + order2v
					else
						-- FastTriWinding.EvaluateOrder1Approx(center, order1, q)
						branch_sum += (1.0 / FourPI) * (fwn[fo + 5] * (dx / len3) + fwn[fo + 6] * (dy / len3) + fwn[fo + 7] * (dz / len3))
					end
				end
			end
			if not used_cache then
				branch_sum += branch(ci, px, py, pz)
			end
		end
		return branch_sum
	end

	return function(iBox: number, px: number, py: number, pz: number, fBeta: number, bOrder2: boolean): number
		beta, order2 = fBeta, bOrder2
		return branch(iBox, px, py, pz)
	end
end

local function ensure_fast_winding_cache(self)
	if self.FastWindingCache == nil or self.fast_winding_cache_timestamp ~= self.mesh:ShapeTimestamp() then
		self:build_fast_winding_cache()
		self.fast_winding_cache_timestamp = self.mesh:ShapeTimestamp()
		self.fwn_branch = make_fwn_branch(self)
	end
end

-- Fast approximation of winding number using far-field approximations
function DMeshAABBTree3:FastWindingNumber(p): number
	check_valid(self, "FastWindingNumber")
	ensure_fast_winding_cache(self)
	return self.fwn_branch(self.root_index, p.x, p.y, p.z, self.FWNBeta, self.FWNApproxOrder == 2)
end

-- allocation-free variant of FastWindingNumber
function DMeshAABBTree3:FastWindingNumberXYZ(x: number, y: number, z: number): number
	check_valid(self, "FastWindingNumber")
	ensure_fast_winding_cache(self)
	return self.fwn_branch(self.root_index, x, y, z, self.FWNBeta, self.FWNApproxOrder == 2)
end

-- C# branch_fast_winding_num(iBox, p) (evaluation helper, normally only used internally)
function DMeshAABBTree3:branch_fast_winding_num(iBox: number, p): number
	ensure_fast_winding_cache(self)
	return self.fwn_branch(iBox, p.x, p.y, p.z, self.FWNBeta, self.FWNApproxOrder == 2)
end

function DMeshAABBTree3:build_fast_winding_cache()
	-- set this to a larger number to ignore caches if number of triangles is too small.
	-- (seems to be no benefit to doing this...is holdover from tree-decomposition FWN code)
	local WINDING_CACHE_THRESH = 1

	local triCache = MeshTriInfoCache.new(self.mesh)

	-- FWNInfo { Center, R, Order1Vec, Order2Mat } per box, stored flat: coeffs[16*iBox + 1 .. 16*iBox + 16]
	-- (layout of FastTriWinding.ComputeCoeffsFlat); has[iBox + 1] == true if box has a cache
	-- (arrays are pre-sized so that they live in the table array part: boxes are filled in post-order)
	local nBoxes = #self.box_to_index
	self.FastWindingCache = { coeffs = table.create(16 * nBoxes, 0), has = table.create(nBoxes, false) }
	self:build_fast_winding_cache_rec(self.root_index, 0, WINDING_CACHE_THRESH, triCache)
end

-- C# build_fast_winding_cache(iBox, depth, tri_count_thresh, out tri_hash, triCache) -> returns count, tri_hash
function DMeshAABBTree3:build_fast_winding_cache_rec(iBox: number, depth: number, tri_count_thresh: number, triCache): (number, any)
	local tri_hash = nil
	local ilist = self.index_list
	local idx = self.box_to_index[iBox + 1]
	if idx < self.triangles_end then -- triange-list case, array is [N t1 t2 ... tN]
		local num_tris = ilist[idx + 1]
		return num_tris, nil
	else -- internal node, either 1 or 2 child boxes
		local iChild1 = ilist[idx + 1]
		if iChild1 < 0 then -- 1 child
			iChild1 = -iChild1 - 1
			local num_child_tris
			num_child_tris, tri_hash = self:build_fast_winding_cache_rec(iChild1, depth + 1, tri_count_thresh, triCache)
			-- if count in child is large enough, we already built a cache at lower node
			return num_child_tris, tri_hash
		else -- 2 children
			iChild1 = iChild1 - 1
			local iChild2 = ilist[idx + 2] - 1

			-- let each child build its own cache if it wants. If so, it will return the
			-- list of its child tris
			local num_tris_1, num_tris_2, child2_hash
			num_tris_1, tri_hash = self:build_fast_winding_cache_rec(iChild1, depth + 1, tri_count_thresh, triCache)
			num_tris_2, child2_hash = self:build_fast_winding_cache_rec(iChild2, depth + 1, tri_count_thresh, triCache)
			local build_cache = (num_tris_1 + num_tris_2 > tri_count_thresh)

			if depth == 0 then
				return num_tris_1 + num_tris_2, tri_hash -- cannot build cache at level 0...
			end

			-- collect up the triangles we need. there are various cases depending on what children already did
			if tri_hash ~= nil or child2_hash ~= nil or build_cache then
				if tri_hash == nil and child2_hash ~= nil then
					self:collect_triangles(iChild1, child2_hash)
					tri_hash = child2_hash
				else
					if tri_hash == nil then
						tri_hash = oset_new()
						self:collect_triangles(iChild1, tri_hash)
					end
					if child2_hash == nil then
						self:collect_triangles(iChild2, tri_hash)
					else
						oset_union(tri_hash, child2_hash)
					end
				end
			end
			if build_cache then
				self:make_box_fast_winding_cache(iBox, tri_hash, triCache)
			end

			return num_tris_1 + num_tris_2, tri_hash
		end
	end
end

-- check if we can use fwn
function DMeshAABBTree3:can_use_fast_winding_cache(iBox: number, q): boolean
	local cache = self.FastWindingCache
	if not cache.has[iBox + 1] then
		return false
	end
	local fo = 16 * iBox
	local c = cache.coeffs
	local dx, dy, dz = q.x - c[fo + 1], q.y - c[fo + 2], q.z - c[fo + 3]
	local dist_qp = sqrt(dx * dx + dy * dy + dz * dz)
	if dist_qp > self.FWNBeta * c[fo + 4] then
		return true
	end
	return false
end

-- compute FWN cache for all triangles underneath this box
function DMeshAABBTree3:make_box_fast_winding_cache(iBox: number, triangles, triCache)
	local cache = self.FastWindingCache
	ComputeCoeffsFlat(self.mesh, triangles.list, triangles.n, triCache, cache.coeffs, 16 * iBox)
	cache.has[iBox + 1] = true
end

-- evaluate the FWN cache for iBox
function DMeshAABBTree3:evaluate_box_fast_winding_cache(iBox: number, q): number
	local FastTriWinding = FastWindingMath.FastTriWinding
	local c = self.FastWindingCache.coeffs
	if self.FWNApproxOrder == 2 then
		return FastTriWinding.EvaluateOrder2ApproxFlat(c, 16 * iBox, q.x, q.y, q.z)
	else
		return FastTriWinding.EvaluateOrder1ApproxFlat(c, 16 * iBox, q.x, q.y, q.z)
	end
end

---------------------------------------------------------------------------------------------------
-- Internals - data structures, construction, etc

-- compute box_bounds (box_eps-expanded min/max) from box_centers/box_extents
function DMeshAABBTree3:finalize_boxes()
	local bc, be = self.box_centers, self.box_extents
	local nBoxes = #self.box_to_index
	local bb = table.create(6 * nBoxes, 0)
	for i = 0, nBoxes - 1 do
		local k = 3 * i
		local cx, cy, cz = bc[k + 1], bc[k + 2], bc[k + 3]
		-- new AxisAlignedBox3d(ref c, e.x + box_eps, e.y + box_eps, e.z + box_eps)
		local hx, hy, hz = be[k + 1] + box_eps, be[k + 2] + box_eps, be[k + 3] + box_eps
		local o = 6 * i
		bb[o + 1], bb[o + 2], bb[o + 3] = cx - hx, cy - hy, cz - hz
		bb[o + 4], bb[o + 5], bb[o + 6] = cx + hx, cy + hy, cz + hz
	end
	self.box_bounds = bb
end

local function new_boxes_set()
	return {
		box_to_index = {},
		box_centers = {},
		box_extents = {},
		index_list = {},
		iBoxCur = 0,
		iIndicesCur = 0,
	}
end

-- store box.Center / box.Extents for box iBox (box given by min/max)
local function set_box_center_extents(centers, extents, iBox: number, minx, miny, minz, maxx, maxy, maxz)
	local k = 3 * iBox
	centers[k + 1], centers[k + 2], centers[k + 3] = 0.5 * (minx + maxx), 0.5 * (miny + maxy), 0.5 * (minz + maxz)
	extents[k + 1], extents[k + 2], extents[k + 3] = (maxx - minx) * 0.5, (maxy - miny) * 0.5, (maxz - minz) * 0.5
end

-- box.Contain(mesh.GetTriBounds(tid))
local function contain_tri_bounds(mtris, mverts, tid: number, minx, miny, minz, maxx, maxy, maxz)
	for j = 1, 3 do
		local vi = 3 * mtris[3 * tid + j]
		local x, y, z = mverts[vi + 1], mverts[vi + 2], mverts[vi + 3]
		if x < minx then
			minx = x
		end
		if x > maxx then
			maxx = x
		end
		if y < miny then
			miny = y
		end
		if y > maxy then
			maxy = y
		end
		if z < minz then
			minz = z
		end
		if z > maxz then
			maxz = z
		end
	end
	return minx, miny, minz, maxx, maxy, maxz
end

-- append a triangle-list leaf box for triangles[iStart+1 .. iStart+iCount]; returns -(iBox+1), box min/max
local function append_leaf_box(self, ctx, iStart: number, iCount: number, tris)
	local mtris, mverts = ctx.mtris, ctx.mverts
	local triangles = ctx.triangles
	-- box = AxisAlignedBox3d.Empty
	local minx, miny, minz = MaxFloat, MaxFloat, MaxFloat
	local maxx, maxy, maxz = -MaxFloat, -MaxFloat, -MaxFloat

	local iBox = tris.iBoxCur
	tris.iBoxCur += 1
	tris.box_to_index[iBox + 1] = tris.iIndicesCur

	local il = tris.index_list
	il[tris.iIndicesCur + 1] = iCount
	tris.iIndicesCur += 1
	for i = 0, iCount - 1 do
		local tid = triangles[iStart + i + 1]
		il[tris.iIndicesCur + 1] = tid
		tris.iIndicesCur += 1
		minx, miny, minz, maxx, maxy, maxz = contain_tri_bounds(mtris, mverts, tid, minx, miny, minz, maxx, maxy, maxz)
	end

	set_box_center_extents(tris.box_centers, tris.box_extents, iBox, minx, miny, minz, maxx, maxy, maxz)
	return -(iBox + 1), minx, miny, minz, maxx, maxy, maxz
end

-- append internal node box with two children; returns iBox
local function append_node_box(nodes, child0: number, child1: number, minx, miny, minz, maxx, maxy, maxz): number
	local iBox = nodes.iBoxCur
	nodes.iBoxCur += 1
	nodes.box_to_index[iBox + 1] = nodes.iIndicesCur

	nodes.index_list[nodes.iIndicesCur + 1] = child0
	nodes.iIndicesCur += 1
	nodes.index_list[nodes.iIndicesCur + 1] = child1
	nodes.iIndicesCur += 1

	set_box_center_extents(nodes.box_centers, nodes.box_extents, iBox, minx, miny, minz, maxx, maxy, maxz)
	return iBox
end

local function split_tri_set_sorted(self, ctx, iStart: number, iCount: number, depth: number, minTriCount: number, tris, nodes)
	if iCount < minTriCount then
		-- append new triangles box
		return append_leaf_box(self, ctx, iStart, iCount, tris)
	end

	-- AxisComp c = new AxisComp() { Axis = depth % 3 };  Array.Sort(centers, triangles, iStart, iCount, c);
	local axis = depth % 3
	local keys = (axis == 0) and ctx.cx or ((axis == 1) and ctx.cy or ctx.cz)
	local triangles, cx, cy, cz = ctx.triangles, ctx.cx, ctx.cy, ctx.cz
	local perm = table.create(iCount, 0)
	for i = 1, iCount do
		perm[i] = iStart + i
	end
	table.sort(perm, function(a, b)
		return keys[a] < keys[b]
	end)
	local t2, x2, y2, z2 = table.create(iCount, 0), table.create(iCount, 0), table.create(iCount, 0), table.create(iCount, 0)
	for i = 1, iCount do
		local k = perm[i]
		t2[i], x2[i], y2[i], z2[i] = triangles[k], cx[k], cy[k], cz[k]
	end
	for i = 1, iCount do
		local k = iStart + i
		triangles[k], cx[k], cy[k], cz[k] = t2[i], x2[i], y2[i], z2[i]
	end

	local mid = iCount // 2
	local n0 = mid
	local n1 = iCount - mid

	-- create child boxes
	local child0, minx, miny, minz, maxx, maxy, maxz = split_tri_set_sorted(self, ctx, iStart, n0, depth + 1, minTriCount, tris, nodes)
	local child1, minx1, miny1, minz1, maxx1, maxy1, maxz1 =
		split_tri_set_sorted(self, ctx, iStart + mid, n1, depth + 1, minTriCount, tris, nodes)
	-- box.Contain(box1)
	minx, miny, minz = min(minx, minx1), min(miny, miny1), min(minz, minz1)
	maxx, maxy, maxz = max(maxx, maxx1), max(maxy, maxy1), max(maxz, maxz1)

	-- append new box
	local iBox = append_node_box(nodes, child0, child1, minx, miny, minz, maxx, maxy, maxz)
	return iBox, minx, miny, minz, maxx, maxy, maxz
end

local function split_tri_set_midpoint(self, ctx, iStart: number, iCount: number, depth: number, minTriCount: number, tris, nodes)
	if iCount < minTriCount then
		-- append new triangles box
		return append_leaf_box(self, ctx, iStart, iCount, tris)
	end

	-- compute interval along an axis and find midpoint
	local axis = depth % 3
	local triangles, cx, cy, cz = ctx.triangles, ctx.cx, ctx.cy, ctx.cz
	local keys = (axis == 0) and cx or ((axis == 1) and cy or cz)
	-- Interval1d interval = Interval1d.Empty; interval.Contain(...)
	local ia, ib = MaxDouble, -MaxDouble
	for i = 1, iCount do
		local d = keys[iStart + i]
		if d < ia then
			ia = d
		end
		if d > ib then
			ib = d
		end
	end
	local midpoint = (ib + ia) * 0.5

	local n0, n1
	if abs(ia - ib) > ZeroTolerance then
		-- we have to re-sort the centers & triangles lists so that centers < midpoint
		-- are first, so that we can recurse on the two subsets. We walk in from each side,
		-- until we find two out-of-order locations, then we swap them.
		local l = 0
		local r = iCount - 1
		while l < r do
			-- [RMS] is <= right here? if v.axis == midpoint, then this loop
			--   can get stuck unless one of these has an equality test. But
			--   I did not think enough about if this is the right thing to do...
			while keys[iStart + l + 1] <= midpoint do
				l += 1
			end
			while keys[iStart + r + 1] > midpoint do
				r -= 1
			end
			if l >= r then
				break --done!
			end
			--swap
			local li, ri = iStart + l + 1, iStart + r + 1
			cx[li], cx[ri] = cx[ri], cx[li]
			cy[li], cy[ri] = cy[ri], cy[li]
			cz[li], cz[ri] = cz[ri], cz[li]
			triangles[li], triangles[ri] = triangles[ri], triangles[li]
		end

		n0 = l
		n1 = iCount - n0
	else
		-- interval is near-empty, so no point trying to do sorting, just split half and half
		n0 = iCount // 2
		n1 = iCount - n0
	end

	-- create child boxes
	local child0, minx, miny, minz, maxx, maxy, maxz = split_tri_set_midpoint(self, ctx, iStart, n0, depth + 1, minTriCount, tris, nodes)
	local child1, minx1, miny1, minz1, maxx1, maxy1, maxz1 =
		split_tri_set_midpoint(self, ctx, iStart + n0, n1, depth + 1, minTriCount, tris, nodes)
	-- box.Contain(box1)
	minx, miny, minz = min(minx, minx1), min(miny, miny1), min(minz, minz1)
	maxx, maxy, maxz = max(maxx, maxx1), max(maxy, maxy1), max(maxz, maxz1)

	-- append new box
	local iBox = append_node_box(nodes, child0, child1, minx, miny, minz, maxx, maxy, maxz)
	return iBox, minx, miny, minz, maxx, maxy, maxz
end

function DMeshAABBTree3:build_top_down(bSorted: boolean)
	local mesh = self.mesh
	local mtris, mverts = mesh.triangles.data, mesh.vertices.data

	-- build list of valid triangles & centers. We skip any
	-- triangles that have infinite/garbage vertices...
	local NT = mesh:TriangleCount()
	local i = 0
	local triangles = table.create(NT, 0)
	local cx, cy, cz = table.create(NT, 0), table.create(NT, 0), table.create(NT, 0)
	local f = 1.0 / 3.0
	for ti in mesh:TriangleIndices() do
		-- centroid = mesh.GetTriCentroid(ti)
		local t3 = 3 * ti
		local a, b, c = 3 * mtris[t3 + 1], 3 * mtris[t3 + 2], 3 * mtris[t3 + 3]
		local x = (mverts[a + 1] + mverts[b + 1] + mverts[c + 1]) * f
		local y = (mverts[a + 2] + mverts[b + 2] + mverts[c + 2]) * f
		local z = (mverts[a + 3] + mverts[b + 3] + mverts[c + 3]) * f
		local d2 = x * x + y * y + z * z
		local bInvalid = (d2 ~= d2) or d2 == math.huge or d2 == -math.huge
		if bInvalid == false then
			i += 1
			triangles[i] = ti
			cx[i], cy[i], cz[i] = x, y, z
		end -- otherwise skip this tri
	end
	-- [note] C# passes mesh.TriangleCount as the count here, which is only correct if no triangles were
	-- skipped; we pass the number of valid triangles.
	local nValid = i

	local ctx = { triangles = triangles, cx = cx, cy = cy, cz = cz, mtris = mtris, mverts = mverts }
	local tris = new_boxes_set()
	local nodes = new_boxes_set()
	local rootnode
	if bSorted then
		rootnode = split_tri_set_sorted(self, ctx, 0, nValid, 0, self.TopDownLeafMaxTriCount, tris, nodes)
	else
		rootnode = split_tri_set_midpoint(self, ctx, 0, nValid, 0, self.TopDownLeafMaxTriCount, tris, nodes)
	end

	self.box_to_index = tris.box_to_index
	self.box_centers = tris.box_centers
	self.box_extents = tris.box_extents
	self.index_list = tris.index_list
	self.triangles_end = tris.iIndicesCur
	local iIndexShift = self.triangles_end
	local iBoxShift = tris.iBoxCur

	local box_to_index, box_centers, box_extents, index_list = self.box_to_index, self.box_centers, self.box_extents, self.index_list

	-- ok now append internal node boxes & index ptrs
	local nbc, nbe = nodes.box_centers, nodes.box_extents
	for k = 0, nodes.iBoxCur - 1 do
		local dst, src = 3 * (iBoxShift + k), 3 * k
		box_centers[dst + 1], box_centers[dst + 2], box_centers[dst + 3] = nbc[src + 1], nbc[src + 2], nbc[src + 3]
		box_extents[dst + 1], box_extents[dst + 2], box_extents[dst + 3] = nbe[src + 1], nbe[src + 2], nbe[src + 3]
		-- internal node indices are shifted
		box_to_index[iBoxShift + k + 1] = iIndexShift + nodes.box_to_index[k + 1]
	end

	-- now append index list
	for k = 0, nodes.iIndicesCur - 1 do
		local child_box = nodes.index_list[k + 1]
		if child_box < 0 then -- this is a triangles box
			child_box = -child_box - 1
		else
			child_box += iBoxShift
		end
		child_box = child_box + 1
		index_list[iIndexShift + k + 1] = child_box
	end

	self.root_index = rootnode + iBoxShift
end

---------------------------------------------------------------------------------------------------
-- bottom-up construction

-- construct box that contains two boxes. returns center xyz, extent xyz
function DMeshAABBTree3:get_combined_box(b0: number, b1: number)
	local bc, be = self.box_centers, self.box_extents
	local k0, k1 = 3 * b0, 3 * b1
	local c0x, c0y, c0z, e0x, e0y, e0z = bc[k0 + 1], bc[k0 + 2], bc[k0 + 3], be[k0 + 1], be[k0 + 2], be[k0 + 3]
	local c1x, c1y, c1z, e1x, e1y, e1z = bc[k1 + 1], bc[k1 + 2], bc[k1 + 3], be[k1 + 1], be[k1 + 2], be[k1 + 3]
	local minx = min(c0x - e0x, c1x - e1x)
	local maxx = max(c0x + e0x, c1x + e1x)
	local miny = min(c0y - e0y, c1y - e1y)
	local maxy = max(c0y + e0y, c1y + e1y)
	local minz = min(c0z - e0z, c1z - e1z)
	local maxz = max(c0z + e0z, c1z + e1z)
	return 0.5 * (minx + maxx), 0.5 * (miny + maxy), 0.5 * (minz + maxz), 0.5 * (maxx - minx), 0.5 * (maxy - miny), 0.5 * (maxz - minz)
end

function DMeshAABBTree3:combined_box_volume(b0: number, b1: number): number
	local bc, be = self.box_centers, self.box_extents
	local k0, k1 = 3 * b0, 3 * b1
	local minx = min(bc[k0 + 1] - be[k0 + 1], bc[k1 + 1] - be[k1 + 1])
	local maxx = max(bc[k0 + 1] + be[k0 + 1], bc[k1 + 1] + be[k1 + 1])
	local miny = min(bc[k0 + 2] - be[k0 + 2], bc[k1 + 2] - be[k1 + 2])
	local maxy = max(bc[k0 + 2] + be[k0 + 2], bc[k1 + 2] + be[k1 + 2])
	local minz = min(bc[k0 + 3] - be[k0 + 3], bc[k1 + 3] - be[k1 + 3])
	local maxz = max(bc[k0 + 3] + be[k0 + 3], bc[k1 + 3] + be[k1 + 3])
	return (maxx - minx) * (maxy - miny) * (maxz - minz)
end

function DMeshAABBTree3:combined_box_length(b0: number, b1: number): number
	local bc, be = self.box_centers, self.box_extents
	local k0, k1 = 3 * b0, 3 * b1
	local minx = min(bc[k0 + 1] - be[k0 + 1], bc[k1 + 1] - be[k1 + 1])
	local maxx = max(bc[k0 + 1] + be[k0 + 1], bc[k1 + 1] + be[k1 + 1])
	local miny = min(bc[k0 + 2] - be[k0 + 2], bc[k1 + 2] - be[k1 + 2])
	local maxy = max(bc[k0 + 2] + be[k0 + 2], bc[k1 + 2] + be[k1 + 2])
	local minz = min(bc[k0 + 3] - be[k0 + 3], bc[k1 + 3] - be[k1 + 3])
	local maxz = max(bc[k0 + 3] + be[k0 + 3], bc[k1 + 3] + be[k1 + 3])
	return (maxx - minx) * (maxx - minx) + (maxy - miny) * (maxy - miny) + (maxz - minz) * (maxz - minz)
end

-- append a 2-child box combining boxes i0 and i1 (st = { iBoxCur, iIndicesCur } replaces C# ref ints)
local function append_pair_box(self, st, i0: number, i1: number)
	local cx, cy, cz, ex, ey, ez = self:get_combined_box(i0, i1)
	local iBox = st.iBoxCur
	st.iBoxCur += 1
	self.box_to_index[iBox + 1] = st.iIndicesCur
	self.index_list[st.iIndicesCur + 1] = i0 + 1
	st.iIndicesCur += 1
	self.index_list[st.iIndicesCur + 1] = i1 + 1
	st.iIndicesCur += 1
	local k = 3 * iBox
	local bc, be = self.box_centers, self.box_extents
	bc[k + 1], bc[k + 2], bc[k + 3] = cx, cy, cz
	be[k + 1], be[k + 2], be[k + 3] = ex, ey, ez
end

function DMeshAABBTree3:duplicate_box(i: number, st)
	-- duplicate box at this level... ?
	local iBox = st.iBoxCur
	st.iBoxCur += 1
	self.box_to_index[iBox + 1] = st.iIndicesCur

	-- negative index means only one child
	self.index_list[st.iIndicesCur + 1] = -(i + 1)
	st.iIndicesCur += 1

	local bc, be = self.box_centers, self.box_extents
	local k, ki = 3 * iBox, 3 * i
	bc[k + 1], bc[k + 2], bc[k + 3] = bc[ki + 1], bc[ki + 2], bc[ki + 3]
	be[k + 1], be[k + 2], be[k + 3] = be[ki + 1], be[ki + 2], be[ki + 3]
end

-- Appends a box that contains free triangles in one-ring of vertex vid.
-- If tri count is < spill threshold, push onto spill list instead.
-- Returns # of free tris found.
function DMeshAABBTree3:add_one_ring_box(
	vid: number,
	used_triangles: { number },
	temp_tris: { number },
	st,
	spill: { number }?,
	nSpillThresh: number
): number
	local mesh = self.mesh
	-- collect free triangles
	local num_free = 0
	for tid in mesh:VtxTrianglesItr(vid) do
		if used_triangles[tid + 1] == 0 then
			num_free += 1
			temp_tris[num_free] = tid
		end
	end

	-- none free, get out
	if num_free == 0 then
		return 0
	end

	-- if we only had a couple free triangles, wait and see if
	-- they get picked up by another vert
	if num_free < nSpillThresh then
		table.insert(spill :: { number }, vid)
		return num_free
	end

	-- append new box
	local mtris, mverts = mesh.triangles.data, mesh.vertices.data
	local minx, miny, minz = MaxFloat, MaxFloat, MaxFloat
	local maxx, maxy, maxz = -MaxFloat, -MaxFloat, -MaxFloat
	local iBox = st.iBoxCur
	st.iBoxCur += 1
	self.box_to_index[iBox + 1] = st.iIndicesCur

	self.index_list[st.iIndicesCur + 1] = num_free
	st.iIndicesCur += 1
	for i = 1, num_free do
		local tid = temp_tris[i]
		self.index_list[st.iIndicesCur + 1] = tid
		st.iIndicesCur += 1
		used_triangles[tid + 1] += 1 -- incrementing for sanity check below, just need to set to 1
		minx, miny, minz, maxx, maxy, maxz = contain_tri_bounds(mtris, mverts, tid, minx, miny, minz, maxx, maxy, maxz)
	end

	set_box_center_extents(self.box_centers, self.box_extents, iBox, minx, miny, minz, maxx, maxy, maxz)
	return num_free
end

-- sort box indices by min coordinate along x axis
local function sort_indices_by_axis_min(self, indices: { number })
	local bc, be = self.box_centers, self.box_extents
	local nDim = 0
	local keys = {}
	for _, b in ipairs(indices) do
		keys[b] = bc[3 * b + nDim + 1] - be[3 * b + nDim + 1]
	end
	table.sort(indices, function(a, b)
		return keys[a] < keys[b]
	end)
end

-- Turn a span of N boxes into N/2 boxes, by pairing boxes
-- Except, of course, if N is odd, then we get N/2+1, where the +1
-- box has a single child box (ie just a copy).
-- [TODO] instead merge that extra box into on of parents? Reduces tree depth by 1
function DMeshAABBTree3:cluster_boxes(iStart: number, iCount: number, st): number
	local indices = table.create(iCount, 0)
	for i = 0, iCount - 1 do
		indices[i + 1] = iStart + i
	end
	sort_indices_by_axis_min(self, indices)

	local nPairs = iCount // 2
	local nLeft = iCount - 2 * nPairs

	-- this is dumb! but lets us test the rest...
	for pi = 0, nPairs - 1 do
		local i0 = indices[2 * pi + 1]
		local i1 = indices[2 * pi + 2]
		append_pair_box(self, st, i0, i1)
	end

	-- [todo] could we merge with last other box? need a way to tell
	--   that there are 3 children though...could use negative index for that?
	if nLeft > 0 then
		if nLeft > 1 then
			error("DMeshAABBTree3.cluster_boxes: nLeft > 1")
		end
		local iLeft = indices[2 * nPairs + 1]
		self:duplicate_box(iLeft, st)
	end

	return nPairs + nLeft
end

-- Turn a span of N boxes into N/2 boxes, by pairing boxes
-- Except, of course, if N is odd, then we get N/2+1, where the +1
-- box has a single child box (ie just a copy).
-- [TODO] instead merge that extra box into on of parents? Reduces tree depth by 1
function DMeshAABBTree3:cluster_boxes_nearsearch(iStart: number, iCount: number, st): number
	local indices = table.create(iCount, 0)
	for i = 0, iCount - 1 do
		indices[i + 1] = iStart + i
	end

	-- sort indices by x axis
	-- cycling axes (ie at each depth) seems to produce much worse results...
	sort_indices_by_axis_min(self, indices)

	local nPairs = iCount // 2
	local nLeft = iCount - 2 * nPairs

	-- bounded greedy clustering.
	-- Search ahead next N boxes in sorted-by-axis list, and find
	-- the one that creates minimal box metric when combined with us.
	local N = self.BottomUpClusterLookahead
	local nextNi = table.create(N, 0)
	local nextNc = table.create(N, 0)
	for pi = 0, iCount - 2 do
		local i0 = indices[pi + 1]
		if i0 < 0 then
			continue
		end
		local nStop = min(N, iCount - pi - 1)
		for k = 1, nStop do
			local pj = pi + k
			nextNi[k] = pj
			local ik = indices[pj + 1]
			if ik < 0 then
				nextNc[k] = MaxDouble
			else
				nextNc[k] = self:combined_box_volume(i0, ik)
			end
		end
		-- Array.Sort(nextNc, nextNi, 0, nStop)  (insertion sort, as .NET uses for small arrays)
		for k = 2, nStop do
			local c, ni = nextNc[k], nextNi[k]
			local j = k - 1
			while j >= 1 and nextNc[j] > c do
				nextNc[j + 1] = nextNc[j]
				nextNi[j + 1] = nextNi[j]
				j -= 1
			end
			nextNc[j + 1] = c
			nextNi[j + 1] = ni
		end
		if nextNc[1] == MaxDouble then
			continue
		end

		local pj = nextNi[1]
		local i1 = indices[pj + 1]
		if i1 < 0 then
			error("DMeshAABBTree3.cluster_boxes_nearsearch: invalid pair")
		end

		append_pair_box(self, st, i0, i1)

		indices[pi + 1] = -(indices[pi + 1] + 1)
		indices[pj + 1] = -(indices[pj + 1] + 1)
	end

	-- [todo] could we merge with last other box? need a way to tell
	--   that there are 3 children though...could use negative index for that?
	if nLeft > 0 then
		local iLeft = -1
		local i = 1
		while iLeft < 0 and i <= #indices do
			if indices[i] >= 0 then
				iLeft = indices[i]
			end
			i += 1
		end
		self:duplicate_box(iLeft, st)
	end

	return nPairs + nLeft
end

-- C# find_smallest_upper(double[,] m, ref ii, ref jj) -> returns v, ii, jj. m is flat row-major n x n
local function find_smallest_upper(m: { number }, n: number, ii: number, jj: number): (number, number, number)
	local v = MaxDouble
	for i = 0, n - 1 do
		for j = i + 1, n - 1 do
			local mij = m[i * n + j + 1]
			if mij < v then
				v = mij
				ii = i
				jj = j
			end
		end
	end
	return v, ii, jj
end

-- greedy-optimal clustering of boxes. Compute all-pairs distance matrix, and
-- then incrementally pull out minimal pairs. This does a great job but it
-- gets insanely slow because the all-pairs matrix takes too long...
function DMeshAABBTree3:cluster_boxes_matrix(iStart: number, iCount: number, st): number
	local indices = table.create(iCount, 0)
	for i = 0, iCount - 1 do
		indices[i + 1] = iStart + i
	end

	local matrix = table.create(iCount * iCount, 0)
	for i = 0, iCount - 1 do
		for j = 0, i do
			matrix[i * iCount + j + 1] = MaxDouble
		end
		for j = i + 1, iCount - 1 do
			matrix[i * iCount + j + 1] = self:combined_box_volume(indices[i + 1], indices[j + 1])
		end
	end

	local nPairs = iCount // 2
	local nLeft = iCount - 2 * nPairs

	for _ = 0, nPairs - 1 do
		local si, sj = 0, 0
		local bFound = false
		while not bFound do
			local _v
			_v, si, sj = find_smallest_upper(matrix, iCount, si, sj)
			if indices[si + 1] >= 0 and indices[sj + 1] >= 0 then
				bFound = true
			end
			matrix[si * iCount + sj + 1] = MaxDouble
		end

		local i0 = indices[si + 1]
		local i1 = indices[sj + 1]
		append_pair_box(self, st, i0, i1)

		indices[si + 1] = -(indices[si + 1] + 1)
		indices[sj + 1] = -(indices[sj + 1] + 1)
	end

	-- [todo] could we merge with last other box? need a way to tell
	--   that there are 3 children though...could use negative index for that?
	if nLeft > 0 then
		local iLeft = -1
		local i = 1
		while iLeft < 0 and i <= #indices do
			if indices[i] >= 0 then
				iLeft = indices[i]
			end
			i += 1
		end
		self:duplicate_box(iLeft, st)
	end

	return nPairs + nLeft
end

-- strategy here is:
--  1) partition triangles by vertex one-rings into leaf boxes
--      1a) first pass where we skip one-rings that have < 3 free tris
--      1b) second pass where we handle any missed tris
--  2) sequentially combine N leaf boxes into (N/2 + N%2) layer 2 boxes
--  3) repeat until layer K has only 1 box, which is root of tree
function DMeshAABBTree3:build_by_one_rings(ePolicy: number)
	local mesh = self.mesh
	self.box_to_index = {}
	self.box_centers = {}
	self.box_extents = {}
	self.index_list = {}
	local st = { iBoxCur = 0, iIndicesCur = 0 }

	-- replace w/ BitArray ?
	local used_triangles = table.create(mesh:MaxTriangleID(), 0)

	-- temporary buffer
	local nMaxEdgeCount = mesh:GetMaxVtxEdgeCount()
	local temp_tris = table.create(2 * nMaxEdgeCount, 0)

	-- first pass: cluster by one-ring, but if # of free tris
	--  in a ring is small (< 3), push onto spill list to try again,
	--  because those tris might be picked up by a bigger cluster
	local spill = {}
	for vid in mesh:VertexIndices() do
		local tCount = self:add_one_ring_box(vid, used_triangles, temp_tris, st, spill, 3)
		if tCount < 3 then
			table.insert(spill, vid)
		end
	end

	-- second pass: check any spill vertices. Most are probably gone
	-- now, but a few stray triangles might still exist
	local N = #spill
	for si = 1, N do
		local vid = spill[si]
		self:add_one_ring_box(vid, used_triangles, temp_tris, st, nil, 0)
	end

	-- keep track of where triangle lists end
	self.triangles_end = st.iIndicesCur

	-- this defines ClusterPolicy.Default
	local clusterF = DMeshAABBTree3.cluster_boxes_nearsearch
	if ePolicy == ClusterPolicy.Fastest then
		clusterF = DMeshAABBTree3.cluster_boxes
	elseif ePolicy == ClusterPolicy.MinimalVolume then
		clusterF = DMeshAABBTree3.cluster_boxes_matrix
	elseif ePolicy == ClusterPolicy.FastVolumeMetric then
		clusterF = DMeshAABBTree3.cluster_boxes_nearsearch
	end

	-- ok, now repeatedly cluster current layer of N boxes into N/2 + N%2 boxes,
	-- until we hit a 1-box layer, which is root of the tree
	local nPrevEnd = st.iBoxCur
	local nLayerSize = clusterF(self, 0, st.iBoxCur, st)
	local iStart = nPrevEnd
	local iCount = st.iBoxCur - nPrevEnd
	while nLayerSize > 1 do
		nPrevEnd = st.iBoxCur
		nLayerSize = clusterF(self, iStart, iCount, st)
		iStart = nPrevEnd
		iCount = st.iBoxCur - nPrevEnd
	end

	self.root_index = st.iBoxCur - 1
end

-- tree-vs-tree queries, traversal and debugging functions
require(script.Parent.DMeshAABBTree3Queries)(DMeshAABBTree3)

return DMeshAABBTree3
