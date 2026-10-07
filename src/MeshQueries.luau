--!native
-- Port of geometry3Sharp queries/MeshQueries.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau API notes (static functions, call with '.'):
--   TriangleDistance(mesh, ti, point) -> DistPoint3Triangle3 (or nil if not a triangle)
--   NearestPointFrame(mesh, spatial, queryPoint, bForceFaceNormal?) -> Frame3d
--   NearestPointDistance(mesh, spatial, queryPoint, maxDist?) -> number
--   TriangleTriangleDistance(mesh1, ti, mesh2, tj, TransformF?) -> DistTriangle3Triangle3 (Compute()d)
--   TriangleIntersection(mesh, ti, ray) -> IntrRay3Triangle3 (Find()-ed)
--   TrianglesIntersection(mesh1, ti, mesh2, tj, TransformF?) -> IntrTriangle3Triangle3 (Find()-ed)
--   TrianglesDistance(mesh1, ti, mesh2, tj, TransformF?) -> DistTriangle3Triangle3
--   RayHitPointFrame(mesh, spatial, ray, bForceFaceNormal?) -> bool, Frame3d   (C# out hitPosFrame)
--   SurfaceFrame(mesh, tID, point, bForceFaceNormal?) -> Frame3d
--   BaryCoords(mesh, tID, point) -> Vector3d
--   TriDistanceSqr(mesh, ti, point) -> number
--   TriDistanceSqrXYZ(mesh, ti, px, py, pz) -> number             (allocation-free variant)
--   FindNearestVertex_LinearSearch(mesh, p), FindNearestTriangle_LinearSearch(mesh, p),
--   FindHitTriangle_LinearSearch(mesh, ray)
--   FindIntersectingTriangles_LinearSearch(mesh1, mesh2) -> Index2i (array {ti, tj}, or Index2i.Max)
--   FindNearestTriangles_LinearSearch(mesh1, mesh2) -> Index2i, fNearest   (C# out fNearestSqr; like C#
--       the returned value is actually the (non-squared) distance)
--   EdgeLengthStats(mesh, samples?) -> minEdgeLen, maxEdgeLen, avgEdgeLen
--   EdgeLengthStatsFromEdges(mesh, EdgeItr, samples?) -> minEdgeLen, maxEdgeLen, avgEdgeLen
--       (EdgeItr is an iterator function or a 1-based array of edge ids)
-- `spatial` is any ISpatial (see SpatialInterfaces.luau), eg DMeshAABBTree3.

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local Triangle3d = require(script.Parent.Triangle3d)
local Frame3d = require(script.Parent.Frame3d)
local Index = require(script.Parent.Index)
local DistPoint3Triangle3 = require(script.Parent.DistPoint3Triangle3)
local IntrRay3Triangle3 = require(script.Parent.IntrRay3Triangle3)
local IntersectionResult = require(script.Parent.IntersectionResult)

local MaxDouble = MathUtil.MaxDouble
local distance_sqr_xyz = DistPoint3Triangle3.distance_sqr_xyz

-- lazily-required modules (avoid load-order dependencies)
local function DistTriangle3Triangle3()
	return require(script.Parent.DistTriangle3Triangle3) :: any
end
local function IntrTriangle3Triangle3()
	return require(script.Parent.IntrTriangle3Triangle3) :: any
end

local MeshQueries = {}

-- construct a DistPoint3Triangle3 object for a mesh triangle
function MeshQueries.TriangleDistance(mesh, ti: number, point)
	if not mesh:IsTriangle(ti) then
		return nil
	end
	local tri = mesh:GetTriVerticesTriangle(ti)
	local q = DistPoint3Triangle3.new(point, tri)
	q:GetSquared()
	return q
end

-- Find point-normal(Z) frame at closest point to queryPoint on mesh.
-- Returns interpolated vertex-normal frame if available, otherwise tri-normal frame.
function MeshQueries.NearestPointFrame(mesh, spatial, queryPoint, bForceFaceNormal: boolean?)
	local tid = spatial:FindNearestTriangle(queryPoint)
	local surfPt = MeshQueries.TriangleDistance(mesh, tid, queryPoint).TriangleClosest
	if mesh:HasVertexNormals() and bForceFaceNormal ~= true then
		return MeshQueries.SurfaceFrame(mesh, tid, surfPt)
	else
		return Frame3d.FromZ(surfPt, mesh:GetTriNormal(tid))
	end
end

-- Find distance from point to mesh
function MeshQueries.NearestPointDistance(mesh, spatial, queryPoint, maxDist: number?): number
	local tid = spatial:FindNearestTriangle(queryPoint, maxDist or MaxDouble)
	if tid == -1 then
		return MaxDouble
	end
	local tri = mesh:GetTriVerticesTriangle(tid)
	local dist_sqr = DistPoint3Triangle3.DistanceSqr(queryPoint, tri)
	return math.sqrt(dist_sqr)
end

local function get_tri_pair(mesh1, ti: number, mesh2, tj: number, TransformF)
	local tri1 = mesh1:GetTriVerticesTriangle(ti)
	local tri2 = mesh2:GetTriVerticesTriangle(tj)
	if TransformF ~= nil then
		tri2.V0 = TransformF(tri2.V0)
		tri2.V1 = TransformF(tri2.V1)
		tri2.V2 = TransformF(tri2.V2)
	end
	return tri1, tri2
end

-- find distance between two triangles, with optional transform on second triangle
function MeshQueries.TriangleTriangleDistance(mesh1, ti: number, mesh2, tj: number, TransformF: ((any) -> any)?)
	if mesh1:IsTriangle(ti) == false or mesh2:IsTriangle(tj) == false then
		return nil
	end
	local tri1, tri2 = get_tri_pair(mesh1, ti, mesh2, tj, TransformF)
	local dist = DistTriangle3Triangle3().new(tri1, tri2)
	dist:Compute()
	return dist
end

-- convenience function to construct a IntrRay3Triangle3 object for a mesh triangle
function MeshQueries.TriangleIntersection(mesh, ti: number, ray)
	if not mesh:IsTriangle(ti) then
		return nil
	end
	local tri = mesh:GetTriVerticesTriangle(ti)
	local q = IntrRay3Triangle3.new(ray, tri)
	q:Find()
	return q
end

-- convenience function to construct a IntrTriangle3Triangle3 object for two mesh triangles
function MeshQueries.TrianglesIntersection(mesh1, ti: number, mesh2, tj: number, TransformF: ((any) -> any)?)
	if mesh1:IsTriangle(ti) == false or mesh2:IsTriangle(tj) == false then
		return nil
	end
	local tri1, tri2 = get_tri_pair(mesh1, ti, mesh2, tj, TransformF)
	local intr = IntrTriangle3Triangle3().new(tri1, tri2)
	intr:Find()
	return intr
end

-- convenience function to construct a DistTriangle3Triangle3 object for two mesh triangles
function MeshQueries.TrianglesDistance(mesh1, ti: number, mesh2, tj: number, TransformF: ((any) -> any)?)
	if mesh1:IsTriangle(ti) == false or mesh2:IsTriangle(tj) == false then
		return nil
	end
	local tri1, tri2 = get_tri_pair(mesh1, ti, mesh2, tj, TransformF)
	local dist = DistTriangle3Triangle3().new(tri1, tri2)
	dist:GetSquared()
	return dist
end

-- Find point-normal frame at ray-intersection point on mesh, or return false if no hit.
-- Returns interpolated vertex-normal frame if available, otherwise tri-normal frame.
-- C# RayHitPointFrame(mesh, spatial, ray, out hitPosFrame, bForceFaceNormal) -> returns bool, hitPosFrame
function MeshQueries.RayHitPointFrame(mesh, spatial, ray, bForceFaceNormal: boolean?)
	local hitPosFrame = Frame3d.new()
	local tid = spatial:FindNearestHitTriangle(ray)
	if tid == -1 then
		return false, hitPosFrame
	end
	local isect = MeshQueries.TriangleIntersection(mesh, tid, ray)
	if isect.Result ~= IntersectionResult.Intersects then
		return false, hitPosFrame
	end
	local surfPt = ray:PointAt(isect.RayParameter)
	if mesh:HasVertexNormals() and bForceFaceNormal ~= true then
		hitPosFrame = MeshQueries.SurfaceFrame(mesh, tid, surfPt) -- TODO isect has bary-coords already!!
	else
		hitPosFrame = Frame3d.FromZ(surfPt, mesh:GetTriNormal(tid))
	end
	return true, hitPosFrame
end

-- Get point-normal frame on surface of mesh. Assumption is that point lies in tID.
-- returns interpolated vertex-normal frame if available, otherwise tri-normal frame.
function MeshQueries.SurfaceFrame(mesh, tID: number, point, bForceFaceNormal: boolean?)
	if not mesh:IsTriangle(tID) then
		error("MeshQueries.SurfaceFrame: triangle " .. tostring(tID) .. " does not exist!")
	end
	local tri = mesh:GetTriVerticesTriangle(tID)
	local bary = tri:BarycentricCoords(point)
	point = tri:PointAt(bary)
	if mesh:HasVertexNormals() and bForceFaceNormal ~= true then
		local normal = mesh:GetTriBaryNormal(tID, bary.x, bary.y, bary.z)
		return Frame3d.FromZ(point, normal)
	else
		return Frame3d.FromZ(point, mesh:GetTriNormal(tID))
	end
end

-- Get barycentric coords of point in triangle
function MeshQueries.BaryCoords(mesh, tID: number, point)
	if not mesh:IsTriangle(tID) then
		error("MeshQueries.SurfaceFrame: triangle " .. tostring(tID) .. " does not exist!")
	end
	local tri = mesh:GetTriVerticesTriangle(tID)
	return tri:BarycentricCoords(point)
end

-- Compute distance from point to triangle in mesh, with minimal extra objects/etc
-- (same algorithm as DistPoint3Triangle3.DistanceSqr)
local function TriDistanceSqrXYZ(mesh, ti: number, px: number, py: number, pz: number): number
	local t, d = mesh.triangles.data, mesh.vertices.data
	local i = 3 * ti
	local a, b, c = 3 * t[i + 1], 3 * t[i + 2], 3 * t[i + 3]
	local sqrDistance =
		distance_sqr_xyz(px, py, pz, d[a + 1], d[a + 2], d[a + 3], d[b + 1], d[b + 2], d[b + 3], d[c + 1], d[c + 2], d[c + 3])
	if sqrDistance < 0 then
		sqrDistance = 0
	end
	return sqrDistance
end
MeshQueries.TriDistanceSqrXYZ = TriDistanceSqrXYZ

function MeshQueries.TriDistanceSqr(mesh, ti: number, point): number
	return TriDistanceSqrXYZ(mesh, ti, point.x, point.y, point.z)
end

-- brute force search for nearest vertex to point
function MeshQueries.FindNearestVertex_LinearSearch(mesh, p): number
	local vNearest = -1
	local fNearestSqr = MaxDouble
	for vid in mesh:VertexIndices() do
		local distSqr = mesh:GetVertex(vid):DistanceSquared(p)
		if distSqr < fNearestSqr then
			fNearestSqr = distSqr
			vNearest = vid
		end
	end
	return vNearest
end

-- brute force search for nearest triangle to point
function MeshQueries.FindNearestTriangle_LinearSearch(mesh, p): number
	local tNearest = -1
	local fNearestSqr = MaxDouble
	local px, py, pz = p.x, p.y, p.z
	for ti in mesh:TriangleIndices() do
		local distSqr = TriDistanceSqrXYZ(mesh, ti, px, py, pz)
		if distSqr < fNearestSqr then
			fNearestSqr = distSqr
			tNearest = ti
		end
	end
	return tNearest
end

function MeshQueries.FindHitTriangle_LinearSearch(mesh, ray): number
	local tNearestID = -1
	local fNearestT = MaxDouble
	for ti in mesh:TriangleIndices() do
		-- [TODO] optimize this
		local tri = mesh:GetTriVerticesTriangle(ti)
		local ray_tri_hit = IntrRay3Triangle3.new(ray, tri)
		if ray_tri_hit:Find() then
			if ray_tri_hit.RayParameter < fNearestT then
				fNearestT = ray_tri_hit.RayParameter
				tNearestID = ti
			end
		end
	end
	return tNearestID
end

-- brute force search for any intersecting triangles on two meshes
function MeshQueries.FindIntersectingTriangles_LinearSearch(mesh1, mesh2): { number }
	local Intr = IntrTriangle3Triangle3()
	for ti in mesh1:TriangleIndices() do
		local a, b, c = mesh1:GetTriVertices(ti)
		for tj in mesh2:TriangleIndices() do
			local e, f, g = mesh2:GetTriVertices(tj)
			local intr = Intr.new(Triangle3d.new(a, b, c), Triangle3d.new(e, f, g))
			if intr:Test() then
				return { ti, tj }
			end
		end
	end
	return Index.Index2i.Max()
end

-- Brute-force search for closest pair of triangles on two meshes
-- C# FindNearestTriangles_LinearSearch(mesh1, mesh2, out fNearestSqr) -> returns Index2i, fNearestSqr
function MeshQueries.FindNearestTriangles_LinearSearch(mesh1, mesh2): ({ number }, number)
	local Dist = DistTriangle3Triangle3()
	local nearPair = Index.Index2i.Max()
	local fNearestSqr = MaxDouble
	for ti in mesh1:TriangleIndices() do
		local a, b, c = mesh1:GetTriVertices(ti)
		for tj in mesh2:TriangleIndices() do
			local e, f, g = mesh2:GetTriVertices(tj)
			local dist = Dist.new(Triangle3d.new(a, b, c), Triangle3d.new(e, f, g))
			if dist:GetSquared() < fNearestSqr then
				fNearestSqr = dist:GetSquared()
				nearPair = { ti, tj }
			end
		end
	end
	fNearestSqr = math.sqrt(fNearestSqr)
	return nearPair, fNearestSqr
end

-- C# EdgeLengthStats(mesh, out minEdgeLen, out maxEdgeLen, out avgEdgeLen, samples) -> returns min, max, avg
function MeshQueries.EdgeLengthStats(mesh, samples: number?): (number, number, number)
	local nsamples = samples or 0
	local minEdgeLen = MaxDouble
	local maxEdgeLen = -MaxDouble
	local avgEdgeLen = 0
	local avg_count = 0
	local MaxID = mesh:MaxEdgeID()

	-- if we are only taking some samples, use a prime-modulo-loop instead of random
	local nPrime = (nsamples == 0) and 1 or 31337
	local max_count = (nsamples == 0) and MaxID or nsamples

	local eid = 0
	local count = 0
	repeat
		if mesh:IsEdge(eid) then
			local _, a, b = mesh:GetEdgeVPositions(eid)
			local len = a:Distance(b)
			if len < minEdgeLen then
				minEdgeLen = len
			end
			if len > maxEdgeLen then
				maxEdgeLen = len
			end
			avgEdgeLen += len
			avg_count += 1
		end
		eid = (eid + nPrime) % MaxID
		-- C#: while (eid != 0 && count++ < max_count)
		local keepGoing = false
		if eid ~= 0 then
			keepGoing = count < max_count
			count += 1
		end
	until not keepGoing

	avgEdgeLen /= avg_count
	return minEdgeLen, maxEdgeLen, avgEdgeLen
end

-- C# EdgeLengthStatsFromEdges(mesh, EdgeItr, out min, out max, out avg, samples) -> returns min, max, avg
function MeshQueries.EdgeLengthStatsFromEdges(mesh, EdgeItr: any, samples: number?): (number, number, number)
	local minEdgeLen = MaxDouble
	local maxEdgeLen = -MaxDouble
	local avgEdgeLen = 0
	local avg_count = 0

	local function visit(eid)
		if mesh:IsEdge(eid) then
			local _, a, b = mesh:GetEdgeVPositions(eid)
			local len = a:Distance(b)
			if len < minEdgeLen then
				minEdgeLen = len
			end
			if len > maxEdgeLen then
				maxEdgeLen = len
			end
			avgEdgeLen += len
			avg_count += 1
		end
	end
	if type(EdgeItr) == "function" then
		for eid in EdgeItr do
			visit(eid)
		end
	else
		for _, eid in ipairs(EdgeItr) do
			visit(eid)
		end
	end
	avgEdgeLen /= avg_count
	return minEdgeLen, maxEdgeLen, avgEdgeLen
end

return MeshQueries
