--!native
-- Port of geometry3Sharp intersection/IntrTriangle3Triangle3.cs (ported from WildMagic5 IntrTriangle3Triangle3)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- use Test() for fast boolean query, does not compute intersection info
-- use Find() to compute full information
-- By default fully-contained co-planar triangles are not reported as intersecting.
-- set ReportCoplanarIntersection=true to handle this case (more expensive)
--
-- Luau API notes:
--   IntrTriangle3Triangle3.new(t0, t1)    (Triangle3d; stored by reference)
--   q:Test(), q:Find(), q:Compute(); q:Triangle0()/SetTriangle0(t), q:Triangle1()/SetTriangle1(t)
--   fields ReportCoplanarIntersection, Quantity, Result, Type,
--          Points: 1-based array of Vector3d (C# Vector3dTuple3 Points[i] -> Points[i+1]), first Quantity valid
--          PolygonPoints: 1-based array of Vector3d (only valid if Type == Polygon)
--   static Intersects(triangle0, triangle1) -> bool
--   static ProjectOntoAxis(triangle, axis) -> fmin, fmax
--   static TrianglePlaneRelations(triangle, plane) -> distance, sign, positive, negative, zero
--       (distance/sign are 1-based arrays of 3 numbers; C# out Vector3d / out Index3i)

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local Vector2d = require(script.Parent.Vector2d)
local Plane3d = require(script.Parent.Plane3d)
local Triangle2d = require(script.Parent.Triangle2d)
local Segment2d = require(script.Parent.Segment2d)
local QueryTuple2d = require(script.Parent.QueryTuple2d)
local IntrSegment2Triangle2 = require(script.Parent.IntrSegment2Triangle2)
local IntrTriangle2Triangle2 = require(script.Parent.IntrTriangle2Triangle2)
local IntersectionResult = require(script.Parent.IntersectionResult)
local IntersectionType = require(script.Parent.IntersectionType)

local V3 = Vector3d.new
local abs = math.abs
local ZeroTolerance = MathUtil.ZeroTolerance

local IntrTriangle3Triangle3 = {}
IntrTriangle3Triangle3.__index = IntrTriangle3Triangle3
IntrTriangle3Triangle3.ClassName = "IntrTriangle3Triangle3"

function IntrTriangle3Triangle3.new(t0, t1)
	return setmetatable({
		triangle0 = t0,
		triangle1 = t1,
		-- If true, will return intersection polygons for co-planar triangles.
		-- This is somewhat expensive, default is false.
		-- Note that when false, co-planar intersections will **NOT** be reported as intersections
		ReportCoplanarIntersection = false,
		-- result flags
		Quantity = 0,
		Result = IntersectionResult.NotComputed,
		Type = IntersectionType.Empty,
		-- intersection points (for point, line); only first Quantity elements are relevant
		Points = { Vector3d.Zero, Vector3d.Zero, Vector3d.Zero },
		-- only valid if intersection type is Polygon
		PolygonPoints = nil,
	}, IntrTriangle3Triangle3)
end

function IntrTriangle3Triangle3:Triangle0()
	return self.triangle0
end
function IntrTriangle3Triangle3:SetTriangle0(value)
	self.triangle0 = value
	self.Result = IntersectionResult.NotComputed
end
function IntrTriangle3Triangle3:Triangle1()
	return self.triangle1
end
function IntrTriangle3Triangle3:SetTriangle1(value)
	self.triangle1 = value
	self.Result = IntersectionResult.NotComputed
end

function IntrTriangle3Triangle3:Compute()
	self:Find()
	return self
end

local function ProjectOntoAxis(triangle, axis): (number, number)
	local dot0 = axis:Dot(triangle.V0)
	local dot1 = axis:Dot(triangle.V1)
	local dot2 = axis:Dot(triangle.V2)

	local fmin = dot0
	local fmax = fmin

	if dot1 < fmin then
		fmin = dot1
	elseif dot1 > fmax then
		fmax = dot1
	end

	if dot2 < fmin then
		fmin = dot2
	elseif dot2 > fmax then
		fmax = dot2
	end
	return fmin, fmax
end
IntrTriangle3Triangle3.ProjectOntoAxis = ProjectOntoAxis

local function TrianglePlaneRelations(triangle, plane)
	-- Compute the signed distances of triangle vertices to the plane.  Use
	-- an epsilon-thick plane test.
	local positive, negative, zero = 0, 0, 0
	local distance = { 0, 0, 0 }
	local sign = { 0, 0, 0 }
	for i = 0, 2 do
		local d = plane:DistanceTo(triangle:Get(i))
		if d > ZeroTolerance then
			sign[i + 1] = 1
			positive += 1
		elseif d < -ZeroTolerance then
			sign[i + 1] = -1
			negative += 1
		else
			d = 0
			sign[i + 1] = 0
			zero += 1
		end
		distance[i + 1] = d
	end
	return distance, sign, positive, negative, zero
end
IntrTriangle3Triangle3.TrianglePlaneRelations = TrianglePlaneRelations

function IntrTriangle3Triangle3:Find(): boolean
	if self.Result ~= IntersectionResult.NotComputed then
		return self.Result ~= IntersectionResult.NoIntersection
	end

	-- in this code the results get initialized in subroutines, so we
	-- set the defautl value here...
	self.Result = IntersectionResult.NoIntersection

	local triangle0, triangle1 = self.triangle0, self.triangle1

	-- Get the plane of triangle0.
	local plane0 = Plane3d.FromPoints(triangle0.V0, triangle0.V1, triangle0.V2)

	-- Compute the signed distances of triangle1 vertices to plane0.  Use
	-- an epsilon-thick plane test.
	local dist1, sign1, pos1, neg1, zero1 = TrianglePlaneRelations(triangle1, plane0)

	if pos1 == 3 or neg1 == 3 then
		-- Triangle1 is fully on one side of plane0.
		return false
	end

	if zero1 == 3 then
		-- Triangle1 is contained by plane0.
		if self.ReportCoplanarIntersection then
			return self:GetCoplanarIntersection(plane0, triangle0, triangle1)
		end
		return false
	end

	-- Check for grazing contact between triangle1 and plane0.
	if pos1 == 0 or neg1 == 0 then
		if zero1 == 2 then
			-- An edge of triangle1 is in plane0.
			for i = 0, 2 do
				if sign1[i + 1] ~= 0 then
					local iM = (i + 2) % 3
					local iP = (i + 1) % 3
					return self:IntersectsSegment(plane0, triangle0, triangle1:Get(iM), triangle1:Get(iP))
				end
			end
		else -- zero1 == 1
			-- A vertex of triangle1 is in plane0.
			for i = 0, 2 do
				if sign1[i + 1] == 0 then
					return self:ContainsPoint(triangle0, plane0, triangle1:Get(i))
				end
			end
		end
	end

	-- At this point, triangle1 tranversely intersects plane 0.  Compute the
	-- line segment of intersection.  Then test for intersection between this
	-- segment and triangle 0.
	local t
	if zero1 == 0 then
		local iSign = (pos1 == 1) and 1 or -1
		for i = 0, 2 do
			if sign1[i + 1] == iSign then
				local iM = (i + 2) % 3
				local iP = (i + 1) % 3
				local ti, tM, tP = triangle1:Get(i), triangle1:Get(iM), triangle1:Get(iP)
				t = dist1[i + 1] / (dist1[i + 1] - dist1[iM + 1])
				local intr0 = ti + t * (tM - ti)
				t = dist1[i + 1] / (dist1[i + 1] - dist1[iP + 1])
				local intr1 = ti + t * (tP - ti)
				return self:IntersectsSegment(plane0, triangle0, intr0, intr1)
			end
		end
	end

	-- zero1 == 1
	for i = 0, 2 do
		if sign1[i + 1] == 0 then
			local iM = (i + 2) % 3
			local iP = (i + 1) % 3
			local tM, tP = triangle1:Get(iM), triangle1:Get(iP)
			t = dist1[iM + 1] / (dist1[iM + 1] - dist1[iP + 1])
			local intr0 = tM + t * (tP - tM)
			return self:IntersectsSegment(plane0, triangle0, triangle1:Get(i), intr0)
		end
	end

	-- should never get here...
	return false
end

-- shared implementation of Test() and static Intersects(). Returns bool, and the IntersectionType
-- that Test() assigns (Unknown or Plane) on success.
local function test_intersection(triangle0, triangle1): (boolean, number?)
	-- Get edge vectors for triangle0.
	local E0 = { triangle0.V1 - triangle0.V0, triangle0.V2 - triangle0.V1, triangle0.V0 - triangle0.V2 }

	-- Get normal vector of triangle0.
	local N0 = E0[1]:UnitCross(E0[2])

	-- Project triangle1 onto normal line of triangle0, test for separation.
	local N0dT0V0 = N0:Dot(triangle0.V0)
	local min1, max1 = ProjectOntoAxis(triangle1, N0)
	if N0dT0V0 < min1 or N0dT0V0 > max1 then
		return false, nil
	end

	-- Get edge vectors for triangle1.
	local E1 = { triangle1.V1 - triangle1.V0, triangle1.V2 - triangle1.V1, triangle1.V0 - triangle1.V2 }

	-- Get normal vector of triangle1.
	local N1 = E1[1]:UnitCross(E1[2])

	local min0, max0

	local N0xN1 = N0:UnitCross(N1)
	if N0xN1:Dot(N0xN1) >= ZeroTolerance then
		-- Triangles are not parallel.

		-- Project triangle0 onto normal line of triangle1, test for
		-- separation.
		local N1dT1V0 = N1:Dot(triangle1.V0)
		min0, max0 = ProjectOntoAxis(triangle0, N1)
		if N1dT1V0 < min0 or N1dT1V0 > max0 then
			return false, nil
		end

		-- Directions E0[i0]xE1[i1].
		for i1 = 1, 3 do
			for i0 = 1, 3 do
				local dir = E0[i0]:UnitCross(E1[i1])
				min0, max0 = ProjectOntoAxis(triangle0, dir)
				min1, max1 = ProjectOntoAxis(triangle1, dir)
				if max0 < min1 or max1 < min0 then
					return false, nil
				end
			end
		end

		-- The test query does not know the intersection set.
		return true, IntersectionType.Unknown
	else -- Triangles are parallel (and, in fact, coplanar).
		-- Directions N0xE0[i0].
		for i0 = 1, 3 do
			local dir = N0:UnitCross(E0[i0])
			min0, max0 = ProjectOntoAxis(triangle0, dir)
			min1, max1 = ProjectOntoAxis(triangle1, dir)
			if max0 < min1 or max1 < min0 then
				return false, nil
			end
		end

		-- Directions N1xE1[i1].
		for i1 = 1, 3 do
			local dir = N1:UnitCross(E1[i1])
			min0, max0 = ProjectOntoAxis(triangle0, dir)
			min1, max1 = ProjectOntoAxis(triangle1, dir)
			if max0 < min1 or max1 < min0 then
				return false, nil
			end
		end

		-- The test query does not know the intersection set.
		return true, IntersectionType.Plane
	end
end

function IntrTriangle3Triangle3:Test(): boolean
	local ok, itype = test_intersection(self.triangle0, self.triangle1)
	if ok then
		self.Type = itype :: number
	end
	return ok
end

function IntrTriangle3Triangle3.Intersects(triangle0, triangle1): boolean
	local ok = test_intersection(triangle0, triangle1)
	return ok
end

function IntrTriangle3Triangle3:ContainsPoint(triangle, plane, point): boolean
	-- Generate a coordinate system for the plane.  The incoming triangle has
	-- vertices <V0,V1,V2>.  The incoming plane has unit-length normal N.
	-- The incoming point is P.  V0 is chosen as the origin for the plane. The
	-- coordinate axis directions are two unit-length vectors, U0 and U1,
	-- constructed so that {U0,U1,N} is an orthonormal set.  Any point Q
	-- in the plane may be written as Q = V0 + x0*U0 + x1*U1.  The coordinates
	-- are computed as x0 = Dot(U0,Q-V0) and x1 = Dot(U1,Q-V0).
	local U0, U1 = Vector3d.GenerateComplementBasis(plane.Normal)

	-- Compute the planar coordinates for the points P, V1, and V2.  To
	-- simplify matters, the origin is subtracted from the points, in which
	-- case the planar coordinates are for P-V0, V1-V0, and V2-V0.
	local PmV0 = point - triangle.V0
	local V1mV0 = triangle.V1 - triangle.V0
	local V2mV0 = triangle.V2 - triangle.V0

	-- The planar representation of P-V0.
	local ProjP = Vector2d.new(U0:Dot(PmV0), U1:Dot(PmV0))

	-- The planar representation of the triangle <V0-V0,V1-V0,V2-V0>.
	local query = QueryTuple2d.new(Vector2d.Zero, Vector2d.new(U0:Dot(V1mV0), U1:Dot(V1mV0)), Vector2d.new(U0:Dot(V2mV0), U1:Dot(V2mV0)))

	-- Test whether P-V0 is in the triangle <0,V1-V0,V2-V0>.
	if query:ToTriangle(ProjP, 0, 1, 2) <= 0 then
		self.Result = IntersectionResult.Intersects
		self.Type = IntersectionType.Point
		self.Quantity = 1
		self.Points[1] = point
		return true
	end

	return false
end

function IntrTriangle3Triangle3:IntersectsSegment(plane, triangle, end0, end1): boolean
	-- Compute the 2D representations of the triangle vertices and the
	-- segment endpoints relative to the plane of the triangle.  Then
	-- compute the intersection in the 2D space.

	-- Project the triangle and segment onto the coordinate plane most
	-- aligned with the plane normal.
	local maxNormal = 0
	local fmax = abs(plane.Normal.x)
	local absMax = abs(plane.Normal.y)
	if absMax > fmax then
		maxNormal = 1
		fmax = absMax
	end
	absMax = abs(plane.Normal.z)
	if absMax > fmax then
		maxNormal = 2
	end

	local projTri = Triangle2d.new(Vector2d.Zero, Vector2d.Zero, Vector2d.Zero)
	local projEnd0, projEnd1

	if maxNormal == 0 then
		-- Project onto yz-plane.
		for i = 0, 2 do
			projTri:Set(i, triangle:Get(i):yz())
		end
		projEnd0 = Vector2d.new(end0.y, end0.z)
		projEnd1 = Vector2d.new(end1.y, end1.z)
	elseif maxNormal == 1 then
		-- Project onto xz-plane.
		for i = 0, 2 do
			projTri:Set(i, triangle:Get(i):xz())
		end
		projEnd0 = Vector2d.new(end0.x, end0.z)
		projEnd1 = Vector2d.new(end1.x, end1.z)
	else
		-- Project onto xy-plane.
		for i = 0, 2 do
			projTri:Set(i, triangle:Get(i):xy())
		end
		projEnd0 = Vector2d.new(end0.x, end0.y)
		projEnd1 = Vector2d.new(end1.x, end1.y)
	end

	local projSeg = Segment2d.new(projEnd0, projEnd1)
	local calc = IntrSegment2Triangle2.new(projSeg, projTri)
	if not calc:Find() then
		return false
	end

	local intr = { Vector2d.Zero, Vector2d.Zero }
	if calc.Type == IntersectionType.Segment then
		self.Result = IntersectionResult.Intersects
		self.Type = IntersectionType.Segment
		self.Quantity = 2
		intr[1] = calc.Point0
		intr[2] = calc.Point1
	else
		--"Intersection must be a point\n";
		self.Result = IntersectionResult.Intersects
		self.Type = IntersectionType.Point
		self.Quantity = 1
		intr[1] = calc.Point0
	end

	-- Unproject the segment of intersection.
	local N, C = plane.Normal, plane.Constant
	if maxNormal == 0 then
		local invNX = 1 / N.x
		for i = 1, self.Quantity do
			local y = intr[i].x
			local z = intr[i].y
			local x = invNX * (C - N.y * y - N.z * z)
			self.Points[i] = V3(x, y, z)
		end
	elseif maxNormal == 1 then
		local invNY = 1 / N.y
		for i = 1, self.Quantity do
			local x = intr[i].x
			local z = intr[i].y
			local y = invNY * (C - N.x * x - N.z * z)
			self.Points[i] = V3(x, y, z)
		end
	else
		local invNZ = 1 / N.z
		for i = 1, self.Quantity do
			local x = intr[i].x
			local y = intr[i].y
			local z = invNZ * (C - N.x * x - N.y * y)
			self.Points[i] = V3(x, y, z)
		end
	end

	return true
end

function IntrTriangle3Triangle3:GetCoplanarIntersection(plane, tri0, tri1): boolean
	-- Project triangles onto coordinate plane most aligned with plane
	-- normal.
	local maxNormal = 0
	local fmax = abs(plane.Normal.x)
	local absMax = abs(plane.Normal.y)
	if absMax > fmax then
		maxNormal = 1
		fmax = absMax
	end
	absMax = abs(plane.Normal.z)
	if absMax > fmax then
		maxNormal = 2
	end

	local projTri0 = Triangle2d.new(Vector2d.Zero, Vector2d.Zero, Vector2d.Zero)
	local projTri1 = Triangle2d.new(Vector2d.Zero, Vector2d.Zero, Vector2d.Zero)

	if maxNormal == 0 then
		-- Project onto yz-plane.
		for i = 0, 2 do
			projTri0:Set(i, tri0:Get(i):yz())
			projTri1:Set(i, tri1:Get(i):yz())
		end
	elseif maxNormal == 1 then
		-- Project onto xz-plane.
		for i = 0, 2 do
			projTri0:Set(i, tri0:Get(i):xz())
			projTri1:Set(i, tri1:Get(i):xz())
		end
	else
		-- Project onto xy-plane.
		for i = 0, 2 do
			projTri0:Set(i, tri0:Get(i):xy())
			projTri1:Set(i, tri1:Get(i):xy())
		end
	end

	-- 2D triangle intersection routines require counterclockwise ordering.
	local edge0 = projTri0.V1 - projTri0.V0
	local edge1 = projTri0.V2 - projTri0.V0
	if edge0:DotPerp(edge1) < 0 then
		-- Triangle is clockwise, reorder it.
		projTri0.V1, projTri0.V2 = projTri0.V2, projTri0.V1
	end

	edge0 = projTri1.V1 - projTri1.V0
	edge1 = projTri1.V2 - projTri1.V0
	if edge0:DotPerp(edge1) < 0 then
		-- Triangle is clockwise, reorder it.
		projTri1.V1, projTri1.V2 = projTri1.V2, projTri1.V1
	end

	local intr = IntrTriangle2Triangle2.new(projTri0, projTri1)
	if not intr:Find() then
		return false
	end

	self.PolygonPoints = table.create(intr.Quantity)

	-- Map 2D intersections back to the 3D triangle space.
	self.Quantity = intr.Quantity
	local N, C = plane.Normal, plane.Constant
	if maxNormal == 0 then
		local invNX = 1 / N.x
		for i = 1, self.Quantity do
			local y = intr.Points[i].x
			local z = intr.Points[i].y
			local x = invNX * (C - N.y * y - N.z * z)
			self.PolygonPoints[i] = V3(x, y, z)
		end
	elseif maxNormal == 1 then
		local invNY = 1 / N.y
		for i = 1, self.Quantity do
			local x = intr.Points[i].x
			local z = intr.Points[i].y
			local y = invNY * (C - N.x * x - N.z * z)
			self.PolygonPoints[i] = V3(x, y, z)
		end
	else
		local invNZ = 1 / N.z
		for i = 1, self.Quantity do
			local x = intr.Points[i].x
			local y = intr.Points[i].y
			local z = invNZ * (C - N.x * x - N.y * y)
			self.PolygonPoints[i] = V3(x, y, z)
		end
	end

	self.Result = IntersectionResult.Intersects
	self.Type = IntersectionType.Polygon
	return true
end

return IntrTriangle3Triangle3
