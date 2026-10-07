--!native
-- Port of geometry3Sharp distance/DistTriangle3Triangle3.cs (ported from WildMagic 5)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau API notes:
--   DistTriangle3Triangle3.new(triangle0, triangle1)    (Triangle3d; stored by reference)
--   q:Compute() -> q, q:Get(), q:GetSquared()
--   fields DistanceSquared, Triangle0Closest, Triangle0BaryCoords, Triangle1Closest, Triangle1BaryCoords
--   C# properties Triangle0/Triangle1 -> q:Triangle0(), q:SetTriangle0(t), q:Triangle1(), q:SetTriangle1(t)
-- Note: when the closest pair comes from an edge of triangle1, the C# code assigns the two closest points
-- to the wrong fields (Triangle0Closest gets the point on triangle1). This port assigns them correctly
-- (as WildMagic5 does); DistanceSquared and the barycentric coords are unchanged.

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local Segment3d = require(script.Parent.Segment3d)
local DistSegment3Triangle3 = require(script.Parent.DistSegment3Triangle3)

local ZeroTolerance = MathUtil.ZeroTolerance
local MaxDouble = MathUtil.MaxDouble

local DistTriangle3Triangle3 = {}
DistTriangle3Triangle3.__index = DistTriangle3Triangle3
DistTriangle3Triangle3.ClassName = "DistTriangle3Triangle3"

function DistTriangle3Triangle3.new(Triangle0in, Triangle1in)
	return setmetatable({
		triangle0 = Triangle0in,
		triangle1 = Triangle1in,
		DistanceSquared = -1.0,
		Triangle0Closest = Vector3d.Zero,
		Triangle0BaryCoords = Vector3d.Zero,
		Triangle1Closest = Vector3d.Zero,
		Triangle1BaryCoords = Vector3d.Zero,
	}, DistTriangle3Triangle3)
end

function DistTriangle3Triangle3:Triangle0()
	return self.triangle0
end
function DistTriangle3Triangle3:SetTriangle0(value)
	self.triangle0 = value
	self.DistanceSquared = -1.0
end
function DistTriangle3Triangle3:Triangle1()
	return self.triangle1
end
function DistTriangle3Triangle3:SetTriangle1(value)
	self.triangle1 = value
	self.DistanceSquared = -1.0
end

function DistTriangle3Triangle3:Compute()
	self:GetSquared()
	return self
end

function DistTriangle3Triangle3:Get(): number
	return math.sqrt(self:GetSquared())
end

local function edge_bary(i0: number, i1: number, ratio: number)
	local bary = { 0, 0, 0 }
	bary[i0 + 1] = 0.5 * (1 - ratio)
	bary[i1 + 1] = 1 - bary[i0 + 1]
	bary[3 - i0 - i1 + 1] = 0
	return Vector3d.new(bary[1], bary[2], bary[3])
end

function DistTriangle3Triangle3:GetSquared(): number
	if self.DistanceSquared >= 0 then
		return self.DistanceSquared
	end
	local triangle0, triangle1 = self.triangle0, self.triangle1

	-- Compare edges of triangle0 to the interior of triangle1.
	local sqrDist = MaxDouble
	local edge = Segment3d.FromCenter(Vector3d.Zero, Vector3d.AxisX, 0)
	local i0 = 2
	for i1 = 0, 2 do
		edge:SetEndpoints(triangle0:Get(i0), triangle0:Get(i1))

		local queryST = DistSegment3Triangle3.new(edge, triangle1)
		local sqrDistTmp = queryST:GetSquared()
		if sqrDistTmp < sqrDist then
			self.Triangle0Closest = queryST.SegmentClosest
			self.Triangle1Closest = queryST.TriangleClosest
			sqrDist = sqrDistTmp

			local ratio = queryST.SegmentParam / edge.Extent
			self.Triangle0BaryCoords = edge_bary(i0, i1, ratio)
			self.Triangle1BaryCoords = queryST.TriangleBaryCoords

			if sqrDist <= ZeroTolerance then
				self.DistanceSquared = 0
				return 0
			end
		end
		i0 = i1
	end

	-- Compare edges of triangle1 to the interior of triangle0.
	i0 = 2
	for i1 = 0, 2 do
		edge:SetEndpoints(triangle1:Get(i0), triangle1:Get(i1))

		local queryST = DistSegment3Triangle3.new(edge, triangle0)
		local sqrDistTmp = queryST:GetSquared()
		if sqrDistTmp < sqrDist then
			-- [note] C# assigns these the other way around (bug); segment is on triangle1 here
			self.Triangle0Closest = queryST.TriangleClosest
			self.Triangle1Closest = queryST.SegmentClosest
			sqrDist = sqrDistTmp

			local ratio = queryST.SegmentParam / edge.Extent
			self.Triangle1BaryCoords = edge_bary(i0, i1, ratio)
			self.Triangle0BaryCoords = queryST.TriangleBaryCoords

			if sqrDist <= ZeroTolerance then
				self.DistanceSquared = 0
				return 0
			end
		end
		i0 = i1
	end

	self.DistanceSquared = sqrDist
	return self.DistanceSquared
end

return DistTriangle3Triangle3
