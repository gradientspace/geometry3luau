--!native
-- Port of geometry3Sharp distance/DistSegment3Triangle3.cs (ported from WildMagic 5)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau API notes:
--   DistSegment3Triangle3.new(segment, triangle)    (Segment3d, Triangle3d; stored by reference)
--   q:Compute() -> q, q:Get(), q:GetSquared()
--   fields DistanceSquared, SegmentClosest, SegmentParam, TriangleClosest, TriangleBaryCoords
--   C# properties Segment/Triangle -> q:Segment(), q:SetSegment(s), q:Triangle(), q:SetTriangle(t)

local Vector3d = require(script.Parent.Vector3d)
local Line3d = require(script.Parent.Line3d)
local DistLine3Triangle3 = require(script.Parent.DistLine3Triangle3)
local DistPoint3Triangle3 = require(script.Parent.DistPoint3Triangle3)

local DistSegment3Triangle3 = {}
DistSegment3Triangle3.__index = DistSegment3Triangle3
DistSegment3Triangle3.ClassName = "DistSegment3Triangle3"

function DistSegment3Triangle3.new(SegmentIn, TriangleIn)
	return setmetatable({
		segment = SegmentIn,
		triangle = TriangleIn,
		DistanceSquared = -1.0,
		SegmentClosest = Vector3d.Zero,
		SegmentParam = 0,
		TriangleClosest = Vector3d.Zero,
		TriangleBaryCoords = Vector3d.Zero,
	}, DistSegment3Triangle3)
end

function DistSegment3Triangle3:Segment()
	return self.segment
end
function DistSegment3Triangle3:SetSegment(value)
	self.segment = value
	self.DistanceSquared = -1.0
end
function DistSegment3Triangle3:Triangle()
	return self.triangle
end
function DistSegment3Triangle3:SetTriangle(value)
	self.triangle = value
	self.DistanceSquared = -1.0
end

function DistSegment3Triangle3:Compute()
	self:GetSquared()
	return self
end

function DistSegment3Triangle3:Get(): number
	return math.sqrt(self:GetSquared())
end

function DistSegment3Triangle3:GetSquared(): number
	if self.DistanceSquared >= 0 then
		return self.DistanceSquared
	end
	local segment, triangle = self.segment, self.triangle
	local line = Line3d.new(segment.Center, segment.Direction)
	local queryLT = DistLine3Triangle3.new(line, triangle)
	local sqrDist = queryLT:GetSquared()
	self.SegmentParam = queryLT.LineParam

	if self.SegmentParam >= -segment.Extent then
		if self.SegmentParam <= segment.Extent then
			self.SegmentClosest = queryLT.LineClosest
			self.TriangleClosest = queryLT.TriangleClosest
			self.TriangleBaryCoords = queryLT.TriangleBaryCoords
		else
			self.SegmentClosest = segment:P1()
			local queryPT = DistPoint3Triangle3.new(self.SegmentClosest, triangle)
			sqrDist = queryPT:GetSquared()
			self.TriangleClosest = queryPT.TriangleClosest
			self.SegmentParam = segment.Extent
			self.TriangleBaryCoords = queryPT.TriangleBaryCoords
		end
	else
		self.SegmentClosest = segment:P0()
		local queryPT = DistPoint3Triangle3.new(self.SegmentClosest, triangle)
		sqrDist = queryPT:GetSquared()
		self.TriangleClosest = queryPT.TriangleClosest
		self.SegmentParam = -segment.Extent
		self.TriangleBaryCoords = queryPT.TriangleBaryCoords
	end

	self.DistanceSquared = sqrDist
	return self.DistanceSquared
end

return DistSegment3Triangle3
