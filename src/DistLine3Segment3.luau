--!native
-- Port of geometry3Sharp distance/DistLine3Segment3.cs (ported from WildMagic 5)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau API notes:
--   DistLine3Segment3.new(line, segment)    (Line3d, Segment3d; stored by reference)
--   q:Compute() -> q, q:Get(), q:GetSquared()
--   fields DistanceSquared, LineClosest, LineParameter, SegmentClosest, SegmentParameter
--   C# properties Line/Segment -> q:Line(), q:SetLine(l), q:Segment(), q:SetSegment(s)
--   static MinDistance(line, segment), MinDistanceLineParam(line, segment)

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)

local abs = math.abs
local ZeroTolerance = MathUtil.ZeroTolerance

local DistLine3Segment3 = {}
DistLine3Segment3.__index = DistLine3Segment3
DistLine3Segment3.ClassName = "DistLine3Segment3"

function DistLine3Segment3.new(LineIn, SegmentIn)
	return setmetatable({
		line = LineIn,
		segment = SegmentIn,
		DistanceSquared = -1.0,
		LineClosest = Vector3d.Zero,
		LineParameter = 0,
		SegmentClosest = Vector3d.Zero,
		SegmentParameter = 0,
	}, DistLine3Segment3)
end

function DistLine3Segment3:Line()
	return self.line
end
function DistLine3Segment3:SetLine(value)
	self.line = value
	self.DistanceSquared = -1.0
end
function DistLine3Segment3:Segment()
	return self.segment
end
function DistLine3Segment3:SetSegment(value)
	self.segment = value
	self.DistanceSquared = -1.0
end

function DistLine3Segment3.MinDistance(line, segment): number
	return DistLine3Segment3.new(line, segment):Get()
end
function DistLine3Segment3.MinDistanceLineParam(line, segment): number
	return DistLine3Segment3.new(line, segment):Compute().LineParameter
end

function DistLine3Segment3:Compute()
	self:GetSquared()
	return self
end

function DistLine3Segment3:Get(): number
	return math.sqrt(self:GetSquared())
end

function DistLine3Segment3:GetSquared(): number
	if self.DistanceSquared >= 0 then
		return self.DistanceSquared
	end
	local line, segment = self.line, self.segment

	local diff = line.Origin - segment.Center
	local a01 = -line.Direction:Dot(segment.Direction)
	local b0 = diff:Dot(line.Direction)
	local c = diff:LengthSquared()
	local det = abs(1 - a01 * a01)
	local b1, s0, s1, sqrDist, extDet

	if det >= ZeroTolerance then
		-- The line and segment are not parallel.
		b1 = -diff:Dot(segment.Direction)
		s1 = a01 * b0 - b1
		extDet = segment.Extent * det

		if s1 >= -extDet then
			if s1 <= extDet then
				-- Two interior points are closest, one on the line and one
				-- on the segment.
				local invDet = 1 / det
				s0 = (a01 * b1 - b0) * invDet
				s1 *= invDet
				sqrDist = s0 * (s0 + a01 * s1 + 2 * b0) + s1 * (a01 * s0 + s1 + 2 * b1) + c
			else
				-- The endpoint e1 of the segment and an interior point of
				-- the line are closest.
				s1 = segment.Extent
				s0 = -(a01 * s1 + b0)
				sqrDist = -s0 * s0 + s1 * (s1 + 2 * b1) + c
			end
		else
			-- The end point e0 of the segment and an interior point of the
			-- line are closest.
			s1 = -segment.Extent
			s0 = -(a01 * s1 + b0)
			sqrDist = -s0 * s0 + s1 * (s1 + 2 * b1) + c
		end
	else
		-- The line and segment are parallel.  Choose the closest pair so that
		-- one point is at segment center.
		s1 = 0
		s0 = -b0
		sqrDist = b0 * s0 + c
	end

	self.LineClosest = line.Origin + s0 * line.Direction
	self.SegmentClosest = segment.Center + s1 * segment.Direction
	self.LineParameter = s0
	self.SegmentParameter = s1

	-- Account for numerical round-off errors.
	if sqrDist < 0 then
		sqrDist = 0
	end

	self.DistanceSquared = sqrDist
	return sqrDist
end

return DistLine3Segment3
