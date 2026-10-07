--!native
-- Port of geometry3Sharp intersection/IntrSegment2Triangle2.cs (ported from WildMagic5)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau API notes:
--   IntrSegment2Triangle2.new(segment, triangle)    (Segment2d, Triangle2d)
--   q:Find(), q:Compute(), q:IsSimpleIntersection(); q:Segment()/SetSegment(s), q:Triangle()/SetTriangle(t)
--   fields Quantity, Result, Type, Point0, Point1, Param0, Param1

local Vector2d = require(script.Parent.Vector2d)
local IntersectionResult = require(script.Parent.IntersectionResult)
local IntersectionType = require(script.Parent.IntersectionType)
local Intersector1 = require(script.Parent.Intersector1)
local IntrLine2Triangle2 = require(script.Parent.IntrLine2Triangle2)

local IntrSegment2Triangle2 = {}
IntrSegment2Triangle2.__index = IntrSegment2Triangle2
IntrSegment2Triangle2.ClassName = "IntrSegment2Triangle2"

function IntrSegment2Triangle2.new(s, t)
	return setmetatable({
		segment = s,
		triangle = t,
		Quantity = 0,
		Result = IntersectionResult.NotComputed,
		Type = IntersectionType.Empty,
		Point0 = Vector2d.Zero,
		Point1 = Vector2d.Zero,
		Param0 = 0,
		Param1 = 0,
	}, IntrSegment2Triangle2)
end

function IntrSegment2Triangle2:Segment()
	return self.segment
end
function IntrSegment2Triangle2:SetSegment(value)
	self.segment = value
	self.Result = IntersectionResult.NotComputed
end
function IntrSegment2Triangle2:Triangle()
	return self.triangle
end
function IntrSegment2Triangle2:SetTriangle(value)
	self.triangle = value
	self.Result = IntersectionResult.NotComputed
end

function IntrSegment2Triangle2:IsSimpleIntersection(): boolean
	return self.Result == IntersectionResult.Intersects and self.Type == IntersectionType.Point
end

function IntrSegment2Triangle2:Compute()
	self:Find()
	return self
end

function IntrSegment2Triangle2:Find(): boolean
	if self.Result ~= IntersectionResult.NotComputed then
		return self.Result == IntersectionResult.Intersects
	end

	-- [RMS] if either line direction is not a normalized vector,
	--   results are garbage, so fail query
	local segment = self.segment
	if segment.Direction:IsNormalized() == false then
		self.Type = IntersectionType.Empty
		self.Result = IntersectionResult.InvalidQuery
		return false
	end

	local dist, sign, positive, negative = IntrLine2Triangle2.TriangleLineRelations(segment.Center, segment.Direction, self.triangle)

	if positive == 3 or negative == 3 then
		-- No intersections.
		self.Quantity = 0
		self.Type = IntersectionType.Empty
	else
		local p0, p1 = IntrLine2Triangle2.GetInterval(segment.Center, segment.Direction, self.triangle, dist, sign)
		-- todo replace with Interval1d.FindIntersection
		local intr = Intersector1.new(p0, p1, -segment.Extent, segment.Extent)
		intr:Find()

		self.Quantity = intr.NumIntersections
		if self.Quantity == 2 then
			-- Segment intersection.
			self.Type = IntersectionType.Segment
			self.Param0 = intr:GetIntersection(0)
			self.Point0 = segment.Center + self.Param0 * segment.Direction
			self.Param1 = intr:GetIntersection(1)
			self.Point1 = segment.Center + self.Param1 * segment.Direction
		elseif self.Quantity == 1 then
			-- Point intersection.
			self.Type = IntersectionType.Point
			self.Param0 = intr:GetIntersection(0)
			self.Point0 = segment.Center + self.Param0 * segment.Direction
		else
			-- No intersections.
			self.Type = IntersectionType.Empty
		end
	end

	self.Result = (self.Type ~= IntersectionType.Empty) and IntersectionResult.Intersects or IntersectionResult.NoIntersection
	return self.Result == IntersectionResult.Intersects
end

return IntrSegment2Triangle2
