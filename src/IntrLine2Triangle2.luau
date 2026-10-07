--!native
-- Port of geometry3Sharp intersection/IntrLine2Triangle2.cs (ported from WildMagic5)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau API notes:
--   IntrLine2Triangle2.new(line, triangle)    (Line2d, Triangle2d)
--   q:Find(), q:Compute(), q:IsSimpleIntersection(); q:Line()/SetLine(l), q:Triangle()/SetTriangle(t)
--   fields Quantity, Result, Type, Point0, Point1, Param0, Param1
--   static TriangleLineRelations(origin, direction, tri) -> dist, sign, positive, negative, zero
--       (dist and sign are 1-based arrays of 3 numbers; C# ref Vector3d dist / ref Vector3i sign)
--   static GetInterval(origin, direction, tri, dist, sign) -> param0, param1   (C# ref Vector2d param)

local MathUtil = require(script.Parent.MathUtil)
local Vector2d = require(script.Parent.Vector2d)
local IntersectionResult = require(script.Parent.IntersectionResult)
local IntersectionType = require(script.Parent.IntersectionType)
local Intersector1 = require(script.Parent.Intersector1)

local ZeroTolerance = MathUtil.ZeroTolerance
local MaxDouble = MathUtil.MaxDouble

local IntrLine2Triangle2 = {}
IntrLine2Triangle2.__index = IntrLine2Triangle2
IntrLine2Triangle2.ClassName = "IntrLine2Triangle2"

function IntrLine2Triangle2.new(l, t)
	return setmetatable({
		line = l,
		triangle = t,
		Quantity = 0,
		Result = IntersectionResult.NotComputed,
		Type = IntersectionType.Empty,
		Point0 = Vector2d.Zero,
		Point1 = Vector2d.Zero,
		Param0 = 0,
		Param1 = 0,
	}, IntrLine2Triangle2)
end

function IntrLine2Triangle2:Line()
	return self.line
end
function IntrLine2Triangle2:SetLine(value)
	self.line = value
	self.Result = IntersectionResult.NotComputed
end
function IntrLine2Triangle2:Triangle()
	return self.triangle
end
function IntrLine2Triangle2:SetTriangle(value)
	self.triangle = value
	self.Result = IntersectionResult.NotComputed
end

function IntrLine2Triangle2:IsSimpleIntersection(): boolean
	return self.Result == IntersectionResult.Intersects and self.Type == IntersectionType.Point
end

function IntrLine2Triangle2:Compute()
	self:Find()
	return self
end

function IntrLine2Triangle2:Find(): boolean
	if self.Result ~= IntersectionResult.NotComputed then
		return self.Result == IntersectionResult.Intersects
	end

	-- [RMS] if either line direction is not a normalized vector,
	--   results are garbage, so fail query
	local line = self.line
	if line.Direction:IsNormalized() == false then
		self.Type = IntersectionType.Empty
		self.Result = IntersectionResult.InvalidQuery
		return false
	end

	local dist, sign, positive, negative = IntrLine2Triangle2.TriangleLineRelations(line.Origin, line.Direction, self.triangle)

	if positive == 3 or negative == 3 then
		-- No intersections.
		self.Quantity = 0
		self.Type = IntersectionType.Empty
	else
		local p0, p1 = IntrLine2Triangle2.GetInterval(line.Origin, line.Direction, self.triangle, dist, sign)
		-- todo replace with Interval1d.FindIntersection
		local intr = Intersector1.new(p0, p1, -MaxDouble, MaxDouble)
		intr:Find()

		self.Quantity = intr.NumIntersections
		if self.Quantity == 2 then
			-- Segment intersection.
			self.Type = IntersectionType.Segment
			self.Param0 = intr:GetIntersection(0)
			self.Point0 = line.Origin + self.Param0 * line.Direction
			self.Param1 = intr:GetIntersection(1)
			self.Point1 = line.Origin + self.Param1 * line.Direction
		elseif self.Quantity == 1 then
			-- Point intersection.
			self.Type = IntersectionType.Point
			self.Param0 = intr:GetIntersection(0)
			self.Point0 = line.Origin + self.Param0 * line.Direction
		else
			-- No intersections.
			self.Type = IntersectionType.Empty
		end
	end

	self.Result = (self.Type ~= IntersectionType.Empty) and IntersectionResult.Intersects or IntersectionResult.NoIntersection
	return self.Result == IntersectionResult.Intersects
end

function IntrLine2Triangle2.TriangleLineRelations(origin, direction, tri)
	local positive, negative, zero = 0, 0, 0
	local dist = { 0, 0, 0 }
	local sign = { 0, 0, 0 }
	for i = 0, 2 do
		local diff = tri:Get(i) - origin
		local d = diff:DotPerp(direction)
		if d > ZeroTolerance then
			sign[i + 1] = 1
			positive += 1
		elseif d < -ZeroTolerance then
			sign[i + 1] = -1
			negative += 1
		else
			d = 0.0
			sign[i + 1] = 0
			zero += 1
		end
		dist[i + 1] = d
	end
	return dist, sign, positive, negative, zero
end

function IntrLine2Triangle2.GetInterval(origin, direction, tri, dist: { number }, sign: { number }): (number, number)
	local param = { 0, 0 }
	-- Project triangle onto line.
	local proj = { 0, 0, 0 }
	for i = 0, 2 do
		local diff = tri:Get(i) - origin
		proj[i + 1] = direction:Dot(diff)
	end

	-- Compute transverse intersections of triangle edges with line.
	local quantity = 0
	local i0 = 2
	for i1 = 0, 2 do
		if sign[i0 + 1] * sign[i1 + 1] < 0 then
			if quantity >= 2 then
				error("IntrLine2Triangle2.GetInterval: too many intersections!")
			end
			local numer = dist[i0 + 1] * proj[i1 + 1] - dist[i1 + 1] * proj[i0 + 1]
			local denom = dist[i0 + 1] - dist[i1 + 1]
			quantity += 1
			param[quantity] = numer / denom
		end
		i0 = i1
	end

	-- Check for grazing contact.
	if quantity < 2 then
		for i2 = 0, 2 do
			if sign[i2 + 1] == 0 then
				if quantity >= 2 then
					error("IntrLine2Triangle2.GetInterval: too many intersections!")
				end
				quantity += 1
				param[quantity] = proj[i2 + 1]
			end
		end
	end

	-- Sort.
	if quantity < 1 then
		error("IntrLine2Triangle2.GetInterval: need at least one intersection")
	end
	if quantity == 2 then
		if param[1] > param[2] then
			param[1], param[2] = param[2], param[1]
		end
	else
		param[2] = param[1]
	end
	return param[1], param[2]
end

return IntrLine2Triangle2
