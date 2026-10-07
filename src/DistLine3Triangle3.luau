--!native
-- Port of geometry3Sharp distance/DistLine3Triangle3.cs (ported from WildMagic 5)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau API notes:
--   DistLine3Triangle3.new(line, triangle)    (Line3d, Triangle3d; stored by reference)
--   q:Compute() -> q, q:Get(), q:GetSquared()
--   fields DistanceSquared, LineClosest, LineParam, TriangleClosest, TriangleBaryCoords
--   C# properties Line/Triangle -> q:Line(), q:SetLine(l), q:Triangle(), q:SetTriangle(t)

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local Segment3d = require(script.Parent.Segment3d)
local DistLine3Segment3 = require(script.Parent.DistLine3Segment3)

local abs = math.abs
local ZeroTolerance = MathUtil.ZeroTolerance
local MaxDouble = MathUtil.MaxDouble

local DistLine3Triangle3 = {}
DistLine3Triangle3.__index = DistLine3Triangle3
DistLine3Triangle3.ClassName = "DistLine3Triangle3"

function DistLine3Triangle3.new(LineIn, TriangleIn)
	return setmetatable({
		line = LineIn,
		triangle = TriangleIn,
		DistanceSquared = -1.0,
		LineClosest = Vector3d.Zero,
		LineParam = 0,
		TriangleClosest = Vector3d.Zero,
		TriangleBaryCoords = Vector3d.Zero,
	}, DistLine3Triangle3)
end

function DistLine3Triangle3:Line()
	return self.line
end
function DistLine3Triangle3:SetLine(value)
	self.line = value
	self.DistanceSquared = -1.0
end
function DistLine3Triangle3:Triangle()
	return self.triangle
end
function DistLine3Triangle3:SetTriangle(value)
	self.triangle = value
	self.DistanceSquared = -1.0
end

function DistLine3Triangle3:Compute()
	self:GetSquared()
	return self
end

function DistLine3Triangle3:Get(): number
	return math.sqrt(self:GetSquared())
end

function DistLine3Triangle3:GetSquared(): number
	if self.DistanceSquared >= 0 then
		return self.DistanceSquared
	end
	local line, triangle = self.line, self.triangle

	-- Test if line intersects triangle.  If so, the squared distance is zero.
	local edge0 = triangle.V1 - triangle.V0
	local edge1 = triangle.V2 - triangle.V0
	local normal = edge0:UnitCross(edge1)
	local NdD = normal:Dot(line.Direction)
	if abs(NdD) > ZeroTolerance then
		-- The line and triangle are not parallel, so the line intersects
		-- the plane of the triangle.
		local diff = line.Origin - triangle.V0
		local U, V = Vector3d.GenerateComplementBasis(line.Direction)
		local UdE0 = U:Dot(edge0)
		local UdE1 = U:Dot(edge1)
		local UdDiff = U:Dot(diff)
		local VdE0 = V:Dot(edge0)
		local VdE1 = V:Dot(edge1)
		local VdDiff = V:Dot(diff)
		local invDet = 1 / (UdE0 * VdE1 - UdE1 * VdE0)

		-- Barycentric coordinates for the point of intersection.
		local b1 = (VdE1 * UdDiff - UdE1 * VdDiff) * invDet
		local b2 = (UdE0 * VdDiff - VdE0 * UdDiff) * invDet
		local b0 = 1 - b1 - b2

		if b0 >= 0 and b1 >= 0 and b2 >= 0 then
			-- Line parameter for the point of intersection.
			local DdE0 = line.Direction:Dot(edge0)
			local DdE1 = line.Direction:Dot(edge1)
			local DdDiff = line.Direction:Dot(diff)
			self.LineParam = b1 * DdE0 + b2 * DdE1 - DdDiff

			-- Barycentric coordinates for the point of intersection.
			self.TriangleBaryCoords = Vector3d.new(b0, b1, b2)

			-- The intersection point is inside or on the triangle.
			self.LineClosest = line.Origin + self.LineParam * line.Direction
			self.TriangleClosest = triangle.V0 + b1 * edge0 + b2 * edge1
			self.DistanceSquared = 0
			return 0
		end
	end

	-- Either (1) the line is not parallel to the triangle and the point of
	-- intersection of the line and the plane of the triangle is outside the
	-- triangle or (2) the line and triangle are parallel.  Regardless, the
	-- closest point on the triangle is on an edge of the triangle.  Compare
	-- the line to all three edges of the triangle.
	local sqrDist = MaxDouble
	local i0 = 2
	for i1 = 0, 2 do
		local segment = Segment3d.new(triangle:Get(i0), triangle:Get(i1))
		local queryLS = DistLine3Segment3.new(line, segment)
		local sqrDistTmp = queryLS:GetSquared()
		if sqrDistTmp < sqrDist then
			self.LineClosest = queryLS.LineClosest
			self.TriangleClosest = queryLS.SegmentClosest
			sqrDist = sqrDistTmp
			self.LineParam = queryLS.LineParameter
			local ratio = queryLS.SegmentParameter / segment.Extent
			local bary = { 0, 0, 0 }
			bary[i0 + 1] = 0.5 * (1 - ratio)
			bary[i1 + 1] = 1 - bary[i0 + 1]
			bary[3 - i0 - i1 + 1] = 0
			self.TriangleBaryCoords = Vector3d.new(bary[1], bary[2], bary[3])
		end
		i0 = i1
	end

	self.DistanceSquared = sqrDist
	return self.DistanceSquared
end

return DistLine3Triangle3
