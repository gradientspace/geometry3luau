--!native
-- Port of geometry3Sharp intersection/IntrRay3Triangle3.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau API notes:
--   IntrRay3Triangle3.new(ray, triangle)    (Ray3d, Triangle3d; stored by reference)
--   q:Find() -> bool, q:Compute() -> q, q:IsSimpleIntersection()
--   fields: Quantity, Result (IntersectionResult), Type (IntersectionType), RayParameter, TriangleBaryCoords
--   C# properties Ray/Triangle -> q:Ray(), q:SetRay(r), q:Triangle(), q:SetTriangle(t) (setters reset Result)
--   static Intersects(ray, V0, V1, V2) -> bool, rayT     (C# Intersects(ref ray, ref V0, ref V1, ref V2, out rayT))
--   static IntersectsXYZ(ox,oy,oz, dx,dy,dz, ax,ay,az, bx,by,bz, cx,cy,cz) -> bool, rayT  (allocation-free)

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local IntersectionResult = require(script.Parent.IntersectionResult)
local IntersectionType = require(script.Parent.IntersectionType)

local ZeroTolerance = MathUtil.ZeroTolerance
local MaxDouble = MathUtil.MaxDouble

local IntrRay3Triangle3 = {}
IntrRay3Triangle3.__index = IntrRay3Triangle3
IntrRay3Triangle3.ClassName = "IntrRay3Triangle3"

function IntrRay3Triangle3.new(r, t)
	return setmetatable({
		ray = r,
		triangle = t,
		Quantity = 0,
		Result = IntersectionResult.NotComputed,
		Type = IntersectionType.Empty,
		RayParameter = 0,
		TriangleBaryCoords = Vector3d.Zero,
	}, IntrRay3Triangle3)
end

function IntrRay3Triangle3:Ray()
	return self.ray
end
function IntrRay3Triangle3:SetRay(value)
	self.ray = value
	self.Result = IntersectionResult.NotComputed
end
function IntrRay3Triangle3:Triangle()
	return self.triangle
end
function IntrRay3Triangle3:SetTriangle(value)
	self.triangle = value
	self.Result = IntersectionResult.NotComputed
end

function IntrRay3Triangle3:IsSimpleIntersection(): boolean
	return self.Result == IntersectionResult.Intersects and self.Type == IntersectionType.Point
end

function IntrRay3Triangle3:Compute()
	self:Find()
	return self
end

function IntrRay3Triangle3:Find(): boolean
	if self.Result ~= IntersectionResult.NotComputed then
		return self.Result ~= IntersectionResult.NoIntersection
	end
	local ray, triangle = self.ray, self.triangle

	-- Compute the offset origin, edges, and normal.
	local diff = ray.Origin - triangle.V0
	local edge1 = triangle.V1 - triangle.V0
	local edge2 = triangle.V2 - triangle.V0
	local normal = edge1:Cross(edge2)

	-- Solve Q + t*D = b1*E1 + b2*E2 (Q = kDiff, D = ray direction,
	-- E1 = kEdge1, E2 = kEdge2, N = Cross(E1,E2)) by
	--   |Dot(D,N)|*b1 = sign(Dot(D,N))*Dot(D,Cross(Q,E2))
	--   |Dot(D,N)|*b2 = sign(Dot(D,N))*Dot(D,Cross(E1,Q))
	--   |Dot(D,N)|*t = -sign(Dot(D,N))*Dot(Q,N)
	local DdN = ray.Direction:Dot(normal)
	local sign
	if DdN > ZeroTolerance then
		sign = 1
	elseif DdN < -ZeroTolerance then
		sign = -1
		DdN = -DdN
	else
		-- Ray and triangle are parallel, call it a "no intersection"
		-- even if the ray does intersect.
		self.Result = IntersectionResult.NoIntersection
		return false
	end

	local DdQxE2 = sign * ray.Direction:Dot(diff:Cross(edge2))
	if DdQxE2 >= 0 then
		local DdE1xQ = sign * ray.Direction:Dot(edge1:Cross(diff))
		if DdE1xQ >= 0 then
			if DdQxE2 + DdE1xQ <= DdN then
				-- Line intersects triangle, check if ray does.
				local QdN = -sign * diff:Dot(normal)
				if QdN >= 0 then
					-- Ray intersects triangle.
					local inv = 1 / DdN
					self.RayParameter = QdN * inv
					local mTriBary1 = DdQxE2 * inv
					local mTriBary2 = DdE1xQ * inv
					self.TriangleBaryCoords = Vector3d.new(1 - mTriBary1 - mTriBary2, mTriBary1, mTriBary2)
					self.Type = IntersectionType.Point
					self.Quantity = 1
					self.Result = IntersectionResult.Intersects
					return true
				end
				-- else: t < 0, no intersection
			end
			-- else: b1+b2 > 1, no intersection
		end
		-- else: b2 < 0, no intersection
	end
	-- else: b1 < 0, no intersection

	self.Result = IntersectionResult.NoIntersection
	return false
end

-- minimal intersection test on scalars, computes ray-t. Returns (hit, rayT), rayT = double.MaxValue on miss.
local function intersects_xyz(
	ox: number,
	oy: number,
	oz: number,
	rdx: number,
	rdy: number,
	rdz: number,
	v0x: number,
	v0y: number,
	v0z: number,
	v1x: number,
	v1y: number,
	v1z: number,
	v2x: number,
	v2y: number,
	v2z: number
): (boolean, number)
	-- Compute the offset origin, edges, and normal.
	local dx, dy, dz = ox - v0x, oy - v0y, oz - v0z -- diff = ray.Origin - V0
	local e1x, e1y, e1z = v1x - v0x, v1y - v0y, v1z - v0z -- edge1 = V1 - V0
	local e2x, e2y, e2z = v2x - v0x, v2y - v0y, v2z - v0z -- edge2 = V2 - V0
	-- normal = edge1.Cross(edge2)
	local nx, ny, nz = e1y * e2z - e1z * e2y, e1z * e2x - e1x * e2z, e1x * e2y - e1y * e2x

	local DdN = rdx * nx + rdy * ny + rdz * nz
	local sign
	if DdN > ZeroTolerance then
		sign = 1
	elseif DdN < -ZeroTolerance then
		sign = -1
		DdN = -DdN
	else
		-- Ray and triangle are parallel, call it a "no intersection"
		-- even if the ray does intersect.
		return false, MaxDouble
	end

	-- cross = diff.Cross(edge2)
	local cx, cy, cz = dy * e2z - dz * e2y, dz * e2x - dx * e2z, dx * e2y - dy * e2x
	local DdQxE2 = sign * (rdx * cx + rdy * cy + rdz * cz)
	if DdQxE2 >= 0 then
		-- cross = edge1.Cross(diff)
		cx, cy, cz = e1y * dz - e1z * dy, e1z * dx - e1x * dz, e1x * dy - e1y * dx
		local DdE1xQ = sign * (rdx * cx + rdy * cy + rdz * cz)
		if DdE1xQ >= 0 then
			if DdQxE2 + DdE1xQ <= DdN then
				-- Line intersects triangle, check if ray does.
				local QdN = -sign * (dx * nx + dy * ny + dz * nz)
				if QdN >= 0 then
					-- Ray intersects triangle.
					local inv = 1 / DdN
					return true, QdN * inv
				end
			end
		end
	end
	return false, MaxDouble
end
IntrRay3Triangle3.IntersectsXYZ = intersects_xyz

-- C# static bool Intersects(ref Ray3d ray, ref Vector3d V0, ref Vector3d V1, ref Vector3d V2, out double rayT)
function IntrRay3Triangle3.Intersects(ray, V0, V1, V2): (boolean, number)
	local o, d = ray.Origin, ray.Direction
	return intersects_xyz(o.x, o.y, o.z, d.x, d.y, d.z, V0.x, V0.y, V0.z, V1.x, V1.y, V1.z, V2.x, V2.y, V2.z)
end

return IntrRay3Triangle3
