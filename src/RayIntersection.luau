--!native
-- Port of geometry3Sharp queries/RayIntersection.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau API notes (static functions; out fRayT becomes the second return value). The C# Vector3f and
-- Vector3d overloads are merged (all math is done in doubles):
--   RayIntersection.Sphere(vOrigin, vDirection, vCenter, fRadius) -> bHit, fRayT   (|t|)
--   RayIntersection.SphereSigned(vOrigin, vDirection, vCenter, fRadius) -> bHit, fRayT
--   RayIntersection.InfiniteCylinder(vOrigin, vDirection, vCylOrigin, vCylAxis, fRadius) -> bHit, fRayT   (|t|)
--   RayIntersection.InfiniteCylinderSigned(vOrigin, vDirection, vCylOrigin, vCylAxis, fRadius) -> bHit, fRayT

local RayIntersection = {}

-- basic ray-sphere intersection
function RayIntersection.Sphere(vOrigin, vDirection, vCenter, fRadius: number): (boolean, number)
	local bHit, fRayT = RayIntersection.SphereSigned(vOrigin, vDirection, vCenter, fRadius)
	fRayT = math.abs(fRayT)
	return bHit, fRayT
end

function RayIntersection.SphereSigned(vOrigin, vDirection, vCenter, fRadius: number): (boolean, number)
	local fRayT = 0.0
	local m = vOrigin - vCenter
	local b = m:Dot(vDirection)
	local c = m:Dot(m) - fRadius * fRadius

	-- Exit if r's origin outside s (c > 0) and r pointing away from s (b > 0)
	if c > 0.0 and b > 0.0 then
		return false, fRayT
	end
	local discr = b * b - c
	-- A negative discriminant corresponds to ray missing sphere
	if discr < 0.0 then
		return false, fRayT
	end
	-- Ray now found to intersect sphere, compute smallest t value of intersection
	fRayT = -b - math.sqrt(discr)
	return true, fRayT
end

function RayIntersection.InfiniteCylinder(vOrigin, vDirection, vCylOrigin, vCylAxis, fRadius: number): (boolean, number)
	local bHit, fRayT = RayIntersection.InfiniteCylinderSigned(vOrigin, vDirection, vCylOrigin, vCylAxis, fRadius)
	fRayT = math.abs(fRayT)
	return bHit, fRayT
end

function RayIntersection.InfiniteCylinderSigned(vOrigin, vDirection, vCylOrigin, vCylAxis, fRadius: number): (boolean, number)
	-- [RMS] ugh this is shit...not even sure it works in general, but works for a ray inside cylinder
	local fRayT = 0.0

	local AB = vCylAxis
	local AO = (vOrigin - vCylOrigin)
	if AO:DistanceSquared(AO:Dot(AB) * AB) > fRadius * fRadius then
		return false, fRayT
	end

	local AOxAB = AO:Cross(AB)
	local VxAB = vDirection:Cross(AB)
	local ab2 = AB:Dot(AB)
	local a = VxAB:Dot(VxAB)
	local b = 2 * VxAB:Dot(AOxAB)
	local c = AOxAB:Dot(AOxAB) - (fRadius * fRadius * ab2)

	local discrim = b * b - 4 * a * c
	if discrim <= 0 then
		return false, fRayT
	end
	discrim = math.sqrt(discrim)
	fRayT = (-b - discrim) / (2 * a)
	--float t1 = (-b + (float)discrim) / (2 * a);

	return true, fRayT
end

return RayIntersection
