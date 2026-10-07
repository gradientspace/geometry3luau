--!native
-- Port of geometry3Sharp queries/IntersectionUtil.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau API notes:
--   IntersectionUtil.IntersectionResult / IntersectionUtil.IntersectionType: the enums (also standalone
--     modules IntersectionResult.luau / IntersectionType.luau)
--   LinearIntersection struct -> table { intersects, numIntersections, parameter = Interval1d }
--     IntersectionUtil.NewLinearIntersection(), IntersectionUtil.LinearIntersectionNone()
--   Intersects(rayOrigin, rayDirection, V0, V1, V2) -> bool
--   LineSphereTest(lineOrigin, lineDirection, sphereCenter, sphereRadius) -> bool
--   LineSphere(lineOrigin, lineDirection, sphereCenter, sphereRadius, result?) -> bool, result
--     (C# overloads: the ref-result version and the one returning a LinearIntersection are merged;
--      if result is omitted a new LinearIntersection is created)
--   RaySphereTest(...), RaySphere(rayOrigin, rayDirection, sphereCenter, sphereRadius, result?) -> bool, result
--   Note: the C# RaySphere overload that returns a LinearIntersection calls LineSphere (so it does not
--   clip to the ray); here both RaySphere forms clip to the ray.

local MathUtil = require(script.Parent.MathUtil)
local Interval1d = require(script.Parent.Interval1d)

local ZeroTolerance = MathUtil.ZeroTolerance

local IntersectionUtil = {}

IntersectionUtil.IntersectionResult = require(script.Parent.IntersectionResult)
IntersectionUtil.IntersectionType = require(script.Parent.IntersectionType)

-- returned by linear-primitive intersection functions
function IntersectionUtil.NewLinearIntersection()
	return { intersects = false, numIntersections = 0, parameter = Interval1d.Zero() }
end
-- LinearIntersection.NoIntersection (fresh value)
function IntersectionUtil.LinearIntersectionNone()
	return { intersects = false, numIntersections = 0, parameter = Interval1d.Zero() }
end

-- same code as IntrRay3Triangle3, but can be called w/o constructing additional data structures
function IntersectionUtil.Intersects(rayOrigin, rayDirection, V0, V1, V2): boolean
	-- Compute the offset origin, edges, and normal.
	local diff = rayOrigin - V0
	local edge1 = V1 - V0
	local edge2 = V2 - V0
	local normal = edge1:Cross(edge2)

	-- Solve Q + t*D = b1*E1 + b2*E2 (Q = kDiff, D = ray direction,
	-- E1 = kEdge1, E2 = kEdge2, N = Cross(E1,E2)) by
	--   |Dot(D,N)|*b1 = sign(Dot(D,N))*Dot(D,Cross(Q,E2))
	--   |Dot(D,N)|*b2 = sign(Dot(D,N))*Dot(D,Cross(E1,Q))
	--   |Dot(D,N)|*t = -sign(Dot(D,N))*Dot(Q,N)
	local DdN = rayDirection:Dot(normal)
	local sign
	if DdN > ZeroTolerance then
		sign = 1
	elseif DdN < -ZeroTolerance then
		sign = -1
		DdN = -DdN
	else
		-- Ray and triangle are parallel, call it a "no intersection"
		-- even if the ray does intersect.
		return false
	end

	local DdQxE2 = sign * rayDirection:Dot(diff:Cross(edge2))
	if DdQxE2 >= 0 then
		local DdE1xQ = sign * rayDirection:Dot(edge1:Cross(diff))
		if DdE1xQ >= 0 then
			if DdQxE2 + DdE1xQ <= DdN then
				-- Line intersects triangle, check if ray does.
				local QdN = -sign * diff:Dot(normal)
				if QdN >= 0 then
					-- Ray intersects triangle.
					return true
				end
				-- else: t < 0, no intersection
			end
			-- else: b1+b2 > 1, no intersection
		end
		-- else: b2 < 0, no intersection
	end
	-- else: b1 < 0, no intersection
	return false
end

-- Test if line intersects sphere
function IntersectionUtil.LineSphereTest(lineOrigin, lineDirection, sphereCenter, sphereRadius: number): boolean
	-- [RMS] adapted from GeometricTools GTEngine
	-- https://www.geometrictools.com/GTEngine/Include/Mathematics/GteIntrLine3Sphere3.h

	-- The sphere is (X-C)^T*(X-C)-1 = 0 and the line is X = P+t*D.
	-- Substitute the line equation into the sphere equation to obtain a
	-- quadratic equation Q(t) = t^2 + 2*a1*t + a0 = 0, where a1 = D^T*(P-C),
	-- and a0 = (P-C)^T*(P-C)-1.
	local diff = lineOrigin - sphereCenter
	local a0 = diff:LengthSquared() - sphereRadius * sphereRadius
	local a1 = lineDirection:Dot(diff)

	-- Intersection occurs when Q(t) has real roots.
	local discr = a1 * a1 - a0
	return discr >= 0
end

-- Intersect ray with sphere and return intersection info (# hits, ray parameters)
function IntersectionUtil.LineSphere(lineOrigin, lineDirection, sphereCenter, sphereRadius: number, result: any?)
	local res = result or IntersectionUtil.NewLinearIntersection()
	-- [RMS] adapted from GeometricTools GTEngine
	-- https://www.geometrictools.com/GTEngine/Include/Mathematics/GteIntrLine3Sphere3.h

	-- The sphere is (X-C)^T*(X-C)-1 = 0 and the line is X = P+t*D.
	-- Substitute the line equation into the sphere equation to obtain a
	-- quadratic equation Q(t) = t^2 + 2*a1*t + a0 = 0, where a1 = D^T*(P-C),
	-- and a0 = (P-C)^T*(P-C)-1.
	local diff = lineOrigin - sphereCenter
	local a0 = diff:LengthSquared() - sphereRadius * sphereRadius
	local a1 = lineDirection:Dot(diff)

	-- Intersection occurs when Q(t) has real roots.
	local discr = a1 * a1 - a0
	if discr > 0 then
		res.intersects = true
		res.numIntersections = 2
		local root = math.sqrt(discr)
		res.parameter.a = -a1 - root
		res.parameter.b = -a1 + root
	elseif discr < 0 then
		res.intersects = false
		res.numIntersections = 0
	else
		res.intersects = true
		res.numIntersections = 1
		res.parameter.a = -a1
		res.parameter.b = -a1
	end
	return res.intersects, res
end

-- Test if ray intersects sphere
function IntersectionUtil.RaySphereTest(rayOrigin, rayDirection, sphereCenter, sphereRadius: number): boolean
	-- [RMS] adapted from GeometricTools GTEngine
	-- https://www.geometrictools.com/GTEngine/Include/Mathematics/GteIntrRay3Sphere3.h

	-- The sphere is (X-C)^T*(X-C)-1 = 0 and the line is X = P+t*D.
	-- Substitute the line equation into the sphere equation to obtain a
	-- quadratic equation Q(t) = t^2 + 2*a1*t + a0 = 0, where a1 = D^T*(P-C),
	-- and a0 = (P-C)^T*(P-C)-1.
	local diff = rayOrigin - sphereCenter
	local a0 = diff:LengthSquared() - sphereRadius * sphereRadius
	if a0 <= 0 then
		return true -- P is inside the sphere.
	end
	-- else: P is outside the sphere
	local a1 = rayDirection:Dot(diff)
	if a1 >= 0 then
		return false
	end

	-- Intersection occurs when Q(t) has double roots.
	local discr = a1 * a1 - a0
	return discr >= 0
end

-- Intersect ray with sphere and return intersection info (# hits, ray parameters)
function IntersectionUtil.RaySphere(rayOrigin, rayDirection, sphereCenter, sphereRadius: number, result: any?)
	local res = result or IntersectionUtil.NewLinearIntersection()
	-- [RMS] adapted from GeometricTools GTEngine
	-- https://www.geometrictools.com/GTEngine/Include/Mathematics/GteIntrRay3Sphere3.h
	IntersectionUtil.LineSphere(rayOrigin, rayDirection, sphereCenter, sphereRadius, res)
	if res.intersects then
		-- The line containing the ray intersects the sphere; the t-interval
		-- is [t0,t1].  The ray intersects the sphere as long as [t0,t1]
		-- overlaps the ray t-interval [0,+infinity).
		if res.parameter.b < 0 then
			res.intersects = false
			res.numIntersections = 0
		elseif res.parameter.a < 0 then
			res.numIntersections -= 1
			res.parameter.a = res.parameter.b
		end
	end
	return res.intersects, res
end

return IntersectionUtil
