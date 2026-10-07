--!native
-- Port of geometry3Sharp intersection/IntersectionLowLevel.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
--   IntersectionLowLevel.Clip(denom, numer, t0, t1) -> bool, t0, t1     (C# ref t0, ref t1)

local IntersectionLowLevel = {}

-- this function is from various WildMagic/GTEngine intersection tests
-- clips segment [t0,t1] against line/plane direction/origin (in plane equation)
function IntersectionLowLevel.Clip(denom: number, numer: number, t0: number, t1: number): (boolean, number, number)
	-- Return value is 'true' if line segment intersects the current test
	-- plane.  Otherwise 'false' is returned in which case the line segment
	-- is entirely clipped.
	if denom > 0 then
		if numer > denom * t1 then
			return false, t0, t1
		end
		if numer > denom * t0 then
			t0 = numer / denom
		end
		return true, t0, t1
	elseif denom < 0 then
		if numer > denom * t0 then
			return false, t0, t1
		end
		if numer > denom * t1 then
			t1 = numer / denom
		end
		return true, t0, t1
	else
		return numer <= 0, t0, t1
	end
end

return IntersectionLowLevel
