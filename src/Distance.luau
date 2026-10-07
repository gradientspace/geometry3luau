--!native
-- Port of geometry3Sharp distance/Distance.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- (C#: "[TODO] get rid of this!!")
--   Distance.ClosestPointOnLineT(p0, dir, pt) -> number

local Distance = {}

function Distance.ClosestPointOnLineT(p0, dir, pt): number
	local t = (pt - p0):Dot(dir)
	return t
end

return Distance
