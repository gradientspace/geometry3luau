-- Port of the IntersectionResult enum from geometry3Sharp queries/IntersectionUtil.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- IntersectionResult.NotComputed == 0, ... (C# enum values). Also re-exported as IntersectionUtil.IntersectionResult.

return table.freeze({
	NotComputed = 0,
	Intersects = 1,
	NoIntersection = 2,
	InvalidQuery = 3,
})
