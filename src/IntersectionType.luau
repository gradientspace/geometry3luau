-- Port of the IntersectionType enum from geometry3Sharp queries/IntersectionUtil.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- IntersectionType.Empty == 0, ... (C# enum values). Also re-exported as IntersectionUtil.IntersectionType.

return table.freeze({
	Empty = 0,
	Point = 1,
	Segment = 2,
	Line = 3,
	Polygon = 4,
	Plane = 5,
	Unknown = 6,
})
