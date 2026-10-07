--!native
-- Port of geometry3Sharp math/Triangle2.cs (Triangle2d; also used for Triangle2f)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Mutable struct with fields V0, V1, V2 (Vector2d). Use :Clone() where C# copies it.
--   Triangle2d.new(v0, v1, v2); tri:Get(key) / tri:Set(key, value) for the C# indexer (key 0..2)
--   tri:PointAt(bary0, bary1, bary2) or tri:PointAt(Vector3d bary)

local Triangle2d = {}
Triangle2d.__index = Triangle2d
Triangle2d.ClassName = "Triangle2d"

function Triangle2d.new(v0, v1, v2)
	return setmetatable({ V0 = v0, V1 = v1, V2 = v2 }, Triangle2d)
end

function Triangle2d:Clone()
	return Triangle2d.new(self.V0, self.V1, self.V2)
end

-- this[key], key in 0..2
function Triangle2d:Get(key: number)
	return (key == 0) and self.V0 or ((key == 1) and self.V1 or self.V2)
end

-- this[key] = value
function Triangle2d:Set(key: number, value)
	if key == 0 then
		self.V0 = value
	elseif key == 1 then
		self.V1 = value
	else
		self.V2 = value
	end
end

-- PointAt(bary0, bary1, bary2) or PointAt(Vector3d bary)
function Triangle2d:PointAt(bary0: any, bary1: number?, bary2: number?)
	if type(bary0) ~= "number" then
		local b = bary0
		return b.x * self.V0 + b.y * self.V1 + b.z * self.V2
	end
	return bary0 * self.V0 + (bary1 :: number) * self.V1 + (bary2 :: number) * self.V2
end

return Triangle2d
