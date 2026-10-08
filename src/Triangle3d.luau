--!native
-- Port of geometry3Sharp math/Triangle3.cs (Triangle3d; Triangle3f is not ported, see PORTING.md)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Mutable struct with fields V0, V1, V2 (Vector3d). Use :Clone() where C# copies it.

local MathUtil = require(script.Parent.MathUtil)

local Triangle3d = {}
Triangle3d.__index = Triangle3d
Triangle3d.ClassName = "Triangle3d"

function Triangle3d.new(v0, v1, v2)
	return setmetatable({ V0 = v0, V1 = v1, V2 = v2 }, Triangle3d)
end

function Triangle3d.Clone(self)
	return Triangle3d.new(self.V0, self.V1, self.V2)
end

-- this[key], key in 0..2
function Triangle3d.Get(self, key: number)
	if key == 0 then
		return self.V0
	elseif key == 1 then
		return self.V1
	end
	return self.V2
end

-- this[key] = value
function Triangle3d.Set(self, key: number, value)
	if key == 0 then
		self.V0 = value
	elseif key == 1 then
		self.V1 = value
	else
		self.V2 = value
	end
end

function Triangle3d.Normal(self)
	return MathUtil.Normal(self.V0, self.V1, self.V2)
end

function Triangle3d.Area(self): number
	return MathUtil.Area(self.V0, self.V1, self.V2)
end

function Triangle3d.AspectRatio(self): number
	return MathUtil.AspectRatio(self.V0, self.V1, self.V2)
end

-- PointAt(bary0, bary1, bary2) or PointAt(Vector3d bary)
function Triangle3d.PointAt(self, bary0: any, bary1: number?, bary2: number?)
	if type(bary0) ~= "number" then
		local b = bary0
		return b.x * self.V0 + b.y * self.V1 + b.z * self.V2
	end
	return bary0 * self.V0 + (bary1 :: number) * self.V1 + (bary2 :: number) * self.V2
end

function Triangle3d.BarycentricCoords(self, point)
	return MathUtil.BarycentricCoords(point, self.V0, self.V1, self.V2)
end

return Triangle3d
