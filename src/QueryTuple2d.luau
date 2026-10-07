--!native
-- Port of geometry3Sharp math/QueryTuple2d.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- variant of Wm5::Query2, for special case of 3 points. This is used in
-- IntrTriangle3Triangle3, to avoid allocating arrays
--
-- Luau API notes:
--   QueryTuple2d.new(v0, v1, v2)  /  QueryTuple2d.FromTuple(t)   (t = 1-based array of 3 Vector2d; C# Vector2dTuple3)
--   q:ToLine(i | test, v0, v1), q:ToTriangle(i | test, v0, v1, v2), q:ToCircumcircle(i | test, v0, v1, v2)
--     (first argument is a vertex index 0..2 or a Vector2d test point; vertex indices are 0-based)
--   static Dot, Det2, Det3
--   static Sort2(v0, v1) -> positive, v0, v1; Sort3(v0, v1, v2) -> positive, v0, v1, v2;
--   Sort4(v0, v1, v2, v3) -> positive, v0, v1, v2, v3          (C# Sort(ref ...) overloads)

local QueryTuple2d = {}
QueryTuple2d.__index = QueryTuple2d
QueryTuple2d.ClassName = "QueryTuple2d"

function QueryTuple2d.new(v0, v1, v2)
	return setmetatable({ mVertices = { v0, v1, v2 } }, QueryTuple2d)
end

function QueryTuple2d.FromTuple(tuple: { any })
	return setmetatable({ mVertices = { tuple[1], tuple[2], tuple[3] } }, QueryTuple2d)
end

-- Helper functions.
function QueryTuple2d.Dot(x0: number, y0: number, x1: number, y1: number): number
	return x0 * x1 + y0 * y1
end
local function Det2(x0: number, y0: number, x1: number, y1: number): number
	return x0 * y1 - x1 * y0
end
QueryTuple2d.Det2 = Det2
local function Det3(x0: number, y0: number, z0: number, x1: number, y1: number, z1: number, x2: number, y2: number, z2: number): number
	local c00 = y1 * z2 - y2 * z1
	local c01 = y2 * z0 - y0 * z2
	local c02 = y0 * z1 - y1 * z0
	return x0 * c00 + x1 * c01 + x2 * c02
end
QueryTuple2d.Det3 = Det3

-- Support for ordering a set of unique indices into the vertex pool.  On
-- output it is guaranteed that:  v0 < v1 < v2.  This is used to guarantee
-- consistent queries when the vertex ordering of a primitive is permuted,
-- a necessity when using floating-point arithmetic that suffers from
-- numerical round-off errors.  The input indices are considered the
-- positive ordering.  The output indices are either positively ordered
-- (an even number of transpositions occurs during sorting) or negatively
-- ordered (an odd number of transpositions occurs during sorting).  The
-- functions return 'true' for a positive ordering and 'false' for a
-- negative ordering.
local function Sort2(v0: number, v1: number): (boolean, number, number)
	if v0 < v1 then
		return true, v0, v1
	end
	return false, v1, v0
end
QueryTuple2d.Sort2 = Sort2

local function Sort3(v0: number, v1: number, v2: number): (boolean, number, number, number)
	local j0, j1, j2, positive
	if v0 < v1 then
		if v2 < v0 then
			j0, j1, j2, positive = 2, 0, 1, true
		elseif v2 < v1 then
			j0, j1, j2, positive = 0, 2, 1, false
		else
			j0, j1, j2, positive = 0, 1, 2, true
		end
	else
		if v2 < v1 then
			j0, j1, j2, positive = 2, 1, 0, false
		elseif v2 < v0 then
			j0, j1, j2, positive = 1, 2, 0, true
		else
			j0, j1, j2, positive = 1, 0, 2, false
		end
	end
	local value = { v0, v1, v2 }
	return positive, value[j0 + 1], value[j1 + 1], value[j2 + 1]
end
QueryTuple2d.Sort3 = Sort3

function QueryTuple2d.Sort4(v0: number, v1: number, v2: number, v3: number): (boolean, number, number, number, number)
	local j0, j1, j2, j3, positive
	if v0 < v1 then
		if v2 < v3 then
			if v1 < v2 then
				j0, j1, j2, j3, positive = 0, 1, 2, 3, true
			elseif v3 < v0 then
				j0, j1, j2, j3, positive = 2, 3, 0, 1, true
			elseif v2 < v0 then
				if v3 < v1 then
					j0, j1, j2, j3, positive = 2, 0, 3, 1, false
				else
					j0, j1, j2, j3, positive = 2, 0, 1, 3, true
				end
			else
				if v3 < v1 then
					j0, j1, j2, j3, positive = 0, 2, 3, 1, true
				else
					j0, j1, j2, j3, positive = 0, 2, 1, 3, false
				end
			end
		else
			if v1 < v3 then
				j0, j1, j2, j3, positive = 0, 1, 3, 2, false
			elseif v2 < v0 then
				j0, j1, j2, j3, positive = 3, 2, 0, 1, false
			elseif v3 < v0 then
				if v2 < v1 then
					j0, j1, j2, j3, positive = 3, 0, 2, 1, true
				else
					j0, j1, j2, j3, positive = 3, 0, 1, 2, false
				end
			else
				if v2 < v1 then
					j0, j1, j2, j3, positive = 0, 3, 2, 1, false
				else
					j0, j1, j2, j3, positive = 0, 3, 1, 2, true
				end
			end
		end
	else
		if v2 < v3 then
			if v0 < v2 then
				j0, j1, j2, j3, positive = 1, 0, 2, 3, false
			elseif v3 < v1 then
				j0, j1, j2, j3, positive = 2, 3, 1, 0, false
			elseif v2 < v1 then
				if v3 < v0 then
					j0, j1, j2, j3, positive = 2, 1, 3, 0, true
				else
					j0, j1, j2, j3, positive = 2, 1, 0, 3, false
				end
			else
				if v3 < v0 then
					j0, j1, j2, j3, positive = 1, 2, 3, 0, false
				else
					j0, j1, j2, j3, positive = 1, 2, 0, 3, true
				end
			end
		else
			if v0 < v3 then
				j0, j1, j2, j3, positive = 1, 0, 3, 2, true
			elseif v2 < v1 then
				j0, j1, j2, j3, positive = 3, 2, 1, 0, true
			elseif v3 < v1 then
				if v2 < v0 then
					j0, j1, j2, j3, positive = 3, 1, 2, 0, false
				else
					j0, j1, j2, j3, positive = 3, 1, 0, 2, true
				end
			else
				if v2 < v0 then
					j0, j1, j2, j3, positive = 1, 3, 2, 0, true
				else
					j0, j1, j2, j3, positive = 1, 3, 0, 2, false
				end
			end
		end
	end
	local value = { v0, v1, v2, v3 }
	return positive, value[j0 + 1], value[j1 + 1], value[j2 + 1], value[j3 + 1]
end

-- Returns:
--   +1, on right of line
--   -1, on left of line
--    0, on the line
-- ToLine(int i, v0, v1) or ToLine(Vector2d test, v0, v1)
function QueryTuple2d:ToLine(test: any, v0: number, v1: number): number
	if type(test) == "number" then
		test = self.mVertices[test + 1]
	end
	local positive
	positive, v0, v1 = Sort2(v0, v1)

	local vec0 = self.mVertices[v0 + 1]
	local vec1 = self.mVertices[v1 + 1]

	local x0 = test.x - vec0.x
	local y0 = test.y - vec0.y
	local x1 = vec1.x - vec0.x
	local y1 = vec1.y - vec0.y

	local det = Det2(x0, y0, x1, y1)
	if not positive then
		det = -det
	end

	return (det > 0) and 1 or ((det < 0) and -1 or 0)
end

-- Returns:
--   +1, outside triangle
--   -1, inside triangle
--    0, on triangle
-- ToTriangle(int i, v0, v1, v2) or ToTriangle(Vector2d test, v0, v1, v2)
function QueryTuple2d:ToTriangle(test: any, v0: number, v1: number, v2: number): number
	if type(test) == "number" then
		test = self.mVertices[test + 1]
	end
	local sign0 = self:ToLine(test, v1, v2)
	if sign0 > 0 then
		return 1
	end

	local sign1 = self:ToLine(test, v0, v2)
	if sign1 < 0 then
		return 1
	end

	local sign2 = self:ToLine(test, v0, v1)
	if sign2 > 0 then
		return 1
	end

	return (sign0 ~= 0 and sign1 ~= 0 and sign2 ~= 0) and -1 or 0
end

-- Returns:
--   +1, outside circumcircle of triangle
--   -1, inside circumcircle of triangle
--    0, on circumcircle of triangle
function QueryTuple2d:ToCircumcircle(test: any, v0: number, v1: number, v2: number): number
	if type(test) == "number" then
		test = self.mVertices[test + 1]
	end
	local positive
	positive, v0, v1, v2 = Sort3(v0, v1, v2)

	local vec0 = self.mVertices[v0 + 1]
	local vec1 = self.mVertices[v1 + 1]
	local vec2 = self.mVertices[v2 + 1]

	local s0x = vec0.x + test.x
	local d0x = vec0.x - test.x
	local s0y = vec0.y + test.y
	local d0y = vec0.y - test.y
	local s1x = vec1.x + test.x
	local d1x = vec1.x - test.x
	local s1y = vec1.y + test.y
	local d1y = vec1.y - test.y
	local s2x = vec2.x + test.x
	local d2x = vec2.x - test.x
	local s2y = vec2.y + test.y
	local d2y = vec2.y - test.y
	local z0 = s0x * d0x + s0y * d0y
	local z1 = s1x * d1x + s1y * d1y
	local z2 = s2x * d2x + s2y * d2y

	local det = Det3(d0x, d0y, z0, d1x, d1y, z1, d2x, d2y, z2)
	if not positive then
		det = -det
	end

	return (det < 0) and 1 or ((det > 0) and -1 or 0)
end

return QueryTuple2d
