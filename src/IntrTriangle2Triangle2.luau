--!native
-- Port of geometry3Sharp intersection/IntrTriangle2Triangle2.cs (ported from WildMagic5)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau API notes:
--   IntrTriangle2Triangle2.new(t0, t1)    (Triangle2d; triangles must be counter-clockwise for Find())
--   q:Test(), q:Find(), q:Compute(), q:IsSimpleIntersection()
--   q:Triangle0()/SetTriangle0(t), q:Triangle1()/SetTriangle1(t)
--   fields Quantity, Result, Type, Points: 1-based array (6 slots after Find), first Quantity valid
--   static WhichSide(V, P, D)
--   static ClipConvexPolygonAgainstLine(N, c, quantity, V) -> quantity   (V: 1-based array, modified in place)

local Vector2d = require(script.Parent.Vector2d)
local IntersectionResult = require(script.Parent.IntersectionResult)
local IntersectionType = require(script.Parent.IntersectionType)

local IntrTriangle2Triangle2 = {}
IntrTriangle2Triangle2.__index = IntrTriangle2Triangle2
IntrTriangle2Triangle2.ClassName = "IntrTriangle2Triangle2"

function IntrTriangle2Triangle2.new(t0, t1)
	return setmetatable({
		triangle0 = t0,
		triangle1 = t1,
		Quantity = 0,
		Result = IntersectionResult.NotComputed,
		Type = IntersectionType.Empty,
		Points = nil,
	}, IntrTriangle2Triangle2)
end

function IntrTriangle2Triangle2:Triangle0()
	return self.triangle0
end
function IntrTriangle2Triangle2:SetTriangle0(value)
	self.triangle0 = value
	self.Result = IntersectionResult.NotComputed
end
function IntrTriangle2Triangle2:Triangle1()
	return self.triangle1
end
function IntrTriangle2Triangle2:SetTriangle1(value)
	self.triangle1 = value
	self.Result = IntersectionResult.NotComputed
end

function IntrTriangle2Triangle2:IsSimpleIntersection(): boolean
	return self.Result == IntersectionResult.Intersects and self.Type == IntersectionType.Point
end

local WhichSide

function IntrTriangle2Triangle2:Test(): boolean
	local triangle0, triangle1 = self.triangle0, self.triangle1

	-- Test edges of triangle0 for separation.
	local i1 = 2
	for i0 = 0, 2 do
		-- Test axis V0[i1] + t*perp(V0[i0]-V0[i1]), perp(x,y) = (y,-x).
		local dir = Vector2d.new(triangle0:Get(i0).y - triangle0:Get(i1).y, triangle0:Get(i1).x - triangle0:Get(i0).x)
		if WhichSide(triangle1, triangle0:Get(i1), dir) > 0 then
			-- Triangle1 is entirely on positive side of triangle0 edge.
			return false
		end
		i1 = i0
	end

	-- Test edges of triangle1 for separation.
	i1 = 2
	for i0 = 0, 2 do
		-- Test axis V1[i1] + t*perp(V1[i0]-V1[i1]), perp(x,y) = (y,-x).
		local dir = Vector2d.new(triangle1:Get(i0).y - triangle1:Get(i1).y, triangle1:Get(i1).x - triangle1:Get(i0).x)
		if WhichSide(triangle0, triangle1:Get(i1), dir) > 0 then
			-- Triangle0 is entirely on positive side of triangle1 edge.
			return false
		end
		i1 = i0
	end

	return true
end

function IntrTriangle2Triangle2:Compute()
	self:Find()
	return self
end

function IntrTriangle2Triangle2:Find(): boolean
	if self.Result ~= IntersectionResult.NotComputed then
		return self.Result == IntersectionResult.Intersects
	end
	local triangle0, triangle1 = self.triangle0, self.triangle1

	-- The potential intersection is initialized to triangle1.  The set of
	-- vertices is refined based on clipping against each edge of triangle0.
	self.Quantity = 3
	self.Points = table.create(6, Vector2d.Zero)
	for i = 0, 2 do
		self.Points[i + 1] = triangle1:Get(i)
	end

	local i1 = 2
	for i0 = 0, 2 do
		-- Clip against edge <V0[i1],V0[i0]>.
		local N = Vector2d.new(triangle0:Get(i1).y - triangle0:Get(i0).y, triangle0:Get(i0).x - triangle0:Get(i1).x)
		local c = N:Dot(triangle0:Get(i1))
		self.Quantity = IntrTriangle2Triangle2.ClipConvexPolygonAgainstLine(N, c, self.Quantity, self.Points)
		if self.Quantity == 0 then
			-- Triangle completely clipped, no intersection occurs.
			self.Type = IntersectionType.Empty
		elseif self.Quantity == 1 then
			self.Type = IntersectionType.Point
		elseif self.Quantity == 2 then
			self.Type = IntersectionType.Segment
		else
			self.Type = IntersectionType.Polygon
		end
		i1 = i0
	end

	self.Result = (self.Type ~= IntersectionType.Empty) and IntersectionResult.Intersects or IntersectionResult.NoIntersection
	return self.Result == IntersectionResult.Intersects
end

function WhichSide(V, P, D): number
	-- Vertices are projected to the form P+t*D.  Return value is +1 if all
	-- t > 0, -1 if all t < 0, 0 otherwise, in which case the line splits the
	-- triangle.
	local positive, negative, zero = 0, 0, 0
	for i = 0, 2 do
		local t = D:Dot(V:Get(i) - P)
		if t > 0 then
			positive += 1
		elseif t < 0 then
			negative += 1
		else
			zero += 1
		end

		if positive > 0 and negative > 0 then
			return 0
		end
	end
	return (zero == 0) and ((positive > 0) and 1 or -1) or 0
end
IntrTriangle2Triangle2.WhichSide = WhichSide

-- Vin is input polygon vertices, returns clipped polygon, vertex count of
--   clipped polygon is returned in quantity
-- **NOTE** returned array may have more elements than quantity!!
-- (V is a 1-based Luau array; C# V[i] -> V[i+1])
function IntrTriangle2Triangle2.ClipConvexPolygonAgainstLine(N, c: number, quantity: number, V: { any }): number
	-- The input vertices are assumed to be in counterclockwise order.  The
	-- ordering is an invariant of this function.

	-- Test on which side of line the vertices are.
	local positive, negative, pIndex = 0, 0, -1
	local test = table.create(6, 0)
	for i = 0, quantity - 1 do
		test[i + 1] = N:Dot(V[i + 1]) - c
		if test[i + 1] > 0 then
			positive += 1
			if pIndex < 0 then
				pIndex = i
			end
		elseif test[i + 1] < 0 then
			negative += 1
		end
	end

	if positive > 0 then
		if negative > 0 then
			-- Line transversely intersects polygon.
			local CV = table.create(6, Vector2d.Zero)
			local cQuantity = 0
			local cur, prv, t

			if pIndex > 0 then
				-- First clip vertex on line.
				cur = pIndex
				prv = cur - 1
				t = test[cur + 1] / (test[cur + 1] - test[prv + 1])
				cQuantity += 1
				CV[cQuantity] = V[cur + 1] + t * (V[prv + 1] - V[cur + 1])

				-- Vertices on positive side of line.
				while cur < quantity and test[cur + 1] > 0 do
					cQuantity += 1
					CV[cQuantity] = V[cur + 1]
					cur += 1
				end

				-- Last clip vertex on line.
				if cur < quantity then
					prv = cur - 1
				else
					cur = 0
					prv = quantity - 1
				end
				t = test[cur + 1] / (test[cur + 1] - test[prv + 1])
				cQuantity += 1
				CV[cQuantity] = V[cur + 1] + t * (V[prv + 1] - V[cur + 1])
			else -- pIndex is 0
				-- Vertices on positive side of line.
				cur = 0
				while cur < quantity and test[cur + 1] > 0 do
					cQuantity += 1
					CV[cQuantity] = V[cur + 1]
					cur += 1
				end

				-- Last clip vertex on line.
				prv = cur - 1
				t = test[cur + 1] / (test[cur + 1] - test[prv + 1])
				cQuantity += 1
				CV[cQuantity] = V[cur + 1] + t * (V[prv + 1] - V[cur + 1])

				-- Skip vertices on negative side.
				while cur < quantity and test[cur + 1] <= 0 do
					cur += 1
				end

				-- First clip vertex on line.
				if cur < quantity then
					prv = cur - 1
					t = test[cur + 1] / (test[cur + 1] - test[prv + 1])
					cQuantity += 1
					CV[cQuantity] = V[cur + 1] + t * (V[prv + 1] - V[cur + 1])

					-- Vertices on positive side of line.
					while cur < quantity and test[cur + 1] > 0 do
						cQuantity += 1
						CV[cQuantity] = V[cur + 1]
						cur += 1
					end
				else
					-- cur = 0
					prv = quantity - 1
					t = test[1] / (test[1] - test[prv + 1])
					cQuantity += 1
					CV[cQuantity] = V[1] + t * (V[prv + 1] - V[1])
				end
			end

			quantity = cQuantity
			for i = 1, cQuantity do
				V[i] = CV[i]
			end
		end
		-- else polygon fully on positive side of line, nothing to do.
	else
		-- Polygon does not intersect positive side of line, clip all.
		quantity = 0
	end
	return quantity
end

return IntrTriangle2Triangle2
