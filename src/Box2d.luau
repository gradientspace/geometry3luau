--!native
-- Port of geometry3Sharp math/Box2.cs (Box2d; partially based on WildMagic5 Box2)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- A box has center C, axis directions U[0] and U[1] (perpendicular and
-- unit-length vectors), and extents e[0] and e[1] (nonnegative numbers).
-- A point X = C+y[0]*U[0]+y[1]*U[1] is inside or on the box whenever
-- |y[i]| <= e[i] for all i.
--
-- Luau API notes:
--   Box2d is a MUTABLE struct (fields Center, AxisX, AxisY, Extent: immutable Vector2d). Use :Clone()
--   where C# would copy it.
--   Constructors (Box2d.new dispatches on its arguments like the C# overloads):
--     Box2d.new(center)                          new Box2d(Vector2d center)
--     Box2d.new(center, x, y, extent)            new Box2d(center, x, y, extent)
--     Box2d.new(center, extent)  / Box2d.FromCenterExtent(center, extent)
--     Box2d.new(aaBox)           / Box2d.FromAABB(aaBox)
--     Box2d.new(seg)             / Box2d.FromSegment(seg)
--     Box2d.new()                                new Box2d() (all-zero default struct)
--     Box2d.Empty()                              (static readonly -> fresh box)
--   C# properties -> methods: MaxExtent(), MinExtent(), Diagonal(), Area()
--   ComputeVertices() -> 1-based array of 4 Vector2d; ComputeVerticesInto(vertex) fills vertex[1..4]
--     (covers the C# Vector2d[] and ref Vector2dTuple4 overloads)
--   Contain(v | Box2d) dispatches to ContainPoint / ContainBox
--   RotateAxes(m): m is a Matrix2d (anything with :Multiply(Vector2d))
--   Box2d.Merge(box0, box1) -> Box2d   (C# ref arguments; inputs are not modified)

local Vector2d = require(script.Parent.Vector2d)

local V2 = Vector2d.new
local abs = math.abs

local Box2d = {}
Box2d.__index = Box2d
Box2d.ClassName = "Box2d"

export type Box2d = typeof(setmetatable(
	{} :: { Center: Vector2d.Vector2d, AxisX: Vector2d.Vector2d, AxisY: Vector2d.Vector2d, Extent: Vector2d.Vector2d },
	Box2d
))

local function make(center, x, y, extent): Box2d
	return setmetatable({ Center = center, AxisX = x, AxisY = y, Extent = extent }, Box2d) :: any
end

function Box2d.FromCenterExtent(center: Vector2d.Vector2d, extent: Vector2d.Vector2d): Box2d
	return make(center, Vector2d.AxisX, Vector2d.AxisY, extent)
end

function Box2d.FromAABB(aaBox: any): Box2d
	local extent = 0.5 * aaBox:Diagonal()
	local center = aaBox.Min + extent
	return make(center, Vector2d.AxisX, Vector2d.AxisY, extent)
end

function Box2d.FromSegment(seg: any): Box2d
	return make(seg.Center, seg.Direction, seg.Direction:Perp(), V2(seg.Extent, 0))
end

function Box2d.new(a: any?, b: any?, c: any?, d: any?): Box2d
	if a == nil then
		-- new Box2d() (default struct)
		return make(Vector2d.Zero, Vector2d.Zero, Vector2d.Zero, Vector2d.Zero)
	end
	local ta = a :: any
	if type(ta.Min) == "table" and type(ta.Max) == "table" then
		return Box2d.FromAABB(ta)
	elseif ta.Direction ~= nil and ta.Extent ~= nil then
		return Box2d.FromSegment(ta)
	elseif d ~= nil then
		return make(ta, b, c, d)
	elseif b ~= nil then
		return Box2d.FromCenterExtent(ta, b)
	end
	return make(ta, Vector2d.AxisX, Vector2d.AxisY, Vector2d.Zero)
end

function Box2d.Empty(): Box2d
	return Box2d.new(Vector2d.Zero)
end

function Box2d.Clone(self: Box2d): Box2d
	return make(self.Center, self.AxisX, self.AxisY, self.Extent)
end

function Box2d.Axis(self: Box2d, i: number): Vector2d.Vector2d
	return (i == 0) and self.AxisX or self.AxisY
end

function Box2d.ComputeVerticesInto(self: Box2d, vertex: { Vector2d.Vector2d })
	local Center = self.Center
	local extAxis0 = self.Extent.x * self.AxisX
	local extAxis1 = self.Extent.y * self.AxisY
	vertex[1] = Center - extAxis0 - extAxis1
	vertex[2] = Center + extAxis0 - extAxis1
	vertex[3] = Center + extAxis0 + extAxis1
	vertex[4] = Center - extAxis0 + extAxis1
end

function Box2d.ComputeVertices(self: Box2d): { Vector2d.Vector2d }
	local v = table.create(4)
	self:ComputeVerticesInto(v)
	return v
end

-- g3 extensions
function Box2d.MaxExtent(self: Box2d): number
	return math.max(self.Extent.x, self.Extent.y)
end
function Box2d.MinExtent(self: Box2d): number
	return math.min(self.Extent.x, self.Extent.y)
end
function Box2d.Diagonal(self: Box2d): Vector2d.Vector2d
	local e = self.Extent
	return (e.x * self.AxisX + e.y * self.AxisY) - (-e.x * self.AxisX - e.y * self.AxisY)
end
function Box2d.Area(self: Box2d): number
	return 2 * self.Extent.x * 2 * self.Extent.y
end

function Box2d.ContainPoint(self: Box2d, v: Vector2d.Vector2d)
	local lv = v - self.Center
	for k = 0, 1 do
		local axis = self:Axis(k)
		local t = lv:Dot(axis)
		local ek = self.Extent:Get(k)
		if abs(t) > ek then
			local min, max = -ek, ek
			if t < min then
				min = t
			elseif t > max then
				max = t
			end
			self.Extent = self.Extent:With(k, (max - min) * 0.5)
			self.Center = self.Center + ((max + min) * 0.5) * axis
		end
	end
end

-- I think this can be more efficient...no? At least could combine
-- all the axis-interval updates before updating Center...
function Box2d.ContainBox(self: Box2d, o: Box2d)
	local v = o:ComputeVertices()
	for k = 1, 4 do
		self:ContainPoint(v[k])
	end
end

-- Contain(Vector2d v) / Contain(Box2d o)
function Box2d.Contain(self: Box2d, x: any)
	if getmetatable(x) == Box2d then
		self:ContainBox(x)
	else
		self:ContainPoint(x)
	end
end

function Box2d.Contains(self: Box2d, v: Vector2d.Vector2d): boolean
	local lv = v - self.Center
	return (abs(lv:Dot(self.AxisX)) <= self.Extent.x) and (abs(lv:Dot(self.AxisY)) <= self.Extent.y)
end

function Box2d.Expand(self: Box2d, f: number)
	self.Extent = self.Extent + f
end

function Box2d.Translate(self: Box2d, v: Vector2d.Vector2d)
	self.Center = self.Center + v
end

function Box2d.RotateAxes(self: Box2d, m: any)
	self.AxisX = m:Multiply(self.AxisX)
	self.AxisY = m:Multiply(self.AxisY)
end

-- Returns distance to box, or 0 if point is inside box.
-- Ported from WildMagic5 Wm5DistPoint2Box2.cpp
function Box2d.DistanceSquared(self: Box2d, v: Vector2d.Vector2d): number
	-- Work in the box's coordinate system.
	v = v - self.Center

	-- Compute squared distance and closest point on box.
	local sqrDistance = 0
	for i = 0, 1 do
		local c, extent
		if i == 0 then
			c = v:Dot(self.AxisX)
			extent = self.Extent.x
		else
			c = v:Dot(self.AxisY)
			extent = self.Extent.y
		end
		if c < -extent then
			local delta = c + extent
			sqrDistance += delta * delta
		elseif c > extent then
			local delta = c - extent
			sqrDistance += delta * delta
		end
	end

	return sqrDistance
end

function Box2d.ClosestPoint(self: Box2d, v: Vector2d.Vector2d): Vector2d.Vector2d
	-- Work in the box's coordinate system.
	local diff = v - self.Center

	-- Compute closest point on box.
	local closest = { 0, 0 }
	for i = 0, 1 do
		local ci = diff:Dot((i == 0) and self.AxisX or self.AxisY)
		local extent = (i == 0) and self.Extent.x or self.Extent.y
		if ci < -extent then
			ci = -extent
		elseif ci > extent then
			ci = extent
		end
		closest[i + 1] = ci
	end

	return self.Center + closest[1] * self.AxisX + closest[2] * self.AxisY
end

-- ported from WildMagic5 Wm5ContBox2.cpp::MergeBoxes
function Box2d.Merge(box0: Box2d, box1: Box2d): Box2d
	-- Construct a box that contains the input boxes.
	local box = Box2d.new()

	-- The first guess at the box center.  This value will be updated later
	-- after the input box vertices are projected onto axes determined by an
	-- average of box axes.
	box.Center = 0.5 * (box0.Center + box1.Center)

	-- The merged box axes are the averages of the input box axes.  The
	-- axes of the second box are negated, if necessary, so they form acute
	-- angles with the axes of the first box.
	if box0.AxisX:Dot(box1.AxisX) >= 0 then
		box.AxisX = (0.5 * (box0.AxisX + box1.AxisX)):Normalized()
	else
		box.AxisX = (0.5 * (box0.AxisX - box1.AxisX)):Normalized()
	end
	box.AxisY = -box.AxisX:Perp()

	-- Project the input box vertices onto the merged-box axes.  Each axis
	-- D[i] containing the current center C has a minimum projected value
	-- min[i] and a maximum projected value max[i].  The corresponding end
	-- points on the axes are C+min[i]*D[i] and C+max[i]*D[i].  The point C
	-- is not necessarily the midpoint for any of the intervals.  The actual
	-- box center will be adjusted from C to a point C' that is the midpoint
	-- of each interval,
	--   C' = C + sum_{i=0}^1 0.5*(min[i]+max[i])*D[i]
	-- The box extents are
	--   e[i] = 0.5*(max[i]-min[i])
	local pmin = { 0, 0 }
	local pmax = { 0, 0 }
	local vertex = table.create(4)

	for _, src in { box0, box1 } do
		src:ComputeVerticesInto(vertex)
		for i = 1, 4 do
			local diff = vertex[i] - box.Center
			for j = 0, 1 do
				local dot = diff:Dot(box:Axis(j))
				if dot > pmax[j + 1] then
					pmax[j + 1] = dot
				elseif dot < pmin[j + 1] then
					pmin[j + 1] = dot
				end
			end
		end
	end

	-- [min,max] is the axis-aligned box in the coordinate system of the
	-- merged box axes.  Update the current box center to be the center of
	-- the new box.  Compute the extents based on the new center.
	box.Extent = V2(0.5 * (pmax[1] - pmin[1]), 0.5 * (pmax[2] - pmin[2]))
	box.Center = box.Center + box.AxisX * (0.5 * (pmax[1] + pmin[1]))
	box.Center = box.Center + box.AxisY * (0.5 * (pmax[2] + pmin[2]))

	return box
end

Box2d.__tostring = function(b: Box2d): string
	return string.format("Box2d(c=%s e=%s)", tostring(b.Center), tostring(b.Extent))
end

return Box2d
