--!native
-- Port of geometry3Sharp math/Box3.cs (Box3d; partially based on WildMagic5 Box3)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- A box has center C, axis directions U[0], U[1], and U[2] (mutually
-- perpendicular unit-length vectors), and extents e[0], e[1], and e[2]
-- (all nonnegative numbers).  A point X = C+y[0]*U[0]+y[1]*U[1]+y[2]*U[2]
-- is inside or on the box whenever |y[i]| <= e[i] for all i.
--
-- Luau API notes:
--   Box3d is a MUTABLE struct (fields Center, AxisX, AxisY, AxisZ, Extent: immutable Vector3d). Use :Clone()
--   where C# would copy it.
--   Constructors (Box3d.new dispatches on its arguments like the C# overloads):
--     Box3d.new(center)                          new Box3d(Vector3d center)
--     Box3d.new(center, x, y, z, extent)         new Box3d(center, x, y, z, extent)
--     Box3d.new(center, extent)  / Box3d.FromCenterExtent(center, extent)
--     Box3d.new(aaBox)           / Box3d.FromAABB(aaBox)
--     Box3d.new(frame, extent)   / Box3d.FromFrame(frame, extent)
--     Box3d.new(seg)             / Box3d.FromSegment(seg)
--     Box3d.Empty(), Box3d.UnitZeroCentered(), Box3d.UnitPositive()   (static readonly -> fresh boxes)
--   C# properties -> methods: MaxExtent(), MinExtent(), Diagonal(), Volume()
--   ComputeVertices() -> 1-based array of 8 Vector3d; ComputeVerticesInto(vertex) fills vertex[1..8]
--   VerticesItr() -> iterator
--   Contain(v | Box3d | points) dispatches to ContainPoint / ContainBox / ContainPoints
--     (points: iterator function or 1-based array)
--   Box3d.Merge(box0, box1) -> Box3d

local Vector3d = require(script.Parent.Vector3d)
local AxisAlignedBox3d = require(script.Parent.AxisAlignedBox3d)
local Matrix3d = require(script.Parent.Matrix3d)
local Quaterniond = require(script.Parent.Quaterniond)

local V3 = Vector3d.new
local abs = math.abs

local Box3d = {}
Box3d.__index = Box3d
Box3d.ClassName = "Box3d"

local function make(center, x, y, z, extent)
	return setmetatable({ Center = center, AxisX = x, AxisY = y, AxisZ = z, Extent = extent }, Box3d)
end

function Box3d.FromCenterExtent(center, extent)
	return make(center, Vector3d.AxisX, Vector3d.AxisY, Vector3d.AxisZ, extent)
end

function Box3d.FromAABB(aaBox)
	-- [RMS] this should produce Empty for aaBox.Empty...
	local extent = V3(aaBox:Width() * 0.5, aaBox:Height() * 0.5, aaBox:Depth() * 0.5)
	return make(aaBox:Center(), Vector3d.AxisX, Vector3d.AxisY, Vector3d.AxisZ, extent)
end

function Box3d.FromFrame(frame, extent)
	return make(frame.Origin, frame:X(), frame:Y(), frame:Z(), extent)
end

function Box3d.FromSegment(seg)
	local axisZ = seg.Direction
	local axisX, axisY = Vector3d.MakePerpVectors(axisZ)
	return make(seg.Center, axisX, axisY, axisZ, V3(0, 0, seg.Extent))
end

function Box3d.new(a: any, b: any?, c: any?, d: any?, e: any?)
	if getmetatable(a) == Vector3d or (type(a) == "table" and a.x ~= nil and a.Origin == nil) then
		if e ~= nil then
			return make(a, b, c, d, e)
		elseif b ~= nil then
			return Box3d.FromCenterExtent(a, b)
		end
		return make(a, Vector3d.AxisX, Vector3d.AxisY, Vector3d.AxisZ, Vector3d.Zero)
	elseif type(a) == "table" and a.Min ~= nil and a.Max ~= nil then
		return Box3d.FromAABB(a)
	elseif type(a) == "table" and a.Origin ~= nil and a.Rotation ~= nil then
		return Box3d.FromFrame(a, b)
	elseif type(a) == "table" and a.Direction ~= nil and a.Extent ~= nil then
		return Box3d.FromSegment(a)
	end
	-- new Box3d() (default struct)
	return make(Vector3d.Zero, Vector3d.Zero, Vector3d.Zero, Vector3d.Zero, Vector3d.Zero)
end

function Box3d.Empty()
	return Box3d.new(Vector3d.Zero)
end
function Box3d.UnitZeroCentered()
	return Box3d.FromCenterExtent(Vector3d.Zero, 0.5 * Vector3d.One)
end
function Box3d.UnitPositive()
	return Box3d.FromCenterExtent(0.5 * Vector3d.One, 0.5 * Vector3d.One)
end

function Box3d:Clone()
	return make(self.Center, self.AxisX, self.AxisY, self.AxisZ, self.Extent)
end

function Box3d:Axis(i: number)
	return (i == 0) and self.AxisX or ((i == 1) and self.AxisY or self.AxisZ)
end

function Box3d:ComputeVerticesInto(vertex: { any })
	local Center = self.Center
	local extAxis0 = self.Extent.x * self.AxisX
	local extAxis1 = self.Extent.y * self.AxisY
	local extAxis2 = self.Extent.z * self.AxisZ
	vertex[1] = Center - extAxis0 - extAxis1 - extAxis2
	vertex[2] = Center + extAxis0 - extAxis1 - extAxis2
	vertex[3] = Center + extAxis0 + extAxis1 - extAxis2
	vertex[4] = Center - extAxis0 + extAxis1 - extAxis2
	vertex[5] = Center - extAxis0 - extAxis1 + extAxis2
	vertex[6] = Center + extAxis0 - extAxis1 + extAxis2
	vertex[7] = Center + extAxis0 + extAxis1 + extAxis2
	vertex[8] = Center - extAxis0 + extAxis1 + extAxis2
end

function Box3d:ComputeVertices(): { any }
	local v = table.create(8)
	self:ComputeVerticesInto(v)
	return v
end

function Box3d:VerticesItr(): () -> any
	local v = self:ComputeVertices()
	local i = 0
	return function()
		i += 1
		return v[i]
	end
end

function Box3d:ToAABB()
	-- [TODO] probably more efficient way to do this...at minimum can move center-shift
	-- to after the containments...
	local Center = self.Center
	local extAxis0 = self.Extent.x * self.AxisX
	local extAxis1 = self.Extent.y * self.AxisY
	local extAxis2 = self.Extent.z * self.AxisZ
	local result = AxisAlignedBox3d.FromPoint(Center - extAxis0 - extAxis1 - extAxis2)
	result:Contain(Center + extAxis0 - extAxis1 - extAxis2)
	result:Contain(Center + extAxis0 + extAxis1 - extAxis2)
	result:Contain(Center - extAxis0 + extAxis1 - extAxis2)
	result:Contain(Center - extAxis0 - extAxis1 + extAxis2)
	result:Contain(Center + extAxis0 - extAxis1 + extAxis2)
	result:Contain(Center + extAxis0 + extAxis1 + extAxis2)
	result:Contain(Center - extAxis0 + extAxis1 + extAxis2)
	return result
end

-- corners [ (-x,-y), (x,-y), (x,y), (-x,y) ], -z, then +z
--
--   7---6     +z       or        3---2     -z
--   |\  |\                       |\  |\
--   4-\-5 \                      0-\-1 \
--    \ 3---2                      \ 7---6
--     \|   |                       \|   |
--      0---1  -z                    4---5  +z
--
-- Note that in RHS system (which is our default), +z is "forward" so -z in this diagram
-- is actually the back of the box (!) This is odd but want to keep consistency w/ ComputeVertices(),
-- and the implementation there needs to stay consistent w/ C++ Wildmagic5
function Box3d:Corner(i: number)
	local c = self.Center
	local b1 = bit32.band(i, 1) ~= 0
	local b2 = bit32.band(i, 2) ~= 0
	c = c + ((b1 ~= b2) and (self.Extent.x * self.AxisX) or (-self.Extent.x * self.AxisX))
	c = c + (((i // 2) % 2 == 0) and (-self.Extent.y * self.AxisY) or (self.Extent.y * self.AxisY))
	c = c + ((i < 4) and (-self.Extent.z * self.AxisZ) or (self.Extent.z * self.AxisZ))
	return c
end

-- g3 extensions
function Box3d:MaxExtent(): number
	local e = self.Extent
	return math.max(e.x, math.max(e.y, e.z))
end
function Box3d:MinExtent(): number
	local e = self.Extent
	-- [note] C# uses Math.Min(Extent.x, Math.Max(Extent.y, Extent.z)); kept as-is
	return math.min(e.x, math.max(e.y, e.z))
end
function Box3d:Diagonal()
	local e = self.Extent
	return (e.x * self.AxisX + e.y * self.AxisY + e.z * self.AxisZ) - (-e.x * self.AxisX - e.y * self.AxisY - e.z * self.AxisZ)
end
function Box3d:Volume(): number
	local e = self.Extent
	return 2 * e.x * 2 * e.y * 2 * e.z
end

function Box3d:ContainPoint(v)
	local lv = v - self.Center
	for k = 0, 2 do
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

-- update the box to contain set of input points. More efficient tha ncalling Contain() many times
-- code ported from GTEngine GteContOrientedBox3.h
function Box3d:ContainPoints(points: any)
	-- Let C be the box center and let U0, U1, and U2 be the box axes.
	-- Each input point is of the form X = C + y0*U0 + y1*U1 + y2*U2.
	-- The following code computes min(y0), max(y0), min(y1), max(y1),
	-- min(y2), and max(y2).  The box center is then adjusted to be
	--   C' = C + 0.5*(min(y0)+max(y0))*U0 + 0.5*(min(y1)+max(y1))*U1 + 0.5*(min(y2)+max(y2))*U2
	local itr = points
	if type(points) ~= "function" then
		local i = 0
		itr = function()
			i += 1
			return points[i]
		end
	end
	local Center, AxisX, AxisY, AxisZ = self.Center, self.AxisX, self.AxisY, self.AxisZ
	local first = itr()
	local diff = first - Center
	local pmin = { diff:Dot(AxisX), diff:Dot(AxisY), diff:Dot(AxisZ) }
	local pmax = { pmin[1], pmin[2], pmin[3] }
	local p = itr()
	while p ~= nil do
		diff = p - Center

		local dotx = diff:Dot(AxisX)
		if dotx < pmin[1] then
			pmin[1] = dotx
		elseif dotx > pmax[1] then
			pmax[1] = dotx
		end

		local doty = diff:Dot(AxisY)
		if doty < pmin[2] then
			pmin[2] = doty
		elseif doty > pmax[2] then
			pmax[2] = doty
		end

		local dotz = diff:Dot(AxisZ)
		if dotz < pmin[3] then
			pmin[3] = dotz
		elseif dotz > pmax[3] then
			pmax[3] = dotz
		end
		p = itr()
	end
	for j = 0, 2 do
		self.Center = self.Center + (0.5 * (pmin[j + 1] + pmax[j + 1])) * self:Axis(j)
		self.Extent = self.Extent:With(j, 0.5 * (pmax[j + 1] - pmin[j + 1]))
	end
end

-- I think this can be more efficient...no? At least could combine
-- all the axis-interval updates before updating Center...
function Box3d:ContainBox(o)
	local v = o:ComputeVertices()
	for k = 1, 8 do
		self:ContainPoint(v[k])
	end
end

-- Contain(Vector3d v) / Contain(IEnumerable<Vector3d> points) / Contain(Box3d o)
function Box3d:Contain(x: any)
	if getmetatable(x) == Box3d then
		self:ContainBox(x)
	elseif type(x) == "function" or (type(x) == "table" and x.x == nil and x[1] ~= nil) then
		self:ContainPoints(x)
	else
		self:ContainPoint(x)
	end
end

function Box3d:Contains(v): boolean
	local lv = v - self.Center
	return (abs(lv:Dot(self.AxisX)) <= self.Extent.x)
		and (abs(lv:Dot(self.AxisY)) <= self.Extent.y)
		and (abs(lv:Dot(self.AxisZ)) <= self.Extent.z)
end

function Box3d:Expand(f: number)
	self.Extent = self.Extent + f
end

function Box3d:Translate(v)
	self.Center = self.Center + v
end

function Box3d:Scale(s)
	self.Center = self.Center * s
	self.Extent = self.Extent * s
	self.AxisX = (self.AxisX * s):NormalizedAndLength()
	self.AxisY = (self.AxisY * s):NormalizedAndLength()
	self.AxisZ = (self.AxisZ * s):NormalizedAndLength()
end

function Box3d:ScaleExtents(s)
	self.Extent = self.Extent * s
end

-- Returns distance to box, or 0 if point is inside box.
-- Ported from WildMagic5 Wm5DistPoint3Box3.cpp
function Box3d:DistanceSquared(v): number
	-- Work in the box's coordinate system.
	v = v - self.Center

	-- Compute squared distance and closest point on box.
	local sqrDistance = 0
	for i = 0, 2 do
		local closest = self:Axis(i):Dot(v)
		local ext = self.Extent:Get(i)
		if closest < -ext then
			local delta = closest + ext
			sqrDistance += delta * delta
		elseif closest > ext then
			local delta = closest - ext
			sqrDistance += delta * delta
		end
	end
	return sqrDistance
end

-- Returns closest point on box (v itself if v is inside box).
-- Ported from WildMagic5 Wm5DistPoint3Box3.cpp
function Box3d:ClosestPoint(v)
	-- Work in the box's coordinate system.
	v = v - self.Center

	-- Compute squared distance and closest point on box.
	local closest = { 0, 0, 0 }
	for i = 0, 2 do
		local c = self:Axis(i):Dot(v)
		local extent = self.Extent:Get(i)
		if c < -extent then
			c = -extent
		elseif c > extent then
			c = extent
		end
		closest[i + 1] = c
	end
	return self.Center + closest[1] * self.AxisX + closest[2] * self.AxisY + closest[3] * self.AxisZ
end

-- ported from WildMagic5 Wm5ContBox3.cpp::MergeBoxes
function Box3d.Merge(box0, box1)
	-- Construct a box that contains the input boxes.
	local box = Box3d.new()

	-- The first guess at the box center.  This value will be updated later
	-- after the input box vertices are projected onto axes determined by an
	-- average of box axes.
	box.Center = 0.5 * (box0.Center + box1.Center)

	-- A box's axes, when viewed as the columns of a matrix, form a rotation
	-- matrix.  The input box axes are converted to quaternions.  The average
	-- quaternion is computed, then normalized to unit length.  The result is
	-- the slerp of the two input quaternions with t-value of 1/2.  The result
	-- is converted back to a rotation matrix and its columns are selected as
	-- the merged box axes.
	local rot0 = Matrix3d.FromVectors(box0.AxisX, box0.AxisY, box0.AxisZ, false)
	local q0 = Quaterniond.FromRotationMatrix(rot0)
	local rot1 = Matrix3d.FromVectors(box1.AxisX, box1.AxisY, box1.AxisZ, false)
	local q1 = Quaterniond.FromRotationMatrix(rot1)
	if q0:Dot(q1) < 0 then
		q1 = -q1
	end

	local q = q0 + q1
	local invLength = 1.0 / math.sqrt(q:Dot(q))
	q = q * invLength
	local q_mat = q:ToRotationMatrix()
	box.AxisX = q_mat:Column(0)
	box.AxisY = q_mat:Column(1)
	box.AxisZ = q_mat:Column(2)

	-- Project the input box vertices onto the merged-box axes.  Each axis
	-- D[i] containing the current center C has a minimum projected value
	-- min[i] and a maximum projected value max[i].  The corresponding end
	-- points on the axes are C+min[i]*D[i] and C+max[i]*D[i].  The point C
	-- is not necessarily the midpoint for any of the intervals.  The actual
	-- box center will be adjusted from C to a point C' that is the midpoint
	-- of each interval,
	--   C' = C + sum_{i=0}^2 0.5*(min[i]+max[i])*D[i]
	-- The box extents are
	--   e[i] = 0.5*(max[i]-min[i])
	local vertex = table.create(8)
	local pmin = { 0, 0, 0 }
	local pmax = { 0, 0, 0 }

	for _, src in { box0, box1 } do
		src:ComputeVerticesInto(vertex)
		for i = 1, 8 do
			local diff = vertex[i] - box.Center
			for j = 0, 2 do
				local dot = box:Axis(j):Dot(diff)
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
	for j = 0, 2 do
		box.Center = box.Center + (0.5 * (pmax[j + 1] + pmin[j + 1])) * box:Axis(j)
		box.Extent = box.Extent:With(j, 0.5 * (pmax[j + 1] - pmin[j + 1]))
	end

	return box
end

Box3d.__tostring = function(b)
	return string.format("Box3d(c=%s e=%s)", tostring(b.Center), tostring(b.Extent))
end

return Box3d
