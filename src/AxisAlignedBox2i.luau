--!native
-- Port of geometry3Sharp math/AxisAlignedBox2i.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- AxisAlignedBox2i is a MUTABLE struct in C#. Mutating methods replace the Min/Max Vector2i fields.
-- Use :Clone() where C# would copy the struct. Integer division truncates toward zero like C#.
--
-- Constructors:
--   AxisAlignedBox2i.new(xmin, ymin, xmax, ymax)
--   AxisAlignedBox2i.Empty(), .Zero(), .UnitPositive(), .Infinite()
--   AxisAlignedBox2i.FromCubeSize(n), .FromSize(w, h), .FromMinMax(vMin, vMax)
--   AxisAlignedBox2i.FromCenterHalfSizes(c, hw, hh), .FromCenterHalfSize(c, h), .FromPoint(c)
-- Overloads:
--   Contain(v) accepts a Vector2i, an AxisAlignedBox2i, a Vector3d (truncated) or an AxisAlignedBox3d (truncated)
--   Contains(v) accepts a Vector2i or an AxisAlignedBox2i
--   MoveMin(Vector2i) or MoveMin(x, y)
--   IndicesInclusive() / IndicesExclusive() return iterators yielding new Vector2i values (x fastest)

local MathUtil = require(script.Parent.MathUtil)
local Vector2i = require(script.Parent.Vector2i)

local V2i = Vector2i.new
local max, min, sqrt = math.max, math.min, math.sqrt
local IntDiv, TruncInt = MathUtil.IntDiv, MathUtil.TruncInt

local AxisAlignedBox2i = {}
AxisAlignedBox2i.__index = AxisAlignedBox2i
AxisAlignedBox2i.ClassName = "AxisAlignedBox2i"

local function make(vmin, vmax)
	return setmetatable({ Min = vmin, Max = vmax }, AxisAlignedBox2i)
end

function AxisAlignedBox2i.new(xmin: number, ymin: number, xmax: number, ymax: number)
	return make(V2i(xmin, ymin), V2i(xmax, ymax))
end
function AxisAlignedBox2i.Empty()
	return make(V2i(MathUtil.MaxInt, MathUtil.MaxInt), V2i(MathUtil.MinInt, MathUtil.MinInt))
end
function AxisAlignedBox2i.Zero()
	return make(V2i(0, 0), V2i(0, 0))
end
function AxisAlignedBox2i.UnitPositive()
	return make(V2i(0, 0), V2i(1, 1))
end
function AxisAlignedBox2i.Infinite()
	return make(V2i(MathUtil.MinInt, MathUtil.MinInt), V2i(MathUtil.MaxInt, MathUtil.MaxInt))
end
function AxisAlignedBox2i.FromCubeSize(n: number)
	return make(V2i(0, 0), V2i(n, n))
end
function AxisAlignedBox2i.FromSize(w: number, h: number)
	return make(V2i(0, 0), V2i(w, h))
end
function AxisAlignedBox2i.FromMinMax(vMin, vMax)
	return make(V2i(min(vMin.x, vMax.x), min(vMin.y, vMax.y)), V2i(max(vMin.x, vMax.x), max(vMin.y, vMax.y)))
end
function AxisAlignedBox2i.FromCenterHalfSizes(vCenter, fHalfWidth: number, fHalfHeight: number)
	return make(V2i(vCenter.x - fHalfWidth, vCenter.y - fHalfHeight), V2i(vCenter.x + fHalfWidth, vCenter.y + fHalfHeight))
end
function AxisAlignedBox2i.FromCenterHalfSize(vCenter, fHalfSize: number)
	return AxisAlignedBox2i.FromCenterHalfSizes(vCenter, fHalfSize, fHalfSize)
end
function AxisAlignedBox2i.FromPoint(vCenter)
	return make(vCenter, vCenter)
end

function AxisAlignedBox2i:Clone()
	return make(self.Min, self.Max)
end

function AxisAlignedBox2i:Width(): number
	return max(self.Max.x - self.Min.x, 0)
end
function AxisAlignedBox2i:Height(): number
	return max(self.Max.y - self.Min.y, 0)
end
function AxisAlignedBox2i:Area(): number
	return self:Width() * self:Height()
end
function AxisAlignedBox2i:DiagonalLength(): number
	local dx, dy = self.Max.x - self.Min.x, self.Max.y - self.Min.y
	return TruncInt(sqrt(dx * dx + dy * dy))
end
function AxisAlignedBox2i:MaxDim(): number
	return max(self:Width(), self:Height())
end
function AxisAlignedBox2i:Diagonal()
	return V2i(self.Max.x - self.Min.x, self.Max.y - self.Min.y)
end
function AxisAlignedBox2i:Extents()
	return V2i(IntDiv(self.Max.x - self.Min.x, 2), IntDiv(self.Max.y - self.Min.y, 2))
end
function AxisAlignedBox2i:Center()
	return V2i(IntDiv(self.Min.x + self.Max.x, 2), IntDiv(self.Min.y + self.Max.y, 2))
end

AxisAlignedBox2i.__eq = function(a, b): boolean
	return a.Min == b.Min and a.Max == b.Max
end
function AxisAlignedBox2i:Equals(other): boolean
	return self.Min == other.Min and self.Max == other.Max
end
function AxisAlignedBox2i:CompareTo(other): number
	local function cmp(a, b)
		if a.x ~= b.x then
			return a.x < b.x and -1 or 1
		elseif a.y ~= b.y then
			return a.y < b.y and -1 or 1
		end
		return 0
	end
	local c = cmp(self.Min, other.Min)
	if c == 0 then
		return cmp(self.Max, other.Max)
	end
	return c
end

-- 0 == bottom-left, 1 = bottom-right, 2 == top-right, 3 == top-left
function AxisAlignedBox2i:GetCorner(i: number)
	return V2i((i % 3 == 0) and self.Min.x or self.Max.x, (i < 2) and self.Min.y or self.Max.y)
end

function AxisAlignedBox2i:Expand(nRadius: number)
	self.Min = V2i(self.Min.x - nRadius, self.Min.y - nRadius)
	self.Max = V2i(self.Max.x + nRadius, self.Max.y + nRadius)
end
function AxisAlignedBox2i:Contract(nRadius: number)
	self.Min = V2i(self.Min.x + nRadius, self.Min.y + nRadius)
	self.Max = V2i(self.Max.x - nRadius, self.Max.y - nRadius)
end
function AxisAlignedBox2i:Scale(sx: number, sy: number, _sz: number?)
	local c = self:Center()
	local e = self:Extents()
	local ex, ey = e.x * sx, e.y * sy
	self.Min = V2i(c.x - ex, c.y - ey)
	self.Max = V2i(c.x + ex, c.y + ey)
end

function AxisAlignedBox2i:Contain(v: any)
	local Min, Max = self.Min, self.Max
	local nx, ny, xx, xy
	if type(v.Min) == "table" then
		local isInt = getmetatable(v.Min) == Vector2i
		nx = isInt and v.Min.x or TruncInt(v.Min.x)
		ny = isInt and v.Min.y or TruncInt(v.Min.y)
		xx = isInt and v.Max.x or TruncInt(v.Max.x)
		xy = isInt and v.Max.y or TruncInt(v.Max.y)
	else
		local isInt = getmetatable(v) == Vector2i
		nx = isInt and v.x or TruncInt(v.x)
		ny = isInt and v.y or TruncInt(v.y)
		xx, xy = nx, ny
	end
	self.Min = V2i(min(Min.x, nx), min(Min.y, ny))
	self.Max = V2i(max(Max.x, xx), max(Max.y, xy))
end

function AxisAlignedBox2i:Intersect(box)
	local intersect =
		AxisAlignedBox2i.new(max(self.Min.x, box.Min.x), max(self.Min.y, box.Min.y), min(self.Max.x, box.Max.x), min(self.Max.y, box.Max.y))
	if intersect:Height() <= 0 or intersect:Width() <= 0 then
		return AxisAlignedBox2i.Empty()
	end
	return intersect
end

function AxisAlignedBox2i:Contains(v): boolean
	if type(v.Min) == "table" then
		return self:Contains(v.Min) and self:Contains(v.Max)
	end
	local Min, Max = self.Min, self.Max
	return (Min.x <= v.x) and (Min.y <= v.y) and (Max.x >= v.x) and (Max.y >= v.y)
end

function AxisAlignedBox2i:Intersects(box): boolean
	local Min, Max = self.Min, self.Max
	return not ((box.Max.x <= Min.x) or (box.Min.x >= Max.x) or (box.Max.y <= Min.y) or (box.Min.y >= Max.y))
end

function AxisAlignedBox2i:DistanceSquared(v): number
	local Min, Max = self.Min, self.Max
	local dx = (v.x < Min.x) and (Min.x - v.x) or ((v.x > Max.x) and (v.x - Max.x) or 0)
	local dy = (v.y < Min.y) and (Min.y - v.y) or ((v.y > Max.y) and (v.y - Max.y) or 0)
	return dx * dx + dy * dy
end
function AxisAlignedBox2i:Distance(v): number
	return TruncInt(sqrt(self:DistanceSquared(v)))
end
function AxisAlignedBox2i:NearestPoint(v)
	local Min, Max = self.Min, self.Max
	local x = (v.x < Min.x) and Min.x or ((v.x > Max.x) and Max.x or v.x)
	local y = (v.y < Min.y) and Min.y or ((v.y > Max.y) and Max.y or v.y)
	return V2i(x, y)
end

function AxisAlignedBox2i:Translate(vTranslate)
	self.Min = V2i(self.Min.x + vTranslate.x, self.Min.y + vTranslate.y)
	self.Max = V2i(self.Max.x + vTranslate.x, self.Max.y + vTranslate.y)
end

function AxisAlignedBox2i:MoveMin(a: any, b: number?)
	local nx, ny
	if type(a) == "number" then
		nx, ny = a, b :: number
	else
		nx, ny = a.x, a.y
	end
	self.Max = V2i(nx + (self.Max.x - self.Min.x), ny + (self.Max.y - self.Min.y))
	self.Min = V2i(nx, ny)
end

local function indices(x0, y0, x1, y1)
	local x, y = x0 - 1, y0
	return function()
		x += 1
		if x > x1 then
			x = x0
			y += 1
		end
		if y > y1 or x0 > x1 then
			return nil
		end
		return V2i(x, y)
	end
end
function AxisAlignedBox2i:IndicesInclusive()
	return indices(self.Min.x, self.Min.y, self.Max.x, self.Max.y)
end
function AxisAlignedBox2i:IndicesExclusive()
	return indices(self.Min.x, self.Min.y, self.Max.x - 1, self.Max.y - 1)
end

AxisAlignedBox2i.__tostring = function(b): string
	return string.format("x[%d,%d] y[%d,%d]", b.Min.x, b.Max.x, b.Min.y, b.Max.y)
end
AxisAlignedBox2i.ToString = AxisAlignedBox2i.__tostring

return AxisAlignedBox2i
