--!native
-- Port of geometry3Sharp math/AxisAlignedBox2d.cs (C# AxisAlignedBox2d; AxisAlignedBox2f is not ported, see PORTING.md)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- AxisAlignedBox2d is a MUTABLE struct in C#. Mutating methods (Expand, Contain, ...) replace the
-- Min/Max vectors (which are themselves immutable). Use :Clone() where C# would copy the struct.
--
-- Constructors:
--   AxisAlignedBox2d.new(xmin, ymin, xmax, ymax)
--   AxisAlignedBox2d.Empty()                         C# AxisAlignedBox2d.Empty / new AxisAlignedBox2d(false)
--   AxisAlignedBox2d.Zero(), .UnitPositive(), .Infinite()   (static readonly -> fresh boxes)
--   AxisAlignedBox2d.FromSquareSize(f)              new AxisAlignedBox2d(double fSquareSize)
--   AxisAlignedBox2d.FromSize(w, h)                 new AxisAlignedBox2d(double fWidth, double fHeight)
--   AxisAlignedBox2d.FromMinMax(vMin, vMax)         new AxisAlignedBox2d(Vector2d vMin, Vector2d vMax) (sorts components)
--   AxisAlignedBox2d.FromCenterHalfSizes(c, hw, hh) new AxisAlignedBox2d(Vector2d vCenter, double fHalfWidth, double fHalfHeight)
--   AxisAlignedBox2d.FromCenterHalfSize(c, h)       new AxisAlignedBox2d(Vector2d vCenter, double fHalfWidth)
--   AxisAlignedBox2d.FromPoint(c)                   new AxisAlignedBox2d(Vector2d vCenter)
--   box:Clone()                                    new AxisAlignedBox2d(AxisAlignedBox2d o)
--
-- Luau API notes:
--   * C# properties -> methods: IsValid(), IsEmpty(), Width(), Height(), Area(), DiagonalLength(), MaxDim(),
--     MinDim(), MaxUnsignedCoordinate(), Diagonal(), Extents(), Center().
--   * Contain(x) dispatches on x: Vector2d point, AxisAlignedBox2d box, or 1-based array of Vector2d (IList overload).
--   * Contains(x) dispatches on x: Vector2d point (strict inequalities, as in C#) or AxisAlignedBox2d.
--   * Translated(v) or Translated(dx, dy); Scale(scale, origin?); MoveMin(v) or MoveMin(x, y).
--   * ScaleMode enum: AxisAlignedBox2d.ScaleMode.ScaleRight etc (C# enum values).
--   * Pad() is [Obsolete] in C# but ported (same as Add()).

local MathUtil = require(script.Parent.MathUtil)
local Vector2d = require(script.Parent.Vector2d)

local V2 = Vector2d.new
local max, min, sqrt, abs = math.max, math.min, math.sqrt, math.abs

local AxisAlignedBox2d = {}
AxisAlignedBox2d.__index = AxisAlignedBox2d
AxisAlignedBox2d.ClassName = "AxisAlignedBox2d"

export type AxisAlignedBox2d = typeof(setmetatable({} :: { Min: Vector2d.Vector2d, Max: Vector2d.Vector2d }, AxisAlignedBox2d))

local function make(vmin, vmax): AxisAlignedBox2d
	return setmetatable({ Min = vmin, Max = vmax }, AxisAlignedBox2d) :: any
end

function AxisAlignedBox2d.new(xmin: number, ymin: number, xmax: number, ymax: number): AxisAlignedBox2d
	return make(V2(xmin, ymin), V2(xmax, ymax))
end

-- new AxisAlignedBox2d(false): "empty" box with inverted min/max
function AxisAlignedBox2d.Empty(): AxisAlignedBox2d
	return make(V2(MathUtil.MaxDouble, MathUtil.MaxDouble), V2(MathUtil.MinDouble, MathUtil.MinDouble))
end

function AxisAlignedBox2d.Zero(): AxisAlignedBox2d
	return make(V2(0, 0), V2(0, 0))
end

function AxisAlignedBox2d.UnitPositive(): AxisAlignedBox2d
	return make(V2(0, 0), V2(1, 1))
end

function AxisAlignedBox2d.Infinite(): AxisAlignedBox2d
	return make(V2(MathUtil.MinDouble, MathUtil.MinDouble), V2(MathUtil.MaxDouble, MathUtil.MaxDouble))
end

function AxisAlignedBox2d.FromSquareSize(fSquareSize: number): AxisAlignedBox2d
	return make(V2(0, 0), V2(fSquareSize, fSquareSize))
end

function AxisAlignedBox2d.FromSize(fWidth: number, fHeight: number): AxisAlignedBox2d
	return make(V2(0, 0), V2(fWidth, fHeight))
end

function AxisAlignedBox2d.FromMinMax(vMin: Vector2d.Vector2d, vMax: Vector2d.Vector2d): AxisAlignedBox2d
	return make(V2(min(vMin.x, vMax.x), min(vMin.y, vMax.y)), V2(max(vMin.x, vMax.x), max(vMin.y, vMax.y)))
end

function AxisAlignedBox2d.FromCenterHalfSizes(vCenter: Vector2d.Vector2d, fHalfWidth: number, fHalfHeight: number): AxisAlignedBox2d
	return make(V2(vCenter.x - fHalfWidth, vCenter.y - fHalfHeight), V2(vCenter.x + fHalfWidth, vCenter.y + fHalfHeight))
end

function AxisAlignedBox2d.FromCenterHalfSize(vCenter: Vector2d.Vector2d, fHalfWidth: number): AxisAlignedBox2d
	return AxisAlignedBox2d.FromCenterHalfSizes(vCenter, fHalfWidth, fHalfWidth)
end

function AxisAlignedBox2d.FromPoint(vCenter: Vector2d.Vector2d): AxisAlignedBox2d
	return make(vCenter, vCenter)
end

function AxisAlignedBox2d.Clone(self: AxisAlignedBox2d): AxisAlignedBox2d
	return make(self.Min, self.Max)
end

function AxisAlignedBox2d.IsValid(self: AxisAlignedBox2d): boolean
	return self.Max.x >= self.Min.x and self.Max.y >= self.Min.y
end
function AxisAlignedBox2d.IsEmpty(self: AxisAlignedBox2d): boolean
	return self.Max.x < self.Min.x or self.Max.y < self.Min.y
end

function AxisAlignedBox2d.Width(self: AxisAlignedBox2d): number
	return max(self.Max.x - self.Min.x, 0)
end
function AxisAlignedBox2d.Height(self: AxisAlignedBox2d): number
	return max(self.Max.y - self.Min.y, 0)
end
function AxisAlignedBox2d.Area(self: AxisAlignedBox2d): number
	return self:Width() * self:Height()
end
function AxisAlignedBox2d.DiagonalLength(self: AxisAlignedBox2d): number
	local dx, dy = self.Max.x - self.Min.x, self.Max.y - self.Min.y
	return sqrt(dx * dx + dy * dy)
end
function AxisAlignedBox2d.MaxDim(self: AxisAlignedBox2d): number
	return max(self:Width(), self:Height())
end
function AxisAlignedBox2d.MinDim(self: AxisAlignedBox2d): number
	return min(self:Width(), self:Height())
end

-- returns absolute value of largest min/max x/y coordinate (ie max axis distance to origin)
function AxisAlignedBox2d.MaxUnsignedCoordinate(self: AxisAlignedBox2d): number
	local Min, Max = self.Min, self.Max
	return max(max(abs(Min.x), abs(Max.x)), max(abs(Min.y), abs(Max.y)))
end

function AxisAlignedBox2d.Diagonal(self: AxisAlignedBox2d): Vector2d.Vector2d
	return V2(self.Max.x - self.Min.x, self.Max.y - self.Min.y)
end
function AxisAlignedBox2d.Extents(self: AxisAlignedBox2d): Vector2d.Vector2d
	return V2((self.Max.x - self.Min.x) * 0.5, (self.Max.y - self.Min.y) * 0.5)
end
function AxisAlignedBox2d.Center(self: AxisAlignedBox2d): Vector2d.Vector2d
	return V2(0.5 * (self.Min.x + self.Max.x), 0.5 * (self.Min.y + self.Max.y))
end

-- 0 == bottom-left, 1 = bottom-right, 2 == top-right, 3 == top-left
function AxisAlignedBox2d.GetCorner(self: AxisAlignedBox2d, i: number): Vector2d.Vector2d
	return V2((i % 3 == 0) and self.Min.x or self.Max.x, (i < 2) and self.Min.y or self.Max.y)
end

-- Point inside box where t,s are in range [0,1]
function AxisAlignedBox2d.SampleT(self: AxisAlignedBox2d, tx: number, sy: number): Vector2d.Vector2d
	local Min, Max = self.Min, self.Max
	return V2((1.0 - tx) * Min.x + tx * Max.x, (1.0 - sy) * Min.y + sy * Max.y)
end

-- value is subtracted from min and added to max
function AxisAlignedBox2d.Expand(self: AxisAlignedBox2d, fRadius: number)
	local Min, Max = self.Min, self.Max
	self.Min = V2(Min.x - fRadius, Min.y - fRadius)
	self.Max = V2(Max.x + fRadius, Max.y + fRadius)
end

-- value is added to min and subtracted from max
function AxisAlignedBox2d.Contract(self: AxisAlignedBox2d, fRadius: number)
	local Min, Max = self.Min, self.Max
	self.Min = V2(Min.x + fRadius, Min.y + fRadius)
	self.Max = V2(Max.x - fRadius, Max.y - fRadius)
end

-- values are all added ([Obsolete] in C#: use Add() instead)
function AxisAlignedBox2d.Pad(self: AxisAlignedBox2d, fPadLeft: number, fPadRight: number, fPadBottom: number, fPadTop: number)
	local Min, Max = self.Min, self.Max
	self.Min = V2(Min.x + fPadLeft, Min.y + fPadBottom)
	self.Max = V2(Max.x + fPadRight, Max.y + fPadTop)
end

function AxisAlignedBox2d.Add(self: AxisAlignedBox2d, left: number, right: number, bottom: number, top: number)
	local Min, Max = self.Min, self.Max
	self.Min = V2(Min.x + left, Min.y + bottom)
	self.Max = V2(Max.x + right, Max.y + top)
end

AxisAlignedBox2d.ScaleMode = table.freeze({
	ScaleRight = 0,
	ScaleLeft = 1,
	ScaleUp = 2,
	ScaleDown = 3,
	ScaleCenter = 4,
})
local ScaleMode = AxisAlignedBox2d.ScaleMode

function AxisAlignedBox2d.SetWidth(self: AxisAlignedBox2d, fNewWidth: number, eScaleMode: number)
	local Min, Max = self.Min, self.Max
	if eScaleMode == ScaleMode.ScaleLeft then
		self.Min = V2(Max.x - fNewWidth, Min.y)
	elseif eScaleMode == ScaleMode.ScaleRight then
		self.Max = V2(Min.x + fNewWidth, Max.y)
	elseif eScaleMode == ScaleMode.ScaleCenter then
		local vCenter = self:Center()
		self.Min = V2(vCenter.x - 0.5 * fNewWidth, Min.y)
		self.Max = V2(vCenter.x + 0.5 * fNewWidth, Max.y)
	else
		error("Invalid scale mode...")
	end
end

function AxisAlignedBox2d.SetHeight(self: AxisAlignedBox2d, fNewHeight: number, eScaleMode: number)
	local Min, Max = self.Min, self.Max
	if eScaleMode == ScaleMode.ScaleDown then
		self.Min = V2(Min.x, Max.y - fNewHeight)
	elseif eScaleMode == ScaleMode.ScaleUp then
		self.Max = V2(Max.x, Min.y + fNewHeight)
	elseif eScaleMode == ScaleMode.ScaleCenter then
		local vCenter = self:Center()
		self.Min = V2(Min.x, vCenter.y - 0.5 * fNewHeight)
		self.Max = V2(Max.x, vCenter.y + 0.5 * fNewHeight)
	else
		error("Invalid scale mode...")
	end
end

function AxisAlignedBox2d.ContainPoint(self: AxisAlignedBox2d, v: Vector2d.Vector2d)
	local Min, Max = self.Min, self.Max
	if v.x < Min.x or v.y < Min.y then
		self.Min = V2(min(Min.x, v.x), min(Min.y, v.y))
	end
	if v.x > Max.x or v.y > Max.y then
		self.Max = V2(max(Max.x, v.x), max(Max.y, v.y))
	end
end

function AxisAlignedBox2d.ContainBox(self: AxisAlignedBox2d, box: AxisAlignedBox2d)
	local Min, Max = self.Min, self.Max
	self.Min = V2(min(Min.x, box.Min.x), min(Min.y, box.Min.y))
	self.Max = V2(max(Max.x, box.Max.x), max(Max.y, box.Max.y))
end

-- Contain(IList<Vector2d> points)
function AxisAlignedBox2d.ContainPoints(self: AxisAlignedBox2d, points: { Vector2d.Vector2d })
	local N = #points
	if N > 0 then
		self:ContainPoint(points[1])
		local mnx, mny, mxx, mxy = self.Min.x, self.Min.y, self.Max.x, self.Max.y
		-- once we are sure we have initialized min/max, we can use if/else
		for i = 2, N do
			local v = points[i]
			if v.x < mnx then
				mnx = v.x
			elseif v.x > mxx then
				mxx = v.x
			end

			if v.y < mny then
				mny = v.y
			elseif v.y > mxy then
				mxy = v.y
			end
		end
		self.Min = V2(mnx, mny)
		self.Max = V2(mxx, mxy)
	end
end

-- Contain(Vector2d v) / Contain(AxisAlignedBox2d box) / Contain(IList<Vector2d> points)
function AxisAlignedBox2d.Contain(self: AxisAlignedBox2d, x: any)
	if type(x.Min) == "table" then
		self:ContainBox(x)
	elseif x.x == nil and x[1] ~= nil then
		self:ContainPoints(x)
	elseif x.x ~= nil then
		self:ContainPoint(x)
	end
end

-- return axis box that contains this box and OtherBox
function AxisAlignedBox2d.CombineWith(self: AxisAlignedBox2d, OtherBox: AxisAlignedBox2d): AxisAlignedBox2d
	local newBox = self:Clone()
	newBox:ContainBox(OtherBox)
	return newBox
end

function AxisAlignedBox2d.Intersect(self: AxisAlignedBox2d, box: AxisAlignedBox2d): AxisAlignedBox2d
	local Min, Max = self.Min, self.Max
	local intersect = AxisAlignedBox2d.new(max(Min.x, box.Min.x), max(Min.y, box.Min.y), min(Max.x, box.Max.x), min(Max.y, box.Max.y))
	if intersect:Height() <= 0 or intersect:Width() <= 0 then
		return AxisAlignedBox2d.Empty()
	end
	return intersect
end

-- Contains(Vector2d v) or Contains(AxisAlignedBox2d box2). Note: strict inequalities (as in C#).
function AxisAlignedBox2d.Contains(self: AxisAlignedBox2d, v: any): boolean
	if type(v.Min) == "table" then
		return self:Contains(v.Min) and self:Contains(v.Max)
	end
	local Min, Max = self.Min, self.Max
	return (Min.x < v.x) and (Min.y < v.y) and (Max.x > v.x) and (Max.y > v.y)
end

function AxisAlignedBox2d.Intersects(self: AxisAlignedBox2d, box: AxisAlignedBox2d): boolean
	local Min, Max = self.Min, self.Max
	return not ((box.Max.x < Min.x) or (box.Min.x > Max.x) or (box.Max.y < Min.y) or (box.Min.y > Max.y))
end

-- return true if this intersects B, where overlapping borders do not count as intersection
function AxisAlignedBox2d.IntersectsExclusive(self: AxisAlignedBox2d, B: AxisAlignedBox2d): boolean
	local Min, Max = self.Min, self.Max
	if not (Max.x <= B.Min.x) and not (Min.x >= B.Max.x) and not (Max.y <= B.Min.y) then
		return not (Min.y >= B.Max.y)
	end
	return false
end

function AxisAlignedBox2d.Distance(self: AxisAlignedBox2d, v: Vector2d.Vector2d): number
	local center = self:Center()
	local dx = abs(v.x - center.x)
	local dy = abs(v.y - center.y)
	local fWidth = self:Width() * 0.5
	local fHeight = self:Height() * 0.5
	if dx < fWidth and dy < fHeight then
		return 0.0
	elseif dx > fWidth and dy > fHeight then
		return sqrt((dx - fWidth) * (dx - fWidth) + (dy - fHeight) * (dy - fHeight))
	elseif dx > fWidth then
		return dx - fWidth
	elseif dy > fHeight then
		return dy - fHeight
	end
	return 0.0
end

-- relative translation
function AxisAlignedBox2d.Translate(self: AxisAlignedBox2d, vTranslate: Vector2d.Vector2d)
	self.Min = self.Min + vTranslate
	self.Max = self.Max + vTranslate
end

-- Translated(Vector2d Translation) or Translated(dx, dy)
function AxisAlignedBox2d.Translated(self: AxisAlignedBox2d, a: any, dy: number?): AxisAlignedBox2d
	if type(a) == "number" then
		local dx = a
		return AxisAlignedBox2d.new(self.Min.x + dx, self.Min.y + (dy :: number), self.Max.x + dx, self.Max.y + (dy :: number))
	end
	return AxisAlignedBox2d.FromMinMax(self.Min + a, self.Max + a)
end

-- Scale(scale) or Scale(scale, origin)
function AxisAlignedBox2d.Scale(self: AxisAlignedBox2d, scale: number, origin: Vector2d.Vector2d?)
	if origin == nil then
		self.Min = self.Min * scale
		self.Max = self.Max * scale
	else
		self.Min = (self.Min - origin) * scale + origin
		self.Max = (self.Max - origin) * scale + origin
	end
end

-- MoveMin(Vector2d vNewMin) or MoveMin(fNewX, fNewY)
function AxisAlignedBox2d.MoveMin(self: AxisAlignedBox2d, a: any, b: number?)
	local nx, ny
	if type(a) == "number" then
		nx, ny = a, b :: number
	else
		nx, ny = a.x, a.y
	end
	local Min, Max = self.Min, self.Max
	self.Max = V2(nx + (Max.x - Min.x), ny + (Max.y - Min.y))
	self.Min = V2(nx, ny)
end

AxisAlignedBox2d.__tostring = function(b: AxisAlignedBox2d): string
	return string.format("X:[%.8f,%.8f] Y:[%.8f,%.8f]", b.Min.x, b.Max.x, b.Min.y, b.Max.y)
end

return AxisAlignedBox2d
