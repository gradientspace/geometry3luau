-- Port of geometry3Sharp curve/DCurve3.cs (plus the CurveUtils.GetTangent / ArcLength helpers it uses)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- DCurve3 is a 3D polyline, either open or closed (via .Closed)
-- Despite the D prefix, it is *not* dynamic
--
-- Luau port notes (see PORTING.md):
--   * Closed and Timestamp are plain fields. Properties are methods:
--       VertexCount(), SegmentCount(), Start(), End(), ArcLength(), Vertices() (iterator)
--   * indexer this[i] -> Get(i) / Set(i, v)   (0-based i)
--   * Constructors:
--       DCurve3.new()                                       new DCurve3()
--       DCurve3.FromVertices(verticesIn, bClosed, bTakeOwnership?)  List<Vector3d> (1-based Luau array) or an iterator
--       DCurve3.FromCopy(copy)                              new DCurve3(DCurve3 copy)
--       DCurve3.FromSampledCurve(icurve)                    new DCurve3(ISampledCurve3d) (needs :Vertices() iterator and .Closed / :Closed())
--   * SetVertices(v, bTakeOwnership?) accepts a Luau array of Vector3d, an iterator, or a VectorArray3d
--     (C# SetVertices(VectorArray3d), always copies).
--   * DistanceSquared(p) -> dist, iNearSeg, fNearSegT   (C# out params as extra returns)
--   * The Polygon2d constructor is not ported (Polygon2d is not part of this port yet).

local Vector3d = require(script.Parent.Vector3d)
local Segment3d = require(script.Parent.Segment3d)
local AxisAlignedBox3d = require(script.Parent.AxisAlignedBox3d)
local MathUtil = require(script.Parent.MathUtil)
local VectorArray3d = require(script.Parent.VectorArray).VectorArray3d

local DCurve3 = {}
DCurve3.__index = DCurve3
DCurve3.ClassName = "DCurve3"

local function toArray(v: any): { any }
	if type(v) == "function" then
		local result = {}
		for x in v do
			table.insert(result, x)
		end
		return result
	end
	return table.clone(v)
end

function DCurve3.new()
	return setmetatable({ vertices = {}, Closed = false, Timestamp = 1 }, DCurve3)
end

function DCurve3.FromVertices(verticesIn: any, bClosed: boolean, bTakeOwnership: boolean?)
	local verts
	if bTakeOwnership and type(verticesIn) == "table" then
		verts = verticesIn
	else
		verts = toArray(verticesIn)
	end
	return setmetatable({ vertices = verts, Closed = bClosed, Timestamp = 1 }, DCurve3)
end

function DCurve3.FromCopy(copy)
	return setmetatable({ vertices = table.clone(copy.vertices), Closed = copy.Closed, Timestamp = 1 }, DCurve3)
end

function DCurve3.FromSampledCurve(icurve)
	local closed = icurve.Closed
	if type(closed) == "function" then
		closed = closed(icurve)
	end
	return setmetatable({ vertices = toArray(icurve:Vertices()), Closed = closed, Timestamp = 1 }, DCurve3)
end

function DCurve3:AppendVertex(v)
	table.insert(self.vertices, v)
	self.Timestamp += 1
end

function DCurve3:VertexCount(): number
	return #self.vertices
end

function DCurve3:SegmentCount(): number
	return self.Closed and #self.vertices or #self.vertices - 1
end

function DCurve3:IsClosed(): boolean
	return self.Closed
end

function DCurve3:GetVertex(i: number)
	return self.vertices[i + 1]
end

function DCurve3:SetVertex(i: number, v)
	self.vertices[i + 1] = v
	self.Timestamp += 1
end

-- SetVertices(IEnumerable<Vector3d>) / SetVertices(List<Vector3d>, bTakeOwnership) / SetVertices(VectorArray3d)
function DCurve3:SetVertices(v: any, bTakeOwnership: boolean?)
	if type(v) == "table" and getmetatable(v) == VectorArray3d then
		self.vertices = v:AsArray()
	elseif bTakeOwnership and type(v) == "table" then
		self.vertices = v
	else
		self.vertices = toArray(v)
	end
	self.Timestamp += 1
end

function DCurve3:ClearVertices()
	self.vertices = {}
	self.Closed = false
	self.Timestamp += 1
end

function DCurve3:RemoveVertex(idx: number)
	table.remove(self.vertices, idx + 1)
	self.Timestamp += 1
end

function DCurve3:Reverse()
	local v = self.vertices
	local n = #v
	for i = 1, n // 2 do
		v[i], v[n - i + 1] = v[n - i + 1], v[i]
	end
	self.Timestamp += 1
end

-- C# indexer get/set
function DCurve3:Get(key: number)
	return self.vertices[key + 1]
end
function DCurve3:Set(key: number, value)
	self.vertices[key + 1] = value
	self.Timestamp += 1
end

function DCurve3:Start()
	return self.vertices[1]
end
function DCurve3:End()
	return self.Closed and self.vertices[1] or self.vertices[#self.vertices]
end

-- IEnumerable<Vector3d> Vertices
function DCurve3:Vertices(): () -> any
	local v = self.vertices
	local i = 0
	return function()
		i += 1
		return v[i]
	end
end

function DCurve3:GetSegment(iSegment: number)
	local v = self.vertices
	if self.Closed then
		return Segment3d.new(v[iSegment + 1], v[((iSegment + 1) % #v) + 1])
	end
	return Segment3d.new(v[iSegment + 1], v[iSegment + 2])
end

function DCurve3:SegmentItr(): () -> any
	local v = self.vertices
	local NV = self.Closed and #v or #v - 1
	local closed = self.Closed
	local i = -1
	return function()
		i += 1
		if i >= NV then
			return nil
		end
		if closed then
			return Segment3d.new(v[i + 1], v[((i + 1) % #v) + 1])
		end
		return Segment3d.new(v[i + 1], v[i + 2])
	end
end

function DCurve3:PointAt(iSegment: number, fSegT: number)
	local v = self.vertices
	local seg = Segment3d.new(v[iSegment + 1], v[((iSegment + 1) % #v) + 1])
	return seg:PointAt(fSegT)
end

function DCurve3:GetBoundingBox()
	local box = AxisAlignedBox3d.Empty()
	for _, v in self.vertices do
		box:Contain(v)
	end
	return box
end

-- CurveUtils.ArcLength(List<Vector3d>, bLoop)
function DCurve3.ArcLengthOf(vertices: { any }, bLoop: boolean?): number
	local sum = 0
	local NV = #vertices
	for i = 2, NV do
		sum += vertices[i]:Distance(vertices[i - 1])
	end
	if bLoop then
		sum += vertices[NV]:Distance(vertices[1])
	end
	return sum
end

-- CurveUtils.GetTangent(List<Vector3d>, i, bLoop) (0-based i)
function DCurve3.TangentOf(vertices: { any }, i: number, bLoop: boolean?)
	local NV = #vertices
	if bLoop then
		if i == 0 then
			return (vertices[2] - vertices[NV]):Normalized()
		end
		return (vertices[((i + 1) % NV) + 1] - vertices[i]):Normalized()
	end
	if i == 0 then
		return (vertices[2] - vertices[1]):Normalized()
	elseif i == NV - 1 then
		return (vertices[NV] - vertices[NV - 1]):Normalized()
	end
	return (vertices[i + 2] - vertices[i]):Normalized()
end

function DCurve3:ArcLength(): number
	return DCurve3.ArcLengthOf(self.vertices, self.Closed)
end

function DCurve3:Tangent(i: number)
	return DCurve3.TangentOf(self.vertices, i, self.Closed)
end

function DCurve3:Centroid(i: number)
	local v = self.vertices
	local NV = #v
	if self.Closed then
		if i == 0 then
			return 0.5 * (v[2] + v[NV])
		end
		return 0.5 * (v[((i + 1) % NV) + 1] + v[i])
	end
	if i == 0 or i == NV - 1 then
		return v[i + 1]
	end
	return 0.5 * (v[i + 2] + v[i])
end

-- returns Index2i {prev, next} (0-based, -1 for none)
function DCurve3:Neighbours(i: number): { number }
	local NV = #self.vertices
	if self.Closed then
		if i == 0 then
			return { NV - 1, 1 }
		end
		return { i - 1, (i + 1) % NV }
	end
	if i == 0 then
		return { -1, 1 }
	elseif i == NV - 1 then
		return { NV - 2, -1 }
	end
	return { i - 1, i + 1 }
end

-- Compute opening angle at vertex i in degrees
function DCurve3:OpeningAngleDeg(i: number): number
	local v = self.vertices
	local prev, nxt = i - 1, i + 1
	if self.Closed then
		local NV = #v
		prev = (i == 0) and NV - 1 or prev
		nxt = nxt % NV
	else
		if i == 0 or i == #v - 1 then
			return 180
		end
	end
	local e1 = (v[prev + 1] - v[i + 1]):Normalized()
	local e2 = (v[nxt + 1] - v[i + 1]):Normalized()
	return Vector3d.AngleD(e1, e2)
end

-- Find nearest vertex to point p
function DCurve3:NearestVertex(p): number
	local nearSqr = MathUtil.MaxDouble
	local i = -1
	local v = self.vertices
	for vi = 0, #v - 1 do
		local distSqr = v[vi + 1]:DistanceSquared(p)
		if distSqr < nearSqr then
			nearSqr = distSqr
			i = vi
		end
	end
	return i
end

-- find squared distance from p to nearest segment on polyline
-- C# DistanceSquared(p, out iNearSeg, out fNearSegT) -> returns dist, iNearSeg, fNearSegT
function DCurve3:DistanceSquared(p): (number, number, number)
	local iNearSeg = -1
	local fNearSegT = MathUtil.MaxDouble
	local dist = MathUtil.MaxDouble
	local v = self.vertices
	local N = self.Closed and #v or #v - 1
	for vi = 0, N - 1 do
		local a = vi
		local b = (vi + 1) % #v
		local seg = Segment3d.new(v[a + 1], v[b + 1])
		local t = (p - seg.Center):Dot(seg.Direction)
		local d
		if t >= seg.Extent then
			d = seg:P1():DistanceSquared(p)
		elseif t <= -seg.Extent then
			d = seg:P0():DistanceSquared(p)
		else
			d = (seg:PointAt(t) - p):LengthSquared()
		end
		if d < dist then
			dist = d
			iNearSeg = vi
			fNearSegT = t
		end
	end
	return dist, iNearSeg, fNearSegT
end

-- Resample curve so that:
--   - if opening angle at vertex is > sharp_thresh, we emit two more vertices at +/- corner_t, where the t is used in prev/next lerps
--   - if opening angle is > flat_thresh, we skip the vertex entirely (simplification)
function DCurve3:ResampleSharpTurns(sharp_thresh: number?, flat_thresh: number?, corner_t: number?)
	local sharp = sharp_thresh or 90
	local flat = flat_thresh or 189
	local ct = corner_t or 0.01
	local v = self.vertices
	local NV = #v
	local resampled = DCurve3.new()
	resampled.Closed = self.Closed
	local prev_t = 1.0 - ct
	for k = 0, NV - 1 do
		local open_angle = math.abs(self:OpeningAngleDeg(k))
		if open_angle > flat and k > 0 then
			-- ignore skip this vertex
		elseif open_angle > sharp then
			resampled:AppendVertex(v[k + 1])
		else
			local n = v[((k + 1) % NV) + 1]
			local p = v[(k == 0) and NV or k]
			resampled:AppendVertex(Vector3d.Lerp(p, v[k + 1], prev_t))
			resampled:AppendVertex(v[k + 1])
			resampled:AppendVertex(Vector3d.Lerp(v[k + 1], n, ct))
		end
	end
	return resampled
end

return DCurve3
