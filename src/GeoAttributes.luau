--!native
-- Port of geometry3Sharp mesh/GeoAttributeTypes.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Fixed-count per-element mesh attributes (one value per vertex or per triangle), stored in a DVector.
-- Values are immutable (numbers, Vector2d/Vector3d, or frozen TriUVs/TriNormals tuples), so they can be
-- shared between elements without copying.
--
-- Classes:
--   BaseGeoAttribute            (IGeoAttribute<T>)
--   IntGeoAttribute, Vector2fGeoAttribute, Vector3fGeoAttribute
--   BaseBaryVertGeoAttribute -> VertexColorsGeoAttribute    (ILinearGeoAttribute, per-vertex)
--   BaseBaryTriGeoAttribute  -> TriUVsGeoAttribute, TriNormalsGeoAttribute (ILinearGeoAttribute, per-triangle)
-- Tuples:
--   TriUVs.new(A, B, C) / TriNormals.new(A, B, C) -> { A = .., B = .., C = .. } (values are Vector2d / Vector3d)
--
-- ILinearGeoAttribute members are UpdateOnPoke, UpdateOnSplit, UpdateOnReverseTriOrientation;
-- objects implementing them have IsLinear = true.

local DVector = require(script.Parent.DVector)
local Vector2d = require(script.Parent.Vector2d)
local Vector3d = require(script.Parent.Vector3d)
local Util = require(script.Parent.Util)
local Index = require(script.Parent.Index)

local GeoAttributes = {}

-- IGeoAttribute.EAttribType
local EAttribType = table.freeze({
	Unspecified = 0,
	Triangle = 1,
	Vertex = 2,
})
GeoAttributes.EAttribType = EAttribType

---------------------------------------------------------------------------------------------------
-- tuple types

local TriUVs = {}
TriUVs.__index = TriUVs
GeoAttributes.TriUVs = TriUVs
function TriUVs.new(a, b, c)
	return table.freeze(setmetatable({ A = a or Vector2d.Zero, B = b or Vector2d.Zero, C = c or Vector2d.Zero }, TriUVs))
end
function TriUVs:Get(key: number)
	return (key == 0) and self.A or ((key == 1) and self.B or self.C)
end
-- returns copy with element key replaced (C# this[key] = value on a struct copy)
function TriUVs:With(key: number, value)
	if key == 0 then
		return TriUVs.new(value, self.B, self.C)
	elseif key == 1 then
		return TriUVs.new(self.A, value, self.C)
	end
	return TriUVs.new(self.A, self.B, value)
end
function TriUVs:BaryPoint(baryCoords)
	return baryCoords.x * self.A + baryCoords.y * self.B + baryCoords.z * self.C
end
TriUVs.Default = TriUVs.new(Vector2d.Zero, Vector2d.Zero, Vector2d.Zero)

local TriNormals = {}
TriNormals.__index = TriNormals
GeoAttributes.TriNormals = TriNormals
function TriNormals.new(a, b, c)
	return table.freeze(setmetatable({ A = a or Vector3d.Zero, B = b or Vector3d.Zero, C = c or Vector3d.Zero }, TriNormals))
end
function TriNormals:Get(key: number)
	return (key == 0) and self.A or ((key == 1) and self.B or self.C)
end
function TriNormals:With(key: number, value)
	if key == 0 then
		return TriNormals.new(value, self.B, self.C)
	elseif key == 1 then
		return TriNormals.new(self.A, value, self.C)
	end
	return TriNormals.new(self.A, self.B, value)
end
function TriNormals:BaryPoint(baryCoords)
	return baryCoords.x * self.A + baryCoords.y * self.B + baryCoords.z * self.C
end
TriNormals.Default = TriNormals.new(Vector3d.Zero, Vector3d.Zero, Vector3d.Zero)

---------------------------------------------------------------------------------------------------
-- base type for a DVector-backed geo attribute

local BaseGeoAttribute = {}
BaseGeoAttribute.__index = BaseGeoAttribute
GeoAttributes.BaseGeoAttribute = BaseGeoAttribute

-- default value for elements: 0 for ints, Vector3d.Zero etc. Subclasses set DefaultValue.
BaseGeoAttribute.DefaultValue = 0

local function get_element_count(self, parentMesh): number
	if self.AttribType == EAttribType.Triangle then
		return parentMesh:MaxTriangleID()
	elseif self.AttribType == EAttribType.Vertex then
		return parentMesh:MaxVertexID()
	end
	return 0
end

-- BaseGeoAttribute(name, parentMesh?, attribType?, initialValue?)
-- (if initialValue is given, it corresponds to the C# constructor taking T InitialValue)
function BaseGeoAttribute.init(self, name: string, parentMesh, attribType: number?, initialValue: any?)
	self.data = DVector.new()
	self.Name = name or "UNNAMED"
	self.AttribType = attribType or EAttribType.Unspecified
	if parentMesh ~= nil then
		if initialValue ~= nil then
			self:Initialize(get_element_count(self, parentMesh), initialValue)
		else
			self:Initialize(get_element_count(self, parentMesh))
		end
	end
	return self
end

function BaseGeoAttribute.new(name: string, parentMesh, attribType: number?, initialValue: any?)
	return BaseGeoAttribute.init(setmetatable({}, BaseGeoAttribute), name, parentMesh, attribType, initialValue)
end

function BaseGeoAttribute:NumElements(): number
	return self.data.size
end

-- Initialize(Count) (no init of values) or Initialize(Count, InitialValue)
function BaseGeoAttribute:Initialize(Count: number, InitialValue: any?)
	if InitialValue == nil then
		self:UpdateCount(Count, false)
	else
		self:UpdateCount(Count, true, InitialValue)
	end
end

-- InitializeToCopy(NewCount, SourceAttrib, MapNewToOldF?) where MapNewToOldF maps new index -> old index
function BaseGeoAttribute:InitializeToCopy(NewCount: number, SourceAttrib, MapNewToOldF: ((number) -> number)?)
	if SourceAttrib.data == nil then
		error("BaseGeoAttribute.InitializeToCopy: source attribute type is not supported")
	end
	if MapNewToOldF == nil then
		Util.gDevAssert(NewCount == SourceAttrib:NumElements())
		self.data:copy(SourceAttrib.data)
		return
	end
	local SourceCount = SourceAttrib:NumElements()
	if SourceCount == NewCount then
		self.data:copy(SourceAttrib.data)
	else
		self:UpdateCount(NewCount, false)
		for i = 0, NewCount - 1 do
			self:SetValue(i, SourceAttrib:GetValue(MapNewToOldF(i)))
		end
	end
end

function BaseGeoAttribute:TrimToLength(NewCount: number)
	Util.gDevAssert(NewCount <= self:NumElements())
	self.data:resize(NewCount)
end

function BaseGeoAttribute:UpdateCount(NewCount: number, bInitializeNew: boolean?, InitValue: any?)
	local prev_size = self.data.size
	self.data:resize(NewCount)
	-- (DVector.resize fills new elements with 0; replace with the typed default)
	local fill = InitValue
	if not bInitializeNew or fill == nil then
		fill = self.DefaultValue
	end
	local d = self.data.data
	for i = prev_size + 1, self.data.size do
		d[i] = fill
	end
end

function BaseGeoAttribute:InsertValue(Index: number, NewValue: any)
	local prev = self.data.size
	self.data:insert(NewValue, Index)
	-- elements created between prev and Index get the default value
	local d, def = self.data.data, self.DefaultValue
	for i = prev + 1, Index do
		d[i] = def
	end
end

function BaseGeoAttribute:InsertValue_Copy(Index: number, CopyIndex: number?)
	local ci = CopyIndex or -1
	if ci >= 0 and ci < self.data.size then
		self:InsertValue(Index, self.data.data[ci + 1])
	else
		self:InsertValue(Index, self.DefaultValue)
	end
end

function BaseGeoAttribute:SetValue(Index: number, NewValue: any)
	self.data.data[Index + 1] = NewValue
end

function BaseGeoAttribute:GetValue(Index: number): any
	return self.data.data[Index + 1]
end

---------------------------------------------------------------------------------------------------

local function subclass(base, defaultValue)
	local cls = setmetatable({}, { __index = base })
	cls.__index = cls
	cls.DefaultValue = defaultValue
	cls.new = function(name, parentMesh, attribType, initialValue)
		return base.init(setmetatable({}, cls), name, parentMesh, attribType, initialValue)
	end
	return cls
end

GeoAttributes.IntGeoAttribute = subclass(BaseGeoAttribute, 0)
GeoAttributes.Vector2fGeoAttribute = subclass(BaseGeoAttribute, Vector2d.Zero)
GeoAttributes.Vector3fGeoAttribute = subclass(BaseGeoAttribute, Vector3d.Zero)

---------------------------------------------------------------------------------------------------
-- per-vertex attributes that have their value interpolated during mesh refinement edits

local BaseBaryVertGeoAttribute = subclass(BaseGeoAttribute, 0)
BaseBaryVertGeoAttribute.IsLinear = true
GeoAttributes.BaseBaryVertGeoAttribute = BaseBaryVertGeoAttribute

function BaseBaryVertGeoAttribute:UpdateOnPoke(pokeInfo)
	self:UpdateOnPoke_Impl(pokeInfo)
end
function BaseBaryVertGeoAttribute:UpdateOnSplit(Mesh, splitInfo)
	self:UpdateOnSplit_Impl(Mesh, splitInfo)
end
function BaseBaryVertGeoAttribute:UpdateOnReverseTriOrientation(TriangleID: number) end

function BaseBaryVertGeoAttribute:UpdateOnPoke_Impl(pokeInfo)
	local a = self:GetValue(pokeInfo.orig_tri[1])
	local b = self:GetValue(pokeInfo.orig_tri[2])
	local c = self:GetValue(pokeInfo.orig_tri[3])
	local newValue = self:GetBaryValue(a, b, c, pokeInfo.new_vid_barycoords)
	self:InsertValue(pokeInfo.new_vid, newValue)
end

function BaseBaryVertGeoAttribute:UpdateOnSplit_Impl(Mesh, splitInfo)
	local a = self:GetValue(splitInfo.eOrigAB[1])
	local b = self:GetValue(splitInfo.eOrigAB[2])
	local newValue = self:GetInterpValue(a, b, splitInfo.split_t)
	self:InsertValue(splitInfo.vNew, newValue)
end

local VertexColorsGeoAttribute = subclass(BaseBaryVertGeoAttribute, Vector3d.Zero)
GeoAttributes.VertexColorsGeoAttribute = VertexColorsGeoAttribute

function VertexColorsGeoAttribute:GetInterpValue(A, B, interp_t: number)
	return Vector3d.Lerp(A, B, interp_t)
end
-- GetBaryValue(A, B, C, baryCoords) with values, or GetBaryValueIdx(ai, bi, ci, baryCoords) with vertex ids
function VertexColorsGeoAttribute:GetBaryValue(A, B, C, baryCoords)
	return baryCoords.x * A + baryCoords.y * B + baryCoords.z * C
end
function VertexColorsGeoAttribute:GetBaryValueIdx(ai: number, bi: number, ci: number, baryCoords)
	return baryCoords.x * self:GetValue(ai) + baryCoords.y * self:GetValue(bi) + baryCoords.z * self:GetValue(ci)
end
-- returns DVector of floats (3 per element)
function VertexColorsGeoAttribute:ToBuffer()
	local buffer = DVector.new()
	local N = self:NumElements()
	buffer:resize(N * 3)
	local d = buffer.data
	for i = 0, N - 1 do
		local v = self:GetValue(i)
		d[3 * i + 1], d[3 * i + 2], d[3 * i + 3] = v.x, v.y, v.z
	end
	return buffer
end
function VertexColorsGeoAttribute:SetFromBuffer(buffer)
	local N = buffer.size // 3
	self:UpdateCount(N)
	local d = buffer.data
	for i = 0, N - 1 do
		self:SetValue(i, Vector3d.new(d[3 * i + 1], d[3 * i + 2], d[3 * i + 3]))
	end
end

---------------------------------------------------------------------------------------------------
-- per-triangle tuple attributes that have their value interpolated during mesh refinement edits

local BaseBaryTriGeoAttribute = subclass(BaseGeoAttribute, 0)
BaseBaryTriGeoAttribute.IsLinear = true
GeoAttributes.BaseBaryTriGeoAttribute = BaseBaryTriGeoAttribute

-- subclasses implement: MakeNewValue(a,b,c), GetBaryValue(tuple, bary), MakeLerpValue(a,b,alpha),
-- GetValueElem(tuple, index), SetValueElem(tuple, index, value) -> returns new tuple

function BaseBaryTriGeoAttribute:UpdateOnPoke(pokeInfo)
	self:UpdateOnPoke_Impl(pokeInfo)
end
function BaseBaryTriGeoAttribute:UpdateOnSplit(Mesh, splitInfo)
	self:UpdateOnSplit_Impl(Mesh, splitInfo)
end
function BaseBaryTriGeoAttribute:UpdateOnReverseTriOrientation(TriangleID: number)
	self:UpdateOnReverseTriOrientation_Impl(TriangleID)
end

function BaseBaryTriGeoAttribute:UpdateOnReverseTriOrientation_Impl(TriangleID: number)
	-- set_triangle(tID, t[1], t[0], t[2]);
	local curVals = self:GetValue(TriangleID)
	local newVals = self:MakeNewValue(self:GetValueElem(curVals, 1), self:GetValueElem(curVals, 0), self:GetValueElem(curVals, 2))
	self:SetValue(TriangleID, newVals)
end

function BaseBaryTriGeoAttribute:UpdateOnPoke_Impl(pokeInfo)
	local orig = self:GetValue(pokeInfo.orig_t0)
	local centerUV = self:GetBaryValue(orig, pokeInfo.new_vid_barycoords)
	local A = self:GetValueElem(orig, 0)
	local B = self:GetValueElem(orig, 1)
	local C = self:GetValueElem(orig, 2)
	self:SetValue(pokeInfo.orig_t0, self:MakeNewValue(A, B, centerUV))
	self:InsertValue(pokeInfo.new_t1, self:MakeNewValue(B, C, centerUV))
	self:InsertValue(pokeInfo.new_t2, self:MakeNewValue(C, A, centerUV))
end

function BaseBaryTriGeoAttribute:UpdateOnSplit_Impl(Mesh, splitInfo)
	local f = splitInfo.vNew
	local Index3i = Index.Index3i

	local orig0 = self:GetValue(splitInfo.eOrigT0)
	local curT0 = Mesh:GetTriangle(splitInfo.eOrigT0)
	local newFBC = Mesh:GetTriangle(splitInfo.eNewT2)
	local fb_Index = Index3i.IndexOf(curT0, f)
	local c = newFBC[3]
	local c_Index = Index3i.IndexOf(curT0, c)
	local a_Index = Index3i.OtherIndex(fb_Index, c_Index)
	local a_Pos = self:GetValueElem(orig0, a_Index)
	local b_Pos = self:GetValueElem(orig0, fb_Index)
	local c_Pos = self:GetValueElem(orig0, c_Index)
	local f_Pos = self:MakeLerpValue(a_Pos, b_Pos, splitInfo.split_t) -- is this always right order?
	orig0 = self:SetValueElem(orig0, fb_Index, f_Pos)
	self:SetValue(splitInfo.eOrigT0, orig0)
	self:InsertValue(splitInfo.eNewT2, self:MakeNewValue(f_Pos, b_Pos, c_Pos)) -- (f,b,c)

	-- note this is exactly the same as above, only with T1/T3 and some slight index permutations...
	-- (have to recompute a/b because tris may not be connected in UV!)
	if splitInfo.eOrigT1 >= 0 then -- tri /a/b/d becomes a/f/d
		local orig1 = self:GetValue(splitInfo.eOrigT1)
		local curT1 = Mesh:GetTriangle(splitInfo.eOrigT1)
		local NewFDB = Mesh:GetTriangle(splitInfo.eNewT3)
		fb_Index = Index3i.IndexOf(curT1, f)
		local d = NewFDB[2]
		local d_index = Index3i.IndexOf(curT1, d)
		a_Index = Index3i.OtherIndex(fb_Index, d_index)
		a_Pos = self:GetValueElem(orig1, a_Index)
		b_Pos = self:GetValueElem(orig1, fb_Index)
		local d_Pos = self:GetValueElem(orig1, d_index)
		f_Pos = self:MakeLerpValue(a_Pos, b_Pos, splitInfo.split_t)
		orig1 = self:SetValueElem(orig1, fb_Index, f_Pos)
		self:SetValue(splitInfo.eOrigT1, orig1)
		self:InsertValue(splitInfo.eNewT3, self:MakeNewValue(f_Pos, d_Pos, b_Pos)) -- (f,d,b)
	end
end

local TriUVsGeoAttribute = subclass(BaseBaryTriGeoAttribute, TriUVs.Default)
GeoAttributes.TriUVsGeoAttribute = TriUVsGeoAttribute

function TriUVsGeoAttribute:GetBaryValue(tuple, baryCoords)
	return tuple:BaryPoint(baryCoords)
end
function TriUVsGeoAttribute:MakeNewValue(a, b, c)
	return TriUVs.new(a, b, c)
end
function TriUVsGeoAttribute:MakeLerpValue(a, b, alpha: number)
	return Vector2d.Lerp(a, b, alpha)
end
function TriUVsGeoAttribute:GetValueElem(tuple, index: number)
	return tuple:Get(index)
end
function TriUVsGeoAttribute:SetValueElem(tuple, index: number, newValue)
	return tuple:With(index, newValue)
end

local TriNormalsGeoAttribute = subclass(BaseBaryTriGeoAttribute, TriNormals.Default)
GeoAttributes.TriNormalsGeoAttribute = TriNormalsGeoAttribute

function TriNormalsGeoAttribute:GetBaryValue(tuple, baryCoords)
	return tuple:BaryPoint(baryCoords)
end
function TriNormalsGeoAttribute:MakeNewValue(a, b, c)
	return TriNormals.new(a, b, c)
end
function TriNormalsGeoAttribute:MakeLerpValue(a, b, alpha: number)
	return Vector3d.Lerp(a, b, alpha)
end
function TriNormalsGeoAttribute:GetValueElem(tuple, index: number)
	return tuple:Get(index)
end
function TriNormalsGeoAttribute:SetValueElem(tuple, index: number, newValue)
	return tuple:With(index, newValue)
end
function TriNormalsGeoAttribute:UpdateOnReverseTriOrientation_Impl(TriangleID: number)
	-- set_triangle(tID, t[1], t[0], t[2]);
	local normals = self:GetValue(TriangleID)
	self:SetValue(TriangleID, TriNormals.new(-normals:Get(1), -normals:Get(0), -normals:Get(2)))
end

return GeoAttributes
