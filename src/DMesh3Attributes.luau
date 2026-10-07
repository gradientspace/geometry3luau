-- Port of geometry3Sharp mesh/DMesh3Attributes.cs (AttribList, AttributeNames, DMesh3Attributes)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- C# properties: VertexColor -> VertexColor(), HasVertexColor -> HasVertexColor(), etc.
-- UV channel indices are 0-based like C#.

local GeoAttributes = require(script.Parent.GeoAttributes)
local Vector3d = require(script.Parent.Vector3d)
local Util = require(script.Parent.Util)

local EAttribType = GeoAttributes.EAttribType

---------------------------------------------------------------------------------------------------
-- AttribList<T>

local AttribList = {}
AttribList.__index = AttribList

function AttribList.new(newAttribFunc: (number) -> any, destroyAttribFunc: (any) -> ())
	return setmetatable({ createNewAttribFunc = newAttribFunc, destroyAttribFunc = destroyAttribFunc, channels = {} }, AttribList)
end

function AttribList:NumChannels(): number
	return #self.channels
end

function AttribList:SetNumChannels(NewNumChannels: number)
	local channels = self.channels
	while #channels > NewNumChannels do
		self.destroyAttribFunc(channels[#channels])
		table.remove(channels)
	end
	while #channels < NewNumChannels do
		local newChannel = self.createNewAttribFunc(#channels)
		table.insert(channels, newChannel)
	end
end

function AttribList:HasChannel(i: number): boolean
	return i >= 0 and i < self:NumChannels()
end

function AttribList:GetChannel(i: number)
	if not self:HasChannel(i) then
		self:SetNumChannels(i + 1)
	end
	return self.channels[i + 1]
end

function AttribList:Channels(): () -> any
	local i = 0
	local channels = self.channels
	return function()
		i += 1
		return channels[i]
	end
end

---------------------------------------------------------------------------------------------------
-- AttributeNames

local AttributeNames = {
	VertexColor = "VertexColor",
	MaterialID = "MaterialID",
	TriNormals = "TriNormals;",
	TriUVsPrefix = "TriUV;",
}
function AttributeNames.TriUV(Index: number): string
	return AttributeNames.TriUVsPrefix .. tostring(Index)
end

---------------------------------------------------------------------------------------------------
-- DMesh3Attributes

local DMesh3Attributes = {}
DMesh3Attributes.__index = DMesh3Attributes
DMesh3Attributes.AttribList = AttribList
DMesh3Attributes.AttributeNames = AttributeNames

function DMesh3Attributes.new(parentMesh)
	return setmetatable({
		parentMesh = parentMesh,
		allAttributes = {},
		vertexColor = nil,
		materialID = nil,
		triNormals = nil,
		triangleUVs = nil,
	}, DMesh3Attributes)
end

function DMesh3Attributes:ParentMesh()
	return self.parentMesh
end

function DMesh3Attributes:register_attribute(geoAttribute)
	table.insert(self.allAttributes, geoAttribute)
	return geoAttribute
end

function DMesh3Attributes:unregister_attribute(attribute)
	if attribute ~= nil then
		local idx = table.find(self.allAttributes, attribute)
		if idx then
			table.remove(self.allAttributes, idx)
		end
	end
	return nil
end

-- vertex colors
function DMesh3Attributes:HasVertexColor(): boolean
	return self.vertexColor ~= nil
end
function DMesh3Attributes:EnableVertexColor(initialColor)
	if self.vertexColor == nil then
		self.vertexColor = self:register_attribute(
			GeoAttributes.VertexColorsGeoAttribute.new(
				AttributeNames.VertexColor,
				self.parentMesh,
				EAttribType.Vertex,
				initialColor or Vector3d.Zero
			)
		)
	end
end
function DMesh3Attributes:DisableVertexColor()
	self.vertexColor = self:unregister_attribute(self.vertexColor)
end
function DMesh3Attributes:VertexColor()
	if not self:HasVertexColor() then
		self:EnableVertexColor(Vector3d.Zero)
	end
	return self.vertexColor
end

-- material ID
function DMesh3Attributes:HasMaterialID(): boolean
	return self.materialID ~= nil
end
function DMesh3Attributes:EnableMaterialID()
	if self.materialID == nil then
		self.materialID =
			self:register_attribute(GeoAttributes.IntGeoAttribute.new(AttributeNames.MaterialID, self.parentMesh, EAttribType.Triangle))
	end
end
function DMesh3Attributes:DisableMaterialID()
	self.materialID = self:unregister_attribute(self.materialID)
end
function DMesh3Attributes:MaterialID()
	if not self:HasMaterialID() then
		self:EnableMaterialID()
	end
	return self.materialID
end

-- per-triangle normals
function DMesh3Attributes:HasTriNormals(): boolean
	return self.triNormals ~= nil
end
function DMesh3Attributes:EnableTriNormals()
	if self.triNormals == nil then
		self.triNormals = self:register_attribute(
			GeoAttributes.TriNormalsGeoAttribute.new(AttributeNames.TriNormals, self.parentMesh, EAttribType.Triangle)
		)
	end
end
function DMesh3Attributes:DisableTriNormals()
	self.triNormals = self:unregister_attribute(self.triNormals)
end
function DMesh3Attributes:TriNormals()
	if not self:HasTriNormals() then
		self:EnableTriNormals()
	end
	return self.triNormals
end

-- per-triangle UVs (multiple channels)
function DMesh3Attributes:HasTriUVs(): boolean
	return self.triangleUVs ~= nil
end
function DMesh3Attributes:NumTriUVChannels(): number
	return (self.triangleUVs == nil) and 0 or self.triangleUVs:NumChannels()
end
function DMesh3Attributes:EnableTriUVs(NumChannels: number?)
	local numChannels = NumChannels or 1
	if self.triangleUVs == nil then
		self.triangleUVs = AttribList.new(function(idx: number)
			return self:register_attribute(
				GeoAttributes.TriUVsGeoAttribute.new(AttributeNames.TriUV(idx), self.parentMesh, EAttribType.Triangle)
			)
		end, function(attrib)
			self:unregister_attribute(attrib)
		end)
	end
	if self.triangleUVs:NumChannels() < numChannels then
		self.triangleUVs:SetNumChannels(numChannels)
	end
end
function DMesh3Attributes:DisableTriUVs()
	if self.triangleUVs ~= nil then
		self:SetNumUVChannels(0)
	end
	self.triangleUVs = nil
end
function DMesh3Attributes:SetNumUVChannels(NewNumChannels: number)
	if self.triangleUVs == nil and NewNumChannels == 0 then
		return
	end
	if self.triangleUVs == nil then
		self:EnableTriUVs(NewNumChannels)
	else
		self.triangleUVs:SetNumChannels(NewNumChannels)
	end
end
function DMesh3Attributes:HasTriUVChannel(channelIndex: number): boolean
	return self.triangleUVs ~= nil and self.triangleUVs:HasChannel(channelIndex)
end
function DMesh3Attributes:TriUVChannel(channelIndex: number)
	if self.triangleUVs == nil then
		self:EnableTriUVs(channelIndex)
	end
	return self.triangleUVs:GetChannel(channelIndex)
end

-- iteration
function DMesh3Attributes:Attributes(): () -> any
	local i = 0
	local all = self.allAttributes
	return function()
		i += 1
		return all[i]
	end
end

local function filtered(all, t)
	local i = 0
	return function()
		while true do
			i += 1
			local a = all[i]
			if a == nil then
				return nil
			end
			if a.AttribType == t then
				return a
			end
		end
	end
end

function DMesh3Attributes:TriAttributes(): () -> any
	return filtered(self.allAttributes, EAttribType.Triangle)
end

function DMesh3Attributes:VertAttributes(): () -> any
	return filtered(self.allAttributes, EAttribType.Vertex)
end

function DMesh3Attributes:FindByName(Name: string)
	local lname = string.lower(Name)
	for _, a in self.allAttributes do
		if string.lower(a.Name) == lname then
			return a
		end
	end
	return nil
end

function DMesh3Attributes:EnableMatching(Other)
	if Other:HasMaterialID() then
		self:EnableMaterialID()
	end
	error("DMesh3Attributes.EnableMatching: not implemented") -- todo (as in C#)
end

-- Copy(other, mapV?, mapT?): mapV/mapT are 1-based arrays mapping new index i -> old index (mapV[i+1])
function DMesh3Attributes:Copy(other, mapV: { number }?, mapT: { number }?)
	local MapV = (mapV ~= nil) and function(a: number)
		return (mapV :: { number })[a + 1]
	end or nil
	local MapT = (mapT ~= nil) and function(a: number)
		return (mapT :: { number })[a + 1]
	end or nil

	if other:HasVertexColor() then
		self:VertexColor():InitializeToCopy(self.parentMesh:VertexCount(), other:VertexColor(), MapV)
	end
	if other:HasMaterialID() then
		self:MaterialID():InitializeToCopy(self.parentMesh:TriangleCount(), other:MaterialID(), MapT)
	end
	if other:HasTriNormals() then
		self:TriNormals():InitializeToCopy(self.parentMesh:TriangleCount(), other:TriNormals(), MapT)
	end
	if other:HasTriUVs() then
		self:EnableTriUVs()
		for i = 0, other:NumTriUVChannels() - 1 do
			self:TriUVChannel(i):InitializeToCopy(self.parentMesh:TriangleCount(), other:TriUVChannel(i), MapT)
		end
	end
end

function DMesh3Attributes:TrimTo(MaxVertID: number, MaxTriID: number)
	for VertAttrib in self:VertAttributes() do
		VertAttrib:TrimToLength(MaxVertID)
	end
	for TriAttrib in self:TriAttributes() do
		TriAttrib:TrimToLength(MaxTriID)
	end
end

function DMesh3Attributes:OnRemoveTriangle(tid: number)
	-- don't actually have to do anything for remove? just leave value?
end

function DMesh3Attributes:OnReverseTriOrientation(tid: number)
	for attrib in self:TriAttributes() do
		if attrib.IsLinear then
			attrib:UpdateOnReverseTriOrientation(tid)
		end
	end
end

function DMesh3Attributes:OnSplitEdge(splitInfo)
	for _, attrib in self.allAttributes do
		if attrib.IsLinear then
			attrib:UpdateOnSplit(self.parentMesh, splitInfo)
		elseif attrib.AttribType == EAttribType.Triangle then
			attrib:InsertValue_Copy(splitInfo.eNewT2, splitInfo.eOrigT0)
			if splitInfo.eOrigT1 >= 0 then
				attrib:InsertValue_Copy(splitInfo.eNewT3, splitInfo.eOrigT1)
			end
		elseif attrib.AttribType == EAttribType.Vertex then
			attrib:InsertValue_Copy(splitInfo.vNew, splitInfo.eOrigAB[1]) -- arbitrary...
		end
	end
end

function DMesh3Attributes:OnFlipEdge(flipInfo)
	-- ?? could try to update some tri attributes...
end

function DMesh3Attributes:OnCollapseEdge(collapseInfo)
	-- ?? possibly want to update some tri attributes...
end

function DMesh3Attributes:OnMergeEdges(mergeInfo)
	-- ??
end

function DMesh3Attributes:OnPokeTriangle(pokeInfo)
	for _, attrib in self.allAttributes do
		if attrib.IsLinear then
			attrib:UpdateOnPoke(pokeInfo)
		elseif attrib.AttribType == EAttribType.Triangle then
			attrib:InsertValue_Copy(pokeInfo.new_t1, pokeInfo.orig_t0)
			attrib:InsertValue_Copy(pokeInfo.new_t2, pokeInfo.orig_t0)
		elseif attrib.AttribType == EAttribType.Vertex then
			attrib:InsertValue_Copy(pokeInfo.new_vid, pokeInfo.orig_tri[1]) -- arbitrary...
		end
	end
end

function DMesh3Attributes:CheckValidity(eFailMode: number?): boolean
	local failMode = eFailMode or Util.FailMode.Throw
	local is_ok = true
	local function CheckOrFailF(b: boolean)
		if failMode == Util.FailMode.Throw then
			if not b then
				error("DMesh3Attributes.CheckValidity: check failed", 2)
			end
		elseif failMode == Util.FailMode.DebugAssert or failMode == Util.FailMode.gDevAssert then
			Util.gDevAssert(b)
		end
		is_ok = is_ok and b
	end

	CheckOrFailF(self.parentMesh ~= nil)
	local TriCount = self.parentMesh:MaxTriangleID()
	local VertCount = self.parentMesh:MaxVertexID()

	if self.vertexColor ~= nil then
		CheckOrFailF(self.vertexColor:NumElements() == VertCount)
		CheckOrFailF(self:FindByName(AttributeNames.VertexColor) == self.vertexColor)
	else
		CheckOrFailF(self:FindByName(AttributeNames.VertexColor) == nil)
	end

	if self.materialID ~= nil then
		CheckOrFailF(self.materialID:NumElements() == TriCount)
		CheckOrFailF(self:FindByName(AttributeNames.MaterialID) == self.materialID)
	else
		CheckOrFailF(self:FindByName(AttributeNames.MaterialID) == nil)
	end

	if self.triNormals ~= nil then
		CheckOrFailF(self.triNormals:NumElements() == TriCount)
		CheckOrFailF(self:FindByName(AttributeNames.TriNormals) == self.triNormals)
	else
		CheckOrFailF(self:FindByName(AttributeNames.TriNormals) == nil)
	end

	if self.triangleUVs ~= nil then
		local n = self.triangleUVs:NumChannels()
		for i = 0, n - 1 do
			CheckOrFailF(self.triangleUVs:HasChannel(i) == true)
			CheckOrFailF(self.triangleUVs:GetChannel(i):NumElements() == TriCount)
			CheckOrFailF(self:FindByName(AttributeNames.TriUV(i)) == self.triangleUVs:GetChannel(i))
		end
		for i = n, 7 do
			CheckOrFailF(self:FindByName(AttributeNames.TriUV(i)) == nil)
		end
	else
		for i = 0, 7 do
			CheckOrFailF(self:FindByName(AttributeNames.TriUV(i)) == nil)
		end
	end
	return is_ok
end

return DMesh3Attributes
