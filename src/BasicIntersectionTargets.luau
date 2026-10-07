--!native
-- Port of geometry3Sharp spatial/BasicIntersectionTargets.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- IIntersectionTarget implementations (see SpatialInterfaces.luau). Module returns
--   { TransformedIntersectionTarget, DMeshIntersectionTarget, PlaneIntersectionTarget }
-- (each is also available as its own module of the same name).
--
-- Luau API notes:
--   target:HasNormal() (C# property), target:RayIntersect(ray) -> bool, vHit, vHitNormal  (C# out params)
--   DMeshIntersectionTarget.new(mesh?, spatial?)  fields Mesh, Spatial, UseFaceNormal
--   PlaneIntersectionTarget.new(planeFrame?, normalAxis?)  fields PlaneFrame, NormalAxis
--   TransformedIntersectionTarget.new(baseTarget?)  fields BaseTarget, MapToBaseF, MapFromBasePosF, MapFromBaseNormalF
--     (the C# BaseTarget is a private field with no way to set it; here it is a constructor argument/field.
--      The C# RayIntersect maps the hit normal with MapFromBasePosF; here MapFromBaseNormalF is used when set.)

local Vector3d = require(script.Parent.Vector3d)

local function MeshQueries()
	return require(script.Parent.MeshQueries) :: any
end

---------------------------------------------------------------------------------------------------
local TransformedIntersectionTarget = {}
TransformedIntersectionTarget.__index = TransformedIntersectionTarget
TransformedIntersectionTarget.ClassName = "TransformedIntersectionTarget"

function TransformedIntersectionTarget.new(baseTarget)
	return setmetatable({
		BaseTarget = baseTarget,
		MapToBaseF = nil,
		MapFromBasePosF = nil,
		MapFromBaseNormalF = nil,
	}, TransformedIntersectionTarget)
end

function TransformedIntersectionTarget:HasNormal(): boolean
	return self.BaseTarget:HasNormal()
end

function TransformedIntersectionTarget:RayIntersect(ray)
	local baseRay = self.MapToBaseF(ray)
	local ok, vHit, vHitNormal = self.BaseTarget:RayIntersect(baseRay)
	if ok then
		vHit = self.MapFromBasePosF(vHit)
		local normalF = self.MapFromBaseNormalF or self.MapFromBasePosF
		vHitNormal = normalF(vHitNormal)
		return true, vHit, vHitNormal
	end
	return false, vHit, vHitNormal
end

---------------------------------------------------------------------------------------------------
local DMeshIntersectionTarget = {}
DMeshIntersectionTarget.__index = DMeshIntersectionTarget
DMeshIntersectionTarget.ClassName = "DMeshIntersectionTarget"

function DMeshIntersectionTarget.new(mesh, spatial)
	return setmetatable({
		Mesh = mesh,
		Spatial = spatial,
		UseFaceNormal = true,
	}, DMeshIntersectionTarget)
end

function DMeshIntersectionTarget:HasNormal(): boolean
	return true
end

function DMeshIntersectionTarget:RayIntersect(ray)
	local vHit = Vector3d.Zero
	local vHitNormal = Vector3d.AxisX
	local tHitID = self.Spatial:FindNearestHitTriangle(ray)
	if tHitID == -1 then
		return false, vHit, vHitNormal
	end
	local mesh = self.Mesh
	local t = MeshQueries().TriangleIntersection(mesh, tHitID, ray)
	vHit = ray:PointAt(t.RayParameter)
	if self.UseFaceNormal == false and mesh:HasVertexNormals() then
		local b = t.TriangleBaryCoords
		vHitNormal = mesh:GetTriBaryNormal(tHitID, b.x, b.y, b.z)
	else
		vHitNormal = mesh:GetTriNormal(tHitID)
	end
	return true, vHit, vHitNormal
end

---------------------------------------------------------------------------------------------------
-- Compute ray-intersection with plane
local PlaneIntersectionTarget = {}
PlaneIntersectionTarget.__index = PlaneIntersectionTarget
PlaneIntersectionTarget.ClassName = "PlaneIntersectionTarget"

function PlaneIntersectionTarget.new(planeFrame, normalAxis: number?)
	return setmetatable({
		PlaneFrame = planeFrame,
		NormalAxis = normalAxis or 2,
	}, PlaneIntersectionTarget)
end

function PlaneIntersectionTarget:HasNormal(): boolean
	return true
end

function PlaneIntersectionTarget:RayIntersect(ray)
	local rayHit = self.PlaneFrame:RayPlaneIntersection(ray.Origin, ray.Direction, self.NormalAxis)
	local vHit = rayHit
	local vHitNormal = Vector3d.AxisY
	return (rayHit ~= Vector3d.Invalid), vHit, vHitNormal
end

return {
	TransformedIntersectionTarget = TransformedIntersectionTarget,
	DMeshIntersectionTarget = DMeshIntersectionTarget,
	PlaneIntersectionTarget = PlaneIntersectionTarget,
}
