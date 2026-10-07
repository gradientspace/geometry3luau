--!native
-- Port of geometry3Sharp spatial/BasicProjectionTargets.cs
--   MeshProjectionTarget, TransformedMeshProjectionTarget, PlaneProjectionTarget,
--   CircleProjectionTarget, CylinderProjectionTarget, SequentialProjectionTarget
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- This module returns a table of the classes. Each class is also available as its own module
-- (G3.MeshProjectionTarget, G3.CircleProjectionTarget, ...), which re-exports the class from here.
--
-- Projection target interfaces (duck typed, used across the package):
--   IProjectionTarget:          target:Project(vPoint, identifier?) -> Vector3d
--   IOrientedProjectionTarget:  target:Project(vPoint, identifier?) -> Vector3d, normal
--       The C# overload Project(vPoint, out Vector3d vProjectNormal, identifier) is the same call: the
--       normal is the second return value. Oriented targets set the field
--       IsOrientedProjectionTarget = true (RemesherPro checks this for face-aligned projection).
--   identifier is the vertex id being projected (or -1 / nil); the basic targets ignore it.
--
-- Luau port notes:
--   * MeshProjectionTarget.new(mesh?, spatial?)   new MeshProjectionTarget(), (mesh), (mesh, spatial)
--       If spatial is nil a DMeshAABBTree3(mesh, true) is built. Fields Mesh, Spatial (C# properties).
--       Spatial must implement FindNearestTriangle(p) (ISpatial); FindNearestTriangleXYZ is used when present.
--     MeshProjectionTarget.Auto(mesh, bForceCopy = true)
--     MeshProjectionTarget.Auto(mesh, triangles, nExpandRings = 5)   (needs MeshFaceSelection + DSubmesh3)
--       (C# overloads; dispatch on the type of the second argument: boolean/nil vs triangle list)
--   * TransformedMeshProjectionTarget.new(mesh?, spatial?), SetTransform(TransformSequence)
--       fields SourceToTargetXForm, TargetToSourceXForm (TransformSequence or any object with
--       TransformP/TransformV)
--   * PlaneProjectionTarget.new(origin?, normal?)  fields Origin, Normal
--   * CircleProjectionTarget.new(circle?)          field Circle (Circle3d)
--   * CylinderProjectionTarget.new(cylinder?)      field Cylinder (Cylinder3d)
--   * SequentialProjectionTarget.new(...)          C# params IProjectionTarget[]; field Targets (Luau array)

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)

local V3 = Vector3d.new
local sqrt = math.sqrt
local Epsilon = MathUtil.Epsilon

local function require_sibling(name: string)
	local child = script.Parent:FindFirstChild(name)
	if child == nil then
		error("BasicProjectionTargets: module " .. name .. " is not available")
	end
	return require(child)
end

-- MathUtil.Normal(v0, v1, v2) on scalars (Triangle3d.Normal)
local function tri_normal(ax, ay, az, bx, by, bz, cx, cy, cz)
	local e1x, e1y, e1z = bx - ax, by - ay, bz - az
	local l1 = sqrt(e1x * e1x + e1y * e1y + e1z * e1z)
	if l1 > Epsilon then
		local s = 1.0 / l1
		e1x, e1y, e1z = e1x * s, e1y * s, e1z * s
	else
		e1x, e1y, e1z = 0, 0, 0
	end
	local e2x, e2y, e2z = cx - bx, cy - by, cz - bz
	local l2 = sqrt(e2x * e2x + e2y * e2y + e2z * e2z)
	if l2 > Epsilon then
		local s = 1.0 / l2
		e2x, e2y, e2z = e2x * s, e2y * s, e2z * s
	else
		e2x, e2y, e2z = 0, 0, 0
	end
	local nx, ny, nz = e1y * e2z - e1z * e2y, e1z * e2x - e1x * e2z, e1x * e2y - e1y * e2x
	local ln = sqrt(nx * nx + ny * ny + nz * nz)
	if ln > Epsilon then
		local s = 1.0 / ln
		return V3(nx * s, ny * s, nz * s)
	end
	return Vector3d.Zero
end

---------------------------------------------------------------------------------------------------
-- MeshProjectionTarget provides an IProjectionTarget interface to a mesh + spatial data structure.
-- Use to project points to mesh surface.

local MeshProjectionTarget = {}
MeshProjectionTarget.__index = MeshProjectionTarget
MeshProjectionTarget.ClassName = "MeshProjectionTarget"
MeshProjectionTarget.IsOrientedProjectionTarget = true

function MeshProjectionTarget.init(self, mesh, spatial)
	self.Mesh = mesh
	self.Spatial = spatial
	if mesh ~= nil and spatial == nil then
		local DMeshAABBTree3 = require_sibling("DMeshAABBTree3")
		self.Spatial = DMeshAABBTree3.new(mesh, true)
	end
	return self
end

function MeshProjectionTarget.new(mesh, spatial)
	return MeshProjectionTarget.init(setmetatable({}, MeshProjectionTarget), mesh, spatial)
end

-- C# Project(Vector3d vPoint, int identifier = -1) and
--    Project(Vector3d vPoint, out Vector3d vProjectNormal, int identifier = -1)
-- returns nearPt, triangle normal
function MeshProjectionTarget:Project(vPoint, identifier: number?)
	local spatial = self.Spatial
	local px, py, pz = vPoint.x, vPoint.y, vPoint.z
	local tNearestID
	if spatial.FindNearestTriangleXYZ ~= nil then
		tNearestID = spatial:FindNearestTriangleXYZ(px, py, pz)
	else
		tNearestID = spatial:FindNearestTriangle(vPoint)
	end
	local ax, ay, az, bx, by, bz, cx, cy, cz = self.Mesh:GetTriVerticesXYZ(tNearestID)
	-- DistPoint3Triangle3.DistanceSqr(ref vPoint, ref triangle, out nearPt, out bary)
	local DistPoint3Triangle3 = require_sibling("DistPoint3Triangle3")
	local _, s, t = DistPoint3Triangle3.DistanceSqrXYZ(px, py, pz, ax, ay, az, bx, by, bz, cx, cy, cz)
	-- nearPt = V0 + s * edge0 + t * edge1
	local e0x, e0y, e0z = bx - ax, by - ay, bz - az
	local e1x, e1y, e1z = cx - ax, cy - ay, cz - az
	local nearPt = V3(ax + s * e0x + t * e1x, ay + s * e0y + t * e1y, az + s * e0z + t * e1z)
	return nearPt, tri_normal(ax, ay, az, bx, by, bz, cx, cy, cz)
end

-- Automatically construct fastest projection target for mesh (or region of mesh)
function MeshProjectionTarget.Auto(mesh, arg2: any?, nExpandRings: number?)
	if arg2 == nil or type(arg2) == "boolean" then
		local bForceCopy = (arg2 ~= false)
		if bForceCopy then
			local DMesh3 = require_sibling("DMesh3")
			return MeshProjectionTarget.new(DMesh3.FromCopyComponents(mesh, false, DMesh3.MeshComponents.None))
		end
		return MeshProjectionTarget.new(mesh)
	end
	-- Auto(DMesh3 mesh, IEnumerable<int> triangles, int nExpandRings = 5)
	local MeshFaceSelection = require_sibling("MeshFaceSelection")
	local DSubmesh3 = require_sibling("DSubmesh3")
	local targetRegion = MeshFaceSelection.new(mesh)
	targetRegion:Select(arg2)
	targetRegion:ExpandToOneRingNeighbours(nExpandRings or 5)
	local submesh = DSubmesh3.new(mesh, targetRegion)
	return MeshProjectionTarget.new(submesh.SubMesh)
end

---------------------------------------------------------------------------------------------------
-- Extension of MeshProjectionTarget that allows the target to have a transformation
-- relative to it's internal space. Call SetTransform(), or initialize the transforms yourself

local TransformedMeshProjectionTarget = setmetatable({}, { __index = MeshProjectionTarget })
TransformedMeshProjectionTarget.__index = TransformedMeshProjectionTarget
TransformedMeshProjectionTarget.ClassName = "TransformedMeshProjectionTarget"

function TransformedMeshProjectionTarget.new(mesh, spatial)
	local self = MeshProjectionTarget.init(setmetatable({}, TransformedMeshProjectionTarget), mesh, spatial)
	self.SourceToTargetXForm = nil
	self.TargetToSourceXForm = nil
	return self
end

function TransformedMeshProjectionTarget:SetTransform(sourceToTargetX)
	self.SourceToTargetXForm = sourceToTargetX
	self.TargetToSourceXForm = sourceToTargetX:MakeInverse()
end

-- returns projected point, normal (C# Project overloads with / without out normal)
function TransformedMeshProjectionTarget:Project(vPoint, identifier: number?)
	local vTargetPt = self.SourceToTargetXForm:TransformP(vPoint)
	local vTargetProj, vTargetProjNormal = MeshProjectionTarget.Project(self, vTargetPt, identifier)
	local vProjectNormal = self.TargetToSourceXForm:TransformV(vTargetProjNormal):Normalized()
	return self.TargetToSourceXForm:TransformP(vTargetProj), vProjectNormal
end

---------------------------------------------------------------------------------------------------

local PlaneProjectionTarget = {}
PlaneProjectionTarget.__index = PlaneProjectionTarget
PlaneProjectionTarget.ClassName = "PlaneProjectionTarget"

function PlaneProjectionTarget.new(origin, normal)
	return setmetatable({ Origin = origin or Vector3d.Zero, Normal = normal or Vector3d.Zero }, PlaneProjectionTarget)
end

function PlaneProjectionTarget:Project(vPoint, identifier: number?)
	local d = vPoint - self.Origin
	return self.Origin + (d - d:Dot(self.Normal) * self.Normal)
end

---------------------------------------------------------------------------------------------------

local CircleProjectionTarget = {}
CircleProjectionTarget.__index = CircleProjectionTarget
CircleProjectionTarget.ClassName = "CircleProjectionTarget"

function CircleProjectionTarget.new(circle)
	return setmetatable({ Circle = circle }, CircleProjectionTarget)
end

function CircleProjectionTarget:Project(vPoint, identifier: number?)
	local DistPoint3Circle3 = require_sibling("DistPoint3Circle3")
	local d = DistPoint3Circle3.new(vPoint, self.Circle)
	d:GetSquared()
	return d.CircleClosest
end

---------------------------------------------------------------------------------------------------

local CylinderProjectionTarget = {}
CylinderProjectionTarget.__index = CylinderProjectionTarget
CylinderProjectionTarget.ClassName = "CylinderProjectionTarget"

function CylinderProjectionTarget.new(cylinder)
	return setmetatable({ Cylinder = cylinder }, CylinderProjectionTarget)
end

function CylinderProjectionTarget:Project(vPoint, identifer: number?)
	local DistPoint3Cylinder3 = require_sibling("DistPoint3Cylinder3")
	local d = DistPoint3Cylinder3.new(vPoint, self.Cylinder)
	d:GetSquared()
	return d.CylinderClosest
end

---------------------------------------------------------------------------------------------------

local SequentialProjectionTarget = {}
SequentialProjectionTarget.__index = SequentialProjectionTarget
SequentialProjectionTarget.ClassName = "SequentialProjectionTarget"

-- SequentialProjectionTarget.new(target1, target2, ...) or SequentialProjectionTarget.new({targets})
function SequentialProjectionTarget.new(...)
	local n = select("#", ...)
	local targets
	if n == 1 and type((...)) == "table" and getmetatable((...)) == nil then
		targets = (...)
	elseif n == 0 then
		targets = nil
	else
		targets = { ... }
	end
	return setmetatable({ Targets = targets }, SequentialProjectionTarget)
end

function SequentialProjectionTarget:Project(vPoint, identifier: number?)
	local vCur = vPoint
	local targets = self.Targets
	for i = 1, #targets do
		vCur = (targets[i]:Project(vCur, identifier))
	end
	return vCur
end

return {
	MeshProjectionTarget = MeshProjectionTarget,
	TransformedMeshProjectionTarget = TransformedMeshProjectionTarget,
	PlaneProjectionTarget = PlaneProjectionTarget,
	CircleProjectionTarget = CircleProjectionTarget,
	CylinderProjectionTarget = CylinderProjectionTarget,
	SequentialProjectionTarget = SequentialProjectionTarget,
}
