-- Port of geometry3Sharp spatial/SpatialInterfaces.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- The C# interfaces are duck-typed in Luau: any object (table with methods) implementing the methods
-- below can be passed where the C# code takes the interface. This module only documents them and
-- provides small helpers to check an object.
--
-- ISpatial   ([TODO] this should be called IMeshSpatial? it is specific to triangles.)
--   obj:SupportsNearestTriangle() -> bool                      (C# property)
--   obj:FindNearestTriangle(p, fMaxDist?) -> tid
--       Find id of triangle nearest to p within distance fMaxDist, or return DMesh3.InvalidID (-1) if not found
--   obj:SupportsTriangleRayIntersection() -> bool              (C# property)
--   obj:FindNearestHitTriangle(ray, fMaxDist?) -> tid
--       Find id of triangle intersected by ray, where intersection point is within distance fMaxDist,
--       or return DMesh3.InvalidID (-1) if not found
--   obj:SupportsPointContainment() -> bool                     (C# property)
--   obj:IsInside(p) -> bool
--       return true if query point is inside mesh
--   Implemented by DMeshAABBTree3.
--
-- IProjectionTarget
--   obj:Project(vPoint, identifier?) -> Vector3d                (identifier defaults to -1)
--
-- IOrientedProjectionTarget : IProjectionTarget
--   obj:Project(vPoint, identifier?) -> Vector3d, vProjectNormal
--       (C# overload Project(Vector3d vPoint, out Vector3d vProjectNormal, int identifier = -1) is the same
--        call: the normal is the second return value). Oriented targets set the field
--        IsOrientedProjectionTarget = true (see BasicProjectionTargets.luau).
--
-- IIntersectionTarget
--   obj:HasNormal() -> bool                                    (C# property)
--   obj:RayIntersect(ray) -> bool, vHit, vHitNormal             (C# out vHit, out vHitNormal)
--   Implemented by DMeshIntersectionTarget, PlaneIntersectionTarget, TransformedIntersectionTarget
--   (BasicIntersectionTargets.luau).

local SpatialInterfaces = {}

local function hasMethods(obj: any, names: { string }): boolean
	if obj == nil then
		return false
	end
	for _, name in names do
		local ok, f = pcall(function()
			return obj[name]
		end)
		if not ok or type(f) ~= "function" then
			return false
		end
	end
	return true
end

SpatialInterfaces.ISpatialMethods = table.freeze({
	"SupportsNearestTriangle",
	"FindNearestTriangle",
	"SupportsTriangleRayIntersection",
	"FindNearestHitTriangle",
	"SupportsPointContainment",
	"IsInside",
})
SpatialInterfaces.IProjectionTargetMethods = table.freeze({ "Project" })
SpatialInterfaces.IOrientedProjectionTargetMethods = table.freeze({ "Project" })
SpatialInterfaces.IIntersectionTargetMethods = table.freeze({ "HasNormal", "RayIntersect" })

function SpatialInterfaces.IsSpatial(obj: any): boolean
	return hasMethods(obj, SpatialInterfaces.ISpatialMethods)
end
function SpatialInterfaces.IsProjectionTarget(obj: any): boolean
	return hasMethods(obj, SpatialInterfaces.IProjectionTargetMethods)
end
function SpatialInterfaces.IsOrientedProjectionTarget(obj: any): boolean
	return hasMethods(obj, SpatialInterfaces.IOrientedProjectionTargetMethods) and obj.IsOrientedProjectionTarget == true
end
function SpatialInterfaces.IsIntersectionTarget(obj: any): boolean
	return hasMethods(obj, SpatialInterfaces.IIntersectionTargetMethods)
end

return SpatialInterfaces
