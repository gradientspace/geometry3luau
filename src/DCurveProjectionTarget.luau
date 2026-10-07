-- Port of geometry3Sharp spatial/DCurveProjection.cs (class DCurveProjectionTarget : IProjectionTarget)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- IProjectionTarget (duck typed): target:Project(vPoint, identifier?) -> Vector3d
-- Luau: DCurveProjectionTarget.new(curve) where curve is a DCurve3.

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)

local V3 = Vector3d.new

local DCurveProjectionTarget = {}
DCurveProjectionTarget.__index = DCurveProjectionTarget
DCurveProjectionTarget.ClassName = "DCurveProjectionTarget"

function DCurveProjectionTarget.new(curve)
	return setmetatable({ Curve = curve }, DCurveProjectionTarget)
end

function DCurveProjectionTarget:Project(vPoint, identifier: number?)
	local nx, ny, nz = 0, 0, 0
	local fNearestSqr = MathUtil.MaxDouble
	local px, py, pz = vPoint.x, vPoint.y, vPoint.z

	local verts = self.Curve.vertices
	local N = #verts
	local NStop = self.Curve.Closed and N or N - 1
	for i = 0, NStop - 1 do
		-- Segment3d seg = new Segment3d(Curve[i], Curve[(i + 1) % N]); pt = seg.NearestPoint(vPoint)
		local p0, p1 = verts[i + 1], verts[((i + 1) % N) + 1]
		local cx, cy, cz = 0.5 * (p0.x + p1.x), 0.5 * (p0.y + p1.y), 0.5 * (p0.z + p1.z)
		local dx, dy, dz = p1.x - p0.x, p1.y - p0.y, p1.z - p0.z
		local len = math.sqrt(dx * dx + dy * dy + dz * dz)
		if len > MathUtil.Epsilon then
			local s = 1.0 / len
			dx, dy, dz = dx * s, dy * s, dz * s
		else
			dx, dy, dz, len = 0, 0, 0, 0
		end
		local extent = 0.5 * len
		local t = (px - cx) * dx + (py - cy) * dy + (pz - cz) * dz
		local qx, qy, qz
		if t >= extent then
			qx, qy, qz = cx + extent * dx, cy + extent * dy, cz + extent * dz
		elseif t <= -extent then
			qx, qy, qz = cx - extent * dx, cy - extent * dy, cz - extent * dz
		else
			qx, qy, qz = cx + t * dx, cy + t * dy, cz + t * dz
		end
		local ex, ey, ez = px - qx, py - qy, pz - qz
		local dsqr = ex * ex + ey * ey + ez * ez
		if dsqr < fNearestSqr then
			fNearestSqr = dsqr
			nx, ny, nz = qx, qy, qz
		end
	end

	if fNearestSqr < MathUtil.MaxDouble then
		return V3(nx, ny, nz)
	end
	return vPoint
end

return DCurveProjectionTarget
