-- Port of geometry3Sharp shapes3/Cylinder3.cs (Cylinder3d)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- ported from GTEngine (WildMagic5 doesn't have cylinder primitive)
-- The cylinder axis is a line.  The origin of the cylinder is chosen to be
-- the line origin.  The cylinder wall is at a distance R units from the axis.
-- An infinite cylinder has infinite height.  A finite cylinder has center C
-- at the line origin and has a finite height H.  The segment for the finite
-- cylinder has endpoints C-(H/2)*D and C+(H/2)*D where D is a unit-length
-- direction of the line.
--
-- Luau port notes:
--   * fields: Axis (Line3d), Radius, Height  (use MathUtil.MaxDouble for an infinite cylinder)
--   * constructors:
--       Cylinder3d.new(axis, radius, height)
--       Cylinder3d.FromCenterAxis(center, axis, radius, height)
--       Cylinder3d.FromFrame(frame, radius, height, nNormalAxis = 1)
--       Cylinder3d.FromRadiusHeight(radius, height)      (axis = +Y through origin)
--   * properties are methods: Circumference(), Diameter(), Volume()

local Vector3d = require(script.Parent.Vector3d)
local Line3d = require(script.Parent.Line3d)
local MathUtil = require(script.Parent.MathUtil)

local Cylinder3d = {}
Cylinder3d.__index = Cylinder3d
Cylinder3d.ClassName = "Cylinder3d"

function Cylinder3d.new(axis, radius: number, height: number)
	return setmetatable({ Axis = axis, Radius = radius, Height = height }, Cylinder3d)
end

function Cylinder3d.FromCenterAxis(center, axis, radius: number, height: number)
	return Cylinder3d.new(Line3d.new(center, axis), radius, height)
end

function Cylinder3d.FromFrame(frame, radius: number, height: number, nNormalAxis: number?)
	return Cylinder3d.new(Line3d.new(frame.Origin, frame:GetAxis(nNormalAxis or 1)), radius, height)
end

function Cylinder3d.FromRadiusHeight(radius: number, height: number)
	return Cylinder3d.new(Line3d.new(Vector3d.Zero, Vector3d.AxisY), radius, height)
end

function Cylinder3d:Circumference(): number
	return MathUtil.TwoPI * self.Radius
end
function Cylinder3d:Diameter(): number
	return 2 * self.Radius
end
function Cylinder3d:Volume(): number
	return math.pi * self.Radius * self.Radius * self.Height
end

return Cylinder3d
