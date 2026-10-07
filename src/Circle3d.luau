-- Port of geometry3Sharp shapes3/Circle3.cs (Circle3d)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- somewhat ported from WildMagic5
-- The plane containing the circle is Dot(N,X-C) = 0, where X is any point
-- in the plane.  Vectors U, V, and N form an orthonormal right-handed set
-- (matrix [U V N] is orthonormal and has determinant 1).  The circle
-- within the plane is parameterized by X = C + R*(cos(t)*U + sin(t)*V),
-- where t is an angle in [-pi,pi).
--
-- Luau port notes:
--   * fields: Center, Normal, PlaneX, PlaneY, Radius, IsReversed
--   * constructors:
--       Circle3d.new(center, radius, axis0, axis1, normal)
--       Circle3d.FromFrame(frame, radius, nNormalAxis = 1)
--       Circle3d.FromCenter(center, radius)              (normal = +Y, plane axes X/Z)
--   * properties are methods: IsClosed(), ParamLength(), HasArcLength(), ArcLength(),
--     Circumference(), Diameter(), Area()

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)

local Circle3d = {}
Circle3d.__index = Circle3d
Circle3d.ClassName = "Circle3d"

function Circle3d.new(center, radius: number, axis0, axis1, normal)
	return setmetatable({
		IsReversed = false, -- use ccw orientation instead of cw
		Center = center,
		Normal = normal,
		PlaneX = axis0,
		PlaneY = axis1,
		Radius = radius,
	}, Circle3d)
end

function Circle3d.FromFrame(frame, radius: number, nNormalAxis: number?)
	local n = nNormalAxis or 1
	return Circle3d.new(frame.Origin, radius, frame:GetAxis((n + 1) % 3), frame:GetAxis((n + 2) % 3), frame:GetAxis(n))
end

function Circle3d.FromCenter(center, radius: number)
	return Circle3d.new(center, radius, Vector3d.AxisX, Vector3d.AxisZ, Vector3d.AxisY)
end

function Circle3d:IsClosed(): boolean
	return true
end

function Circle3d:Reverse()
	self.IsReversed = not self.IsReversed
end

-- angle in range [0,360] (but works for any value, obviously)
function Circle3d:SampleDeg(degrees: number)
	local theta = degrees * MathUtil.Deg2Rad
	local c, s = math.cos(theta), math.sin(theta)
	return self.Center + c * self.Radius * self.PlaneX + s * self.Radius * self.PlaneY
end

-- angle in range [0,2pi] (but works for any value, obviously)
function Circle3d:SampleRad(radians: number)
	local c, s = math.cos(radians), math.sin(radians)
	return self.Center + c * self.Radius * self.PlaneX + s * self.Radius * self.PlaneY
end

function Circle3d:ParamLength(): number
	return 1.0
end

-- t in range[0,1] spans circle [0,2pi]
function Circle3d:SampleT(t: number)
	local theta = self.IsReversed and -t * MathUtil.TwoPI or t * MathUtil.TwoPI
	local c, s = math.cos(theta), math.sin(theta)
	return self.Center + c * self.Radius * self.PlaneX + s * self.Radius * self.PlaneY
end

function Circle3d:HasArcLength(): boolean
	return true
end

function Circle3d:ArcLength(): number
	return MathUtil.TwoPI * self.Radius
end

function Circle3d:SampleArcLength(a: number)
	local t = a / self:ArcLength()
	local theta = self.IsReversed and -t * MathUtil.TwoPI or t * MathUtil.TwoPI
	local c, s = math.cos(theta), math.sin(theta)
	return self.Center + c * self.Radius * self.PlaneX + s * self.Radius * self.PlaneY
end

function Circle3d:Circumference(): number
	return MathUtil.TwoPI * self.Radius
end
function Circle3d:Diameter(): number
	return 2 * self.Radius
end
function Circle3d:Area(): number
	return math.pi * self.Radius * self.Radius
end

return Circle3d
