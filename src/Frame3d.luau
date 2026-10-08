--!native
-- Port of geometry3Sharp math/Frame3d.cs (Frame3d; Frame3f is not ported, see PORTING.md)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Frame3d is a MUTABLE struct in C# (Translate, Rotate, ...). Use :Clone() where C# copies it.
-- The C# Origin/Rotation properties are trivial wrappers, so they are plain fields here:
--   frame.Origin (Vector3d), frame.Rotation (Quaterniond). X/Y/Z are methods: frame:X().
-- Constructors:
--   Frame3d.new(origin?, rotation?)            new Frame3d(Vector3d origin, Quaterniond orientation)
--   Frame3d.Identity()                         (fresh identity frame)
--   Frame3d.FromZ(origin, setZ)                new Frame3d(Vector3d origin, Vector3d setZ)
--   Frame3d.FromAxis(origin, setAxis, nAxis)   new Frame3d(Vector3d origin, Vector3d setAxis, int nAxis)
--   Frame3d.FromXYZ(origin, x, y, z)           new Frame3d(Vector3d origin, Vector3d x, Vector3d y, Vector3d z)
-- (Box3d overloads of ToFrame/FromFrame are not ported.)

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local Quaterniond = require(script.Parent.Quaterniond)
local Matrix3d = require(script.Parent.Matrix3d)

local Frame3d = {}
Frame3d.__index = Frame3d
Frame3d.ClassName = "Frame3d"

export type Frame3d = typeof(setmetatable({} :: { Origin: Vector3d.Vector3d, Rotation: Quaterniond.Quaterniond }, Frame3d))

local function new(origin: Vector3d.Vector3d?, rotation: Quaterniond.Quaterniond?): Frame3d
	return setmetatable({ Origin = origin or Vector3d.Zero, Rotation = rotation or Quaterniond.Identity }, Frame3d) :: any
end
Frame3d.new = new

function Frame3d.Identity(): Frame3d
	return new(Vector3d.Zero, Quaterniond.Identity)
end

function Frame3d.FromZ(origin: Vector3d.Vector3d, setZ: Vector3d.Vector3d): Frame3d
	return new(origin, Quaterniond.FromTo(Vector3d.AxisZ, setZ))
end

function Frame3d.FromAxis(origin: Vector3d.Vector3d, setAxis: Vector3d.Vector3d, nAxis: number): Frame3d
	local rotation
	if nAxis == 0 then
		rotation = Quaterniond.FromTo(Vector3d.AxisX, setAxis)
	elseif nAxis == 1 then
		rotation = Quaterniond.FromTo(Vector3d.AxisY, setAxis)
	else
		rotation = Quaterniond.FromTo(Vector3d.AxisZ, setAxis)
	end
	return new(origin, rotation)
end

function Frame3d.FromXYZ(origin: Vector3d.Vector3d, x: Vector3d.Vector3d, y: Vector3d.Vector3d, z: Vector3d.Vector3d): Frame3d
	local m = Matrix3d.FromVectors(x, y, z, false)
	return new(origin, m:ToQuaternion())
end

function Frame3d.Clone(self: Frame3d): Frame3d
	return new(self.Origin, self.Rotation)
end

function Frame3d.X(self: Frame3d): Vector3d.Vector3d
	return self.Rotation:AxisX()
end
function Frame3d.Y(self: Frame3d): Vector3d.Vector3d
	return self.Rotation:AxisY()
end
function Frame3d.Z(self: Frame3d): Vector3d.Vector3d
	return self.Rotation:AxisZ()
end

function Frame3d.GetAxis(self: Frame3d, nAxis: number): Vector3d.Vector3d
	if nAxis == 0 then
		return self.Rotation * Vector3d.AxisX
	elseif nAxis == 1 then
		return self.Rotation * Vector3d.AxisY
	elseif nAxis == 2 then
		return self.Rotation * Vector3d.AxisZ
	end
	error("Frame3d.GetAxis: nAxis out of range")
end

function Frame3d.GetXYZ(self: Frame3d): (Vector3d.Vector3d, Vector3d.Vector3d, Vector3d.Vector3d)
	return self.Rotation:GetXYZ()
end

function Frame3d.Translate(self: Frame3d, v: Vector3d.Vector3d)
	self.Origin = self.Origin + v
end

-- Translated(Vector3d v) or Translated(double fDistance, int nAxis)
function Frame3d.Translated(self: Frame3d, v: any, nAxis: number?): Frame3d
	if type(v) == "number" then
		return new(self.Origin + v * self:GetAxis(nAxis :: number), self.Rotation)
	end
	return new(self.Origin + v, self.Rotation)
end

-- Scale(double) or Scale(Vector3d)
function Frame3d.Scale(self: Frame3d, scale: any)
	self.Origin = self.Origin * scale
end

function Frame3d.Scaled(self: Frame3d, scale: any): Frame3d
	return new(scale * self.Origin, self.Rotation)
end

function Frame3d.Rotate(self: Frame3d, q: Quaterniond.Quaterniond)
	self.Rotation = q * self.Rotation
end

-- Rotated(Quaterniond q) or Rotated(double AngleDeg, int nAxis)
function Frame3d.Rotated(self: Frame3d, q: any, nAxis: number?): Frame3d
	if type(q) == "number" then
		return self:Rotated(Quaterniond.AxisAngleD(self:GetAxis(nAxis :: number), q))
	end
	return new(self.Origin, q * self.Rotation)
end

-- this rotates the frame around its own axes, rather than around the world axes,
-- which is what Rotate() does. So, RotateAroundAxis(AxisAngleD(Z,180)) is equivalent
-- to Rotate(AxisAngleD(My_AxisZ,180)).
function Frame3d.RotateAroundAxes(self: Frame3d, q: Quaterniond.Quaterniond)
	self.Rotation = self.Rotation * q
end

function Frame3d.RotateAround(self: Frame3d, point: Vector3d.Vector3d, q: Quaterniond.Quaterniond)
	local dv = q * (self.Origin - point)
	self.Rotation = q * self.Rotation
	self.Origin = point + dv
end

function Frame3d.RotatedAround(self: Frame3d, point: Vector3d.Vector3d, q: Quaterniond.Quaterniond): Frame3d
	local dv = q * (self.Origin - point)
	return new(point + dv, q * self.Rotation)
end

function Frame3d.AlignAxis(self: Frame3d, nAxis: number, vTo: Vector3d.Vector3d)
	local rot = Quaterniond.FromTo(self:GetAxis(nAxis), vTo)
	self:Rotate(rot)
end

function Frame3d.GetAlignAxis(self: Frame3d, nAxis: number, vTo: Vector3d.Vector3d): Frame3d
	local rot = Quaterniond.FromTo(self:GetAxis(nAxis), vTo)
	return self:Rotated(rot)
end

function Frame3d.ConstrainedAlignAxis(self: Frame3d, nAxis: number, vTo: Vector3d.Vector3d, vAround: Vector3d.Vector3d)
	local axis = self:GetAxis(nAxis)
	local fAngle = MathUtil.PlaneAngleSignedD(axis, vTo, vAround)
	local rot = Quaterniond.AxisAngleD(vAround, fAngle)
	self:Rotate(rot)
end

function Frame3d.GetAlignAxisConstrained(self: Frame3d, nAxis: number, vTo: Vector3d.Vector3d, vAround: Vector3d.Vector3d): Frame3d
	local axis = self:GetAxis(nAxis)
	local fAngle = MathUtil.PlaneAngleSignedD(axis, vTo, vAround)
	local rot = Quaterniond.AxisAngleD(vAround, fAngle)
	return self:Rotated(rot)
end

-- 3D projection of point p onto frame-axis plane orthogonal to normal axis
function Frame3d.ProjectToPlane(self: Frame3d, p: Vector3d.Vector3d, nNormal: number): Vector3d.Vector3d
	local d = p - self.Origin
	local n = self:GetAxis(nNormal)
	return self.Origin + (d - d:Dot(n) * n)
end

-- map from 2D coordinates in frame-axes plane perpendicular to normal axis, to 3D
-- [TODO] check that mapping preserves orientation?
function Frame3d.FromPlaneUV(self: Frame3d, v: any, nPlaneNormalAxis: number): Vector3d.Vector3d
	local dv = Vector3d.new(v.x, v.y, 0)
	if nPlaneNormalAxis == 0 then
		dv = Vector3d.new(0, dv.y, v.x)
	elseif nPlaneNormalAxis == 1 then
		dv = Vector3d.new(dv.x, 0, v.y)
	end
	return self.Rotation * dv + self.Origin
end

-- Project p onto plane axes
-- [TODO] check that mapping preserves orientation?
function Frame3d.ToPlaneUV(self: Frame3d, p: Vector3d.Vector3d, nNormal: number)
	local Vector2d = require(script.Parent.Vector2d)
	local nAxis0, nAxis1 = 0, 1
	if nNormal == 0 then
		nAxis0 = 2
	elseif nNormal == 1 then
		nAxis1 = 2
	end
	local d = p - self.Origin
	local fu = d:Dot(self:GetAxis(nAxis0))
	local fv = d:Dot(self:GetAxis(nAxis1))
	return Vector2d.new(fu, fv)
end

function Frame3d.DistanceToPlane(self: Frame3d, p: Vector3d.Vector3d, nNormal: number): number
	return math.abs((p - self.Origin):Dot(self:GetAxis(nNormal)))
end

function Frame3d.DistanceToPlaneSigned(self: Frame3d, p: Vector3d.Vector3d, nNormal: number): number
	return (p - self.Origin):Dot(self:GetAxis(nNormal))
end

-- Map point *into* local coordinates of Frame
function Frame3d.ToFrameP(self: Frame3d, v: Vector3d.Vector3d): Vector3d.Vector3d
	local o = self.Origin
	return self.Rotation:InverseMultiply(Vector3d.new(v.x - o.x, v.y - o.y, v.z - o.z))
end

-- Map point *from* local frame coordinates into "world" coordinates
function Frame3d.FromFrameP(self: Frame3d, v: Vector3d.Vector3d): Vector3d.Vector3d
	return self.Rotation * v + self.Origin
end

-- Map vector *into* local coordinates of Frame
function Frame3d.ToFrameV(self: Frame3d, v: Vector3d.Vector3d): Vector3d.Vector3d
	return self.Rotation:InverseMultiply(v)
end

-- Map vector *from* local frame coordinates into "world" coordinates
function Frame3d.FromFrameV(self: Frame3d, v: Vector3d.Vector3d): Vector3d.Vector3d
	return self.Rotation * v
end

-- ToFrame(Quaterniond), ToFrame(Ray3d), ToFrame(Frame3d)
function Frame3d.ToFrame(self: Frame3d, x: any): any
	local mt = getmetatable(x)
	if mt == Quaterniond then
		return Quaterniond.Inverse(self.Rotation) * x
	elseif mt == Frame3d then
		return new(self:ToFrameP(x.Origin), Quaterniond.Inverse(self.Rotation) * x.Rotation)
	elseif x.Direction ~= nil then
		local Ray3d = require(script.Parent.Ray3d)
		return Ray3d.new(self:ToFrameP(x.Origin), self:ToFrameV(x.Direction))
	end
	error("Frame3d.ToFrame: unsupported argument type")
end

-- FromFrame(Quaterniond), FromFrame(Ray3d), FromFrame(Frame3d)
function Frame3d.FromFrame(self: Frame3d, x: any): any
	local mt = getmetatable(x)
	if mt == Quaterniond then
		return self.Rotation * x
	elseif mt == Frame3d then
		return new(self:FromFrameP(x.Origin), self.Rotation * x.Rotation)
	elseif x.Direction ~= nil then
		local Ray3d = require(script.Parent.Ray3d)
		return Ray3d.new(self:FromFrameP(x.Origin), self:FromFrameV(x.Direction))
	end
	error("Frame3d.FromFrame: unsupported argument type")
end

-- Compute intersection of ray with plane passing through frame origin, normal
-- to the specified axis. If the ray is parallel to the plane, no intersection can be found, and
-- we return Vector3d.Invalid
function Frame3d.RayPlaneIntersection(
	self: Frame3d,
	ray_origin: Vector3d.Vector3d,
	ray_direction: Vector3d.Vector3d,
	nAxisAsNormal: number
): Vector3d.Vector3d
	local N = self:GetAxis(nAxisAsNormal)
	local d = -self.Origin:Dot(N)
	local div = ray_direction:Dot(N)
	if MathUtil.EpsilonEqual(div, 0, MathUtil.ZeroTolerance) then
		return Vector3d.Invalid
	end
	local t = -(ray_origin:Dot(N) + d) / div
	return ray_origin + t * ray_direction
end

-- Interpolate between two frames - Lerp for origin, Slerp for rotation
function Frame3d.Interpolate(f1: Frame3d, f2: Frame3d, t: number): Frame3d
	return new(Vector3d.Lerp(f1.Origin, f2.Origin, t), Quaterniond.Slerp(f1.Rotation, f2.Rotation, t))
end

function Frame3d.EpsilonEqual(self: Frame3d, f2: Frame3d, epsilon: number): boolean
	return self.Origin:EpsilonEqual(f2.Origin, epsilon) and self.Rotation:EpsilonEqual(f2.Rotation, epsilon)
end

Frame3d.__tostring = function(f: Frame3d): string
	local fmt = "%.4f"
	return string.format(
		"[Frame3d: Origin=%s, X=%s, Y=%s, Z=%s]",
		f.Origin:ToString(fmt),
		f:X():ToString(fmt),
		f:Y():ToString(fmt),
		f:Z():ToString(fmt)
	)
end

-- finds minimal rotation that aligns source frame with axes of target frame.
-- considers all signs
--   1) find smallest angle(axis_source, axis_target), considering all sign permutations
--   2) rotate source to align axis_source with sign*axis_target
--   3) now rotate around alined_axis_source to align second-best pair of axes
function Frame3d.SolveMinRotation(source: Frame3d, target: Frame3d): Frame3d
	local best_i, best_j = -1, -1
	local fMaxAbsDot, fMaxSign = 0, 0
	for i = 0, 2 do
		for j = 0, 2 do
			local d = source:GetAxis(i):Dot(target:GetAxis(j))
			local a = math.abs(d)
			if a > fMaxAbsDot then
				fMaxAbsDot = a
				fMaxSign = MathUtil.Sign(d)
				best_i = i
				best_j = j
			end
		end
	end

	local R1 = source:Rotated(Quaterniond.FromTo(source:GetAxis(best_i), fMaxSign * target:GetAxis(best_j)))
	local vAround = R1:GetAxis(best_i)

	local second_i, second_j = -1, -1
	local fSecondDot, fSecondSign = 0, 0
	for i = 0, 2 do
		if i ~= best_i then
			for j = 0, 2 do
				if j ~= best_j then
					local d = R1:GetAxis(i):Dot(target:GetAxis(j))
					local a = math.abs(d)
					if a > fSecondDot then
						fSecondDot = a
						fSecondSign = MathUtil.Sign(d)
						second_i = i
						second_j = j
					end
				end
			end
		end
	end

	R1:ConstrainedAlignAxis(second_i, fSecondSign * target:GetAxis(second_j), vAround)
	return R1
end

return Frame3d
