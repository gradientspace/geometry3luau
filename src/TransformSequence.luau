-- Port of geometry3Sharp math/TransformSequence.cs (dependency of TransformedMeshProjectionTarget; package B)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- TransformSequence stores an ordered list of basic transformations.
-- This can be useful if you need to construct some modifications and want
-- to use the same set later. For example, if you have a hierarchy of objects
-- with relative transformations and want to "save" the nested transform sequence
-- without having to hold references to the original objects.
--
-- Use the Append() functions to add different transform types, and the TransformX()
-- to apply the sequence
--
-- Luau port notes:
--   * TransformSequence.new(), TransformSequence.FromCopy(copy)
--   * overloads: AppendTranslation(dv) or AppendTranslation(dx, dy, dz);
--     AppendRotation(q, aroundPt?); AppendScale(s, aroundPt?)
--   * Store/Restore (BinaryWriter/BinaryReader serialization) are not ported.

local Vector3d = require(script.Parent.Vector3d)
local Quaterniond = require(script.Parent.Quaterniond)
local Frame3d = require(script.Parent.Frame3d)

local TransformSequence = {}
TransformSequence.__index = TransformSequence
TransformSequence.ClassName = "TransformSequence"

local XFormType = table.freeze({
	Translation = 0,
	QuaterionRotation = 1,
	QuaternionRotateAroundPoint = 2,
	Scale = 3,
	ScaleAroundPoint = 4,
	ToFrame = 5,
	FromFrame = 6,
})
TransformSequence.XFormType = XFormType

-- XForm: { type, V0, V1, V2 }  (C# Vector3dTuple3 data)
local function xf_quat(op)
	return Quaterniond.new(op.V0.x, op.V0.y, op.V0.z, op.V1.x)
end
local function xf_frame(op)
	return Frame3d.new(op.V2, xf_quat(op))
end

function TransformSequence.new()
	return setmetatable({ Operations = {} }, TransformSequence)
end

function TransformSequence.FromCopy(copy)
	return setmetatable({ Operations = table.clone(copy.Operations) }, TransformSequence)
end

local function add(self, t: number, v0, v1, v2)
	table.insert(self.Operations, table.freeze({ type = t, V0 = v0, V1 = v1, V2 = v2 }))
end

function TransformSequence:Append(sequence)
	for _, op in sequence.Operations do
		table.insert(self.Operations, op)
	end
end

function TransformSequence:AppendTranslation(dv: any, dy: number?, dz: number?)
	if type(dv) == "number" then
		dv = Vector3d.new(dv, dy :: number, dz :: number)
	end
	add(self, XFormType.Translation, dv, Vector3d.Zero, Vector3d.Zero)
end

function TransformSequence:AppendRotation(q, aroundPt)
	if aroundPt == nil then
		add(self, XFormType.QuaterionRotation, Vector3d.new(q.x, q.y, q.z), Vector3d.new(q.w, 0, 0), Vector3d.Zero)
	else
		add(self, XFormType.QuaternionRotateAroundPoint, Vector3d.new(q.x, q.y, q.z), Vector3d.new(q.w, 0, 0), aroundPt)
	end
end

function TransformSequence:AppendScale(s, aroundPt)
	if aroundPt == nil then
		add(self, XFormType.Scale, s, Vector3d.Zero, Vector3d.Zero)
	else
		add(self, XFormType.ScaleAroundPoint, s, Vector3d.Zero, aroundPt)
	end
end

function TransformSequence:AppendToFrame(frame)
	local q = frame.Rotation
	add(self, XFormType.ToFrame, Vector3d.new(q.x, q.y, q.z), Vector3d.new(q.w, 0, 0), frame.Origin)
end

function TransformSequence:AppendFromFrame(frame)
	local q = frame.Rotation
	add(self, XFormType.FromFrame, Vector3d.new(q.x, q.y, q.z), Vector3d.new(q.w, 0, 0), frame.Origin)
end

-- Apply transforms to point
function TransformSequence:TransformP(p)
	for _, op in self.Operations do
		local t = op.type
		if t == XFormType.Translation then
			p = p + op.V0
		elseif t == XFormType.QuaterionRotation then
			p = xf_quat(op) * p
		elseif t == XFormType.QuaternionRotateAroundPoint then
			p = p - op.V2
			p = xf_quat(op) * p
			p = p + op.V2
		elseif t == XFormType.Scale then
			p = p * op.V0
		elseif t == XFormType.ScaleAroundPoint then
			p = p - op.V2
			p = p * op.V0
			p = p + op.V2
		elseif t == XFormType.ToFrame then
			p = xf_frame(op):ToFrameP(p)
		elseif t == XFormType.FromFrame then
			p = xf_frame(op):FromFrameP(p)
		else
			error("TransformSequence.TransformP: unhandled type!")
		end
	end
	return p
end

-- Apply transforms to vector. Includes scaling.
function TransformSequence:TransformV(v)
	for _, op in self.Operations do
		local t = op.type
		if t == XFormType.Translation then
			-- nothing
		elseif t == XFormType.QuaternionRotateAroundPoint or t == XFormType.QuaterionRotation then
			v = xf_quat(op) * v
		elseif t == XFormType.ScaleAroundPoint or t == XFormType.Scale then
			v = v * op.V0
		elseif t == XFormType.ToFrame then
			v = xf_frame(op):ToFrameV(v)
		elseif t == XFormType.FromFrame then
			v = xf_frame(op):FromFrameV(v)
		else
			error("TransformSequence.TransformV: unhandled type!")
		end
	end
	return v
end

-- construct inverse transformation sequence
function TransformSequence:MakeInverse()
	local reverse = TransformSequence.new()
	local ops = self.Operations
	for i = #ops, 1, -1 do
		local op = ops[i]
		local t = op.type
		if t == XFormType.Translation then
			reverse:AppendTranslation(-op.V0)
		elseif t == XFormType.QuaterionRotation then
			reverse:AppendRotation(xf_quat(op):Inverse())
		elseif t == XFormType.QuaternionRotateAroundPoint then
			reverse:AppendRotation(xf_quat(op):Inverse(), op.V2)
		elseif t == XFormType.Scale then
			reverse:AppendScale(1.0 / op.V0)
		elseif t == XFormType.ScaleAroundPoint then
			reverse:AppendScale(1.0 / op.V0, op.V2)
		elseif t == XFormType.ToFrame then
			reverse:AppendFromFrame(xf_frame(op))
		elseif t == XFormType.FromFrame then
			reverse:AppendToFrame(xf_frame(op))
		else
			error("TransformSequence.MakeInverse: unhandled type!")
		end
	end
	return reverse
end

return TransformSequence
