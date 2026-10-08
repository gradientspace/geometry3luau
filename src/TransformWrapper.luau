-- Port of geometry3Sharp mesh/MeshTransforms.cs (struct TransformWrapper)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- TransformWrapper is a struct-of-Func<>'s that can be used to pass around arbitrary,
-- optionally-defined position/normal transforms.
--
-- Use TransformWrapper.new() for no-transform (C# `TransformWrapper wrapper = default`).
-- Use TransformP() / TransformN() to handle all cases.
--
-- Fields: TransformPosition, TransformNormal: ((Vector3d) -> Vector3d)? (nil = identity).
-- Constructors:
--   TransformWrapper.new(transformPosition?, transformNormal?)   new TransformWrapper() + field assignments
--   TransformWrapper.MakeFromAffine(transform)                   static MakeFromAffine(Matrix4d Transform)
-- TransformWrapper is also available as MeshTransforms.TransformWrapper.

local Vector3d = require(script.Parent.Vector3d)

local TransformWrapper = {}
TransformWrapper.__index = TransformWrapper
TransformWrapper.ClassName = "TransformWrapper"

export type TransformWrapper = typeof(setmetatable(
	{} :: {
		TransformPosition: ((Vector3d.Vector3d) -> Vector3d.Vector3d)?,
		TransformNormal: ((Vector3d.Vector3d) -> Vector3d.Vector3d)?,
	},
	TransformWrapper
))

function TransformWrapper.new(
	transformPosition: ((Vector3d.Vector3d) -> Vector3d.Vector3d)?,
	transformNormal: ((Vector3d.Vector3d) -> Vector3d.Vector3d)?
): TransformWrapper
	return setmetatable({ TransformPosition = transformPosition, TransformNormal = transformNormal }, TransformWrapper) :: any
end

function TransformWrapper.Clone(self: TransformWrapper): TransformWrapper
	return TransformWrapper.new(self.TransformPosition, self.TransformNormal)
end

function TransformWrapper.TransformP(self: TransformWrapper, p: Vector3d.Vector3d): Vector3d.Vector3d
	local f = self.TransformPosition
	return (f ~= nil) and f(p) or p
end

function TransformWrapper.TransformN(self: TransformWrapper, n: Vector3d.Vector3d): Vector3d.Vector3d
	local f = self.TransformNormal
	return (f ~= nil) and f(n) or n
end

-- Transform is a Matrix4d
function TransformWrapper.MakeFromAffine(Transform: any): TransformWrapper
	local ok, NormalTransform = Transform:GetAffineNormalTransform()
	if ok == false then
		NormalTransform = Transform:GetAffineTransform()
	end
	return TransformWrapper.new(function(p)
		return Transform:TransformPointAffine(p)
	end, function(n)
		return NormalTransform:Multiply(n)
	end)
end

return TransformWrapper
