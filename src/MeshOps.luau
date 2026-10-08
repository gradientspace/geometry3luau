-- Port of geometry3Sharp mesh_ops/MeshOps.cs (SetGroupBehavior; the C# static class MeshOps is empty)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- SetGroupBehavior: policy for the face-group id assigned to newly created triangles.
--
-- Luau port notes (see PORTING.md):
--   * SetGroupBehavior is immutable (its C# fields are private), so the C# static properties
--     SetGroupBehavior.Ignore / .AutoGenerate are frozen values, not functions:
--       SetGroupBehavior.Ignore                  C# SetGroupBehavior.Ignore
--       SetGroupBehavior.AutoGenerate            C# SetGroupBehavior.AutoGenerate
--       SetGroupBehavior.SetTo(groupID)          C# SetGroupBehavior.SetTo(groupID)
--       SetGroupBehavior.new(mode, id?)          new SetGroupBehavior(Modes mode, int id = 0)
--   * Modes is an enum table: SetGroupBehavior.Modes.Ignore == 0, AutoGenerate == 1, UseConstant == 2.
--   * The module table is the (empty) C# MeshOps class; SetGroupBehavior is MeshOps.SetGroupBehavior.

local MeshOps = {}

local SetGroupBehavior = {}
SetGroupBehavior.__index = SetGroupBehavior
SetGroupBehavior.ClassName = "SetGroupBehavior"

local Modes = table.freeze({
	Ignore = 0,
	AutoGenerate = 1,
	UseConstant = 2,
})
SetGroupBehavior.Modes = Modes

function SetGroupBehavior.new(mode: number, id: number?)
	return setmetatable({ Mode = mode, SetGroupID = id or 0 }, SetGroupBehavior)
end

function SetGroupBehavior:GetGroupID(mesh): number
	if self.Mode == Modes.Ignore then
		return -1
	elseif self.Mode == Modes.AutoGenerate then
		return mesh:AllocateTriangleGroup()
	else
		return self.SetGroupID
	end
end

SetGroupBehavior.Ignore = table.freeze(SetGroupBehavior.new(Modes.Ignore, 0))
SetGroupBehavior.AutoGenerate = table.freeze(SetGroupBehavior.new(Modes.AutoGenerate, 0))

function SetGroupBehavior.SetTo(groupID: number)
	return SetGroupBehavior.new(Modes.UseConstant, groupID)
end

MeshOps.SetGroupBehavior = SetGroupBehavior

return MeshOps
