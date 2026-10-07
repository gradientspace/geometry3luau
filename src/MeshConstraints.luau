-- Port of geometry3Sharp mesh/MeshConstraints.cs (EdgeRefineFlags, EdgeConstraint, VertexConstraint, MeshConstraints)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau port notes (see PORTING.md):
--   * This module returns the MeshConstraints class; the related types are fields of it and are also
--     available as their own modules (G3.EdgeConstraint, G3.VertexConstraint, G3.EdgeRefineFlags):
--       MeshConstraints.EdgeRefineFlags, MeshConstraints.EdgeConstraint, MeshConstraints.VertexConstraint
--   * EdgeConstraint / VertexConstraint are C# structs. Here they are small tables that are treated as
--     IMMUTABLE once stored in a MeshConstraints (the same constraint object may be shared by many
--     edges/vertices, and the static constants are frozen). To modify one, :Clone() it first, eg
--       local ec = EdgeConstraint.Unconstrained:Clone(); ec.TrackingSetID = setID
--   * EdgeConstraint.new(rflags, target?)            new EdgeConstraint(rflags[, target])
--     EdgeConstraint fields: refineFlags, Target, TrackingSetID. Properties are methods:
--       CanFlip(), CanSplit(), CanCollapse(), NoModifications(), IsUnconstrained()
--   * VertexConstraint.new(isFixed, setID?)          new VertexConstraint(bool isFixed, int setID = InvalidSetID)
--     VertexConstraint.FromTarget(target)            new VertexConstraint(IProjectionTarget target)
--     VertexConstraint fields: Fixed, FixedSetID, Target
--   * MeshConstraints:GetVertexConstraint(vid) -> VertexConstraint (Unconstrained if none);
--     C# bool GetVertexConstraint(vid, ref vc) is TryGetVertexConstraint(vid) -> found, vc
--   * VertexConstraintsItr() is used as `for vid, vc in cons:VertexConstraintsItr() do` (C# KeyValuePair)
--   * FindConstrainedEdgesBySetID returns edge ids sorted ascending (C# returns Dictionary order).
--   * HasConstraints() is a method.
--   * Projection targets (IProjectionTarget) are duck typed: target:Project(vPoint, identifier?) -> Vector3d.
--     Targets are compared by identity (==), like C# reference equality.

local EdgeRefineFlags = table.freeze({
	NoConstraint = 0,
	NoFlip = 1,
	NoSplit = 2,
	NoCollapse = 4,
	FullyConstrained = 7, -- NoFlip | NoSplit | NoCollapse
	PreserveTopology = 8, -- this flag just means we want to avoid 'losing' this edge
	-- when we collapse a neighbour (but allow the edge itself to be collapsed).
	-- Eg when constraining edges for Reduce
})

local band = bit32.band

---------------------------------------------------------------------------------------------------
-- EdgeConstraint

local EdgeConstraint = {}
EdgeConstraint.__index = EdgeConstraint
EdgeConstraint.ClassName = "EdgeConstraint"

function EdgeConstraint.new(rflags: number, target: any?)
	return setmetatable({ refineFlags = rflags, Target = target, TrackingSetID = -1 }, EdgeConstraint)
end

function EdgeConstraint:Clone()
	return setmetatable({ refineFlags = self.refineFlags, Target = self.Target, TrackingSetID = self.TrackingSetID }, EdgeConstraint)
end

function EdgeConstraint:CanFlip(): boolean
	return band(self.refineFlags, 1) == 0
end
function EdgeConstraint:CanSplit(): boolean
	return band(self.refineFlags, 2) == 0
end
function EdgeConstraint:CanCollapse(): boolean
	return band(self.refineFlags, 4) == 0
end
function EdgeConstraint:NoModifications(): boolean
	return band(self.refineFlags, 7) == 7
end
function EdgeConstraint:IsUnconstrained(): boolean
	return self.refineFlags == 0 and self.Target == nil
end

EdgeConstraint.Unconstrained = table.freeze(EdgeConstraint.new(EdgeRefineFlags.NoConstraint))
EdgeConstraint.NoFlips = table.freeze(EdgeConstraint.new(EdgeRefineFlags.NoFlip))
EdgeConstraint.FullyConstrained = table.freeze(EdgeConstraint.new(EdgeRefineFlags.FullyConstrained))

---------------------------------------------------------------------------------------------------
-- VertexConstraint

local VertexConstraint = {}
VertexConstraint.__index = VertexConstraint
VertexConstraint.ClassName = "VertexConstraint"

-- clients should interpret negative values as invalid
-- (in case you wanted to use negative values for something else...)
VertexConstraint.InvalidSetID = -1

function VertexConstraint.new(isFixed: boolean, setID: number?)
	return setmetatable({ Fixed = isFixed, FixedSetID = setID or -1, Target = nil }, VertexConstraint)
end

function VertexConstraint.FromTarget(target: any)
	return setmetatable({ Fixed = false, FixedSetID = -1, Target = target }, VertexConstraint)
end

function VertexConstraint:Clone()
	return setmetatable({ Fixed = self.Fixed, FixedSetID = self.FixedSetID, Target = self.Target }, VertexConstraint)
end

VertexConstraint.Unconstrained = table.freeze(VertexConstraint.new(false, -1))
VertexConstraint.Pinned = table.freeze(VertexConstraint.new(true, -1))

---------------------------------------------------------------------------------------------------
-- MeshConstraints

local MeshConstraints = {}
MeshConstraints.__index = MeshConstraints
MeshConstraints.ClassName = "MeshConstraints"

MeshConstraints.EdgeRefineFlags = EdgeRefineFlags
MeshConstraints.EdgeConstraint = EdgeConstraint
MeshConstraints.VertexConstraint = VertexConstraint

local EdgeUnconstrained = EdgeConstraint.Unconstrained
local VertexUnconstrained = VertexConstraint.Unconstrained

function MeshConstraints.new()
	return setmetatable({
		Edges = {}, -- [eid] = EdgeConstraint
		Vertices = {}, -- [vid] = VertexConstraint
		nEdges = 0,
		nVertices = 0,
		set_id_counter = 0, -- use this to allocate FixedSetIDs
	}, MeshConstraints)
end

function MeshConstraints.IsMeshConstraints(o: any): boolean
	return type(o) == "table" and getmetatable(o) == MeshConstraints
end

function MeshConstraints:AllocateSetID(): number
	local id = self.set_id_counter
	self.set_id_counter += 1
	return id
end

function MeshConstraints:HasEdgeConstraint(eid: number): boolean
	return self.Edges[eid] ~= nil
end

function MeshConstraints:GetEdgeConstraint(eid: number)
	return self.Edges[eid] or EdgeUnconstrained
end

function MeshConstraints:SetOrUpdateEdgeConstraint(eid: number, ec)
	local E = self.Edges
	if E[eid] == nil then
		self.nEdges += 1
	end
	E[eid] = ec
end

function MeshConstraints:ClearEdgeConstraint(eid: number)
	local E = self.Edges
	if E[eid] ~= nil then
		self.nEdges -= 1
		E[eid] = nil
	end
end

function MeshConstraints:FindConstrainedEdgesBySetID(setID: number): { number }
	local result = {}
	for eid, ec in self.Edges do
		if ec.TrackingSetID == setID then
			table.insert(result, eid)
		end
	end
	table.sort(result)
	return result
end

function MeshConstraints:HasVertexConstraint(vid: number): boolean
	return self.Vertices[vid] ~= nil
end

function MeshConstraints:GetVertexConstraint(vid: number)
	return self.Vertices[vid] or VertexUnconstrained
end

-- C# bool GetVertexConstraint(int vid, ref VertexConstraint vc)
function MeshConstraints:TryGetVertexConstraint(vid: number)
	local vc = self.Vertices[vid]
	if vc ~= nil then
		return true, vc
	end
	return false, nil
end

function MeshConstraints:SetOrUpdateVertexConstraint(vid: number, vc)
	local V = self.Vertices
	if V[vid] == nil then
		self.nVertices += 1
	end
	V[vid] = vc
end

function MeshConstraints:ClearVertexConstraint(vid: number)
	local V = self.Vertices
	if V[vid] ~= nil then
		self.nVertices -= 1
		V[vid] = nil
	end
end

-- `for vid, vc in cons:VertexConstraintsItr() do`
function MeshConstraints:VertexConstraintsItr()
	return next, self.Vertices, nil
end

function MeshConstraints:HasConstraints(): boolean
	return self.nEdges > 0 or self.nVertices > 0
end

return MeshConstraints
