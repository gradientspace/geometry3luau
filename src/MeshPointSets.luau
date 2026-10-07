-- Port of geometry3Sharp mesh/MeshPointSets.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- IPointSet adapters that present mesh edge midpoints as point sets:
--   MeshPointSets.MeshEdgeMidpoints.new(mesh)          all edge midpoints
--   MeshPointSets.MeshBoundaryEdgeMidpoints.new(mesh)  boundary-edge midpoints
--
-- IPointSet (duck-typed) methods: VertexCount(), MaxVertexID(), HasVertexNormals(), HasVertexColors(),
-- GetVertex(i), GetVertexNormal(i), GetVertexColor(i), IsVertex(i), VertexIndices(), Timestamp().

local Vector3d = require(script.Parent.Vector3d)

local MeshPointSets = {}

---------------------------------------------------------------------------------------------------
-- Present mesh edge midpoints as a point set
local MeshEdgeMidpoints = {}
MeshEdgeMidpoints.__index = MeshEdgeMidpoints
MeshEdgeMidpoints.ClassName = "MeshEdgeMidpoints"
MeshPointSets.MeshEdgeMidpoints = MeshEdgeMidpoints

function MeshEdgeMidpoints.new(mesh)
	return setmetatable({ Mesh = mesh }, MeshEdgeMidpoints)
end
function MeshEdgeMidpoints:VertexCount(): number
	return self.Mesh:EdgeCount()
end
function MeshEdgeMidpoints:MaxVertexID(): number
	return self.Mesh:MaxEdgeID()
end
function MeshEdgeMidpoints:HasVertexNormals(): boolean
	return false
end
function MeshEdgeMidpoints:HasVertexColors(): boolean
	return false
end
function MeshEdgeMidpoints:GetVertex(i: number)
	return self.Mesh:GetEdgePoint(i, 0.5)
end
function MeshEdgeMidpoints:GetVertexNormal(i: number)
	return Vector3d.AxisY
end
function MeshEdgeMidpoints:GetVertexColor(i: number)
	return Vector3d.One
end
function MeshEdgeMidpoints:IsVertex(vID: number): boolean
	return self.Mesh:IsEdge(vID)
end
-- iterators allow us to work with gaps in index space
function MeshEdgeMidpoints:VertexIndices(): () -> number?
	return self.Mesh:EdgeIndices()
end
-- Timestamp is incremented any time any change is made to the mesh
function MeshEdgeMidpoints:Timestamp(): number
	return self.Mesh:Timestamp()
end

---------------------------------------------------------------------------------------------------
-- Present mesh boundary-edge midpoints as a point set
local MeshBoundaryEdgeMidpoints = {}
MeshBoundaryEdgeMidpoints.__index = MeshBoundaryEdgeMidpoints
MeshBoundaryEdgeMidpoints.ClassName = "MeshBoundaryEdgeMidpoints"
MeshPointSets.MeshBoundaryEdgeMidpoints = MeshBoundaryEdgeMidpoints

function MeshBoundaryEdgeMidpoints.new(mesh)
	local self = setmetatable({ Mesh = mesh }, MeshBoundaryEdgeMidpoints)
	-- [RMS] we could count boundary edges really fast by iterating by +4's in
	--  the mesh edge buffer... (also have to check that edge is valid though!)
	local n = 0
	for _ in mesh:BoundaryEdgeIndices() do
		n += 1
	end
	self.num_boundary_edges = n
	return self
end
function MeshBoundaryEdgeMidpoints:VertexCount(): number
	return self.num_boundary_edges
end
function MeshBoundaryEdgeMidpoints:MaxVertexID(): number
	return self.Mesh:MaxEdgeID()
end
function MeshBoundaryEdgeMidpoints:HasVertexNormals(): boolean
	return false
end
function MeshBoundaryEdgeMidpoints:HasVertexColors(): boolean
	return false
end
function MeshBoundaryEdgeMidpoints:GetVertex(i: number)
	return self.Mesh:GetEdgePoint(i, 0.5)
end
function MeshBoundaryEdgeMidpoints:GetVertexNormal(i: number)
	return Vector3d.AxisY
end
function MeshBoundaryEdgeMidpoints:GetVertexColor(i: number)
	return Vector3d.One
end
function MeshBoundaryEdgeMidpoints:IsVertex(vID: number): boolean
	return self.Mesh:IsEdge(vID) and self.Mesh:IsBoundaryEdge(vID)
end
-- iterators allow us to work with gaps in index space
function MeshBoundaryEdgeMidpoints:VertexIndices(): () -> number?
	return self.Mesh:BoundaryEdgeIndices()
end
-- Timestamp is incremented any time any change is made to the mesh
function MeshBoundaryEdgeMidpoints:Timestamp(): number
	return self.Mesh:Timestamp()
end

return MeshPointSets
