-- Port of geometry3Sharp mesh_ops/SimpleHoleFiller.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Fill a boundary loop with a triangle fan around a new centroid vertex (or a single triangle for a 3-loop).
--
-- Luau port notes:
--   * SimpleHoleFiller.new(mesh, loop) where loop is an EdgeLoop (only loop.Vertices is used).
--   * Outputs: NewVertex (InvalidID if none), NewTriangles (1-based array of tids, or nil).
--   * Fill(group_id = -1) -> bool; Validate() -> ValidationStatus value.

local Vector3d = require(script.Parent.Vector3d)
local MeshValidation = require(script.Parent.MeshValidation)
local MeshEditor = require(script.Parent.MeshEditor)

local InvalidID = -1

local SimpleHoleFiller = {}
SimpleHoleFiller.__index = SimpleHoleFiller
SimpleHoleFiller.ClassName = "SimpleHoleFiller"

function SimpleHoleFiller.new(mesh, loop)
	local self = setmetatable({}, SimpleHoleFiller)
	self.Mesh = mesh
	self.Loop = loop
	self.NewVertex = InvalidID
	self.NewTriangles = nil
	return self
end

function SimpleHoleFiller:Validate(): number
	local loopStatus = MeshValidation.IsBoundaryLoop(self.Mesh, self.Loop)
	return loopStatus
end

function SimpleHoleFiller:Fill(group_id: number?): boolean
	local gid = group_id or -1
	local Mesh = self.Mesh
	local Vertices = self.Loop.Vertices
	local N = #Vertices
	if N < 3 then
		return false
	end

	-- this just needs one triangle
	if N == 3 then
		local new_tid = Mesh:AppendTriangle(Vertices[1], Vertices[3], Vertices[2], gid)
		if new_tid < 0 then
			return false
		end
		self.NewTriangles = { new_tid }
		self.NewVertex = InvalidID
		return true
	end

	-- [TODO] 4-case? could check nbr normals to figure out best internal edge...

	-- compute centroid
	local c = Vector3d.Zero
	for i = 1, N do
		c = c + Mesh:GetVertex(Vertices[i])
	end
	c = c * (1.0 / N)

	-- add centroid vtx
	self.NewVertex = Mesh:AppendVertex(c)

	-- stitch triangles
	local editor = MeshEditor.new(Mesh)
	local ok, result = pcall(function()
		return editor:AddTriangleFan_OrderedVertexLoop(self.NewVertex, Vertices, gid)
	end)
	self.NewTriangles = ok and result or nil

	-- if fill failed, back out vertex-add
	if self.NewTriangles == nil then
		Mesh:RemoveVertex(self.NewVertex, true, false)
		self.NewVertex = InvalidID
		return false
	else
		return true
	end
end

return SimpleHoleFiller
