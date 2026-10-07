-- Port of geometry3Sharp queries/MeshValidation.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Static validation helpers returning ValidationStatus values (see ValidationStatus.luau).
-- EdgeLoop is duck-typed: only loop.Vertices (1-based array of vertex ids) is used.

local ValidationStatus = require(script.Parent.ValidationStatus)

local InvalidID = -1

local MeshValidation = {}
MeshValidation.ValidationStatus = ValidationStatus

function MeshValidation.IsEdgeLoop(mesh, loop): number
	local Vertices = loop.Vertices
	local N = #Vertices
	for i = 1, N do
		if not mesh:IsVertex(Vertices[i]) then
			return ValidationStatus.NotAVertex
		end
	end
	for i = 0, N - 1 do
		local a = Vertices[i + 1]
		local b = Vertices[((i + 1) % N) + 1]

		local eid = mesh:FindEdge(a, b)
		if eid == InvalidID then
			return ValidationStatus.VerticesNotConnectedByEdge
		end
	end
	return ValidationStatus.Ok
end

function MeshValidation.IsBoundaryLoop(mesh, loop): number
	local Vertices = loop.Vertices
	local N = #Vertices

	for i = 1, N do
		if not mesh:IsBoundaryVertex(Vertices[i]) then
			return ValidationStatus.NotBoundaryVertex
		end
	end

	for i = 0, N - 1 do
		local a = Vertices[i + 1]
		local b = Vertices[((i + 1) % N) + 1]

		local eid = mesh:FindEdge(a, b)
		if eid == InvalidID then
			return ValidationStatus.VerticesNotConnectedByEdge
		end

		if mesh:IsBoundaryEdge(eid) == false then
			return ValidationStatus.NotBoundaryEdge
		end

		local ev = mesh:GetOrientedBoundaryEdgeV(eid)
		if not (ev[1] == a and ev[2] == b) then
			return ValidationStatus.IncorrectLoopOrientation
		end
	end

	return ValidationStatus.Ok
end

function MeshValidation.HasDuplicateTriangles(mesh): number
	for tid in mesh:TriangleIndices() do
		local nbrs = mesh:GetTriNeighbourTris(tid)
		if nbrs[1] == nbrs[2] and nbrs[2] == nbrs[3] and nbrs[1] ~= InvalidID then
			return ValidationStatus.DuplicateTriangles
		end
	end

	return ValidationStatus.Ok
end

return MeshValidation
