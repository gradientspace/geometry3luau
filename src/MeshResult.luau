-- Port of the MeshResult enum from geometry3Sharp mesh/DMesh3.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- MeshResult.Ok == 0 etc. MeshResult.Name(r) returns the enum name for messages.
-- (MeshComponents and MeshHints live on DMesh3: DMesh3.MeshComponents, DMesh3.MeshHints.)

local MeshResult = {
	Ok = 0,
	Failed_NotAVertex = 1,
	Failed_NotATriangle = 2,
	Failed_NotAnEdge = 3,

	Failed_BrokenTopology = 10,
	Failed_HitValenceLimit = 11,

	Failed_IsBoundaryEdge = 20,
	Failed_FlippedEdgeExists = 21,
	Failed_IsBowtieVertex = 22,
	Failed_InvalidNeighbourhood = 23, -- these are all failures for CollapseEdge
	Failed_FoundDuplicateTriangle = 24,
	Failed_CollapseTetrahedron = 25,
	Failed_CollapseTriangle = 26,
	Failed_NotABoundaryEdge = 27,
	Failed_SameOrientation = 28,

	Failed_WouldCreateBowtie = 30,
	Failed_VertexAlreadyExists = 31,
	Failed_CannotAllocateVertex = 32,

	Failed_WouldCreateNonmanifoldEdge = 50,
	Failed_TriangleAlreadyExists = 51,
	Failed_CannotAllocateTriangle = 52,
}

local names = {}
for k, v in MeshResult do
	names[v] = k
end

function MeshResult.Name(r: number): string
	return names[r] or tostring(r)
end

return table.freeze(MeshResult)
