-- Port of the ValidationStatus enum from geometry3Sharp queries/MeshValidation.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Enum values match the C# declaration order (Ok == 0). Also available as MeshValidation.ValidationStatus.

return table.freeze({
	Ok = 0,

	NotAVertex = 1,
	NotBoundaryVertex = 2,
	NotBoundaryEdge = 3,

	NotATriangle = 4,

	VerticesNotConnectedByEdge = 5,
	IncorrectLoopOrientation = 6,

	DuplicateTriangles = 7,

	NearDegenerateMeshEdges = 8,
	NearDenegerateInputGeometry = 9,
})
