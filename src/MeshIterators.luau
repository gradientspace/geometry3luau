-- Port of geometry3Sharp mesh/MeshIterators.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau port notes (see PORTING.md):
--   * every function returns an iterator usable in a generic for: `for vid in MeshIterators.BoundaryVertices(mesh) do`
--   * FilterF(mesh, id) -> boolean is a plain function

local MeshIterators = {}

-- generic helper: iterate i in [0, N) where validF(i) and filterF(i)
local function filtered(N: number, validF: (number) -> boolean, filterF: (number) -> boolean): () -> number?
	local i = -1
	return function()
		while true do
			i += 1
			if i >= N then
				return nil
			end
			if validF(i) and filterF(i) then
				return i
			end
		end
	end
end

function MeshIterators.FilteredVertices(mesh, FilterF: (any, number) -> boolean): () -> number?
	return filtered(mesh:MaxVertexID(), function(i)
		return mesh:IsVertex(i)
	end, function(i)
		return FilterF(mesh, i)
	end)
end

function MeshIterators.FilteredEdges(mesh, FilterF: (any, number) -> boolean): () -> number?
	return filtered(mesh:MaxEdgeID(), function(i)
		return mesh:IsEdge(i)
	end, function(i)
		return FilterF(mesh, i)
	end)
end

function MeshIterators.FilteredTriangles(mesh, FilterF: (any, number) -> boolean): () -> number?
	return filtered(mesh:MaxTriangleID(), function(i)
		return mesh:IsTriangle(i)
	end, function(i)
		return FilterF(mesh, i)
	end)
end

-- Boundary vertices of mesh
function MeshIterators.BoundaryVertices(mesh): () -> number?
	return filtered(mesh:MaxVertexID(), function(i)
		return mesh:IsVertex(i)
	end, function(i)
		return mesh:IsBoundaryVertex(i)
	end)
end

-- boundary vertices of mesh, but based on edges, so returns each vertex twice!
function MeshIterators.BoundaryEdgeVertices(mesh): () -> number?
	local N = mesh:MaxEdgeID()
	local i = -1
	local pending: number? = nil
	return function()
		if pending ~= nil then
			local p = pending
			pending = nil
			return p
		end
		while true do
			i += 1
			if i >= N then
				return nil
			end
			if mesh:IsEdge(i) and mesh:IsBoundaryEdge(i) then
				local ev = mesh:GetEdgeV(i)
				pending = ev[2]
				return ev[1]
			end
		end
	end
end

function MeshIterators.InteriorVertices(mesh): () -> number?
	return filtered(mesh:MaxVertexID(), function(i)
		return mesh:IsVertex(i)
	end, function(i)
		return mesh:IsBoundaryVertex(i) == false
	end)
end

function MeshIterators.GroupBoundaryVertices(mesh): () -> number?
	return filtered(mesh:MaxVertexID(), function(i)
		return mesh:IsVertex(i)
	end, function(i)
		return mesh:IsGroupBoundaryVertex(i)
	end)
end

function MeshIterators.GroupJunctionVertices(mesh): () -> number?
	return filtered(mesh:MaxVertexID(), function(i)
		return mesh:IsVertex(i)
	end, function(i)
		return mesh:IsGroupJunctionVertex(i)
	end)
end

function MeshIterators.BoundaryEdges(mesh): () -> number?
	return filtered(mesh:MaxEdgeID(), function(i)
		return mesh:IsEdge(i)
	end, function(i)
		return mesh:IsBoundaryEdge(i)
	end)
end

function MeshIterators.InteriorEdges(mesh): () -> number?
	return filtered(mesh:MaxEdgeID(), function(i)
		return mesh:IsEdge(i)
	end, function(i)
		return mesh:IsBoundaryEdge(i) == false
	end)
end

function MeshIterators.GroupBoundaryEdges(mesh): () -> number?
	return filtered(mesh:MaxEdgeID(), function(i)
		return mesh:IsEdge(i)
	end, function(i)
		return mesh:IsGroupBoundaryEdge(i)
	end)
end

function MeshIterators.BowtieVertices(mesh): () -> number?
	return filtered(mesh:MaxVertexID(), function(i)
		return mesh:IsVertex(i)
	end, function(i)
		return mesh:IsBowtieVertex(i)
	end)
end

return MeshIterators
