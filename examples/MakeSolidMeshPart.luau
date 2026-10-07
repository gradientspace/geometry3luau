-- Example: turn a MeshPart (any mesh: open, self-intersecting, multiple overlapping pieces) into a clean,
-- closed solid MeshPart, using fast winding number + marching cubes + remesh + simplify.
--
-- In Studio (development package):
--   local G3 = require(game.ServerStorage.geometry3luau)
--   local makeSolid = require(game.ServerStorage.geometry3luau.Examples.MakeSolidMeshPart)
--   local newPart = makeSolid(G3, workspace.MyMeshPart, { GridResolution = 96, TargetTriangleCount = 8000 })
--
-- Returns the new MeshPart (parented next to the source part, which is left unchanged) and the
-- MakeSolid object (for timings / intermediate meshes).

return function(G3, sourcePart: MeshPart, options: { [string]: any }?)
	local opts = options or {}

	-- 1. read the part's mesh in world space (published meshes come back with split vertices; weld them)
	local mesh = G3.RobloxMesh.FromMeshPart(sourcePart, { weld = "positions", normals = "none", uvs = "none" })

	-- 2. make solid. Yield regularly so Studio does not time out on large inputs.
	local solid = G3.MakeSolid.new(mesh)
	solid.GridResolution = opts.GridResolution or 64
	solid.TargetTriangleCount = opts.TargetTriangleCount or math.min(G3.RobloxMesh.MaxTriangles, 10000)
	solid.RemeshPasses = opts.RemeshPasses or 10
	solid.Progress = G3.ProgressCancel.Yielding(0.1)
	local result = solid:Compute()
	assert(result, "MakeSolid was cancelled")

	-- 3. write a new MeshPart at the same world position (normals are computed by the engine)
	local part = G3.RobloxMesh.ToMeshPart(result, {
		name = sourcePart.Name .. "_Solid",
		parent = sourcePart.Parent,
		-- Hull is fast; PreciseConvexDecomposition gives accurate collisions but can take several seconds
		collisionFidelity = opts.CollisionFidelity or Enum.CollisionFidelity.Hull,
	})
	part.Color = sourcePart.Color
	part.Material = sourcePart.Material
	return part, solid
end
