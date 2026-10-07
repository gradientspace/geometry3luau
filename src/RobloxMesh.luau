-- Roblox engine bindings: convert DMesh3 to/from EditableMesh and MeshPart instances.
-- (Not part of geometry3Sharp.) This is the only module that touches Roblox engine services, and it only
-- does so when one of its functions is called. The format conversion itself lives in EditableMeshIO
-- (duck-typed, engine independent) and MeshArrays (partitioning).
--
-- Coordinates: an EditableMesh / MeshPart mesh is in "mesh space". A MeshPart displays it scaled by
-- part.Size / part.MeshSize and placed at part.CFrame. FromMeshPart returns world-space geometry by default.
--
-- RobloxMesh.FromEditableMesh(editable, options?) -> DMesh3, info        (options: see EditableMeshIO.Read)
-- RobloxMesh.ToEditableMesh(mesh, options?) -> EditableMesh, info
--     options: EditableMeshIO.Write options, plus
--       center: boolean (default false) - subtract the mesh bounds center from positions; info.offset is the
--               Vector3 that was subtracted
--       maxTriangles / maxVertices: limits to check (defaults 20000 / 60000)
--       editableOptions: table passed to AssetService:CreateEditableMesh
-- RobloxMesh.FromMeshPart(part, options?) -> DMesh3, info
--     options: EditableMeshIO.Read options (weld defaults to "positions"), plus
--       space: "world" (default) | "local" (part-local, with part scale) | "mesh" (raw mesh space)
-- RobloxMesh.ToMeshPart(mesh, options?) -> MeshPart, EditableMesh
--     the mesh is centered in its EditableMesh and the part is placed so that it appears at the mesh's own
--     coordinates transformed by options.cframe (default CFrame.identity). options:
--       cframe, name, parent, anchored (default true), collisionFidelity (default Enum.CollisionFidelity.Hull),
--       renderFidelity (default Enum.RenderFidelity.Precise), plus EditableMeshIO.Write options and limits
-- RobloxMesh.ToMeshParts(mesh, options?) -> { MeshPart }, { EditableMesh }
--     like ToMeshPart, but splits meshes that exceed the EditableMesh limits into several parts
--     (options.maxTriangles default 20000, options.maxVertices default 60000); if options.parent is given
--     the parts are parented to a new Model named options.name (returned as third value)
-- RobloxMesh.ApplyToMeshPart(part, mesh, options?) -> EditableMesh
--     replace the mesh of an existing MeshPart (via MeshPart:ApplyMesh), keeping its other properties;
--     mesh is in world space unless options.space = "mesh"

local EditableMeshIO = require(script.Parent.EditableMeshIO)
local MeshArrays = require(script.Parent.MeshArrays)

local RobloxMesh = {}

RobloxMesh.MaxTriangles = 20000
RobloxMesh.MaxVertices = 60000

local function assetService()
	return game:GetService("AssetService")
end

local function checkLimits(mesh, options)
	local maxT = options.maxTriangles or RobloxMesh.MaxTriangles
	local maxV = options.maxVertices or RobloxMesh.MaxVertices
	if mesh:TriangleCount() > maxT or mesh:VertexCount() > maxV then
		error(
			string.format(
				"RobloxMesh: mesh has %d triangles / %d vertices, EditableMesh limit is %d / %d (use RobloxMesh.ToMeshParts, or simplify with Reducer)",
				mesh:TriangleCount(),
				mesh:VertexCount(),
				maxT,
				maxV
			),
			3
		)
	end
end

function RobloxMesh.FromEditableMesh(editable, options: any?)
	return EditableMeshIO.Read(editable, options)
end

function RobloxMesh.ToEditableMesh(mesh, options: any?)
	local opts = table.clone(options or {})
	checkLimits(mesh, opts)
	local offset = Vector3.zero
	if opts.center then
		local c = mesh:GetBounds():Center()
		offset = Vector3.new(c.x, c.y, c.z)
		opts.offset = offset
	end
	local editable = assetService():CreateEditableMesh(opts.editableOptions or {})
	assert(editable, "RobloxMesh: EditableMesh allocation failed")
	local ok, result = pcall(EditableMeshIO.Write, mesh, editable, opts)
	if not ok then
		editable:Destroy()
		error(result, 2)
	end
	result.offset = offset
	return editable, result
end

-- returns an EditableMesh for the part's mesh, and whether we own it (and must destroy it)
local function partEditable(part)
	local content = part.MeshContent
	if content.SourceType == Enum.ContentSourceType.Object then
		local object = content.Object
		if object and object:IsA("EditableMesh") then
			return object, false
		end
	end
	local editable = assetService():CreateEditableMeshAsync(content)
	return editable, true
end

function RobloxMesh.FromMeshPart(part, options: any?)
	local opts = table.clone(options or {})
	opts.weld = opts.weld or "positions"
	local space = opts.space or "world"
	assert(space == "world" or space == "local" or space == "mesh", "RobloxMesh.FromMeshPart: unknown space " .. tostring(space))

	if space ~= "mesh" and opts.transform == nil then
		local scale = part.Size / part.MeshSize
		local sx, sy, sz = scale.X, scale.Y, scale.Z
		if space == "world" then
			local cf = part.CFrame
			opts.transform = function(x, y, z)
				local p = cf * Vector3.new(x * sx, y * sy, z * sz)
				return p.X, p.Y, p.Z
			end
			-- normals: inverse-transpose of (rotation * scale) = rotation * inverse(scale)
			opts.normalTransform = function(x, y, z)
				local n = cf:VectorToWorldSpace(Vector3.new(x / sx, y / sy, z / sz))
				return n.X, n.Y, n.Z
			end
		else
			opts.transform = function(x, y, z)
				return x * sx, y * sy, z * sz
			end
			opts.normalTransform = function(x, y, z)
				return x / sx, y / sy, z / sz
			end
		end
	end

	local editable, owned = partEditable(part)
	local ok, mesh, info = pcall(EditableMeshIO.Read, editable, opts)
	if owned then
		editable:Destroy()
	end
	if not ok then
		error(mesh, 2)
	end
	return mesh, info
end

local function makePart(editable, center, options)
	local part = assetService():CreateMeshPartAsync(Content.fromObject(editable), {
		CollisionFidelity = options.collisionFidelity or Enum.CollisionFidelity.Hull,
		RenderFidelity = options.renderFidelity or Enum.RenderFidelity.Precise,
	})
	part.Name = options.name or "geometry3luau"
	part.Anchored = options.anchored ~= false
	local placement = options.cframe or CFrame.identity
	-- the part pivot is at the center of the mesh's bounding box in mesh space
	part.CFrame = placement * CFrame.new(center + editable:GetCenter())
	return part
end

function RobloxMesh.ToMeshPart(mesh, options: any?)
	local opts = table.clone(options or {})
	opts.center = true
	local editable, info = RobloxMesh.ToEditableMesh(mesh, opts)
	local ok, part = pcall(makePart, editable, info.offset, opts)
	if not ok then
		editable:Destroy()
		error(part, 2)
	end
	if opts.parent then
		part.Parent = opts.parent
	end
	return part, editable
end

function RobloxMesh.ToMeshParts(mesh, options: any?)
	local opts = table.clone(options or {})
	local maxT = opts.maxTriangles or RobloxMesh.MaxTriangles
	local maxV = opts.maxVertices or RobloxMesh.MaxVertices
	local groups
	if mesh:TriangleCount() <= maxT and mesh:VertexCount() <= maxV then
		groups = nil
	else
		groups = MeshArrays.PartitionTriangles(mesh, maxT, maxV)
	end
	local parent = opts.parent
	opts.parent = nil
	local parts, editables = {}, {}
	local ok, err = pcall(function()
		if groups == nil then
			local part, editable = RobloxMesh.ToMeshPart(mesh, opts)
			table.insert(parts, part)
			table.insert(editables, editable)
		else
			local baseName = opts.name or "geometry3luau"
			for i, tids in groups do
				local sub = MeshArrays.ExtractSubmesh(mesh, tids)
				opts.name = baseName .. "_" .. i
				local part, editable = RobloxMesh.ToMeshPart(sub, opts)
				table.insert(parts, part)
				table.insert(editables, editable)
			end
		end
	end)
	if not ok then
		for _, p in parts do
			p:Destroy()
		end
		for _, e in editables do
			e:Destroy()
		end
		error(err, 2)
	end
	local model = nil
	if parent then
		model = Instance.new("Model")
		model.Name = options and options.name or "geometry3luau"
		for _, p in parts do
			p.Parent = model
		end
		model.Parent = parent
	end
	return parts, editables, model
end

function RobloxMesh.ApplyToMeshPart(part, mesh, options: any?)
	local opts = table.clone(options or {})
	local space = opts.space or "world"
	if space == "world" then
		-- express the world-space mesh in the part's frame
		local inv = part.CFrame:Inverse()
		opts.transform = function(x, y, z)
			local p = inv * Vector3.new(x, y, z)
			return p.X, p.Y, p.Z
		end
	end
	opts.center = false
	local editable, _ = RobloxMesh.ToEditableMesh(mesh, opts)
	local temp = assetService():CreateMeshPartAsync(Content.fromObject(editable), {
		CollisionFidelity = part.CollisionFidelity,
		RenderFidelity = part.RenderFidelity,
	})
	part:ApplyMesh(temp)
	temp:Destroy()
	-- keep the mesh at the same place: the pivot moves to the new mesh's bounds center
	local center = editable:GetCenter()
	part.Size = editable:GetSize()
	part.CFrame = part.CFrame * CFrame.new(center)
	return editable
end

return RobloxMesh
