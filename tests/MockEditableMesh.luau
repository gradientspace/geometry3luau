-- A minimal stand-in for Roblox's EditableMesh, for headless tests of EditableMeshIO.
-- Implements the subset of the EditableMesh API used by EditableMeshIO. Ids are arbitrary (non-sequential)
-- like the engine's. Vectors are stored as given (Roblox Vector3/Vector2/Color3 under Lune).

local MockEditableMesh = {}
MockEditableMesh.__index = MockEditableMesh

function MockEditableMesh.new()
	return setmetatable({
		nextId = 1000,
		positions = {},
		vertexOrder = {},
		faces = {},
		faceOrder = {},
		normals = {},
		uvs = {},
		colors = {},
		alphas = {},
		faceNormals = {},
		faceUVs = {},
		faceColors = {},
	}, MockEditableMesh)
end

function MockEditableMesh:newId()
	self.nextId += 7
	return self.nextId
end

function MockEditableMesh:AddVertex(p)
	local id = self:newId()
	self.positions[id] = p
	table.insert(self.vertexOrder, id)
	return id
end
function MockEditableMesh:GetVertices()
	return table.clone(self.vertexOrder)
end
function MockEditableMesh:GetPosition(id)
	return self.positions[id]
end
function MockEditableMesh:AddTriangle(a, b, c)
	assert(self.positions[a] and self.positions[b] and self.positions[c], "invalid vertex id")
	local id = self:newId()
	self.faces[id] = { a, b, c }
	table.insert(self.faceOrder, id)
	return id
end
function MockEditableMesh:GetFaces()
	return table.clone(self.faceOrder)
end
function MockEditableMesh:GetFaceVertices(fid)
	return table.clone(self.faces[fid])
end
function MockEditableMesh:AddNormal(n)
	local id = self:newId()
	self.normals[id] = n
	return id
end
function MockEditableMesh:GetNormal(id)
	return self.normals[id]
end
function MockEditableMesh:SetFaceNormals(fid, ids)
	self.faceNormals[fid] = table.clone(ids)
end
function MockEditableMesh:GetFaceNormals(fid)
	return self.faceNormals[fid] or { 0, 0, 0 }
end
function MockEditableMesh:AddUV(uv)
	local id = self:newId()
	self.uvs[id] = uv
	return id
end
function MockEditableMesh:GetUV(id)
	return self.uvs[id]
end
function MockEditableMesh:SetFaceUVs(fid, ids)
	self.faceUVs[fid] = table.clone(ids)
end
function MockEditableMesh:GetFaceUVs(fid)
	return self.faceUVs[fid] or { 0, 0, 0 }
end
function MockEditableMesh:AddColor(c, alpha)
	local id = self:newId()
	self.colors[id] = c
	self.alphas[id] = alpha
	return id
end
function MockEditableMesh:GetColor(id)
	return self.colors[id]
end
function MockEditableMesh:GetColorAlpha(id)
	return self.alphas[id]
end
function MockEditableMesh:SetFaceColors(fid, ids)
	self.faceColors[fid] = table.clone(ids)
end
function MockEditableMesh:GetFaceColors(fid)
	return self.faceColors[fid] or { 0, 0, 0 }
end

return MockEditableMesh
