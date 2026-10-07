-- Simple test-mesh generators for the test suites (not part of the library API).
-- All meshes are DMesh3 with consistent outward orientation (counter-clockwise when viewed from outside).
--
--   TestMeshes.Octahedron(G3)                  closed, 6 verts / 8 tris
--   TestMeshes.Grid(G3, n, size)               open planar grid in XY, n x n quads
--   TestMeshes.UVSphere(G3, radius, slices, stacks, center?)   closed sphere
--   TestMeshes.Box(G3, sizeX, sizeY, sizeZ, n, center?)        closed box, each face an n x n grid (shared verts)
--   TestMeshes.Torus(G3, R, r, nu, nv)         closed torus around Y axis
--   TestMeshes.Translate(G3, mesh, dx, dy, dz) translates in place, returns mesh
--   TestMeshes.Append(G3, target, source)      appends source mesh (as separate component) into target

local TestMeshes = {}

function TestMeshes.Octahedron(G3)
	local mesh = G3.DMesh3.new()
	local V = G3.Vector3d.new
	for _, v in { V(1, 0, 0), V(-1, 0, 0), V(0, 1, 0), V(0, -1, 0), V(0, 0, 1), V(0, 0, -1) } do
		mesh:AppendVertex(v)
	end
	for _, t in { { 0, 2, 4 }, { 2, 1, 4 }, { 1, 3, 4 }, { 3, 0, 4 }, { 2, 0, 5 }, { 1, 2, 5 }, { 3, 1, 5 }, { 0, 3, 5 } } do
		mesh:AppendTriangle(t[1], t[2], t[3])
	end
	return mesh
end

function TestMeshes.Grid(G3, n: number, size: number)
	local mesh = G3.DMesh3.new()
	for j = 0, n do
		for i = 0, n do
			mesh:AppendVertex(G3.Vector3d.new(size * i / n, size * j / n, 0))
		end
	end
	for j = 0, n - 1 do
		for i = 0, n - 1 do
			local a = j * (n + 1) + i
			local b, c, d = a + 1, a + n + 2, a + n + 1
			mesh:AppendTriangle(a, b, c)
			mesh:AppendTriangle(a, c, d)
		end
	end
	return mesh
end

function TestMeshes.UVSphere(G3, radius: number, slices: number, stacks: number, center)
	local c = center or G3.Vector3d.Zero
	local mesh = G3.DMesh3.new()
	local top = mesh:AppendVertex(G3.Vector3d.new(c.x, c.y + radius, c.z))
	local rings = {}
	for j = 1, stacks - 1 do
		local phi = math.pi * j / stacks
		local ring = {}
		for i = 0, slices - 1 do
			local theta = 2 * math.pi * i / slices
			local x = radius * math.sin(phi) * math.cos(theta)
			local z = radius * math.sin(phi) * math.sin(theta)
			local y = radius * math.cos(phi)
			ring[i + 1] = mesh:AppendVertex(G3.Vector3d.new(c.x + x, c.y + y, c.z + z))
		end
		rings[j] = ring
	end
	local bottom = mesh:AppendVertex(G3.Vector3d.new(c.x, c.y - radius, c.z))
	-- outward orientation: viewed from outside, CCW. theta increases toward +z from +x,
	-- so at the top cap, (top, ring[i+1], ring[i]) is CCW from above.
	for i = 0, slices - 1 do
		local i2 = (i + 1) % slices
		mesh:AppendTriangle(top, rings[1][i2 + 1], rings[1][i + 1])
	end
	for j = 1, stacks - 2 do
		local r0, r1 = rings[j], rings[j + 1]
		for i = 0, slices - 1 do
			local i2 = (i + 1) % slices
			mesh:AppendTriangle(r0[i + 1], r0[i2 + 1], r1[i2 + 1])
			mesh:AppendTriangle(r0[i + 1], r1[i2 + 1], r1[i + 1])
		end
	end
	local last = rings[stacks - 1]
	for i = 0, slices - 1 do
		local i2 = (i + 1) % slices
		mesh:AppendTriangle(bottom, last[i + 1], last[i2 + 1])
	end
	return mesh
end

function TestMeshes.Box(G3, sx: number, sy: number, sz: number, n: number, center)
	local c = center or G3.Vector3d.Zero
	local mesh = G3.DMesh3.new()
	local lookup = {}
	local function vert(i, j, k) -- lattice coords in [0, n]
		local key = i .. "," .. j .. "," .. k
		local vid = lookup[key]
		if vid == nil then
			vid = mesh:AppendVertex(G3.Vector3d.new(c.x + sx * (i / n - 0.5), c.y + sy * (j / n - 0.5), c.z + sz * (k / n - 0.5)))
			lookup[key] = vid
		end
		return vid
	end
	-- each face: origin corner + two axes (u, v) such that u x v points outward
	local faces = {
		{ { 0, 0, n }, { 1, 0, 0 }, { 0, 1, 0 } }, -- +z
		{ { n, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 } }, -- -z
		{ { n, 0, n }, { 0, 0, -1 }, { 0, 1, 0 } }, -- +x
		{ { 0, 0, 0 }, { 0, 0, 1 }, { 0, 1, 0 } }, -- -x
		{ { 0, n, n }, { 1, 0, 0 }, { 0, 0, -1 } }, -- +y
		{ { 0, 0, 0 }, { 1, 0, 0 }, { 0, 0, 1 } }, -- -y
	}
	for _, f in faces do
		local o, u, v = f[1], f[2], f[3]
		local function p(a, b)
			return vert(o[1] + u[1] * a + v[1] * b, o[2] + u[2] * a + v[2] * b, o[3] + u[3] * a + v[3] * b)
		end
		for b = 0, n - 1 do
			for a = 0, n - 1 do
				local v00, v10, v11, v01 = p(a, b), p(a + 1, b), p(a + 1, b + 1), p(a, b + 1)
				mesh:AppendTriangle(v00, v10, v11)
				mesh:AppendTriangle(v00, v11, v01)
			end
		end
	end
	return mesh
end

function TestMeshes.Torus(G3, R: number, r: number, nu: number, nv: number)
	local mesh = G3.DMesh3.new()
	local ids = {}
	for i = 0, nu - 1 do
		local u = 2 * math.pi * i / nu
		for j = 0, nv - 1 do
			local v = 2 * math.pi * j / nv
			local x = (R + r * math.cos(v)) * math.cos(u)
			local z = (R + r * math.cos(v)) * math.sin(u)
			local y = r * math.sin(v)
			ids[i * nv + j] = mesh:AppendVertex(G3.Vector3d.new(x, y, z))
		end
	end
	for i = 0, nu - 1 do
		local i2 = (i + 1) % nu
		for j = 0, nv - 1 do
			local j2 = (j + 1) % nv
			local a, b, c, d = ids[i * nv + j], ids[i2 * nv + j], ids[i2 * nv + j2], ids[i * nv + j2]
			mesh:AppendTriangle(a, c, b)
			mesh:AppendTriangle(a, d, c)
		end
	end
	return mesh
end

function TestMeshes.Translate(G3, mesh, dx: number, dy: number, dz: number)
	for vid in mesh:VertexIndices() do
		local v = mesh:GetVertex(vid)
		mesh:SetVertex(vid, G3.Vector3d.new(v.x + dx, v.y + dy, v.z + dz))
	end
	return mesh
end

function TestMeshes.Append(G3, target, source)
	local map = {}
	for vid in source:VertexIndices() do
		map[vid] = target:AppendVertex(source:GetVertex(vid))
	end
	for tid in source:TriangleIndices() do
		local t = source:GetTriangle(tid)
		target:AppendTriangle(map[t[1]], map[t[2]], map[t[3]])
	end
	return target
end

return TestMeshes
