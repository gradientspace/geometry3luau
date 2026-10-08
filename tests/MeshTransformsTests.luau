-- Tests for MeshTransforms and TransformWrapper.

return function(G3, h)
	local TM = require(script.Parent.TestMeshes)
	local MT = G3.MeshTransforms
	local V = G3.Vector3d.new

	local function nearV3(a, b, eps, msg)
		h.truthy(a:EpsilonEqual(b, eps or 1e-9), (msg or "") .. ": " .. tostring(a) .. " vs " .. tostring(b))
	end

	local function signedVolume(mesh)
		local vol = 0
		for tid in mesh:TriangleIndices() do
			local a, b, c = mesh:GetTriVertices(tid)
			vol += a:Dot(b:Cross(c)) / 6
		end
		return vol
	end

	local function copyPositions(mesh)
		local t = {}
		for vid in mesh:VertexIndices() do
			t[vid] = mesh:GetVertex(vid)
		end
		return t
	end

	local function copyNormals(mesh)
		local t = {}
		for vid in mesh:VertexIndices() do
			t[vid] = mesh:GetVertexNormal(vid)
		end
		return t
	end

	local function samePositions(mesh, ref, eps, msg)
		for vid in mesh:VertexIndices() do
			nearV3(mesh:GetVertex(vid), ref[vid], eps, (msg or "pos") .. " v" .. vid)
		end
	end

	local function sameNormals(mesh, ref, eps, msg)
		for vid in mesh:VertexIndices() do
			nearV3(mesh:GetVertexNormal(vid), ref[vid], eps, (msg or "normal") .. " v" .. vid)
		end
	end

	-- sphere with computed vertex normals
	local function sphere()
		local mesh = TM.UVSphere(G3, 1, 12, 8)
		mesh:EnableVertexNormals(V(0, 1, 0))
		G3.MeshNormals.QuickCompute(mesh)
		return mesh
	end

	h:test("Translate and Scale", function()
		local mesh = TM.Box(G3, 1, 2, 3, 2)
		local p0 = copyPositions(mesh)
		MT.Translate(mesh, V(1, 2, 3))
		MT.Translate(mesh, -1, -2, -3)
		samePositions(mesh, p0, 1e-12, "translate round trip")
		local vol = signedVolume(mesh)
		MT.Scale(mesh, 2)
		h.near(signedVolume(mesh), 8 * vol, 1e-9)
		MT.Scale(mesh, 0.5, 1, 2)
		h.near(signedVolume(mesh), 8 * vol, 1e-9)
		MT.Scale(mesh, V(1, 0.5, 0.25), V(0, 0, 0))
		samePositions(mesh, p0, 1e-12, "scale round trip")
		-- scale around a point keeps that point fixed
		local mesh2 = TM.Octahedron(G3)
		MT.Scale(mesh2, V(3, 3, 3), V(1, 0, 0))
		nearV3(mesh2:GetVertex(0), V(1, 0, 0), 1e-12, "fixed point")
		nearV3(mesh2:GetVertex(1), V(-5, 0, 0), 1e-12, "scaled point")
	end)

	h:test("Rotate point, frame and mesh", function()
		local q = G3.Quaterniond.AxisAngleD(V(0, 0, 1), 90)
		local origin = V(1, 1, 0)
		nearV3(MT.Rotate(V(2, 1, 0), origin, q), V(1, 2, 0), 1e-12, "point")
		local f = G3.Frame3d.new(V(2, 1, 0))
		local fr = MT.Rotate(f, origin, q)
		nearV3(fr.Origin, V(1, 2, 0), 1e-12, "frame origin")
		nearV3(fr:X(), V(0, 1, 0), 1e-12, "frame X")
		h.eq(f.Origin, V(2, 1, 0), "input frame not modified")
		local mesh = sphere()
		local p0, n0 = copyPositions(mesh), copyNormals(mesh)
		MT.Rotate(mesh, origin, q)
		for vid in mesh:VertexIndices() do
			nearV3(mesh:GetVertex(vid), q * (p0[vid] - origin) + origin, 1e-12, "mesh pos")
			nearV3(mesh:GetVertexNormal(vid), q * n0[vid], 1e-12, "mesh normal")
		end
		MT.Rotate(mesh, origin, q:Inverse())
		samePositions(mesh, p0, 1e-12)
		sameNormals(mesh, n0, 1e-12)
	end)

	h:test("ToFrame / FromFrame round trip", function()
		local mesh = sphere()
		local p0, n0 = copyPositions(mesh), copyNormals(mesh)
		local frame = G3.Frame3d.new(V(3, -2, 5), G3.Quaterniond.AxisAngleD(V(1, 2, 3):Normalized(), 37))
		MT.ToFrame(mesh, frame)
		nearV3(mesh:GetVertex(0), frame:ToFrameP(p0[0]), 1e-12, "ToFrame pos")
		nearV3(mesh:GetVertexNormal(0), frame:ToFrameV(n0[0]), 1e-12, "ToFrame normal")
		MT.FromFrame(mesh, frame)
		samePositions(mesh, p0, 1e-12)
		sameNormals(mesh, n0, 1e-12)
	end)

	h:test("Up-axis conversions", function()
		h.eq(MT.ConvertZUpToYUp(V(1, 2, 3)), V(1, 3, -2))
		h.eq(MT.ConvertYUpToZUp(V(1, 3, -2)), V(1, 2, 3))
		h.eq(MT.FlipLeftRightCoordSystems(V(1, 2, 3)), V(1, 2, -3))
		local f = G3.Frame3d.new(V(1, 2, 3), G3.Quaterniond.AxisAngleD(V(0, 1, 0), 30))
		local fy = MT.ConvertZUpToYUp(f)
		nearV3(fy.Origin, V(1, 3, -2), 1e-12)
		nearV3(fy:Z(), MT.ConvertZUpToYUp(f:Z()), 1e-12)
		h.truthy(MT.ConvertYUpToZUp(fy):EpsilonEqual(f, 1e-9), "frame round trip")
		h.truthy(not pcall(MT.FlipLeftRightCoordSystems, f), "Frame3d flip errors like C#")

		local mesh = sphere()
		local p0, n0 = copyPositions(mesh), copyNormals(mesh)
		local vol = signedVolume(mesh)
		MT.ConvertZUpToYUp(mesh)
		h.near(signedVolume(mesh), vol, 1e-9, "rotation keeps volume")
		nearV3(mesh:GetVertex(0), MT.ConvertZUpToYUp(p0[0]), 1e-12)
		MT.ConvertYUpToZUp(mesh)
		samePositions(mesh, p0, 1e-12)
		sameNormals(mesh, n0, 1e-12)

		-- mirror + reversed orientation: volume stays positive, mesh stays valid
		MT.FlipLeftRightCoordSystems(mesh)
		h.near(signedVolume(mesh), vol, 1e-9, "flip keeps outward orientation")
		h.truthy(mesh:CheckValidity())
		MT.FlipLeftRightCoordSystems(mesh)
		samePositions(mesh, p0, 1e-12)
		sameNormals(mesh, n0, 1e-12)
	end)

	h:test("VertexNormalOffset", function()
		local mesh = sphere()
		MT.VertexNormalOffset(mesh, 0.5)
		for vid in mesh:VertexIndices() do
			-- vertex normals of a UV sphere are close to radial
			h.near(mesh:GetVertex(vid):Length(), 1.5, 0.02)
		end
	end)

	h:test("PerVertexTransform overloads", function()
		local mesh = sphere()
		local p0, n0 = copyPositions(mesh), copyNormals(mesh)
		MT.PerVertexTransform(mesh, function(v)
			return v * 2
		end)
		nearV3(mesh:GetVertex(3), p0[3] * 2, 1e-12, "Func<v,v>")
		MT.PerVertexTransformVN(mesh, function(v, n)
			return v - n
		end)
		nearV3(mesh:GetVertex(3), p0[3] * 2 - n0[3], 1e-12, "Func<v,n,v>")
		MT.PerVertexTransformPN(mesh, function(_, n)
			return G3.VectorTuple.Vector3dTuple2(n, -n)
		end)
		nearV3(mesh:GetVertex(3), n0[3], 1e-12, "Func<v,n,tuple> pos")
		nearV3(mesh:GetVertexNormal(3), -n0[3], 1e-12, "Func<v,n,tuple> normal")

		-- TransformSequence (positions and normals)
		mesh = sphere()
		local seq = G3.TransformSequence.new()
		local q = G3.Quaterniond.AxisAngleD(V(1, 0, 0), 45)
		seq:AppendRotation(q)
		seq:AppendTranslation(V(0, 0, 10))
		MT.PerVertexTransform(mesh, seq)
		nearV3(mesh:GetVertex(5), q * p0[5] + V(0, 0, 10), 1e-12, "sequence pos")
		nearV3(mesh:GetVertexNormal(5), q * n0[5], 1e-12, "sequence normal")

		-- subset, as array and as iterator
		mesh = sphere()
		local seen = {}
		MT.PerVertexTransform(mesh, { 1, 2 }, function(v, vid)
			seen[vid] = true
			return v + V(1, 0, 0)
		end)
		h.truthy(seen[1] and seen[2] and not seen[0])
		nearV3(mesh:GetVertex(1), p0[1] + V(1, 0, 0), 1e-12)
		nearV3(mesh:GetVertex(0), p0[0], 1e-12)
		MT.PerVertexTransform(mesh, G3.Util.ArrayItr({ 1 }), function(v)
			return v - V(1, 0, 0)
		end)
		nearV3(mesh:GetVertex(1), p0[1], 1e-12, "iterator subset")

		-- subset through MapV
		MT.PerVertexTransform(mesh, { 0 }, function(vid)
			return vid + 4
		end, function(v, vid, map_vid)
			h.eq(vid, 0)
			h.eq(map_vid, 4)
			return V(0, 0, 0)
		end)
		h.eq(mesh:GetVertex(4), V(0, 0, 0))

		-- target / source / mapV
		local target = TM.Octahedron(G3)
		local source = TM.Octahedron(G3)
		local mapV = {}
		for vid in source:VertexIndices() do
			mapV[vid + 1] = 5 - vid
		end
		MT.PerVertexTransform(target, source, mapV, function(v, vid, map_vid)
			h.eq(map_vid, 5 - vid)
			return v * 3
		end)
		nearV3(target:GetVertex(0), V(3, 0, 0), 1e-12)
		nearV3(target:GetVertex(5), V(0, 0, -3), 1e-12)
	end)

	h:test("TransformMesh and TransformWrapper", function()
		local mesh = sphere()
		local p0, n0 = copyPositions(mesh), copyNormals(mesh)
		local frame = G3.Frame3d.new(V(1, 2, 3), G3.Quaterniond.AxisAngleD(V(0, 1, 1):Normalized(), 70))
		MT.TransformMesh(mesh, G3.Matrix4d.FromFrame(frame))
		for vid in mesh:VertexIndices() do
			nearV3(mesh:GetVertex(vid), frame:FromFrameP(p0[vid]), 1e-12, "TransformMesh pos")
			nearV3(mesh:GetVertexNormal(vid), frame:FromFrameV(n0[vid]), 1e-12, "TransformMesh normal")
		end

		-- non-uniform scale: normals use the inverse-transpose
		local mesh2 = sphere()
		local S = G3.Matrix4d.Scale(V(1, 3, 1))
		MT.TransformMesh(mesh2, S)
		nearV3(mesh2:GetVertex(0), V(p0[0].x, 3 * p0[0].y, p0[0].z), 1e-12)
		nearV3(mesh2:GetVertexNormal(1), V(n0[1].x, n0[1].y / 3, n0[1].z), 1e-12)

		local id = G3.TransformWrapper.new()
		h.eq(id:TransformP(V(1, 2, 3)), V(1, 2, 3))
		h.eq(id:TransformN(V(1, 2, 3)), V(1, 2, 3))
		local w = MT.TransformWrapper.MakeFromAffine(S)
		h.eq(w:TransformP(V(1, 2, 3)), V(1, 6, 3))
		nearV3(w:TransformN(V(0, 3, 0)), V(0, 1, 0), 1e-12)
		local m3 = G3.Matrix4d.FromMatrix3(G3.Matrix3d.new(1, 2, 0, 0, 1, 0, 0, 0, 1))
		local w2 = G3.TransformWrapper.MakeFromAffine(m3)
		nearV3(w2:TransformP(V(1, 1, 1)), V(3, 1, 1), 1e-12)
	end)

	return h:results()
end
