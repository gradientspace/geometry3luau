-- Sanity checks for the test mesh generators (orientation and closedness).
return function(G3, h)
	local TM = require(script.Parent.TestMeshes)
	h:test("generators produce valid closed outward meshes", function()
		local cases = {
			{ "octahedron", TM.Octahedron(G3), 4 / 3 },
			{ "sphere", TM.UVSphere(G3, 1, 32, 16), nil },
			{ "box", TM.Box(G3, 2, 3, 4, 3), 24 },
			{ "torus", TM.Torus(G3, 2, 0.5, 24, 12), nil },
		}
		for _, c in cases do
			local name, mesh = c[1], c[2]
			h.truthy(mesh:CheckValidity(false), name .. " valid")
			h.truthy(mesh:IsClosed(), name .. " closed")
			-- signed volume via divergence theorem must be positive for outward orientation
			local vol = 0
			for tid in mesh:TriangleIndices() do
				local a, b, cc = mesh:GetTriVertices(tid)
				vol += a:Dot(b:Cross(cc)) / 6
			end
			h.truthy(vol > 0, name .. " outward (volume " .. vol .. ")")
			if c[3] then
				h.near(vol, c[3], 1e-9, name .. " volume")
			end
		end
		local sphere = TM.UVSphere(G3, 1, 32, 16)
		h.near(sphere:WindingNumber(G3.Vector3d.Zero), 1, 1e-6)
	end)
	h:test("grid is open", function()
		local g = TM.Grid(G3, 4, 1)
		h.truthy(g:CheckValidity(false))
		h.truthy(not g:IsClosed())
	end)
	return h:results()
end
