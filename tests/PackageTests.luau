-- Every module in the package loads, and nested names resolve.
return function(G3, h)
	h:test("all modules load", function()
		G3.LoadAll()
		local n = 0
		for _, child in script.Parent.Parent:GetChildren() do
			if child:IsA("ModuleScript") then
				h.truthy(G3[child.Name] ~= nil, "module " .. child.Name)
				n += 1
			end
		end
		h.truthy(n > 100, "module count " .. n)
	end)
	h:test("names", function()
		h.eq(G3.Vector3d.ClassName, "Vector3d")
		-- only the double types are ported (no Vector3f), and nothing shadows the Roblox Vector3
		h.eq(G3.Vector3f, nil)
		h.eq(G3.Vector3, nil)
		h.truthy(G3.Index3i.new ~= nil)
		h.eq(G3.MeshComponents.VertexNormals, 1)
		h.truthy(type(G3.Version) == "string")
	end)
	return h:results()
end
