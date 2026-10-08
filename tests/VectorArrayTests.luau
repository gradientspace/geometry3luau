-- Tests for VectorArray (VectorArray3d/2, VectorArray3i, IndexArray2i/3i/4i, generic bases) and DCurve3.SetVertices(VectorArray3d)
return function(G3, h)
	local VA = G3.VectorArray
	local Vector3d, Vector2d = G3.Vector3d, G3.Vector2d

	local function collect(iter)
		local t = {}
		for x in iter do
			table.insert(t, x)
		end
		return t
	end

	h:test("VectorArray3d", function()
		local a = VA.VectorArray3d.new(3)
		h.eq(a:Count(), 3)
		h.eq(#a.array, 9)
		h.eq(a:Get(2), Vector3d.Zero)
		a:Set(0, Vector3d.new(1, 2, 3))
		a:Set(1, 4, 5, 6)
		a:SetComponents(2, 7, 8, 9)
		h.eq(a:Get(0), Vector3d.new(1, 2, 3))
		h.eq(a:Get(1), Vector3d.new(4, 5, 6))
		h.eq(a.array[9], 9)
		h.eq(#collect(a:GetEnumerator()), 9)
		local vs = collect(a:AsVector3())
		h.eq(#vs, 3)
		h.eq(vs[3], Vector3d.new(7, 8, 9))
		local arr = a:AsArray()
		h.eq(#arr, 3)
		h.eq(arr[2], Vector3d.new(4, 5, 6))

		-- wrap without copy
		local data = { 1, 1, 1, 2, 2, 2 }
		local b = VA.VectorArray3d.FromArray(data)
		h.eq(b:Count(), 2)
		b:Set(1, Vector3d.new(3, 3, 3))
		h.eq(data[4], 3)

		-- Set(iStart, iCount, source) copies source's first iCount elements to iStart
		local c = VA.VectorArray3d.new(4)
		c:Set(1, 2, b)
		h.eq(c:Get(0), Vector3d.Zero)
		h.eq(c:Get(1), Vector3d.new(1, 1, 1))
		h.eq(c:Get(2), Vector3d.new(3, 3, 3))
		h.eq(c:Get(3), Vector3d.Zero)

		c:Resize(1)
		h.eq(c:Count(), 1)
		h.eq(c:Get(0), Vector3d.Zero)
	end)

	h:test("VectorArray3d debug mode", function()
		local Util = G3.Util
		local old = Util.DebugChecks
		Util.DebugChecks = true
		local ok, err = pcall(function()
			local a = VA.VectorArray3d.new(2, true)
			a:Set(0, Vector3d.One)
			a:Set(0, Vector3d.One) -- already set
		end)
		Util.DebugChecks = old
		h.truthy(not ok and tostring(err):find("already set") ~= nil, tostring(err))
	end)

	h:test("VectorArray3i and IndexArray3i", function()
		local a = VA.VectorArray3i.new(2)
		a:Set(0, 1, 2, 3)
		a:Set(1, 1, 2, 3, true)
		h.eq(a:Get(0), G3.Vector3i.new(1, 2, 3))
		h.eq(a:Get(1), G3.Vector3i.new(1, 3, 2))
		a:Set(1, G3.Vector3i.new(4, 5, 6))
		local vs = collect(a:AsVector3i())
		h.eq(#vs, 2)
		h.eq(vs[2], G3.Vector3i.new(4, 5, 6))

		local t = VA.IndexArray3i.new(2)
		t:Set(0, { 0, 1, 2 })
		t:Set(1, 3, 4, 5, true)
		local t1 = t:Get(1)
		h.eq(t1[1], 3)
		h.eq(t1[2], 5)
		h.eq(t1[3], 4)
		local ts = collect(t:AsIndex3i())
		h.eq(#ts, 2)
		h.eq(ts[1][3], 2)

		local u = VA.IndexArray3i.FromArray({ 9, 9, 9, 9, 9, 9 })
		u:Set(1, 1, t)
		h.eq(u:Get(1)[3], 2)
		h.eq(u:Get(0)[1], 9)
	end)

	h:test("VectorArray2d and IndexArray2i", function()
		local a = VA.VectorArray2d.new(2)
		a:Set(0, Vector2d.new(1, 2))
		a:Set(1, 3, 4)
		h.eq(a:Count(), 2)
		h.eq(a:Get(1), Vector2d.new(3, 4))
		local vs = collect(a:AsVector2())
		h.eq(#vs, 2)
		h.eq(vs[1], Vector2d.new(1, 2))
		h.eq(a:AsArray()[2], Vector2d.new(3, 4))

		local b = VA.VectorArray2d.new(3)
		b:Set(2, 1, a)
		h.eq(b:Get(2), Vector2d.new(1, 2))

		local t = VA.IndexArray2i.new(2)
		t:Set(0, { 5, 6 })
		t:Set(1, 7, 8)
		h.eq(t:Get(0)[2], 6)
		local ts = collect(t:AsIndex2i())
		h.eq(#ts, 2)
		h.eq(ts[2][1], 7)

		local g = VA.VectorArray2.new(1)
		g:Set(0, 1, 2)
		h.eq(g.array[2], 2)
		h.eq(#collect(g:GetEnumerator()), 2)
	end)

	h:test("VectorArray4 and IndexArray4i", function()
		local t = VA.IndexArray4i.new(2)
		t:Set(0, { 1, 2, 3, 4 })
		t:Set(1, 5, 6, 7, 8)
		h.eq(t:Count(), 2)
		local q = t:Get(0)
		h.eq(q[4], 4)
		local ts = collect(t:AsIndex4i())
		h.eq(#ts, 2)
		h.eq(ts[2][4], 8)

		local g = VA.VectorArray4.new(3)
		g:Set(1, 2, t)
		g:Set(0, 9, 9, 9, 9)
		h.eq(g:Count(), 3)
		h.eq(g.array[1], 9)
		h.eq(g.array[5], 1)
		h.eq(g.array[12], 8)
		h.eq(#collect(g:GetEnumerator()), 12)
	end)

	h:test("DCurve3.SetVertices(VectorArray3d)", function()
		local a = VA.VectorArray3d.FromArray({ 0, 0, 0, 1, 0, 0, 1, 1, 0 })
		local c = G3.DCurve3.new()
		c:SetVertices(a)
		h.eq(c:VertexCount(), 3)
		h.eq(c:Get(2), Vector3d.new(1, 1, 0))
	end)

	return h:results()
end
