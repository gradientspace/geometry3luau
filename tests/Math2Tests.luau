-- Tests for the 2D math types: AxisAlignedBox2d, Matrix2d, Box2d, and the 2D BoundsUtil overloads

return function(G3, h)
	local Vector2d = G3.Vector2d
	local AxisAlignedBox2d = G3.AxisAlignedBox2d
	local Matrix2d = G3.Matrix2d
	local Box2d = G3.Box2d
	local V2 = Vector2d.new

	local function vnear(a, b, eps, msg)
		h.near(a.x, b.x, eps, (msg or "") .. ".x")
		h.near(a.y, b.y, eps, (msg or "") .. ".y")
	end
	local function boxeq(box, xmin, ymin, xmax, ymax, msg)
		vnear(box.Min, V2(xmin, ymin), 1e-12, (msg or "box") .. " Min")
		vnear(box.Max, V2(xmax, ymax), 1e-12, (msg or "box") .. " Max")
	end
	local function mnear(m, m00, m01, m10, m11, eps, msg)
		h.near(m.m00, m00, eps, (msg or "m") .. ".m00")
		h.near(m.m01, m01, eps, (msg or "m") .. ".m01")
		h.near(m.m10, m10, eps, (msg or "m") .. ".m10")
		h.near(m.m11, m11, eps, (msg or "m") .. ".m11")
	end

	------------------------------------------------------------------ AxisAlignedBox2d

	h:test("AxisAlignedBox2d constructors and properties", function()
		local b = AxisAlignedBox2d.new(0, 0, 2, 1)
		h.eq(b:Width(), 2)
		h.eq(b:Height(), 1)
		h.eq(b:Area(), 2)
		h.near(b:DiagonalLength(), math.sqrt(5))
		h.eq(b:MaxDim(), 2)
		h.eq(b:MinDim(), 1)
		h.eq(b:Center(), V2(1, 0.5))
		h.eq(b:Extents(), V2(1, 0.5))
		h.eq(b:Diagonal(), V2(2, 1))
		h.truthy(b:IsValid() and not b:IsEmpty())
		h.eq(AxisAlignedBox2d.new(-3, 1, 2, 2):MaxUnsignedCoordinate(), 3)

		boxeq(AxisAlignedBox2d.FromMinMax(V2(2, 0), V2(0, 1)), 0, 0, 2, 1, "FromMinMax sorts")
		boxeq(AxisAlignedBox2d.FromSquareSize(3), 0, 0, 3, 3)
		boxeq(AxisAlignedBox2d.FromSize(3, 4), 0, 0, 3, 4)
		boxeq(AxisAlignedBox2d.FromCenterHalfSizes(V2(1, 1), 2, 3), -1, -2, 3, 4)
		boxeq(AxisAlignedBox2d.FromCenterHalfSize(V2(1, 1), 2), -1, -1, 3, 3)
		boxeq(AxisAlignedBox2d.FromPoint(V2(5, 6)), 5, 6, 5, 6)
		boxeq(AxisAlignedBox2d.Zero(), 0, 0, 0, 0)
		boxeq(AxisAlignedBox2d.UnitPositive(), 0, 0, 1, 1)
		h.truthy(AxisAlignedBox2d.Empty():IsEmpty(), "Empty is empty")
		h.truthy(AxisAlignedBox2d.Infinite():Contains(V2(1e300, -1e300)), "Infinite contains")

		-- corners: 0 == bottom-left, 1 = bottom-right, 2 == top-right, 3 == top-left
		h.eq(b:GetCorner(0), V2(0, 0))
		h.eq(b:GetCorner(1), V2(2, 0))
		h.eq(b:GetCorner(2), V2(2, 1))
		h.eq(b:GetCorner(3), V2(0, 1))
		h.eq(b:SampleT(0.5, 0.25), V2(1, 0.25))

		-- Clone is a copy
		local c = b:Clone()
		c:Expand(1)
		boxeq(c, -1, -1, 3, 2, "Expand")
		boxeq(b, 0, 0, 2, 1, "original unchanged")
		c:Contract(1)
		boxeq(c, 0, 0, 2, 1, "Contract")
		c:Add(-1, 1, -2, 2)
		boxeq(c, -1, -2, 3, 3, "Add")
		c:Pad(1, -1, 2, -2)
		boxeq(c, 0, 0, 2, 1, "Pad")
		h.eq(tostring(b), "X:[0.00000000,2.00000000] Y:[0.00000000,1.00000000]")
	end)

	h:test("AxisAlignedBox2d contain / contains / intersect", function()
		local e = AxisAlignedBox2d.Empty()
		e:Contain(V2(1, 2))
		boxeq(e, 1, 2, 1, 2, "Contain point into empty")
		e:Contain(V2(-1, 0))
		boxeq(e, -1, 0, 1, 2)

		local p = AxisAlignedBox2d.Empty()
		p:Contain({ V2(1, 2), V2(-1, 0), V2(3, 1) })
		boxeq(p, -1, 0, 3, 2, "Contain(IList)")

		local b = AxisAlignedBox2d.new(0, 0, 2, 1)
		local cb = b:Clone()
		cb:Contain(AxisAlignedBox2d.new(1, -1, 3, 0.5))
		boxeq(cb, 0, -1, 3, 1, "Contain(box)")
		boxeq(b:CombineWith(AxisAlignedBox2d.new(1, -1, 3, 0.5)), 0, -1, 3, 1, "CombineWith")
		boxeq(b, 0, 0, 2, 1, "CombineWith does not modify")

		-- Contains uses strict inequalities
		h.truthy(b:Contains(V2(1, 0.5)))
		h.truthy(not b:Contains(V2(0, 0.5)), "boundary point is not contained")
		h.truthy(b:Contains(AxisAlignedBox2d.new(0.5, 0.25, 1, 0.75)))
		h.truthy(not b:Contains(AxisAlignedBox2d.new(0.5, 0.25, 2, 0.75)))

		-- touching boxes intersect, but not exclusively
		local t = AxisAlignedBox2d.new(2, 0, 3, 1)
		h.truthy(b:Intersects(t))
		h.truthy(not b:IntersectsExclusive(t))
		h.truthy(b:IntersectsExclusive(AxisAlignedBox2d.new(1.5, 0.5, 3, 2)))
		h.truthy(not b:Intersects(AxisAlignedBox2d.new(2.5, 0, 3, 1)))

		boxeq(b:Intersect(AxisAlignedBox2d.new(1, 0.5, 3, 3)), 1, 0.5, 2, 1, "Intersect")
		h.truthy(b:Intersect(AxisAlignedBox2d.new(5, 5, 6, 6)):IsEmpty(), "disjoint Intersect is Empty")
	end)

	h:test("AxisAlignedBox2d distance and transforms", function()
		local b = AxisAlignedBox2d.new(0, 0, 2, 1)
		h.eq(b:Distance(V2(1, 0.5)), 0)
		h.near(b:Distance(V2(3, 0.5)), 1)
		h.near(b:Distance(V2(1, -2)), 2)
		h.near(b:Distance(V2(3, 2)), math.sqrt(2))

		boxeq(b:Translated(1, 2), 1, 2, 3, 3, "Translated(dx, dy)")
		boxeq(b:Translated(V2(-1, 1)), -1, 1, 1, 2, "Translated(v)")
		local c = b:Clone()
		c:Translate(V2(1, 1))
		boxeq(c, 1, 1, 3, 2, "Translate")

		c = b:Clone()
		c:Scale(2)
		boxeq(c, 0, 0, 4, 2, "Scale")
		c = b:Clone()
		c:Scale(2, V2(1, 0.5))
		boxeq(c, -1, -0.5, 3, 1.5, "Scale about origin")

		c = b:Clone()
		c:MoveMin(10, 10)
		boxeq(c, 10, 10, 12, 11, "MoveMin(x, y)")
		c:MoveMin(V2(0, 0))
		boxeq(c, 0, 0, 2, 1, "MoveMin(v)")

		local SM = AxisAlignedBox2d.ScaleMode
		c = b:Clone()
		c:SetWidth(4, SM.ScaleCenter)
		boxeq(c, -1, 0, 3, 1, "SetWidth center")
		c:SetWidth(1, SM.ScaleRight)
		boxeq(c, -1, 0, 0, 1, "SetWidth right")
		c:SetWidth(3, SM.ScaleLeft)
		boxeq(c, -3, 0, 0, 1, "SetWidth left")
		c:SetHeight(3, SM.ScaleUp)
		boxeq(c, -3, 0, 0, 3, "SetHeight up")
		c:SetHeight(1, SM.ScaleDown)
		boxeq(c, -3, 2, 0, 3, "SetHeight down")
		c:SetHeight(3, SM.ScaleCenter)
		boxeq(c, -3, 1, 0, 4, "SetHeight center")
		h.truthy(not pcall(function()
			c:SetWidth(1, SM.ScaleUp)
		end), "invalid scale mode errors")
	end)

	------------------------------------------------------------------ Matrix2d

	h:test("Matrix2d basics, inverse, determinant", function()
		local m = Matrix2d.new(4, 1, 2, 3)
		h.eq(m:Determinant(), 10)
		h.eq(m:Get(0, 0), 4)
		h.eq(m:Get(0, 1), 1)
		h.eq(m:Get(1, 0), 2)
		h.eq(m:Get(1, 1), 3)
		h.eq(m:Row(0), V2(4, 1))
		h.eq(m:Row(1), V2(2, 3))
		h.eq(m:Column(0), V2(4, 2))
		h.eq(m:Column(1), V2(1, 3))
		mnear(m:Transpose(), 4, 2, 1, 3, 0, "Transpose")
		mnear(m:Adjoint(), 3, -1, -2, 4, 0, "Adjoint")

		local inv = m:Inverse()
		mnear(inv, 0.3, -0.1, -0.2, 0.4, 1e-15, "Inverse")
		for _, v in { V2(1, 0), V2(0, 1), V2(-2.5, 7) } do
			vnear(m * (inv * v), v, 1e-12, "M * Minv * v")
		end
		h.eq(Matrix2d.new(1, 2, 2, 4):Inverse(), Matrix2d.Zero, "singular -> Zero")
		h.eq(Matrix2d.new(1, 0, 0, 1e-3):Inverse(0.01), Matrix2d.Zero, "Inverse(epsilon)")

		-- m * v (row*vector) and v * m (vector*column)
		h.eq(m * V2(1, 2), V2(6, 8))
		h.eq(m:Multiply(V2(1, 2)), V2(6, 8))
		h.eq(Matrix2d.VectorMultiply(V2(1, 2), m), V2(8, 7))
		-- u^T M v
		h.eq(m:QForm(V2(1, 1), V2(1, 2)), 14)

		-- operators
		mnear(-m, -4, -1, -2, -3, 0, "unm")
		mnear(m + m, 8, 2, 4, 6, 0, "m+m")
		mnear(m + 1, 5, 2, 3, 4, 0, "m+f")
		mnear(m - Matrix2d.One, 3, 0, 1, 2, 0, "m-m")
		mnear(m - 1, 3, 0, 1, 2, 0, "m-f")
		mnear(m * 2, 8, 2, 4, 6, 0, "m*f")
		mnear(2 * m, 8, 2, 4, 6, 0, "f*m")
		mnear(m / 2, 2, 0.5, 1, 1.5, 0, "m/f")

		-- constructors
		mnear(Matrix2d.FromIdentity(true), 1, 0, 0, 1, 0)
		mnear(Matrix2d.FromIdentity(false), 0, 0, 0, 0, 0)
		mnear(Matrix2d.Diagonal(2, 3), 2, 0, 0, 3, 0)
		mnear(Matrix2d.FromVectors(V2(1, 2), V2(3, 4), true), 1, 3, 2, 4, 0, "columns")
		mnear(Matrix2d.FromVectors(V2(1, 2), V2(3, 4), false), 1, 2, 3, 4, 0, "rows")
		mnear(Matrix2d.OuterProduct(V2(1, 2), V2(3, 4)), 3, 4, 6, 8, 0, "outer product")

		-- mutation
		local c = m:Clone()
		c:SetToDiagonal(5, 6)
		mnear(c, 5, 0, 0, 6, 0, "SetToDiagonal")
		mnear(m, 4, 1, 2, 3, 0, "Clone is a copy")
		h.truthy(not pcall(function()
			Matrix2d.Identity:SetToDiagonal(2, 2)
		end), "static constants are frozen")
	end)

	h:test("Matrix2d rotation", function()
		local r = Matrix2d.FromAngle(90, true)
		vnear(r * V2(1, 0), V2(0, 1), 1e-15, "CCW rotation")
		h.near(r:ExtractAngle(), math.pi / 2, 1e-15)
		local r2 = Matrix2d.FromAngle(math.pi / 6)
		h.near(r2:ExtractAngle(), math.pi / 6, 1e-15)
		h.near(r2:Determinant(), 1, 1e-15)
		mnear(r2:Inverse(), r2.m00, r2.m10, r2.m01, r2.m11, 1e-15, "inverse == transpose")
		local s = Matrix2d.new(0, 0, 0, 0)
		s:SetToRotationDeg(-45)
		vnear(s * V2(1, 1), V2(math.sqrt(2), 0), 1e-15, "SetToRotationDeg")
	end)

	h:test("Matrix2d orthonormalize and eigen decomposition", function()
		local m = Matrix2d.new(2, 1, 0, 1)
		m:Orthonormalize()
		mnear(m, 1, 0, 0, 1, 1e-15, "Orthonormalize")
		local m2 = Matrix2d.new(1, 2, 1, 0)
		m2:Orthonormalize()
		h.near(m2:Column(0):Dot(m2:Column(1)), 0, 1e-15, "orthogonal columns")
		h.near(m2:Column(0):Length(), 1, 1e-15)
		h.near(m2:Column(1):Length(), 1, 1e-15)

		-- symmetric [[2,1],[1,2]]: eigenvalues 1, 3
		local a = Matrix2d.new(2, 1, 1, 2)
		local rot, diag = a:EigenDecomposition()
		mnear(diag, 1, 0, 0, 3, 1e-14, "eigenvalues")
		local s = 1 / math.sqrt(2)
		mnear(rot, s, s, -s, s, 1e-15, "eigenvectors")
		-- columns of rot are eigenvectors: A * col_i == lambda_i * col_i
		vnear(a * rot:Column(0), 1 * rot:Column(0), 1e-14, "eigvec0")
		vnear(a * rot:Column(1), 3 * rot:Column(1), 1e-14, "eigvec1")

		-- diff < 0 branch
		local b = Matrix2d.new(1, 2, 2, 5)
		local rotb, diagb = b:EigenDecomposition()
		for i = 0, 1 do
			local lam = diagb:Get(i, i)
			vnear(b * rotb:Column(i), lam * rotb:Column(i), 1e-12, "eigvec b" .. i)
		end
		h.truthy(diagb.m00 <= diagb.m11, "sorted eigenvalues")

		-- already diagonal; fills the passed-in matrices
		local R, D = Matrix2d.new(0, 0, 0, 0), Matrix2d.new(9, 9, 9, 9)
		local R2, D2 = Matrix2d.Diagonal(2, 5):EigenDecomposition(R, D)
		h.eq(R2, R)
		h.eq(D2, D)
		mnear(R, 1, 0, 0, 1, 0, "diag rot")
		mnear(D, 2, 0, 0, 5, 0, "diag diag")
	end)

	------------------------------------------------------------------ Box2d

	h:test("Box2d constructors and properties", function()
		local b = Box2d.FromCenterExtent(V2(0, 0), V2(2, 1))
		h.eq(b:Area(), 8)
		h.eq(b:MaxExtent(), 2)
		h.eq(b:MinExtent(), 1)
		h.eq(b:Diagonal(), V2(4, 2))
		h.eq(b:Axis(0), Vector2d.AxisX)
		h.eq(b:Axis(1), Vector2d.AxisY)
		local v = b:ComputeVertices()
		h.eq(#v, 4)
		h.eq(v[1], V2(-2, -1))
		h.eq(v[2], V2(2, -1))
		h.eq(v[3], V2(2, 1))
		h.eq(v[4], V2(-2, 1))

		local p = Box2d.new(V2(1, 1))
		h.eq(p.Center, V2(1, 1))
		h.eq(p.Extent, Vector2d.Zero)
		h.eq(Box2d.Empty().Center, Vector2d.Zero)
		local d = Box2d.new()
		h.eq(d.AxisX, Vector2d.Zero)
		local c = Box2d.new(V2(1, 2), V2(0, 1), V2(-1, 0), V2(3, 4))
		h.eq(c.AxisX, V2(0, 1))
		h.eq(c.Extent, V2(3, 4))
		h.eq(Box2d.new(V2(1, 2), V2(3, 4)).Extent, V2(3, 4))

		local fa = Box2d.new(AxisAlignedBox2d.new(0, 0, 4, 2))
		h.eq(fa.Center, V2(2, 1))
		h.eq(fa.Extent, V2(2, 1))

		local seg = G3.Segment2d.new(V2(0, 0), V2(4, 0))
		local fs = Box2d.new(seg)
		h.eq(fs.Center, V2(2, 0))
		h.eq(fs.AxisX, V2(1, 0))
		h.eq(fs.AxisY, V2(0, -1))
		h.eq(fs.Extent, V2(2, 0))

		local cl = b:Clone()
		cl:Expand(1)
		h.eq(cl.Extent, V2(3, 2))
		h.eq(b.Extent, V2(2, 1), "Clone is a copy")
		cl:Translate(V2(1, 1))
		h.eq(cl.Center, V2(1, 1))
	end)

	h:test("Box2d contains / distance / closest point / rotation", function()
		local b = Box2d.FromCenterExtent(V2(0, 0), V2(2, 1))
		h.truthy(b:Contains(V2(2, 1)), "boundary is contained")
		h.truthy(b:Contains(V2(-1, 0.5)))
		h.truthy(not b:Contains(V2(2.1, 0)))
		h.eq(b:DistanceSquared(V2(1, 0)), 0)
		h.near(b:DistanceSquared(V2(4, 0)), 4)
		h.near(b:DistanceSquared(V2(3, 2)), 2)
		h.eq(b:ClosestPoint(V2(3, 2)), V2(2, 1))
		h.eq(b:ClosestPoint(V2(0.5, -0.5)), V2(0.5, -0.5))
		h.eq(b:ClosestPoint(V2(-5, 0)), V2(-2, 0))

		-- rotate axes by 90 degrees: AxisX -> +y, AxisY -> -x
		local r = b:Clone()
		r:RotateAxes(Matrix2d.FromAngle(90, true))
		vnear(r.AxisX, V2(0, 1), 1e-15)
		vnear(r.AxisY, V2(-1, 0), 1e-15)
		h.truthy(r:Contains(V2(0.5, 1.9)))
		h.truthy(not r:Contains(V2(1.9, 0.5)))
		h.near(r:DistanceSquared(V2(0, 3)), 1, 1e-12)
		vnear(r:ClosestPoint(V2(3, 3)), V2(1, 2), 1e-12, "rotated ClosestPoint")
	end)

	h:test("Box2d contain and merge", function()
		local b = Box2d.FromCenterExtent(V2(0, 0), V2(1, 1))
		b:Contain(V2(3, 0))
		h.eq(b.Center, V2(1, 0))
		h.eq(b.Extent, V2(2, 1))
		h.truthy(b:Contains(V2(3, 0)) and b:Contains(V2(-1, 0)))
		b:Contain(V2(0, -3))
		h.eq(b.Center, V2(1, -1))
		h.eq(b.Extent, V2(2, 2))

		local c = Box2d.FromCenterExtent(V2(0, 0), V2(1, 1))
		c:Contain(Box2d.FromCenterExtent(V2(5, 5), V2(1, 1)))
		for _, v in Box2d.FromCenterExtent(V2(5, 5), V2(1, 1)):ComputeVertices() do
			h.truthy(c:Contains(v), "Contain(box) contains its vertices")
		end

		local b0 = Box2d.FromCenterExtent(V2(0, 0), V2(1, 1))
		local b1 = Box2d.FromCenterExtent(V2(4, 0), V2(1, 1))
		local m = Box2d.Merge(b0, b1)
		vnear(m.Center, V2(2, 0), 1e-15, "merge center")
		vnear(m.AxisX, V2(1, 0), 1e-15)
		vnear(m.AxisY, V2(0, 1), 1e-15)
		vnear(m.Extent, V2(3, 1), 1e-15, "merge extent")

		-- rotated input boxes: merged box contains every input vertex
		local r1 = Box2d.FromCenterExtent(V2(1, 2), V2(2, 0.5))
		r1:RotateAxes(Matrix2d.FromAngle(30, true))
		local r2 = Box2d.FromCenterExtent(V2(-2, 1), V2(1, 1.5))
		r2:RotateAxes(Matrix2d.FromAngle(-20, true))
		local mr = Box2d.Merge(r1, r2)
		h.near(mr.AxisX:Dot(mr.AxisY), 0, 1e-12, "merged axes orthogonal")
		for _, src in { r1, r2 } do
			for _, v in src:ComputeVertices() do
				h.truthy(mr:DistanceSquared(v) < 1e-18, "merged contains vertex")
			end
		end
	end)

	------------------------------------------------------------------ BoundsUtil 2D

	h:test("BoundsUtil 2D overloads", function()
		local BoundsUtil = G3.BoundsUtil
		local a, b, c = V2(1, 5), V2(3, -1), V2(0, 2)
		boxeq(BoundsUtil.BoundsVectors2(a, b, c), 0, -1, 3, 5, "BoundsVectors2")
		local bb = BoundsUtil.Bounds(a, b, c)
		h.eq(getmetatable(bb), AxisAlignedBox2d)
		boxeq(bb, 0, -1, 3, 5, "Bounds(v0, v1, v2)")
		boxeq(BoundsUtil.Bounds({ a, b, c, V2(-4, 0) }), -4, -1, 3, 5, "Bounds(array of Vector2d)")
		local pts = { a, b, c }
		local i = 0
		local itr = function()
			i += 1
			return pts[i]
		end
		boxeq(BoundsUtil.BoundsPositions2(itr), 0, -1, 3, 5, "BoundsPositions2(iterator)")
		-- 3D overloads still dispatch to 3D
		local V3 = G3.Vector3d.new
		h.eq(getmetatable(BoundsUtil.Bounds(V3(0, 0, 0), V3(1, 1, 1), V3(2, 0, 1))), G3.AxisAlignedBox3d)
		h.eq(getmetatable(BoundsUtil.Bounds({ V3(0, 0, 0), V3(1, 1, 1) })), G3.AxisAlignedBox3d)
	end)

	return h:results()
end
