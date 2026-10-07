--!native
-- Port of geometry3Sharp mesh/DMesh3.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- DMesh3 is a dynamic triangle mesh class. The mesh has has connectivity,
-- is an indexed mesh, and allows for gaps in the index space.
--
-- internally, all data is stored in POD-type buffers, except for the vertex->edge
-- links, which are stored in a SmallListSet. The arrays of POD data are stored in DVector's.
--
-- Reference counts for verts/tris/edges are stored as separate RefCountVector instances.
--
-- Vertices are stored as doubles. Per-vertex Vertex Normals, Colors, and UVs are optional.
--
-- For each vertex, vertex_edges[i] is the unordered list of connected edges. The
-- elements of the list are indices into the edges list.
-- This list is unsorted but can be traversed in-order (ie cw/ccw) at some additional cost.
--
-- Triangles are stored as 3 ints, with optionally a per-triangle integer group id.
--
-- The edges of a triangle are similarly stored as 3 ints, in triangle_edes. If the
-- triangle is [v1,v2,v3], then the triangle edges [e1,e2,e3] are
-- e1=edge(v1,v2), e2=edge(v2,v3), e3=edge(v3,v1), where the e# are indexes into edges.
--
-- Edges are stored as tuples of 4 ints. If the edge is between v1 and v2, with neighbour
-- tris t1 and t2, then the edge is [min(v1,v2), max(v1,v2), t1, t2]. For a boundary
-- edge, t2 is InvalidID. t1 is never InvalidID.
--
-- Most of the class assumes that the mesh is manifold. Many functions will
-- work if the topology is non-manifold, but behavior of operators like Split/Flip/Collapse
-- edge is untested.
--
-- The function CheckValidity() does extensive sanity checking on the mesh data structure.
-- Use this to test your code, both for mesh construction and editing!!
--
-- ===================================================================================================
-- Luau port notes (see PORTING.md):
--   * vertex/triangle/edge ids are 0-based, InvalidID == -1
--   * Index2i/Index3i/Index4i results are Luau arrays {a,b(,c(,d))}
--   * C# properties are methods: mesh:VertexCount(), mesh:MaxVertexID(), mesh:CachedBounds(), ...
--   * Constructors:
--       DMesh3.new(bWantNormals=true, bWantColors=false, bWantUVs=false, bWantTriGroups=false)
--       DMesh3.FromComponents(flags)                          (MeshComponents flags)
--       DMesh3.FromCopy(copy, bCompact=false, bWantNormals=true, bWantColors=true, bWantUVs=true)
--       DMesh3.FromCopyComponents(copy, bCompact, flags)
--       DMesh3.FromIMesh(copy, hints, bWantNormals=true, bWantColors=true, bWantUVs=true)
--   * Overloads with distinct names:
--       GetVertexInfo(vID, bWantNormals, bWantColors, bWantUVs) -> ok, NewVertexInfo   (C# GetVertex(vID, ref vinfo, ...))
--       GetTriVertices(tID) -> v0, v1, v2;  GetTriVerticesTriangle(tID) -> Triangle3d
--       GetTriBaryPointInfo(tID, b0, b1, b2) -> NewVertexInfo                          (C# GetTriBaryPoint(..., out vinfo))
--       GetEdgeVPositions(eID) -> ok, Vector3d a, Vector3d b                           (C# GetEdgeV(eID, ref a, ref b))
--       GetEdge4(eID) -> ok, a, b, t0, t1                                               (C# GetEdge(eID, ref a, ref b, ref t0, ref t1))
--       SplitEdge(eab, split_t=0.5) / SplitEdgeVerts(vA, vB) -> MeshResult, EdgeSplitInfo
--       FlipEdge(eab) / FlipEdgeVerts(vA, vB) -> MeshResult, EdgeFlipInfo
--       CopyIMesh(copy, hints, ...)                                                     (C# Copy(IMesh, hints, ...))
--   * AppendVertex accepts a Vector3d, a NewVertexInfo table, or (DMesh3 from, int fromVID).
--   * AppendTriangle accepts (v0, v1, v2, gid?) or (Index3i, gid?).

local Util = require(script.Parent.Util)
local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local Vector2d = require(script.Parent.Vector2d)
local AxisAlignedBox3d = require(script.Parent.AxisAlignedBox3d)
local Frame3d = require(script.Parent.Frame3d)
local Triangle3d = require(script.Parent.Triangle3d)
local DVector = require(script.Parent.DVector)
local RefCountVector = require(script.Parent.RefCountVector)
local SmallListSet = require(script.Parent.SmallListSet)
local IndexUtil = require(script.Parent.IndexUtil)
local IndexMap = require(script.Parent.IndexMap)
local MeshResult = require(script.Parent.MeshResult)
local NewVertexInfo = require(script.Parent.NewVertexInfo)
local DMesh3Attributes = require(script.Parent.DMesh3Attributes)

local V3 = Vector3d.new
local sqrt, atan2 = math.sqrt, math.atan2

local InvalidID = -1
local NonManifoldID = -2
local DegenerateID = -3

-- SmallListSet internals, used for fast iteration in hot paths
local SLS_Null = -1
local SLS_BLOCKSIZE = SmallListSet.BLOCKSIZE
local SLS_BLOCK_LIST_OFFSET = SmallListSet.BLOCK_LIST_OFFSET

local DMesh3 = {}
DMesh3.__index = DMesh3
DMesh3.ClassName = "DMesh3"

DMesh3.InvalidID = InvalidID
DMesh3.NonManifoldID = NonManifoldID
DMesh3.DegenerateID = DegenerateID
DMesh3.InvalidVertex = Vector3d.new(MathUtil.MaxDouble, 0, 0)

-- static readonly Index3i InvalidTriangle / Index2i InvalidEdge (functions return fresh arrays)
function DMesh3.InvalidTriangle(): { number }
	return { InvalidID, InvalidID, InvalidID }
end
function DMesh3.InvalidEdge(): { number }
	return { InvalidID, InvalidID }
end

-- [Flags] enum MeshComponents
DMesh3.MeshComponents = table.freeze({
	None = 0,
	VertexNormals = 1,
	VertexColors = 2,
	VertexUVs = 4,
	FaceGroups = 8,
	All = 15,
})
local MeshComponents = DMesh3.MeshComponents

-- [Flags] enum MeshHints
DMesh3.MeshHints = table.freeze({
	None = 0,
	IsCompact = 1,
})

local function def(b: boolean?, d: boolean): boolean
	if b == nil then
		return d
	end
	return b
end

local function hasflag(flags: number, f: number): boolean
	return bit32.band(flags, f) ~= 0
end

local function init_empty(self)
	self.vertices = DVector.new()
	self.normals = nil
	self.uv = nil
	self.vertex_edges = SmallListSet.new()
	self.vertices_refcount = RefCountVector.new()
	self.triangles = DVector.new()
	self.triangle_edges = DVector.new()
	self.triangles_refcount = RefCountVector.new()
	self.triangle_groups = nil
	self.max_group_id = 0
	self.edges = DVector.new()
	self.edges_refcount = RefCountVector.new()
	self.timestamp = 0
	self.shape_timestamp = 0
	self.attributes = nil
	self.Metadata = nil
	self.cached_bounds = nil
	self.cached_bounds_timestamp = -1
	self.cached_is_closed = false
	self.cached_is_closed_timestamp = -1
end

function DMesh3.new(bWantNormals: boolean?, bWantColors: boolean?, bWantUVs: boolean?, bWantTriGroups: boolean?)
	local self = setmetatable({}, DMesh3)
	init_empty(self)
	if def(bWantNormals, true) then
		self.normals = DVector.new()
	end
	if def(bWantColors, false) then
		self:EnableVertexColors(Vector3d.Zero)
	end
	if def(bWantUVs, false) then
		self.uv = DVector.new()
	end
	if def(bWantTriGroups, false) then
		self.triangle_groups = DVector.new()
	end
	self.max_group_id = 0
	return self
end

function DMesh3.FromComponents(flags: number)
	return DMesh3.new(
		hasflag(flags, MeshComponents.VertexNormals),
		hasflag(flags, MeshComponents.VertexColors),
		hasflag(flags, MeshComponents.VertexUVs),
		hasflag(flags, MeshComponents.FaceGroups)
	)
end

-- normals/colors/uvs will only be copied if they exist
function DMesh3.FromCopy(copy, bCompact: boolean?, bWantNormals: boolean?, bWantColors: boolean?, bWantUVs: boolean?)
	local self = setmetatable({}, DMesh3)
	init_empty(self)
	if bCompact then
		self:CompactCopy(copy, def(bWantNormals, true), def(bWantColors, true), def(bWantUVs, true))
	else
		self:Copy(copy, def(bWantNormals, true), def(bWantColors, true), def(bWantUVs, true))
	end
	return self
end

function DMesh3.FromCopyComponents(copy, bCompact: boolean, flags: number)
	return DMesh3.FromCopy(
		copy,
		bCompact,
		hasflag(flags, MeshComponents.VertexNormals),
		hasflag(flags, MeshComponents.VertexColors),
		hasflag(flags, MeshComponents.VertexUVs)
	)
end

function DMesh3.FromIMesh(copy, hints: number?, bWantNormals: boolean?, bWantColors: boolean?, bWantUVs: boolean?)
	local self = setmetatable({}, DMesh3)
	init_empty(self)
	self:CopyIMesh(copy, hints, def(bWantNormals, true), def(bWantColors, true), def(bWantUVs, true))
	return self
end

function DMesh3.IsDMesh3(m: any): boolean
	return getmetatable(m) == DMesh3
end

-- struct CompactInfo { IIndexMap MapV }
function DMesh3:CompactCopy(copy, bNormals: boolean?, bColors: boolean?, bUVs: boolean?)
	bNormals, bColors, bUVs = def(bNormals, true), def(bColors, true), def(bUVs, true)
	if copy:IsCompact() then
		self:Copy(copy, bNormals, bColors, bUVs)
		return { MapV = IndexMap.IdentityIndexMap.new() }
	end

	self:DisableAttributes()
	self.vertices = DVector.new()
	self.vertex_edges = SmallListSet.new()
	self.vertices_refcount = RefCountVector.new()
	self.triangles = DVector.new()
	self.triangle_edges = DVector.new()
	self.triangles_refcount = RefCountVector.new()
	self.edges = DVector.new()
	self.edges_refcount = RefCountVector.new()
	self.max_group_id = 0

	self.normals = (bNormals and copy.normals ~= nil) and DVector.new() or nil
	self.uv = (bUVs and copy.uv ~= nil) and DVector.new() or nil
	self.triangle_groups = (copy.triangle_groups ~= nil) and DVector.new() or nil

	-- [TODO] if we knew some of these were dense we could copy directly...

	local bBuildInvMapV = (copy:IsCompactV() == false) and copy:HasAttributes()
	local mapNewToOldV = bBuildInvMapV and table.create(copy:VertexCount(), 0) or nil

	local mapV = table.create(copy:MaxVertexID(), 0) -- mapV[vid+1] = new vid
	for vid in copy.vertices_refcount:Indices() do
		local _, vinfo = copy:GetVertexInfo(vid, bNormals, bColors, bUVs)
		local new_vid = self:AppendVertex(vinfo)
		mapV[vid + 1] = new_vid
		if mapNewToOldV ~= nil then
			mapNewToOldV[new_vid + 1] = vid
		end
	end

	local bBuildInvMapT = (copy:IsCompactT() == false) and copy:HasAttributes()
	local mapNewToOldT = bBuildInvMapT and table.create(copy:TriangleCount(), 0) or nil

	-- [TODO] would be much faster to explicitly copy triangle & edge data structures!!
	for tid in copy.triangles_refcount:Indices() do
		local t = copy:GetTriangle(tid)
		t[1], t[2], t[3] = mapV[t[1] + 1], mapV[t[2] + 1], mapV[t[3] + 1]
		local g = copy:HasTriangleGroups() and copy:GetTriangleGroup(tid) or InvalidID
		local new_tid = self:AppendTriangle(t, g)
		if mapNewToOldT ~= nil then
			mapNewToOldT[new_tid + 1] = tid
		end
		self.max_group_id = math.max(self.max_group_id, g + 1)
	end

	if copy:HasAttributes() then
		self:EnableAttributes(true)
		self.attributes:Copy(copy.attributes, mapNewToOldV, mapNewToOldT)
	end

	return { MapV = IndexMap.FromDense(mapV, self:MaxVertexID()) }
end

function DMesh3:Copy(copy, bNormals: boolean?, bColors: boolean?, bUVs: boolean?)
	bNormals, bColors, bUVs = def(bNormals, true), def(bColors, true), def(bUVs, true)
	self.vertices = DVector.Copy(copy.vertices)

	self.normals = (bNormals and copy.normals ~= nil) and DVector.Copy(copy.normals) or nil
	self.uv = (bUVs and copy.uv ~= nil) and DVector.Copy(copy.uv) or nil

	self.vertices_refcount = RefCountVector.Copy(copy.vertices_refcount)

	self.vertex_edges = SmallListSet.Copy(copy.vertex_edges)

	self.triangles = DVector.Copy(copy.triangles)
	self.triangle_edges = DVector.Copy(copy.triangle_edges)
	self.triangles_refcount = RefCountVector.Copy(copy.triangles_refcount)
	if copy.triangle_groups ~= nil then
		self.triangle_groups = DVector.Copy(copy.triangle_groups)
	end
	self.max_group_id = copy.max_group_id

	self.edges = DVector.Copy(copy.edges)
	self.edges_refcount = RefCountVector.Copy(copy.edges_refcount)

	if copy:HasAttributes() then
		self:EnableAttributes(true)
		self.attributes:Copy(copy.attributes, nil, nil)
	end
end

-- Copy IMesh into this mesh. Currently always compacts.
-- IMesh is duck-typed: VertexIndices(), TriangleIndices(), GetVertexAll(vid), GetTriangle(tid),
-- MaxVertexID(), HasVertexNormals(), HasVertexUVs(), HasVertexColors(), HasTriangleGroups(), GetTriangleGroup(tid)
function DMesh3:CopyIMesh(copy, hints: number?, bNormals: boolean?, bColors: boolean?, bUVs: boolean?)
	bNormals, bColors, bUVs = def(bNormals, true), def(bColors, true), def(bUVs, true)
	if DMesh3.IsDMesh3(copy) then
		return self:CompactCopy(copy, bNormals, bColors, bUVs)
	end

	self.vertices = DVector.new()
	self.vertex_edges = SmallListSet.new()
	self.vertices_refcount = RefCountVector.new()
	self.triangles = DVector.new()
	self.triangle_edges = DVector.new()
	self.triangles_refcount = RefCountVector.new()
	self.edges = DVector.new()
	self.edges_refcount = RefCountVector.new()
	self.max_group_id = 0

	self.normals = (bNormals and copy:HasVertexNormals()) and DVector.new() or nil
	self.uv = (bUVs and copy:HasVertexUVs()) and DVector.new() or nil
	self.triangle_groups = copy:HasTriangleGroups() and DVector.new() or nil

	if copy:HasVertexColors() then
		self:EnableVertexColors()
	end

	local mapV = table.create(copy:MaxVertexID(), 0)
	for vid in copy:VertexIndices() do
		local vinfo = copy:GetVertexAll(vid)
		mapV[vid + 1] = self:AppendVertex(vinfo)
	end

	for tid in copy:TriangleIndices() do
		local t = table.clone(copy:GetTriangle(tid))
		t[1], t[2], t[3] = mapV[t[1] + 1], mapV[t[2] + 1], mapV[t[3] + 1]
		local g = copy:HasTriangleGroups() and copy:GetTriangleGroup(tid) or InvalidID
		self:AppendTriangle(t, g)
		self.max_group_id = math.max(self.max_group_id, g + 1)
	end

	return { MapV = IndexMap.FromDense(mapV, self:MaxVertexID()) }
end

function DMesh3:updateTimeStamp(bShapeChange: boolean)
	self.timestamp += 1
	if bShapeChange then
		self.shape_timestamp += 1
	end
end

-- Timestamp is incremented any time any change is made to the mesh
function DMesh3:Timestamp(): number
	return self.timestamp
end

-- ShapeTimestamp is incremented any time any vertex position is changed or the mesh topology is modified
function DMesh3:ShapeTimestamp(): number
	return self.shape_timestamp
end

-- IMesh impl

function DMesh3:VertexCount(): number
	return self.vertices_refcount.used_count
end
function DMesh3:TriangleCount(): number
	return self.triangles_refcount.used_count
end
function DMesh3:EdgeCount(): number
	return self.edges_refcount.used_count
end

-- these values are (max_used+1), ie so an iteration should be < MaxTriangleID, not <=
function DMesh3:MaxVertexID(): number
	return self.vertices_refcount.ref_counts.size
end
function DMesh3:MaxTriangleID(): number
	return self.triangles_refcount.ref_counts.size
end
function DMesh3:MaxEdgeID(): number
	return self.edges_refcount.ref_counts.size
end
function DMesh3:MaxGroupID(): number
	return self.max_group_id
end

function DMesh3:HasVertexColors(): boolean
	return self.attributes ~= nil and self.attributes:HasVertexColor()
end
function DMesh3:HasVertexNormals(): boolean
	return self.normals ~= nil
end
function DMesh3:HasVertexUVs(): boolean
	return self.uv ~= nil
end
function DMesh3:HasTriangleGroups(): boolean
	return self.triangle_groups ~= nil
end
function DMesh3:HasAttributes(): boolean
	return self.attributes ~= nil
end

function DMesh3:Components(): number
	local c = 0
	if self.normals ~= nil then
		c = bit32.bor(c, MeshComponents.VertexNormals)
	end
	if self:Attribs():HasVertexColor() then
		c = bit32.bor(c, MeshComponents.VertexColors)
	end
	if self.uv ~= nil then
		c = bit32.bor(c, MeshComponents.VertexUVs)
	end
	if self.triangle_groups ~= nil then
		c = bit32.bor(c, MeshComponents.FaceGroups)
	end
	return c
end

-- info

function DMesh3:IsVertex(vID: number): boolean
	return self.vertices_refcount:isValid(vID)
end
function DMesh3:IsTriangle(tID: number): boolean
	return self.triangles_refcount:isValid(tID)
end
function DMesh3:IsEdge(eID: number): boolean
	return self.edges_refcount:isValid(eID)
end

-- [Conditional("DEBUG")] checks
function DMesh3:debug_check_is_vertex(v: number)
	if Util.DebugChecks and not self:IsVertex(v) then
		error("DMesh3.debug_is_vertex - not a vertex!", 2)
	end
end
function DMesh3:debug_check_is_triangle(t: number)
	if Util.DebugChecks and not self:IsTriangle(t) then
		error("DMesh3.debug_is_triangle - not a triangle!", 2)
	end
end
function DMesh3:debug_check_is_edge(e: number)
	if Util.DebugChecks and not self:IsEdge(e) then
		error("DMesh3.debug_is_edge - not an edge!", 2)
	end
end

-- getters

function DMesh3:GetVertex(vID: number)
	if Util.DebugChecks then
		self:debug_check_is_vertex(vID)
	end
	local d = self.vertices.data
	local i = 3 * vID
	return V3(d[i + 1], d[i + 2], d[i + 3])
end
DMesh3.GetVertexf = DMesh3.GetVertex

-- returns x, y, z of vertex position without allocating a Vector3d
function DMesh3:GetVertexXYZ(vID: number): (number, number, number)
	local d = self.vertices.data
	local i = 3 * vID
	return d[i + 1], d[i + 2], d[i + 3]
end

function DMesh3:SetVertex(vID: number, vNewPos)
	Util.gDevAssert(vNewPos:IsFinite()) -- this will really catch a lot of bugs...
	if Util.DebugChecks then
		self:debug_check_is_vertex(vID)
	end
	local d = self.vertices.data
	local i = 3 * vID
	d[i + 1], d[i + 2], d[i + 3] = vNewPos.x, vNewPos.y, vNewPos.z
	self:updateTimeStamp(true)
end

-- set vertex position from scalars (avoids allocating a Vector3d)
function DMesh3:SetVertexXYZ(vID: number, x: number, y: number, z: number)
	local d = self.vertices.data
	local i = 3 * vID
	d[i + 1], d[i + 2], d[i + 3] = x, y, z
	self:updateTimeStamp(true)
end

function DMesh3:GetVertexNormal(vID: number)
	if self.normals == nil then
		return Vector3d.AxisY
	end
	if Util.DebugChecks then
		self:debug_check_is_vertex(vID)
	end
	local d = self.normals.data
	local i = 3 * vID
	return V3(d[i + 1], d[i + 2], d[i + 3])
end

function DMesh3:SetVertexNormal(vID: number, vNewNormal)
	if self.normals ~= nil then
		if Util.DebugChecks then
			self:debug_check_is_vertex(vID)
		end
		local d = self.normals.data
		local i = 3 * vID
		d[i + 1], d[i + 2], d[i + 3] = vNewNormal.x, vNewNormal.y, vNewNormal.z
		self:updateTimeStamp(false)
	end
end

function DMesh3:GetVertexColor(vID: number)
	if self.attributes ~= nil and self.attributes:HasVertexColor() then
		return self:Attribs():VertexColor():GetValue(vID)
	end
	return Vector3d.One
end

function DMesh3:SetVertexColor(vID: number, vNewColor)
	if self:HasVertexColors() then
		if Util.DebugChecks then
			self:debug_check_is_vertex(vID)
		end
		self:Attribs():VertexColor():SetValue(vID, vNewColor)
		self:updateTimeStamp(false)
	end
end

function DMesh3:GetVertexUV(vID: number)
	if self.uv == nil then
		return Vector2d.Zero
	end
	if Util.DebugChecks then
		self:debug_check_is_vertex(vID)
	end
	local d = self.uv.data
	local i = 2 * vID
	return Vector2d.new(d[i + 1], d[i + 2])
end

function DMesh3:SetVertexUV(vID: number, vNewUV)
	if self.uv ~= nil then
		if Util.DebugChecks then
			self:debug_check_is_vertex(vID)
		end
		local d = self.uv.data
		local i = 2 * vID
		d[i + 1], d[i + 2] = vNewUV.x, vNewUV.y
		self:updateTimeStamp(false)
	end
end

-- C# bool GetVertex(int vID, ref NewVertexInfo vinfo, bool bWantNormals, bool bWantColors, bool bWantUVs)
-- -> returns ok, vinfo
function DMesh3:GetVertexInfo(vID: number, bWantNormals: boolean, bWantColors: boolean, bWantUVs: boolean)
	if not self.vertices_refcount:isValid(vID) then
		return false, nil
	end
	local d = self.vertices.data
	local vinfo = NewVertexInfo.new(V3(d[3 * vID + 1], d[3 * vID + 2], d[3 * vID + 3]))
	vinfo.bHaveN, vinfo.bHaveUV, vinfo.bHaveC = false, false, false
	if self:HasVertexNormals() and bWantNormals then
		vinfo.bHaveN = true
		local n = self.normals.data
		vinfo.n = V3(n[3 * vID + 1], n[3 * vID + 2], n[3 * vID + 3])
	end
	if self:HasVertexColors() and bWantColors then
		vinfo.bHaveC = true
		vinfo.c = self:Attribs():VertexColor():GetValue(vID)
	end
	if self:HasVertexUVs() and bWantUVs then
		vinfo.bHaveUV = true
		local u = self.uv.data
		vinfo.uv = Vector2d.new(u[2 * vID + 1], u[2 * vID + 2])
	end
	return true, vinfo
end

function DMesh3:GetVtxEdgeCount(vID: number): number
	return self.vertices_refcount:isValid(vID) and self.vertex_edges:Count(vID) or -1
end

function DMesh3:GetMaxVtxEdgeCount(): number
	local max = 0
	for vid in self.vertices_refcount:Indices() do
		max = math.max(max, self.vertex_edges:Count(vid))
	end
	return max
end

function DMesh3:GetVertexAll(i: number)
	local vi = NewVertexInfo.new(self:GetVertex(i))
	if self:HasVertexNormals() then
		vi.bHaveN = true
		vi.n = self:GetVertexNormal(i)
	else
		vi.bHaveN = false
	end
	if self:HasVertexColors() then
		vi.bHaveC = true
		vi.c = self:GetVertexColor(i)
	else
		vi.bHaveC = false
	end
	if self:HasVertexUVs() then
		vi.bHaveUV = true
		vi.uv = self:GetVertexUV(i)
	else
		vi.bHaveUV = false
	end
	return vi
end

-- Compute a normal/tangent frame at vertex that is "stable" as long as
-- the mesh topology doesn't change, meaning that one axis of the frame
-- will be computed from projection of outgoing edge.
-- Requires that vertex normals are available.
-- by default, frame.Z is normal, and .X points along mesh edge
-- if bFrameNormalY, then frame.Y is normal (X still points along mesh edge)
function DMesh3:GetVertexFrame(vID: number, bFrameNormalY: boolean?)
	Util.gDevAssert(self:HasVertexNormals())
	local d, n = self.vertices.data, self.normals.data
	local vi = 3 * vID
	local v = V3(d[vi + 1], d[vi + 2], d[vi + 3])
	local normal = V3(n[vi + 1], n[vi + 2], n[vi + 3])
	local eid = self.vertex_edges:First(vID)
	local ovi = 3 * self:edge_other_v(eid, vID)
	local ov = V3(d[ovi + 1], d[ovi + 2], d[ovi + 3])
	local edge = (ov - v):Normalized()

	local other = normal:Cross(edge)
	edge = other:Cross(normal)
	if bFrameNormalY then
		return Frame3d.FromXYZ(v, edge, normal, -other)
	end
	return Frame3d.FromXYZ(v, edge, other, normal)
end

function DMesh3:GetTriangle(tID: number): { number }
	if Util.DebugChecks then
		self:debug_check_is_triangle(tID)
	end
	local t = self.triangles.data
	local i = 3 * tID
	return { t[i + 1], t[i + 2], t[i + 3] }
end

-- returns a, b, c without allocating
function DMesh3:GetTriangleABC(tID: number): (number, number, number)
	local t = self.triangles.data
	local i = 3 * tID
	return t[i + 1], t[i + 2], t[i + 3]
end

function DMesh3:GetTriEdges(tID: number): { number }
	if Util.DebugChecks then
		self:debug_check_is_triangle(tID)
	end
	local t = self.triangle_edges.data
	local i = 3 * tID
	return { t[i + 1], t[i + 2], t[i + 3] }
end

function DMesh3:GetTriEdge(tid: number, j: number): number
	if Util.DebugChecks then
		self:debug_check_is_triangle(tid)
	end
	return self.triangle_edges.data[3 * tid + j + 1]
end

function DMesh3:GetTriNeighbourTris(tID: number): { number }
	if self.triangles_refcount:isValid(tID) then
		local te, ed = self.triangle_edges.data, self.edges.data
		local tei = 3 * tID
		local nbr_t = { 0, 0, 0 }
		for j = 0, 2 do
			local ei = 4 * te[tei + j + 1]
			nbr_t[j + 1] = (ed[ei + 3] == tID) and ed[ei + 4] or ed[ei + 3]
		end
		return nbr_t
	end
	return DMesh3.InvalidTriangle()
end

function DMesh3:TriTrianglesItr(tID: number): () -> number?
	if not self.triangles_refcount:isValid(tID) then
		return function()
			return nil
		end
	end
	local te, ed = self.triangle_edges.data, self.edges.data
	local tei = 3 * tID
	local j = 0
	return function()
		while j < 3 do
			local ei = 4 * te[tei + j + 1]
			j += 1
			local nbr_t = (ed[ei + 3] == tID) and ed[ei + 4] or ed[ei + 3]
			if nbr_t ~= InvalidID then
				return nbr_t
			end
		end
		return nil
	end
end

function DMesh3:GetTriangleGroup(tID: number): number
	if self.triangle_groups == nil then
		return -1
	end
	return self.triangles_refcount:isValid(tID) and self.triangle_groups.data[tID + 1] or 0
end

function DMesh3:SetTriangleGroup(tid: number, group_id: number)
	if self.triangle_groups ~= nil then
		if Util.DebugChecks then
			self:debug_check_is_triangle(tid)
		end
		self.triangle_groups.data[tid + 1] = group_id
		self.max_group_id = math.max(self.max_group_id, group_id + 1)
		self:updateTimeStamp(false)
	end
end

function DMesh3:AllocateTriangleGroup(): number
	self.max_group_id += 1
	return self.max_group_id
end

-- C# GetTriVertices(int tID, ref Vector3d v0, ref Vector3d v1, ref Vector3d v2) -> returns v0, v1, v2
function DMesh3:GetTriVertices(tID: number)
	local t, d = self.triangles.data, self.vertices.data
	local ti = 3 * tID
	local ai, bi, ci = 3 * t[ti + 1], 3 * t[ti + 2], 3 * t[ti + 3]
	return V3(d[ai + 1], d[ai + 2], d[ai + 3]), V3(d[bi + 1], d[bi + 2], d[bi + 3]), V3(d[ci + 1], d[ci + 2], d[ci + 3])
end

-- C# GetTriVertices(int tID, out Triangle3d Tri)
function DMesh3:GetTriVerticesTriangle(tID: number)
	local v0, v1, v2 = self:GetTriVertices(tID)
	return Triangle3d.new(v0, v1, v2)
end

-- returns the 9 coordinates of the triangle vertices without allocating
function DMesh3:GetTriVerticesXYZ(tID: number): (number, number, number, number, number, number, number, number, number)
	local t, d = self.triangles.data, self.vertices.data
	local ti = 3 * tID
	local ai, bi, ci = 3 * t[ti + 1], 3 * t[ti + 2], 3 * t[ti + 3]
	return d[ai + 1], d[ai + 2], d[ai + 3], d[bi + 1], d[bi + 2], d[bi + 3], d[ci + 1], d[ci + 2], d[ci + 3]
end

function DMesh3:GetTriVertex(tid: number, j: number)
	local a = self.triangles.data[3 * tid + j + 1]
	local d = self.vertices.data
	return V3(d[3 * a + 1], d[3 * a + 2], d[3 * a + 3])
end

function DMesh3:GetTriBaryPoint(tID: number, bary0: number, bary1: number, bary2: number)
	local t, d = self.triangles.data, self.vertices.data
	local ai, bi, ci = 3 * t[3 * tID + 1], 3 * t[3 * tID + 2], 3 * t[3 * tID + 3]
	return V3(
		(bary0 * d[ai + 1] + bary1 * d[bi + 1] + bary2 * d[ci + 1]),
		(bary0 * d[ai + 2] + bary1 * d[bi + 2] + bary2 * d[ci + 2]),
		(bary0 * d[ai + 3] + bary1 * d[bi + 3] + bary2 * d[ci + 3])
	)
end

function DMesh3:GetTriNormal(tID: number)
	local v0, v1, v2 = self:GetTriVertices(tID)
	return MathUtil.Normal(v0, v1, v2)
end

function DMesh3:GetTriArea(tID: number): number
	local v0, v1, v2 = self:GetTriVertices(tID)
	return MathUtil.Area(v0, v1, v2)
end

-- Compute triangle normal, area, and centroid all at once. Re-uses vertex
-- lookups and computes normal & area simultaneously. *However* does not produce
-- the same normal/area as separate calls, because of this.
-- C# GetTriInfo(int tID, out Vector3d normal, out double fArea, out Vector3d vCentroid) -> returns normal, fArea, vCentroid
function DMesh3:GetTriInfo(tID: number)
	local v0, v1, v2 = self:GetTriVertices(tID)
	local vCentroid = (1.0 / 3.0) * (v0 + v1 + v2)
	local normal, fArea = MathUtil.FastNormalArea(v0, v1, v2)
	return normal, fArea, vCentroid
end

-- interpolate vertex normals of triangle using barycentric coordinates
function DMesh3:GetTriBaryNormal(tID: number, bary0: number, bary1: number, bary2: number)
	local t, n = self.triangles.data, self.normals.data
	local ai, bi, ci = 3 * t[3 * tID + 1], 3 * t[3 * tID + 2], 3 * t[3 * tID + 3]
	local nv = V3(
		(bary0 * n[ai + 1] + bary1 * n[bi + 1] + bary2 * n[ci + 1]),
		(bary0 * n[ai + 2] + bary1 * n[bi + 2] + bary2 * n[ci + 2]),
		(bary0 * n[ai + 3] + bary1 * n[bi + 3] + bary2 * n[ci + 3])
	)
	return (nv:NormalizedAndLength())
end

-- efficiently compute centroid of triangle
function DMesh3:GetTriCentroid(tID: number)
	local t, d = self.triangles.data, self.vertices.data
	local ai, bi, ci = 3 * t[3 * tID + 1], 3 * t[3 * tID + 2], 3 * t[3 * tID + 3]
	local f = (1.0 / 3.0)
	return V3((d[ai + 1] + d[bi + 1] + d[ci + 1]) * f, (d[ai + 2] + d[bi + 2] + d[ci + 2]) * f, (d[ai + 3] + d[bi + 3] + d[ci + 3]) * f)
end

-- Compute interpolated vertex attributes at point of triangle
-- C# GetTriBaryPoint(int tID, double bary0, double bary1, double bary2, out NewVertexInfo vinfo)
function DMesh3:GetTriBaryPointInfo(tID: number, bary0: number, bary1: number, bary2: number)
	local vinfo = NewVertexInfo.new()
	local t, d = self.triangles.data, self.vertices.data
	local ta, tb, tc = t[3 * tID + 1], t[3 * tID + 2], t[3 * tID + 3]
	local ai, bi, ci = 3 * ta, 3 * tb, 3 * tc
	vinfo.v = V3(
		(bary0 * d[ai + 1] + bary1 * d[bi + 1] + bary2 * d[ci + 1]),
		(bary0 * d[ai + 2] + bary1 * d[bi + 2] + bary2 * d[ci + 2]),
		(bary0 * d[ai + 3] + bary1 * d[bi + 3] + bary2 * d[ci + 3])
	)
	vinfo.bHaveN = self:HasVertexNormals()
	if vinfo.bHaveN then
		local n = self.normals.data
		vinfo.n = V3(
			(bary0 * n[ai + 1] + bary1 * n[bi + 1] + bary2 * n[ci + 1]),
			(bary0 * n[ai + 2] + bary1 * n[bi + 2] + bary2 * n[ci + 2]),
			(bary0 * n[ai + 3] + bary1 * n[bi + 3] + bary2 * n[ci + 3])
		):NormalizedAndLength()
	end
	vinfo.bHaveC = self:HasVertexColors()
	if vinfo.bHaveC then
		-- NOTE: C# passes the *buffer offsets* (3*vid) here, which looks like a bug; we pass vertex ids.
		vinfo.c = self:Attribs():VertexColor():GetBaryValueIdx(ta, tb, tc, V3(bary0, bary1, bary2))
	end
	vinfo.bHaveUV = self:HasVertexUVs()
	if vinfo.bHaveUV then
		local u = self.uv.data
		ai, bi, ci = 2 * ta, 2 * tb, 2 * tc
		vinfo.uv = Vector2d.new(
			(bary0 * u[ai + 1] + bary1 * u[bi + 1] + bary2 * u[ci + 1]),
			(bary0 * u[ai + 2] + bary1 * u[bi + 2] + bary2 * u[ci + 2])
		)
	end
	return vinfo
end

-- construct bounding box of triangle as efficiently as possible
function DMesh3:GetTriBounds(tID: number)
	local t, d = self.triangles.data, self.vertices.data
	local vi = 3 * t[3 * tID + 1]
	local x, y, z = d[vi + 1], d[vi + 2], d[vi + 3]
	local minx, maxx, miny, maxy, minz, maxz = x, x, y, y, z, z
	for i = 1, 2 do
		vi = 3 * t[3 * tID + i + 1]
		x, y, z = d[vi + 1], d[vi + 2], d[vi + 3]
		if x < minx then
			minx = x
		elseif x > maxx then
			maxx = x
		end
		if y < miny then
			miny = y
		elseif y > maxy then
			maxy = y
		end
		if z < minz then
			minz = z
		elseif z > maxz then
			maxz = z
		end
	end
	return AxisAlignedBox3d.new(minx, miny, minz, maxx, maxy, maxz)
end

-- Construct stable frame at triangle centroid, where frame.Z is face normal,
-- and frame.X is aligned with edge nEdge of triangle.
function DMesh3:GetTriFrame(tID: number, nEdge: number?)
	local e = nEdge or 0
	local t, d = self.triangles.data, self.vertices.data
	local ti = 3 * tID
	local a = 3 * t[ti + (e % 3) + 1]
	local b = 3 * t[ti + ((e + 1) % 3) + 1]
	local c = 3 * t[ti + ((e + 2) % 3) + 1]
	local v1 = V3(d[a + 1], d[a + 2], d[a + 3])
	local v2 = V3(d[b + 1], d[b + 2], d[b + 3])
	local v3 = V3(d[c + 1], d[c + 2], d[c + 3])

	local edge1 = (v2 - v1):Normalized()
	local edge2 = (v3 - v2):Normalized()
	local normal = edge1:Cross(edge2):Normalized()

	local other = normal:Cross(edge1)

	local center = (v1 + v2 + v3) / 3
	return Frame3d.FromXYZ(center, edge1, other, normal)
end

-- compute solid angle of oriented triangle tID relative to point p - see WindingNumber()
function DMesh3:GetTriSolidAngle(tID: number, p): number
	local t, d = self.triangles.data, self.vertices.data
	local ti = 3 * tID
	local ta = 3 * t[ti + 1]
	local ax, ay, az = d[ta + 1] - p.x, d[ta + 2] - p.y, d[ta + 3] - p.z
	local tb = 3 * t[ti + 2]
	local bx, by, bz = d[tb + 1] - p.x, d[tb + 2] - p.y, d[tb + 3] - p.z
	local tc = 3 * t[ti + 3]
	local cx, cy, cz = d[tc + 1] - p.x, d[tc + 2] - p.y, d[tc + 3] - p.z
	-- note: top and bottom are reversed here from formula in the paper? but it doesn't work otherwise...
	local la = sqrt(ax * ax + ay * ay + az * az)
	local lb = sqrt(bx * bx + by * by + bz * bz)
	local lc = sqrt(cx * cx + cy * cy + cz * cz)
	local bottom = (la * lb * lc)
		+ (ax * bx + ay * by + az * bz) * lc
		+ (bx * cx + by * cy + bz * cz) * la
		+ (cx * ax + cy * ay + cz * az) * lb
	local top = ax * (by * cz - cy * bz) - ay * (bx * cz - cx * bz) + az * (bx * cy - cx * by)
	return 2.0 * atan2(top, bottom)
end

-- compute internal angle at vertex i of triangle (where i is 0,1,2);
-- TODO can be more efficient here, probably...
function DMesh3:GetTriInternalAngleR(tID: number, i: number): number
	local a, b, c = self:GetTriVertices(tID)
	if i == 0 then
		return (b - a):Normalized():AngleR((c - a):Normalized())
	elseif i == 1 then
		return (a - b):Normalized():AngleR((c - b):Normalized())
	end
	return (a - c):Normalized():AngleR((b - c):Normalized())
end

function DMesh3:GetEdgeV(eID: number): { number }
	if Util.DebugChecks then
		self:debug_check_is_edge(eID)
	end
	local e = self.edges.data
	local i = 4 * eID
	return { e[i + 1], e[i + 2] }
end

-- C# bool GetEdgeV(int eID, ref Vector3d a, ref Vector3d b) -> returns true, a, b
function DMesh3:GetEdgeVPositions(eID: number)
	if Util.DebugChecks then
		self:debug_check_is_edge(eID)
	end
	local e, d = self.edges.data, self.vertices.data
	local iv0 = 3 * e[4 * eID + 1]
	local iv1 = 3 * e[4 * eID + 2]
	return true, V3(d[iv0 + 1], d[iv0 + 2], d[iv0 + 3]), V3(d[iv1 + 1], d[iv1 + 2], d[iv1 + 3])
end

function DMesh3:GetEdgeT(eID: number): { number }
	if Util.DebugChecks then
		self:debug_check_is_edge(eID)
	end
	local e = self.edges.data
	local i = 4 * eID
	return { e[i + 3], e[i + 4] }
end

-- return [v0,v1,t0,t1], or Index4i.Max if eid is invalid
function DMesh3:GetEdge(eID: number): { number }
	if Util.DebugChecks then
		self:debug_check_is_edge(eID)
	end
	local e = self.edges.data
	local i = 4 * eID
	return { e[i + 1], e[i + 2], e[i + 3], e[i + 4] }
end

-- C# bool GetEdge(int eID, ref int a, ref int b, ref int t0, ref int t1) -> returns true, a, b, t0, t1
function DMesh3:GetEdge4(eID: number): (boolean, number, number, number, number)
	if Util.DebugChecks then
		self:debug_check_is_edge(eID)
	end
	local e = self.edges.data
	local i = 4 * eID
	return true, e[i + 1], e[i + 2], e[i + 3], e[i + 4]
end

-- return same indices as GetEdgeV, but oriented based on attached triangle
function DMesh3:GetOrientedBoundaryEdgeV(eID: number): { number }
	if self.edges_refcount:isValid(eID) then
		local e = self.edges.data
		local ei = 4 * eID
		if e[ei + 4] == InvalidID then
			local a, b = e[ei + 1], e[ei + 2]
			local ti = 3 * e[ei + 3]
			local t = self.triangles.data
			local tri = { t[ti + 1], t[ti + 2], t[ti + 3] }
			local ai = IndexUtil.find_edge_index_in_tri(a, b, tri)
			return { tri[ai + 1], tri[((ai + 1) % 3) + 1] }
		end
	end
	Util.gDevAssert(false)
	return DMesh3.InvalidEdge()
end

-- average of 1 or 2 face normals
function DMesh3:GetEdgeNormal(eID: number)
	if self.edges_refcount:isValid(eID) then
		local e = self.edges.data
		local ei = 4 * eID
		local n = self:GetTriNormal(e[ei + 3])
		if e[ei + 4] ~= InvalidID then
			n = n + self:GetTriNormal(e[ei + 4])
			n = n:NormalizedAndLength()
		end
		return n
	end
	Util.gDevAssert(false)
	return Vector3d.Zero
end

function DMesh3:GetEdgePoint(eID: number, t: number)
	if self.edges_refcount:isValid(eID) then
		local e, d = self.edges.data, self.vertices.data
		local ei = 4 * eID
		local iv0 = 3 * e[ei + 1]
		local iv1 = 3 * e[ei + 2]
		local mt = 1.0 - t
		return V3(mt * d[iv0 + 1] + t * d[iv1 + 1], mt * d[iv0 + 2] + t * d[iv1 + 2], mt * d[iv0 + 3] + t * d[iv1 + 3])
	end
	Util.gDevAssert(false)
	return Vector3d.Zero
end

---------------------------------------------------------------------------------------------------
-- mesh-building

-- AppendVertex(Vector3d v) / AppendVertex(NewVertexInfo info) / AppendVertex(DMesh3 from, int fromVID)
-- returns new vid
function DMesh3:AppendVertex(info: any, fromVID: number?): number
	if fromVID ~= nil then
		return self:AppendVertexFromMesh(info, fromVID)
	end
	local v, n, c, uvv, bHaveN, bHaveC, bHaveUV
	if info.v ~= nil then
		v, n, c, uvv = info.v, info.n, info.c, info.uv
		bHaveN, bHaveC, bHaveUV = info.bHaveN, info.bHaveC, info.bHaveUV
	else
		v = info
		bHaveN, bHaveC, bHaveUV = false, false, false
	end

	local vid = self.vertices_refcount:allocate()
	local i = 3 * vid
	local verts = self.vertices
	verts:insert(v.z, i + 2)
	verts:insert(v.y, i + 1)
	verts:insert(v.x, i)
	self:on_new_vertex(vid)

	if self.normals ~= nil then
		local nn = bHaveN and n or Vector3d.AxisY
		local normals = self.normals
		normals:insert(nn.z, i + 2)
		normals:insert(nn.y, i + 1)
		normals:insert(nn.x, i)
	end

	if self:HasVertexColors() and bHaveC then
		self:Attribs():VertexColor():SetValue(vid, c)
	end

	if self.uv ~= nil then
		local u = bHaveUV and uvv or Vector2d.Zero
		local j = 2 * vid
		self.uv:insert(u.y, j + 1)
		self.uv:insert(u.x, j)
	end

	self:allocate_edges_list(vid)

	self:updateTimeStamp(true)
	return vid
end

-- append vertex at position given as scalars (fast path, no normal/uv/color info)
function DMesh3:AppendVertexXYZ(x: number, y: number, z: number): number
	local vid = self.vertices_refcount:allocate()
	local i = 3 * vid
	local verts = self.vertices
	verts:insert(z, i + 2)
	verts:insert(y, i + 1)
	verts:insert(x, i)
	self:on_new_vertex(vid)
	if self.normals ~= nil then
		local normals = self.normals
		normals:insert(0, i + 2)
		normals:insert(1, i + 1)
		normals:insert(0, i)
	end
	if self.uv ~= nil then
		local j = 2 * vid
		self.uv:insert(0, j + 1)
		self.uv:insert(0, j)
	end
	self:allocate_edges_list(vid)
	self:updateTimeStamp(true)
	return vid
end

-- copy vertex fromVID from existing source mesh, returns new vid
function DMesh3:AppendVertexFromMesh(from, fromVID: number): number
	local bi = 3 * fromVID
	local fv = from.vertices.data

	local vid = self.vertices_refcount:allocate()
	local i = 3 * vid
	local verts = self.vertices
	verts:insert(fv[bi + 3], i + 2)
	verts:insert(fv[bi + 2], i + 1)
	verts:insert(fv[bi + 1], i)
	self:on_new_vertex(vid)

	if self.normals ~= nil then
		local normals = self.normals
		if from.normals ~= nil then
			local fn = from.normals.data
			normals:insert(fn[bi + 3], i + 2)
			normals:insert(fn[bi + 2], i + 1)
			normals:insert(fn[bi + 1], i)
		else
			normals:insert(0, i + 2)
			normals:insert(1, i + 1) -- y-up
			normals:insert(0, i)
		end
	end

	if self:HasVertexColors() and from:HasVertexColors() then
		local setColor = from:Attribs():VertexColor():GetValue(fromVID)
		self:Attribs():VertexColor():SetValue(vid, setColor)
	end

	if self.uv ~= nil then
		local j = 2 * vid
		if from.uv ~= nil then
			local bj = 2 * fromVID
			local fu = from.uv.data
			self.uv:insert(fu[bj + 2], j + 1)
			self.uv:insert(fu[bj + 1], j)
		else
			self.uv:insert(0, j + 1)
			self.uv:insert(0, j)
		end
	end

	self:allocate_edges_list(vid)

	self:updateTimeStamp(true)
	return vid
end

-- insert vertex at given index, assuming it is unused
-- If bUnsafe, we use fast id allocation that does not update free list.
-- You should only be using this between BeginUnsafeVerticesInsert() / EndUnsafeVerticesInsert() calls
function DMesh3:InsertVertex(vid: number, info, bUnsafe: boolean?): number
	if self.vertices_refcount:isValid(vid) then
		return MeshResult.Failed_VertexAlreadyExists
	end

	local bOK
	if bUnsafe then
		bOK = self.vertices_refcount:allocate_at_unsafe(vid)
	else
		bOK = self.vertices_refcount:allocate_at(vid)
	end
	if not bOK then
		return MeshResult.Failed_CannotAllocateVertex
	end

	local i = 3 * vid
	local verts = self.vertices
	verts:insert(info.v.z, i + 2)
	verts:insert(info.v.y, i + 1)
	verts:insert(info.v.x, i)
	self:on_new_vertex(vid)

	if self.normals ~= nil then
		local n = info.bHaveN and info.n or Vector3d.AxisY
		self.normals:insert(n.z, i + 2)
		self.normals:insert(n.y, i + 1)
		self.normals:insert(n.x, i)
	end

	if self:HasVertexColors() and info.bHaveC then
		self:Attribs():VertexColor():InsertValue(vid, info.c)
	end

	if self.uv ~= nil then
		local u = info.bHaveUV and info.uv or Vector2d.Zero
		local j = 2 * vid
		self.uv:insert(u.y, j + 1)
		self.uv:insert(u.x, j)
	end

	self:allocate_edges_list(vid)

	self:updateTimeStamp(true)
	return MeshResult.Ok
end

function DMesh3:BeginUnsafeVerticesInsert()
	-- do nothing...
end
function DMesh3:EndUnsafeVerticesInsert()
	self.vertices_refcount:rebuild_free_list()
end

-- helper fn for AppendTriangle/InsertTriangle, just makes code cleaner
function DMesh3:add_tri_edge(tid: number, v0: number, v1: number, j: number, eid: number)
	if eid ~= InvalidID then
		self.edges.data[4 * eid + 4] = tid
		self.triangle_edges:insert(eid, 3 * tid + j)
	else
		self.triangle_edges:insert(self:add_edge(v0, v1, tid), 3 * tid + j)
	end
end

-- AppendTriangle(v0, v1, v2, gid = -1) or AppendTriangle(Index3i tv, gid = -1)
function DMesh3:AppendTriangle(a: any, b: number?, c: number?, gid: number?): number
	local v0, v1, v2
	if type(a) == "table" then
		v0, v1, v2 = a[1], a[2], a[3]
		gid = b
	else
		v0, v1, v2 = a, b :: number, c :: number
	end
	local g = gid or -1

	local vrc = self.vertices_refcount
	if not vrc:isValid(v0) or not vrc:isValid(v1) or not vrc:isValid(v2) then
		return InvalidID
	end
	if v0 == v1 or v0 == v2 or v1 == v2 then
		return DegenerateID
	end

	-- look up edges. if any already have two triangles, this would
	-- create non-manifold geometry and so we do not allow it
	local e0 = self:find_edge(v0, v1)
	local e1 = self:find_edge(v1, v2)
	local e2 = self:find_edge(v2, v0)
	local ed = self.edges.data
	if (e0 ~= InvalidID and ed[4 * e0 + 4] ~= InvalidID) or (e1 ~= InvalidID and ed[4 * e1 + 4] ~= InvalidID) or (e2 ~= InvalidID and ed[4 * e2 + 4] ~= InvalidID) then
		return NonManifoldID
	end

	-- now safe to insert triangle
	local tid = self.triangles_refcount:allocate()
	local i = 3 * tid
	local tris = self.triangles
	tris:insert(v2, i + 2)
	tris:insert(v1, i + 1)
	tris:insert(v0, i)
	if self.triangle_groups ~= nil then
		self.triangle_groups:insert(g, tid)
		self.max_group_id = math.max(self.max_group_id, g + 1)
	end
	self:on_new_triangle(tid)

	-- increment ref counts and update/create edges
	local vrcd = vrc.ref_counts.data
	vrcd[v0 + 1] += 1
	vrcd[v1 + 1] += 1
	vrcd[v2 + 1] += 1

	self:add_tri_edge(tid, v0, v1, 0, e0)
	self:add_tri_edge(tid, v1, v2, 1, e1)
	self:add_tri_edge(tid, v2, v0, 2, e2)

	self:updateTimeStamp(true)
	return tid
end

-- Insert triangle at given index, assuming it is unused.
-- If bUnsafe, we use fast id allocation that does not update free list.
-- You should only be using this between BeginUnsafeTrianglesInsert() / EndUnsafeTrianglesInsert() calls
function DMesh3:InsertTriangle(tid: number, tv: { number }, gid: number?, bUnsafe: boolean?): number
	local g = gid or -1
	if self.triangles_refcount:isValid(tid) then
		return MeshResult.Failed_TriangleAlreadyExists
	end
	local v0, v1, v2 = tv[1], tv[2], tv[3]
	if not self:IsVertex(v0) or not self:IsVertex(v1) or not self:IsVertex(v2) then
		return MeshResult.Failed_NotAVertex
	end
	if v0 == v1 or v0 == v2 or v1 == v2 then
		return MeshResult.Failed_InvalidNeighbourhood
	end

	-- look up edges. if any already have two triangles, this would
	-- create non-manifold geometry and so we do not allow it
	local e0 = self:find_edge(v0, v1)
	local e1 = self:find_edge(v1, v2)
	local e2 = self:find_edge(v2, v0)
	if (e0 ~= InvalidID and not self:IsBoundaryEdge(e0)) or (e1 ~= InvalidID and not self:IsBoundaryEdge(e1)) or (e2 ~= InvalidID and not self:IsBoundaryEdge(e2)) then
		return MeshResult.Failed_WouldCreateNonmanifoldEdge
	end

	local bOK
	if bUnsafe then
		bOK = self.triangles_refcount:allocate_at_unsafe(tid)
	else
		bOK = self.triangles_refcount:allocate_at(tid)
	end
	if not bOK then
		return MeshResult.Failed_CannotAllocateTriangle
	end

	-- now safe to insert triangle
	local i = 3 * tid
	self.triangles:insert(v2, i + 2)
	self.triangles:insert(v1, i + 1)
	self.triangles:insert(v0, i)
	if self.triangle_groups ~= nil then
		self.triangle_groups:insert(g, tid)
		self.max_group_id = math.max(self.max_group_id, g + 1)
	end
	self:on_new_triangle(tid)

	-- increment ref counts and update/create edges
	self.vertices_refcount:increment(v0)
	self.vertices_refcount:increment(v1)
	self.vertices_refcount:increment(v2)

	self:add_tri_edge(tid, v0, v1, 0, e0)
	self:add_tri_edge(tid, v1, v2, 1, e1)
	self:add_tri_edge(tid, v2, v0, 2, e2)

	self:updateTimeStamp(true)
	return MeshResult.Ok
end

function DMesh3:BeginUnsafeTrianglesInsert()
	-- do nothing...
end
function DMesh3:EndUnsafeTrianglesInsert()
	self.triangles_refcount:rebuild_free_list()
end

function DMesh3:EnableVertexNormals(initial_normal)
	if self:HasVertexNormals() then
		return
	end
	self.normals = DVector.new()
	local NV = self:MaxVertexID()
	self.normals:resize(3 * NV)
	local n = self.normals.data
	for i = 0, NV - 1 do
		local vi = 3 * i
		n[vi + 1], n[vi + 2], n[vi + 3] = initial_normal.x, initial_normal.y, initial_normal.z
	end
end
function DMesh3:DiscardVertexNormals()
	self.normals = nil
end

function DMesh3:EnableVertexColors(initial_color)
	if not self:HasVertexColors() then
		self:Attribs():EnableVertexColor(initial_color or Vector3d.Zero)
	end
end
function DMesh3:DiscardVertexColors()
	if self.attributes ~= nil then
		self.attributes:DisableVertexColor()
	end
end

function DMesh3:EnableVertexUVs(initial_uv)
	if self:HasVertexUVs() then
		return
	end
	self.uv = DVector.new()
	local NV = self:MaxVertexID()
	self.uv:resize(2 * NV)
	local u = self.uv.data
	for i = 0, NV - 1 do
		local vi = 2 * i
		u[vi + 1], u[vi + 2] = initial_uv.x, initial_uv.y
	end
end
function DMesh3:DiscardVertexUVs()
	self.uv = nil
end

function DMesh3:EnableTriangleGroups(initial_group: number?)
	if self:HasTriangleGroups() then
		return
	end
	self.triangle_groups = DVector.new()
	local NT = self:MaxTriangleID()
	self.triangle_groups:resize(NT)
	local g = self.triangle_groups.data
	for i = 1, NT do
		g[i] = initial_group or 0
	end
	self.max_group_id = 0
end
function DMesh3:DiscardTriangleGroups()
	self.triangle_groups = nil
	self.max_group_id = 0
end

function DMesh3:EnableAttributes(bResetIfExists: boolean?)
	if self:HasAttributes() and not bResetIfExists then
		return
	end
	self.attributes = DMesh3Attributes.new(self)
end
function DMesh3:DisableAttributes()
	self.attributes = nil
end
function DMesh3:Attribs()
	if self.attributes == nil then
		self:EnableAttributes()
	end
	return self.attributes
end

function DMesh3:on_new_triangle(tid: number, ref_tid: number?)
	if self.attributes == nil then
		return
	end
	for attrib in self.attributes:TriAttributes() do
		attrib:InsertValue_Copy(tid, ref_tid or -1)
	end
end

function DMesh3:on_new_vertex(vid: number, ref_vid: number?)
	if self.attributes == nil then
		return
	end
	for attrib in self.attributes:VertAttributes() do
		attrib:InsertValue_Copy(vid, ref_vid or -1)
	end
end

---------------------------------------------------------------------------------------------------
-- iterators

function DMesh3:VertexIndices(): () -> number?
	return self.vertices_refcount:Indices()
end
function DMesh3:TriangleIndices(): () -> number?
	return self.triangles_refcount:Indices()
end
function DMesh3:EdgeIndices(): () -> number?
	return self.edges_refcount:Indices()
end

-- Enumerate ids of boundary edges
function DMesh3:BoundaryEdgeIndices(): () -> number?
	local itr = self.edges_refcount:Indices()
	local ed = self.edges.data
	return function()
		while true do
			local eid = itr()
			if eid == nil then
				return nil
			end
			if ed[4 * eid + 4] == InvalidID then
				return eid
			end
		end
	end
end

-- Enumerate vertices
function DMesh3:Vertices(): () -> any
	local itr = self.vertices_refcount:Indices()
	local d = self.vertices.data
	return function()
		local vid = itr()
		if vid == nil then
			return nil
		end
		local i = 3 * vid
		return V3(d[i + 1], d[i + 2], d[i + 3])
	end
end

-- Enumerate triangles
function DMesh3:Triangles(): () -> any
	local itr = self.triangles_refcount:Indices()
	local t = self.triangles.data
	return function()
		local tid = itr()
		if tid == nil then
			return nil
		end
		local i = 3 * tid
		return { t[i + 1], t[i + 2], t[i + 3] }
	end
end

-- Enumerate edges. return value is [v0,v1,t0,t1], where t1 will be InvalidID if this is a boundary edge
function DMesh3:Edges(): () -> any
	local itr = self.edges_refcount:Indices()
	local e = self.edges.data
	return function()
		local eid = itr()
		if eid == nil then
			return nil
		end
		local i = 4 * eid
		return { e[i + 1], e[i + 2], e[i + 3], e[i + 4] }
	end
end

---------------------------------------------------------------------------------------------------
-- queries

-- Find edgeid for edge [a,b]
function DMesh3:FindEdge(vA: number, vB: number): number
	if Util.DebugChecks then
		self:debug_check_is_vertex(vA)
		self:debug_check_is_vertex(vB)
	end
	return self:find_edge(vA, vB)
end

-- Find edgeid for edge [a,b] from triangle that contains the edge.
-- This is faster than FindEdge() because it is constant-time
function DMesh3:FindEdgeFromTri(vA: number, vB: number, tID: number): number
	return self:find_edge_from_tri(vA, vB, tID)
end

-- If edge has vertices [a,b], and is connected two triangles [a,b,c] and [a,b,d],
-- this returns [c,d], or [c,InvalidID] for a boundary edge
function DMesh3:GetEdgeOpposingV(eID: number): { number }
	-- ** it is important that verts returned maintain [c,d] order!!
	local e = self.edges.data
	local i = 4 * eID
	local a, b = e[i + 1], e[i + 2]
	local t0, t1 = e[i + 3], e[i + 4]
	local c = IndexUtil.find_tri_other_vtx_dv(a, b, self.triangles, t0)
	if t1 ~= InvalidID then
		local d = IndexUtil.find_tri_other_vtx_dv(a, b, self.triangles, t1)
		return { c, d }
	end
	return { c, InvalidID }
end

-- Find triangle made up of any permutation of vertices [a,b,c]
function DMesh3:FindTriangle(a: number, b: number, c: number): number
	local eid = self:find_edge(a, b)
	if eid == InvalidID then
		return InvalidID
	end
	local e, t = self.edges.data, self.triangles.data
	local ei = 4 * eid

	-- triangles attached to edge [a,b] must contain verts a and b...
	local ti = 3 * e[ei + 3]
	if t[ti + 1] == c or t[ti + 2] == c or t[ti + 3] == c then
		return e[ei + 3]
	end
	if e[ei + 4] ~= InvalidID then
		ti = 3 * e[ei + 4]
		if t[ti + 1] == c or t[ti + 2] == c or t[ti + 3] == c then
			return e[ei + 4]
		end
	end
	return InvalidID
end

-- Enumerate "other" vertices of edges connected to vertex (ie vertex one-ring)
function DMesh3:VtxVerticesItr(vID: number): () -> number?
	if not self.vertices_refcount:isValid(vID) then
		return function()
			return nil
		end
	end
	local itr = self.vertex_edges:ValueItr(vID)
	return function()
		local eid = itr()
		if eid == nil then
			return nil
		end
		return self:edge_other_v(eid, vID)
	end
end

-- Enumerate edge ids connected to vertex (ie edge one-ring)
function DMesh3:VtxEdgesItr(vID: number): () -> number?
	if self.vertices_refcount:isValid(vID) then
		return self.vertex_edges:ValueItr(vID)
	end
	return function()
		return nil
	end
end

-- Returns count of boundary edges at vertex, and the first two boundary edges if found.
-- If return is > 2, call VtxAllBoundaryEdges
-- C# int VtxBoundaryEdges(int vID, ref int e0, ref int e1) -> returns count, e0, e1
function DMesh3:VtxBoundaryEdges(vID: number, e0In: number?, e1In: number?): (number, number, number)
	local e0, e1 = e0In or InvalidID, e1In or InvalidID
	if self.vertices_refcount:isValid(vID) then
		local count = 0
		local ed = self.edges.data
		for eid in self.vertex_edges:ValueItr(vID) do
			if ed[4 * eid + 4] == InvalidID then
				if count == 0 then
					e0 = eid
				elseif count == 1 then
					e1 = eid
				end
				count += 1
			end
		end
		return count, e0, e1
	end
	Util.gDevAssert(false)
	return -1, e0, e1
end

-- Find edge ids of boundary edges connected to vertex.
-- e is a Luau array that is filled starting at e[1]. returns count, ie number of elements of e that were filled
function DMesh3:VtxAllBoundaryEdges(vID: number, e: { number }): number
	if self.vertices_refcount:isValid(vID) then
		local count = 0
		local ed = self.edges.data
		for eid in self.vertex_edges:ValueItr(vID) do
			if ed[4 * eid + 4] == InvalidID then
				count += 1
				e[count] = eid
			end
		end
		return count
	end
	Util.gDevAssert(false)
	return -1
end

-- Get triangle one-ring at vertex. vTriangles is a Luau array that tris are appended to.
-- bUseOrientation is more efficient but returns incorrect result if vertex is a bowtie
function DMesh3:GetVtxTriangles(vID: number, vTriangles: { number }, bUseOrientation: boolean): number
	if not self:IsVertex(vID) then
		return MeshResult.Failed_NotAVertex
	end
	local ed = self.edges.data
	if bUseOrientation then
		for eid in self.vertex_edges:ValueItr(vID) do
			local vOther = self:edge_other_v(eid, vID)
			local i = 4 * eid
			local et0 = ed[i + 3]
			if self:tri_has_sequential_v(et0, vID, vOther) then
				table.insert(vTriangles, et0)
			end
			local et1 = ed[i + 4]
			if et1 ~= InvalidID and self:tri_has_sequential_v(et1, vID, vOther) then
				table.insert(vTriangles, et1)
			end
		end
	else
		-- brute-force method
		for eid in self.vertex_edges:ValueItr(vID) do
			local i = 4 * eid
			local t0 = ed[i + 3]
			if table.find(vTriangles, t0) == nil then
				table.insert(vTriangles, t0)
			end
			local t1 = ed[i + 4]
			if t1 ~= InvalidID and table.find(vTriangles, t1) == nil then
				table.insert(vTriangles, t1)
			end
		end
	end
	return MeshResult.Ok
end

-- return # of triangles attached to vID, or -1 if invalid vertex
-- if bBruteForce = true, explicitly checks, which creates a list and is expensive
-- default is false, uses orientation, no memory allocation
function DMesh3:GetVtxTriangleCount(vID: number, bBruteForce: boolean?): number
	if bBruteForce then
		local vTriangles = {}
		if self:GetVtxTriangles(vID, vTriangles, false) ~= MeshResult.Ok then
			return -1
		end
		return #vTriangles
	end

	if not self:IsVertex(vID) then
		return -1
	end
	local N = 0
	local ed = self.edges.data
	for eid in self.vertex_edges:ValueItr(vID) do
		local vOther = self:edge_other_v(eid, vID)
		local i = 4 * eid
		local et0 = ed[i + 3]
		if self:tri_has_sequential_v(et0, vID, vOther) then
			N += 1
		end
		local et1 = ed[i + 4]
		if et1 ~= InvalidID and self:tri_has_sequential_v(et1, vID, vOther) then
			N += 1
		end
	end
	return N
end

-- iterate over triangle IDs of vertex one-ring
function DMesh3:VtxTrianglesItr(vID: number): () -> number?
	if not self:IsVertex(vID) then
		return function()
			return nil
		end
	end
	local itr = self.vertex_edges:ValueItr(vID)
	local ed = self.edges.data
	local pending = InvalidID
	return function()
		if pending ~= InvalidID then
			local p = pending
			pending = InvalidID
			return p
		end
		while true do
			local eid = itr()
			if eid == nil then
				return nil
			end
			local vOther = self:edge_other_v(eid, vID)
			local i = 4 * eid
			local et0 = ed[i + 3]
			local et1 = ed[i + 4]
			local ok0 = self:tri_has_sequential_v(et0, vID, vOther)
			local ok1 = et1 ~= InvalidID and self:tri_has_sequential_v(et1, vID, vOther)
			if ok0 then
				if ok1 then
					pending = et1
				end
				return et0
			elseif ok1 then
				return et1
			end
		end
	end
end

-- from edge and vert, returns other vert, two opposing verts, and two triangles
-- C# GetVtxNbrhood(int eID, int vID, ref vOther, ref oppV1, ref oppV2, ref t1, ref t2)
-- -> returns vOther, oppV1, oppV2, t1, t2  (oppV2 is InvalidID for a boundary edge)
function DMesh3:GetVtxNbrhood(eID: number, vID: number): (number, number, number, number, number)
	local e = self.edges.data
	local i = 4 * eID
	local vOther = (e[i + 1] == vID) and e[i + 2] or e[i + 1]
	local t1 = e[i + 3]
	local oppV1 = IndexUtil.find_tri_other_vtx_dv(vID, vOther, self.triangles, t1)
	local t2 = e[i + 4]
	local oppV2 = InvalidID
	if t2 ~= InvalidID then
		oppV2 = IndexUtil.find_tri_other_vtx_dv(vID, vOther, self.triangles, t2)
	else
		t2 = InvalidID
	end
	return vOther, oppV1, oppV2, t1, t2
end

-- Fastest possible one-ring centroid. This is used inside many other algorithms
-- so it helps to have it be maximally efficient
-- C# VtxOneRingCentroid(int vID, ref Vector3d centroid) -> returns centroid
function DMesh3:VtxOneRingCentroid(vID: number)
	local cx, cy, cz = 0, 0, 0
	if self.vertices_refcount:isValid(vID) then
		local n = 0
		local d = self.vertices.data
		local ed = self.edges.data
		for eid in self.vertex_edges:ValueItr(vID) do
			local ei = 4 * eid
			local ev0 = ed[ei + 1]
			local other = (ev0 == vID) and ed[ei + 2] or ev0
			local other_idx = 3 * other
			cx += d[other_idx + 1]
			cy += d[other_idx + 2]
			cz += d[other_idx + 3]
			n += 1
		end
		if n > 0 then
			local f = 1.0 / n
			cx *= f
			cy *= f
			cz *= f
		end
	end
	return V3(cx, cy, cz)
end

function DMesh3:tri_has_v(tID: number, vID: number): boolean
	local t = self.triangles.data
	local i = 3 * tID
	return t[i + 1] == vID or t[i + 2] == vID or t[i + 3] == vID
end

function DMesh3:tri_is_boundary(tID: number): boolean
	local te = self.triangle_edges.data
	local i = 3 * tID
	return self:IsBoundaryEdge(te[i + 1]) or self:IsBoundaryEdge(te[i + 2]) or self:IsBoundaryEdge(te[i + 3])
end

function DMesh3:tri_has_neighbour_t(tCheck: number, tNbr: number): boolean
	local te = self.triangle_edges.data
	local i = 3 * tCheck
	return self:edge_has_t(te[i + 1], tNbr) or self:edge_has_t(te[i + 2], tNbr) or self:edge_has_t(te[i + 3], tNbr)
end

function DMesh3:tri_has_sequential_v(tID: number, vA: number, vB: number): boolean
	local t = self.triangles.data
	local i = 3 * tID
	local v0, v1, v2 = t[i + 1], t[i + 2], t[i + 3]
	if v0 == vA and v1 == vB then
		return true
	end
	if v1 == vA and v2 == vB then
		return true
	end
	if v2 == vA and v0 == vB then
		return true
	end
	return false
end

--! returns edge ID
function DMesh3:find_tri_neighbour_edge(tID: number, vA: number, vB: number): number
	local t, te = self.triangles.data, self.triangle_edges.data
	local i = 3 * tID
	local tv0, tv1 = t[i + 1], t[i + 2]
	if IndexUtil.same_pair_unordered(tv0, tv1, vA, vB) then
		return te[3 * tID + 1]
	end
	local tv2 = t[i + 3]
	if IndexUtil.same_pair_unordered(tv1, tv2, vA, vB) then
		return te[3 * tID + 2]
	end
	if IndexUtil.same_pair_unordered(tv2, tv0, vA, vB) then
		return te[3 * tID + 3]
	end
	return InvalidID
end

-- returns 0/1/2
function DMesh3:find_tri_neighbour_index(tID: number, vA: number, vB: number): number
	local t = self.triangles.data
	local i = 3 * tID
	local tv0, tv1 = t[i + 1], t[i + 2]
	if IndexUtil.same_pair_unordered(tv0, tv1, vA, vB) then
		return 0
	end
	local tv2 = t[i + 3]
	if IndexUtil.same_pair_unordered(tv1, tv2, vA, vB) then
		return 1
	end
	if IndexUtil.same_pair_unordered(tv2, tv0, vA, vB) then
		return 2
	end
	return InvalidID
end

function DMesh3:IsBoundaryEdge(eid: number): boolean
	return self.edges.data[4 * eid + 4] == InvalidID
end

function DMesh3:edge_has_v(eid: number, vid: number): boolean
	local e = self.edges.data
	local i = 4 * eid
	return (e[i + 1] == vid) or (e[i + 2] == vid)
end

function DMesh3:edge_has_t(eid: number, tid: number): boolean
	local e = self.edges.data
	local i = 4 * eid
	return (e[i + 3] == tid) or (e[i + 4] == tid)
end

function DMesh3:edge_other_v(eID: number, vID: number): number
	local e = self.edges.data
	local i = 4 * eID
	local ev0, ev1 = e[i + 1], e[i + 2]
	if ev0 == vID then
		return ev1
	elseif ev1 == vID then
		return ev0
	end
	return InvalidID
end

function DMesh3:edge_other_t(eID: number, tid: number): number
	local e = self.edges.data
	local i = 4 * eID
	local et0, et1 = e[i + 3], e[i + 4]
	if et0 == tid then
		return et1
	elseif et1 == tid then
		return et0
	end
	return InvalidID
end

function DMesh3:IsBoundaryVertex(vID: number): boolean
	local ed = self.edges.data
	for e in self.vertex_edges:ValueItr(vID) do
		if ed[4 * e + 4] == InvalidID then
			return true
		end
	end
	return false
end

-- Returns true if any edge of triangle is a boundary edge
function DMesh3:IsBoundaryTriangle(tID: number): boolean
	if Util.DebugChecks then
		self:debug_check_is_triangle(tID)
	end
	local te = self.triangle_edges.data
	local i = 3 * tID
	return self:IsBoundaryEdge(te[i + 1]) or self:IsBoundaryEdge(te[i + 2]) or self:IsBoundaryEdge(te[i + 3])
end

function DMesh3:find_edge(vA: number, vB: number): number
	-- [RMS] edge vertices must be sorted (min,max),
	--   that means we only need one index-check in inner loop.
	local vO = (vA > vB) and vA or vB
	local vI = (vA > vB) and vB or vA
	-- inlined iteration over vertex_edges.ValueItr(vI)
	local sls = self.vertex_edges
	local block_ptr = sls.list_heads[vI + 1]
	if block_ptr == nil or block_ptr == SLS_Null then
		return InvalidID
	end
	local ed = self.edges.data
	local bs = sls.block_store
	local N = bs[block_ptr + 1]
	local iEnd = block_ptr + ((N < SLS_BLOCKSIZE) and N or SLS_BLOCKSIZE)
	for i = block_ptr + 2, iEnd + 1 do
		local eid = bs[i]
		if ed[4 * eid + 2] == vO then
			return eid
		end
	end
	if N > SLS_BLOCKSIZE then
		local ls = sls.linked_store
		local cur_ptr = bs[block_ptr + SLS_BLOCK_LIST_OFFSET + 1]
		while cur_ptr ~= SLS_Null do
			local eid = ls[cur_ptr + 1]
			if ed[4 * eid + 2] == vO then
				return eid
			end
			cur_ptr = ls[cur_ptr + 2]
		end
	end
	return InvalidID
end

function DMesh3:find_edge_from_tri(vA: number, vB: number, tID: number): number
	local t, te = self.triangles.data, self.triangle_edges.data
	local i = 3 * tID
	local t0, t1 = t[i + 1], t[i + 2]
	if IndexUtil.same_pair_unordered(vA, vB, t0, t1) then
		return te[i + 1]
	end
	local t2 = t[i + 3]
	if IndexUtil.same_pair_unordered(vA, vB, t1, t2) then
		return te[i + 2]
	end
	if IndexUtil.same_pair_unordered(vA, vB, t2, t0) then
		return te[i + 3]
	end
	return InvalidID
end

-- queries

-- Returns true if the two triangles connected to edge have different group IDs
function DMesh3:IsGroupBoundaryEdge(eID: number): boolean
	if not self:IsEdge(eID) then
		error("DMesh3.IsGroupBoundaryEdge: " .. eID .. " is not a valid edge")
	end
	if self.triangle_groups == nil then
		error("DMesh3.IsGroupBoundaryEdge: no triangle groups!")
	end
	local e, g = self.edges.data, self.triangle_groups.data
	local et1 = e[4 * eID + 4]
	if et1 == InvalidID then
		return false
	end
	local g1 = g[et1 + 1]
	local et0 = e[4 * eID + 3]
	local g0 = g[et0 + 1]
	return g1 ~= g0
end

-- returns true if vertex has more than one tri group in its tri nbrhood
function DMesh3:IsGroupBoundaryVertex(vID: number): boolean
	if not self:IsVertex(vID) then
		error("DMesh3.IsGroupBoundaryVertex: " .. vID .. " is not a valid vertex")
	end
	if self.triangle_groups == nil then
		error("DMesh3.IsGroupBoundaryVertex: no triangle groups!")
	end
	local e, g = self.edges.data, self.triangle_groups.data
	local group_id = MathUtil.MinInt
	for eID in self.vertex_edges:ValueItr(vID) do
		local et0 = e[4 * eID + 3]
		local g0 = g[et0 + 1]
		if group_id ~= g0 then
			if group_id == MathUtil.MinInt then
				group_id = g0
			else
				return true -- saw multiple group IDs
			end
		end
		local et1 = e[4 * eID + 4]
		if et1 ~= InvalidID then
			local g1 = g[et1 + 1]
			if group_id ~= g1 then
				return true -- saw multiple group IDs
			end
		end
	end
	return false
end

-- returns true if more than two group boundary edges meet at vertex (ie 3+ groups meet at this vertex)
function DMesh3:IsGroupJunctionVertex(vID: number): boolean
	if not self:IsVertex(vID) then
		error("DMesh3.IsGroupJunctionVertex: " .. vID .. " is not a valid vertex")
	end
	if self.triangle_groups == nil then
		error("DMesh3.IsGroupJunctionVertex: no triangle groups!")
	end
	local e, g = self.edges.data, self.triangle_groups.data
	local MaxInt = MathUtil.MaxInt
	local ga, gb = MaxInt, MaxInt
	for eID in self.vertex_edges:ValueItr(vID) do
		local et = { e[4 * eID + 3], e[4 * eID + 4] }
		for k = 1, 2 do
			if et[k] ~= InvalidID then
				local g0 = g[et[k] + 1]
				if g0 ~= ga and g0 ~= gb then
					if ga ~= MaxInt and gb ~= MaxInt then
						return true
					end
					if ga == MaxInt then
						ga = g0
					else
						gb = g0
					end
				end
			end
		end
	end
	return false
end

-- returns up to 4 group IDs at vertex. Returns false if > 4 encountered
-- C# bool GetVertexGroups(int vID, out Index4i groups) -> returns ok, groups
function DMesh3:GetVertexGroups(vID: number): (boolean, { number })
	local MaxInt = MathUtil.MaxInt
	local groups = { MaxInt, MaxInt, MaxInt, MaxInt }
	local ng = 0

	if not self:IsVertex(vID) then
		error("DMesh3.GetVertexGroups: " .. vID .. " is not a valid vertex")
	end
	if self.triangle_groups == nil then
		error("DMesh3.GetVertexGroups: no triangle groups!")
	end
	local e, g = self.edges.data, self.triangle_groups.data
	for eID in self.vertex_edges:ValueItr(vID) do
		local et0 = e[4 * eID + 3]
		local g0 = g[et0 + 1]
		if table.find(groups, g0) == nil then
			ng += 1
			groups[ng] = g0
		end
		if ng == 4 then
			return false, groups
		end
		local et1 = e[4 * eID + 4]
		if et1 ~= InvalidID then
			local g1 = g[et1 + 1]
			if table.find(groups, g1) == nil then
				ng += 1
				groups[ng] = g1
			end
			if ng == 4 then
				return false, groups
			end
		end
	end
	return true, groups
end

-- returns all group IDs at vertex. If groups is given, ids are appended to it. returns the list.
function DMesh3:GetAllVertexGroups(vID: number, groups: { number }?): { number }
	local result = groups or {}
	if not self:IsVertex(vID) then
		error("DMesh3.GetAllVertexGroups: " .. vID .. " is not a valid vertex")
	end
	if self.triangle_groups == nil then
		error("DMesh3.GetAllVertexGroups: no triangle groups!")
	end
	local e, g = self.edges.data, self.triangle_groups.data
	for eID in self.vertex_edges:ValueItr(vID) do
		local et0 = e[4 * eID + 3]
		local g0 = g[et0 + 1]
		if table.find(result, g0) == nil then
			table.insert(result, g0)
		end
		local et1 = e[4 * eID + 4]
		if et1 ~= InvalidID then
			local g1 = g[et1 + 1]
			if table.find(result, g1) == nil then
				table.insert(result, g1)
			end
		end
	end
	return result
end

-- returns true if vID is a "bowtie" vertex, ie multiple disjoint triangle sets in one-ring
function DMesh3:IsBowtieVertex(vID: number): boolean
	if self.vertices_refcount:isValid(vID) then
		local nEdges = self.vertex_edges:Count(vID)
		if nEdges == 0 then
			return false
		end
		local e = self.edges.data
		local t, te = self.triangles.data, self.triangle_edges.data

		-- find a boundary edge to start at
		local start_eid = -1
		local start_at_boundary = false
		for eid in self.vertex_edges:ValueItr(vID) do
			if e[4 * eid + 4] == InvalidID then
				start_at_boundary = true
				start_eid = eid
				break
			end
		end
		-- if no boundary edge, start at arbitrary edge
		if start_eid == -1 then
			start_eid = self.vertex_edges:First(vID)
		end
		-- initial triangle
		local start_tid = e[4 * start_eid + 3]

		local prev_tid = start_tid
		local prev_eid = start_eid

		-- walk forward to next edge. if we hit start edge or boundary edge,
		-- we are done the walk. count number of edges as we go.
		local count = 1
		while true do
			local i = 3 * prev_tid
			local tv = { t[i + 1], t[i + 2], t[i + 3] }
			local vert_idx = IndexUtil.find_tri_index(vID, tv)
			local e1 = te[i + vert_idx + 1]
			local e2 = te[i + ((vert_idx + 2) % 3) + 1]
			local next_eid = (e1 == prev_eid) and e2 or e1
			if next_eid == start_eid then
				break
			end
			local na, nb = e[4 * next_eid + 3], e[4 * next_eid + 4]
			local next_tid = (na == prev_tid) and nb or na
			if next_tid == InvalidID then
				break
			end
			prev_eid = next_eid
			prev_tid = next_tid
			count += 1
		end

		-- if we did not see all edges at vertex, we have a bowtie
		local target_count = start_at_boundary and (nEdges - 1) or nEdges
		return target_count ~= count
	end
	error("DMesh3.IsBowtieVertex: " .. vID .. " is not a valid vertex")
end

-- Computes bounding box of all vertices.
function DMesh3:GetBounds()
	local d = self.vertices.data
	local x, y, z = 0, 0, 0
	for vi in self.vertices_refcount:Indices() do
		x, y, z = d[3 * vi + 1], d[3 * vi + 2], d[3 * vi + 3]
		break
	end
	local minx, maxx, miny, maxy, minz, maxz = x, x, y, y, z, z
	for vi in self.vertices_refcount:Indices() do
		x, y, z = d[3 * vi + 1], d[3 * vi + 2], d[3 * vi + 3]
		if x < minx then
			minx = x
		elseif x > maxx then
			maxx = x
		end
		if y < miny then
			miny = y
		elseif y > maxy then
			maxy = y
		end
		if z < minz then
			minz = z
		elseif z > maxz then
			maxz = z
		end
	end
	return AxisAlignedBox3d.new(minx, miny, minz, maxx, maxy, maxz)
end

-- cached bounding box, lazily re-computed on access if mesh has changed (returns a copy)
function DMesh3:CachedBounds()
	if self.cached_bounds_timestamp ~= self.timestamp then
		self.cached_bounds = self:GetBounds()
		self.cached_bounds_timestamp = self.timestamp
	end
	return self.cached_bounds:Clone()
end

function DMesh3:IsClosed(): boolean
	if self:TriangleCount() == 0 then
		return false
	end
	local erc = self.edges_refcount
	for eid in erc:Indices() do
		if self:IsBoundaryEdge(eid) then
			return false
		end
	end
	return true
end

function DMesh3:CachedIsClosed(): boolean
	if self.cached_is_closed_timestamp ~= self.timestamp then
		self.cached_is_closed = self:IsClosed()
		self.cached_is_closed_timestamp = self.timestamp
	end
	return self.cached_is_closed
end

-- returns true if vertices, edges, and triangles are all "dense" (Count == MaxID)
function DMesh3:IsCompact(): boolean
	return self.vertices_refcount:is_dense() and self.edges_refcount:is_dense() and self.triangles_refcount:is_dense()
end

-- Returns true if vertex count == max vertex id
function DMesh3:IsCompactV(): boolean
	return self.vertices_refcount:is_dense()
end

-- returns true if triangle count == max triangle id
function DMesh3:IsCompactT(): boolean
	return self.triangles_refcount:is_dense()
end

-- returns measure of compactness in range [0,1], where 1 is fully compacted
function DMesh3:CompactMetric(): number
	return (self:VertexCount() / self:MaxVertexID() + self:TriangleCount() / self:MaxTriangleID()) * 0.5
end

-- Compute mesh winding number, from Jacobson et al, Robust Inside-Outside Segmentation using Generalized Winding Numbers
-- http://igl.ethz.ch/projects/winding-number/
-- returns ~0 for points outside a closed, consistently oriented mesh, and a positive or negative integer
-- for points inside, with value > 1 depending on how many "times" the point inside the mesh (like in 2D polygon winding)
function DMesh3:WindingNumber(v): number
	local sum = 0
	for tid in self.triangles_refcount:Indices() do
		sum += self:GetTriSolidAngle(tid, v)
	end
	return sum / (4.0 * math.pi)
end

-- Metadata support

function DMesh3:HasMetadata(): boolean
	return self.Metadata ~= nil and next(self.Metadata) ~= nil
end
function DMesh3:AttachMetadata(key: string, o: any)
	if self.Metadata == nil then
		self.Metadata = {}
	end
	if self.Metadata[key] ~= nil then
		error("DMesh3.AttachMetadata: key already exists: " .. key) -- (C# Dictionary.Add throws)
	end
	self.Metadata[key] = o
end
function DMesh3:FindMetadata(key: string): any
	if self.Metadata == nil then
		return nil
	end
	return self.Metadata[key]
end
function DMesh3:RemoveMetadata(key: string): boolean
	if self.Metadata == nil or self.Metadata[key] == nil then
		return false
	end
	self.Metadata[key] = nil
	return true
end
function DMesh3:ClearMetadata()
	self.Metadata = nil
end

-- direct access to internal dvectors - dangerous!!

function DMesh3:VerticesBuffer()
	return self.vertices
end
function DMesh3:SetVerticesBuffer(value)
	self.vertices = value
end
function DMesh3:VerticesRefCounts()
	return self.vertices_refcount
end
function DMesh3:SetVerticesRefCounts(value)
	self.vertices_refcount = value
end
function DMesh3:NormalsBuffer()
	return self.normals
end
function DMesh3:SetNormalsBuffer(value)
	self.normals = value
end
function DMesh3:UVBuffer()
	return self.uv
end
function DMesh3:SetUVBuffer(value)
	self.uv = value
end
function DMesh3:ColorsBuffer()
	return self:HasVertexColors() and self:Attribs():VertexColor():ToBuffer() or nil
end
function DMesh3:SetColorsBuffer(value)
	self:EnableVertexColors()
	self:Attribs():VertexColor():SetFromBuffer(value)
end
function DMesh3:TrianglesBuffer()
	return self.triangles
end
function DMesh3:SetTrianglesBuffer(value)
	self.triangles = value
end
function DMesh3:TrianglesRefCounts()
	return self.triangles_refcount
end
function DMesh3:SetTrianglesRefCounts(value)
	self.triangles_refcount = value
end
function DMesh3:GroupsBuffer()
	return self.triangle_groups
end
function DMesh3:SetGroupsBuffer(value)
	self.triangle_groups = value
end
function DMesh3:EdgesBuffer()
	return self.edges
end
function DMesh3:SetEdgesBuffer(value)
	self.edges = value
end
function DMesh3:EdgesRefCounts()
	return self.edges_refcount
end
function DMesh3:SetEdgesRefCounts(value)
	self.edges_refcount = value
end
function DMesh3:VertexEdges()
	return self.vertex_edges
end
function DMesh3:SetVertexEdges(value)
	self.vertex_edges = value
end

-- Rebuild mesh topology.
-- assumes that we have initialized vertices, triangles, and edges buffers,
-- and edges refcounts. Rebuilds vertex and tri refcounts, triangle edges, vertex edges.
function DMesh3:RebuildFromEdgeRefcounts()
	local MaxVID = self.vertices.size // 3
	local MaxTID = self.triangles.size // 3

	self.triangle_edges:resize(self.triangles.size)
	self.triangles_refcount:RawRefCounts():resize(MaxTID)

	self.vertex_edges:Resize(MaxVID)
	self.vertices_refcount:RawRefCounts():resize(MaxVID)

	local e = self.edges.data
	local MaxEID = self.edges.size // 4
	for eid = 0, MaxEID - 1 do
		if self.edges_refcount:isValid(eid) then
			local va = e[4 * eid + 1]
			local vb = e[4 * eid + 2]
			local t0 = e[4 * eid + 3]
			local t1 = e[4 * eid + 4]

			-- set vertex and tri refcounts to 1
			-- find edges [a,b] in each triangle and set its tri-edge to this edge

			if not self.vertices_refcount:isValidUnsafe(va) then
				self:allocate_edges_list(va)
				self.vertices_refcount:set_Unsafe(va, 1)
			end
			if not self.vertices_refcount:isValidUnsafe(vb) then
				self:allocate_edges_list(vb)
				self.vertices_refcount:set_Unsafe(vb, 1)
			end
			self.triangles_refcount:set_Unsafe(t0, 1)
			local tri0 = self:GetTriangle(t0)
			local idx0 = IndexUtil.find_edge_index_in_tri(va, vb, tri0)
			self.triangle_edges.data[3 * t0 + idx0 + 1] = eid

			if t1 ~= InvalidID then
				self.triangles_refcount:set_Unsafe(t1, 1)
				local tri1 = self:GetTriangle(t1)
				local idx1 = IndexUtil.find_edge_index_in_tri(va, vb, tri1)
				self.triangle_edges.data[3 * t1 + idx1 + 1] = eid
			end

			-- add this edge to both vertices
			self.vertex_edges:Insert(va, eid)
			self.vertex_edges:Insert(vb, eid)
		end
	end

	-- iterate over triangles and increment vtx refcount for each tri
	local has_groups = self:HasTriangleGroups()
	self.max_group_id = 0
	local t = self.triangles.data
	for tid = 0, MaxTID - 1 do
		if self.triangles_refcount:isValid(tid) then
			local a, b, c = t[3 * tid + 1], t[3 * tid + 2], t[3 * tid + 3]
			self.vertices_refcount:increment(a)
			self.vertices_refcount:increment(b)
			self.vertices_refcount:increment(c)

			if has_groups then
				self.max_group_id = math.max(self.max_group_id, self.triangle_groups.data[tid + 1])
			end
		end
	end
	self.max_group_id += 1

	self.vertices_refcount:rebuild_free_list()
	self.triangles_refcount:rebuild_free_list()
	self.edges_refcount:rebuild_free_list()

	self:updateTimeStamp(true)
end

-- Compact mesh in-place, by moving vertices around and rewriting indices.
-- Should be faster if the amount of compacting is not too significant, and
-- is useful in some places.
-- [TODO] vertex_edges is not compacted. does not affect indices, but does keep memory.
--
-- If bComputeCompactInfo=false, the returned CompactInfo is not initialized
function DMesh3:CompactInPlace(bComputeCompactInfo: boolean?)
	local mapV = bComputeCompactInfo and IndexMap.WithEstimate(self:MaxVertexID(), self:VertexCount()) or nil
	local ci = { MapV = mapV }

	local vrc = self.vertices_refcount
	-- find first free vertex, and last used vertex
	local iLastV, iCurV = self:MaxVertexID() - 1, 0
	while iLastV >= 0 and not vrc:isValidUnsafe(iLastV) do
		iLastV -= 1
	end
	while iCurV < self:MaxVertexID() and vrc:isValidUnsafe(iCurV) do
		iCurV += 1
	end

	local vref = vrc:RawRefCounts().data
	local verts = self.vertices.data
	local normals = self.normals and self.normals.data or nil
	local uvd = self.uv and self.uv.data or nil
	local ed = self.edges.data

	while iCurV < iLastV do
		local kc, kl = iCurV * 3, iLastV * 3
		verts[kc + 1], verts[kc + 2], verts[kc + 3] = verts[kl + 1], verts[kl + 2], verts[kl + 3]
		if normals ~= nil then
			normals[kc + 1], normals[kc + 2], normals[kc + 3] = normals[kl + 1], normals[kl + 2], normals[kl + 3]
		end
		if uvd ~= nil then
			local ukc, ukl = iCurV * 2, iLastV * 2
			uvd[ukc + 1], uvd[ukc + 2] = uvd[ukl + 1], uvd[ukl + 2]
		end

		for eid in self.vertex_edges:ValueItr(iLastV) do
			-- replace vertex in edges
			self:replace_edge_vertex(eid, iLastV, iCurV)

			-- replace vertex in triangles
			local t0 = ed[4 * eid + 3]
			self:replace_tri_vertex(t0, iLastV, iCurV)
			local t1 = ed[4 * eid + 4]
			if t1 ~= InvalidID then
				self:replace_tri_vertex(t1, iLastV, iCurV)
			end
		end

		-- shift vertex refcount to new position
		vref[iCurV + 1] = vref[iLastV + 1]
		vref[iLastV + 1] = RefCountVector.invalid

		-- update attributes
		self:on_new_vertex(iCurV, iLastV)

		-- move edge list
		self.vertex_edges:Move(iLastV, iCurV)

		if mapV ~= nil then
			mapV:Set(iLastV, iCurV)
		end

		-- move cur forward one, last back one, and  then search for next valid
		iLastV -= 1
		iCurV += 1
		while iLastV >= 0 and not vrc:isValidUnsafe(iLastV) do
			iLastV -= 1
		end
		while iCurV < iLastV and vrc:isValidUnsafe(iCurV) do
			iCurV += 1
		end
	end

	-- trim vertices data structures
	vrc:trim(self:VertexCount())
	self.vertices:resize(self:VertexCount() * 3)
	if self.normals ~= nil then
		self.normals:resize(self:VertexCount() * 3)
	end
	if self.uv ~= nil then
		self.uv:resize(self:VertexCount() * 2)
	end

	-- [TODO] vertex_edges!!!

	--* shift triangles *

	local trc = self.triangles_refcount
	-- find first free triangle, and last valid triangle
	local iLastT, iCurT = self:MaxTriangleID() - 1, 0
	while iLastT >= 0 and not trc:isValidUnsafe(iLastT) do
		iLastT -= 1
	end
	while iCurT < self:MaxTriangleID() and trc:isValidUnsafe(iCurT) do
		iCurT += 1
	end

	local tref = trc:RawRefCounts().data
	local tris = self.triangles.data
	local tedges = self.triangle_edges.data

	while iCurT < iLastT do
		local kc, kl = iCurT * 3, iLastT * 3

		-- shift triangle
		for j = 1, 3 do
			tris[kc + j] = tris[kl + j]
			tedges[kc + j] = tedges[kl + j]
		end
		if self.triangle_groups ~= nil then
			local g = self.triangle_groups.data
			g[iCurT + 1] = g[iLastT + 1]
		end

		self:on_new_triangle(iCurT, iLastT)

		-- update edges
		for j = 1, 3 do
			local eid = tedges[kc + j]
			self:replace_edge_triangle(eid, iLastT, iCurT)
		end

		-- shift triangle refcount to new position
		tref[iCurT + 1] = tref[iLastT + 1]
		tref[iLastT + 1] = RefCountVector.invalid

		-- move cur forward one, last back one, and  then search for next valid
		iLastT -= 1
		iCurT += 1
		while iLastT >= 0 and not trc:isValidUnsafe(iLastT) do
			iLastT -= 1
		end
		while iCurT < iLastT and trc:isValidUnsafe(iCurT) do
			iCurT += 1
		end
	end

	-- trim triangles data structures
	trc:trim(self:TriangleCount())
	self.triangles:resize(self:TriangleCount() * 3)
	self.triangle_edges:resize(self:TriangleCount() * 3)
	if self.triangle_groups ~= nil then
		self.triangle_groups:resize(self:TriangleCount())
	end

	--* shift edges *

	local erc = self.edges_refcount
	-- find first free edge, and last used edge
	local iLastE, iCurE = self:MaxEdgeID() - 1, 0
	while iLastE >= 0 and not erc:isValidUnsafe(iLastE) do
		iLastE -= 1
	end
	while iCurE < self:MaxEdgeID() and erc:isValidUnsafe(iCurE) do
		iCurE += 1
	end

	local eref = erc:RawRefCounts().data

	while iCurE < iLastE do
		local kc, kl = iCurE * 4, iLastE * 4

		-- shift edge
		for j = 1, 4 do
			ed[kc + j] = ed[kl + j]
		end

		-- replace edge in vertex edges lists
		local v0, v1 = ed[kc + 1], ed[kc + 2]
		self.vertex_edges:ReplaceValue(v0, iLastE, iCurE)
		self.vertex_edges:ReplaceValue(v1, iLastE, iCurE)

		-- replace edge in triangles
		self:replace_triangle_edge(ed[kc + 3], iLastE, iCurE)
		if ed[kc + 4] ~= InvalidID then
			self:replace_triangle_edge(ed[kc + 4], iLastE, iCurE)
		end

		-- shift triangle refcount to new position
		eref[iCurE + 1] = eref[iLastE + 1]
		eref[iLastE + 1] = RefCountVector.invalid

		-- move cur forward one, last back one, and  then search for next valid
		iLastE -= 1
		iCurE += 1
		while iLastE >= 0 and not erc:isValidUnsafe(iLastE) do
			iLastE -= 1
		end
		while iCurE < iLastE and erc:isValidUnsafe(iCurE) do
			iCurE += 1
		end
	end

	-- trim edge data structures
	erc:trim(self:EdgeCount())
	self.edges:resize(self:EdgeCount() * 4)

	-- trim attributes
	if self:HasAttributes() then
		self.attributes:TrimTo(self:VertexCount(), self:TriangleCount())
	end

	return ci
end

-- install edge operators and debug functions (C# partial class DMesh3)
require(script.Parent.DMesh3EdgeOps)(DMesh3)
require(script.Parent.DMesh3Debug)(DMesh3)

return DMesh3
