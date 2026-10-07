-- Port of geometry3Sharp core/Indexing.cs (IIndexMap implementations, IndexFlagSet)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- IIndexMap: C# indexer map[index] -> map:Get(index). Setter map[index] = v -> map:Set(index, v).
-- All indices are 0-based.

local MathUtil = require(script.Parent.MathUtil)

local IndexMap = {}
IndexMap.__index = IndexMap
IndexMap.InvalidIndex = MathUtil.MinInt

-- i = i index map
local IdentityIndexMap = {}
IdentityIndexMap.__index = IdentityIndexMap
function IdentityIndexMap.new()
	return setmetatable({}, IdentityIndexMap)
end
function IdentityIndexMap:Get(index: number): number
	return index
end
IndexMap.IdentityIndexMap = IdentityIndexMap

-- i = i + constant index map
local ShiftIndexMap = {}
ShiftIndexMap.__index = ShiftIndexMap
function ShiftIndexMap.new(n: number)
	return setmetatable({ Shift = n }, ShiftIndexMap)
end
function ShiftIndexMap:Get(index: number): number
	return index + self.Shift
end
IndexMap.ShiftIndexMap = ShiftIndexMap

-- i = constant index map
local ConstantIndexMap = {}
ConstantIndexMap.__index = ConstantIndexMap
function ConstantIndexMap.new(c: number)
	return setmetatable({ Constant = c }, ConstantIndexMap)
end
function ConstantIndexMap:Get(index: number): number
	return self.Constant
end
IndexMap.ConstantIndexMap = ConstantIndexMap

-- dense or sparse index map
--   IndexMap.new(bForceSparse: boolean, MaxIndex?)
--   IndexMap.FromDense(use_dense_map: {number} (1-based array, map[i+1] is value for index i), MaxIndex?)
--   IndexMap.WithEstimate(MaxIndex, SubsetCountEst)
local function make(dense, sparse, maxIndex)
	return setmetatable({ dense_map = dense, sparse_map = sparse, MaxIndex = maxIndex or -1 }, IndexMap)
end

function IndexMap.new(bForceSparse: boolean, MaxIndex: number?)
	local m
	if bForceSparse then
		m = make(nil, {}, MaxIndex)
	else
		m = make(table.create(MaxIndex or 0, IndexMap.InvalidIndex), nil, MaxIndex)
	end
	m:SetToInvalid()
	return m
end

function IndexMap.FromDense(use_dense_map: { number }, MaxIndex: number?)
	return make(use_dense_map, nil, MaxIndex)
end

function IndexMap.WithEstimate(MaxIndex: number, SubsetCountEst: number)
	local bSmall = MaxIndex < 32000 -- if buffer is less than 128k, just use dense map
	local fPercent = SubsetCountEst / MaxIndex
	local fPercentThresh = 0.1
	local m
	if bSmall or fPercent > fPercentThresh then
		m = make(table.create(MaxIndex, IndexMap.InvalidIndex), nil, MaxIndex)
	else
		m = make(nil, {}, MaxIndex)
	end
	m:SetToInvalid()
	return m
end

-- no effect on sparse map
function IndexMap:SetToInvalid()
	local dense = self.dense_map
	if dense ~= nil then
		for i = 1, #dense do
			dense[i] = IndexMap.InvalidIndex
		end
	end
end

-- dense variant: returns true unless you have set index to InvalidIndex (eg via SetToInvalid)
-- sparse variant: returns true if index is in map
-- either: returns false if index is out-of-bounds
function IndexMap:Contains(index: number): boolean
	if self.MaxIndex > 0 and index >= self.MaxIndex then
		return false
	end
	if self.dense_map ~= nil then
		local v = self.dense_map[index + 1]
		return v ~= nil and v ~= IndexMap.InvalidIndex
	end
	return self.sparse_map[index] ~= nil
end

function IndexMap:Get(index: number): number
	if self.dense_map ~= nil then
		return self.dense_map[index + 1]
	end
	local to = self.sparse_map[index]
	if to ~= nil then
		return to
	end
	return IndexMap.InvalidIndex
end

function IndexMap:Set(index: number, value: number)
	if self.dense_map ~= nil then
		self.dense_map[index + 1] = value
	else
		self.sparse_map[index] = value
	end
end

---------------------------------------------------------------------------------------------------
-- IndexFlagSet: like BitArray but can optionally use a sparse (hash) representation

local IndexFlagSet = {}
IndexFlagSet.__index = IndexFlagSet
IndexMap.IndexFlagSet = IndexFlagSet

-- IndexFlagSet.new(bSparse, MaxIndex?) / IndexFlagSet.WithEstimate(MaxIndex, SubsetCountEst)
function IndexFlagSet.new(bSparse: boolean, MaxIndex: number?)
	if bSparse then
		return setmetatable({ bits = nil, hash = {}, count = 0 }, IndexFlagSet)
	end
	return setmetatable({ bits = table.create(MaxIndex or 0, false), hash = nil, count = 0 }, IndexFlagSet)
end

function IndexFlagSet.WithEstimate(MaxIndex: number, SubsetCountEst: number)
	local bSmall = MaxIndex < 128000 -- 16k in bits is a small buffer?
	local fPercent = SubsetCountEst / MaxIndex
	local fPercentThresh = 0.05
	return IndexFlagSet.new(not (bSmall or fPercent > fPercentThresh), MaxIndex)
end

function IndexFlagSet:Contains(i: number): boolean
	if self.bits ~= nil then
		return self.bits[i + 1] == true
	end
	return self.hash[i] == true
end

function IndexFlagSet:Add(i: number)
	if self.bits ~= nil then
		if self.bits[i + 1] ~= true then
			self.bits[i + 1] = true
			self.count += 1
		end
	elseif self.hash[i] == nil then
		self.hash[i] = true
		self.count += 1
	end
end

function IndexFlagSet:Count(): number
	return self.count
end

-- C# indexer: get
function IndexFlagSet:Get(key: number): boolean
	return self:Contains(key)
end

-- C# indexer: set
function IndexFlagSet:Set(key: number, value: boolean)
	if value then
		self:Add(key)
	elseif self.bits ~= nil then
		if self.bits[key + 1] == true then
			self.bits[key + 1] = false
			self.count -= 1
		end
	elseif self.hash[key] ~= nil then
		self.hash[key] = nil
		self.count -= 1
	end
end

-- enumerate over indices w/ value = true
function IndexFlagSet:Indices(): () -> number?
	if self.bits ~= nil then
		local bits = self.bits
		local n, i = #bits, 0
		return function()
			while i < n do
				i += 1
				if bits[i] then
					return i - 1
				end
			end
			return nil
		end
	end
	local k = nil
	local hash = self.hash
	return function()
		k = next(hash, k)
		return k
	end
end

return IndexMap
