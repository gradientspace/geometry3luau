--!native
-- Port of geometry3Sharp math/IndexTypes.cs (Index2i, Index3i, Index4i).
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- IndexNi values are plain Luau arrays: Index3i(a,b,c) == { a, b, c }.
--   C# t.a -> t[1], t.b -> t[2], t.c -> t[3], t.d -> t[4], C# t[j] -> t[j+1]
-- These helper functions take/return 0-based positions where the C# does (IndexOf, OtherIndex).
-- Arrays are mutable: clone with table.clone() before modifying one you did not create.

local MathUtil = require(script.Parent.MathUtil)

local Index = {}

---------------------------------------------------------------------------------------------------
local Index2i = {}
Index.Index2i = Index2i

function Index2i.new(a: number, b: number): { number }
	return { a, b }
end
function Index2i.Zero(): { number }
	return { 0, 0 }
end
function Index2i.One(): { number }
	return { 1, 1 }
end
function Index2i.Max(): { number }
	return { MathUtil.MaxInt, MathUtil.MaxInt }
end
function Index2i.Min(): { number }
	return { MathUtil.MinInt, MathUtil.MinInt }
end
function Index2i.Equals(x: { number }, y: { number }): boolean
	return x[1] == y[1] and x[2] == y[2]
end
function Index2i.CompareTo(x: { number }, y: { number }): number
	if x[1] ~= y[1] then
		return x[1] < y[1] and -1 or 1
	elseif x[2] ~= y[2] then
		return x[2] < y[2] and -1 or 1
	end
	return 0
end
function Index2i.ToString(t: { number }): string
	return string.format("[%d,%d]", t[1], t[2])
end
function Index2i.HashKey(t: { number }): string
	return t[1] .. "," .. t[2]
end

---------------------------------------------------------------------------------------------------
local Index3i = {}
Index.Index3i = Index3i

function Index3i.new(a: number, b: number, c: number): { number }
	return { a, b, c }
end
-- Index3i(ii, jj, kk, bool cycle): reverse last two indices if cycle is true (useful for cw/ccw codes)
function Index3i.newCycle(a: number, b: number, c: number, cycle: boolean): { number }
	if cycle then
		return { a, c, b }
	end
	return { a, b, c }
end
function Index3i.Zero(): { number }
	return { 0, 0, 0 }
end
function Index3i.One(): { number }
	return { 1, 1, 1 }
end
function Index3i.Max(): { number }
	return { MathUtil.MaxInt, MathUtil.MaxInt, MathUtil.MaxInt }
end
function Index3i.Min(): { number }
	return { MathUtil.MinInt, MathUtil.MinInt, MathUtil.MinInt }
end
function Index3i.LengthSquared(t: { number }): number
	return t[1] * t[1] + t[2] * t[2] + t[3] * t[3]
end
-- returns 0-based index of value in [a,b,c]. no error handling.
function Index3i.IndexOf(t: { number }, value: number): number
	if t[1] == value then
		return 0
	elseif t[2] == value then
		return 1
	end
	return 2
end
-- assuming value is one of [a,b,c], returns other two values in cycle-order. no error handling.
function Index3i.FindOthers(t: { number }, value: number): (number, number)
	if t[1] == value then
		return t[2], t[3]
	elseif t[2] == value then
		return t[3], t[1]
	end
	return t[1], t[2]
end
-- assuming i0 and i1 are in range [0,1,2], returns third index. no error handling.
function Index3i.OtherIndex(i0: number, i1: number): number
	if i1 < i0 then
		i0, i1 = i1, i0
	end
	if i0 == 0 then
		return (i1 == 1) and 2 or 1
	end
	return 0
end
function Index3i.Contains(t: { number }, value: number): boolean
	return t[1] == value or t[2] == value or t[3] == value
end
function Index3i.Equals(x: { number }, y: { number }): boolean
	return x[1] == y[1] and x[2] == y[2] and x[3] == y[3]
end
function Index3i.CompareTo(x: { number }, y: { number }): number
	for i = 1, 3 do
		if x[i] ~= y[i] then
			return x[i] < y[i] and -1 or 1
		end
	end
	return 0
end
function Index3i.ToString(t: { number }): string
	return string.format("[%d,%d,%d]", t[1], t[2], t[3])
end
function Index3i.HashKey(t: { number }): string
	return t[1] .. "," .. t[2] .. "," .. t[3]
end

---------------------------------------------------------------------------------------------------
local Index4i = {}
Index.Index4i = Index4i

function Index4i.new(a: number, b: number, c: number, d: number): { number }
	return { a, b, c, d }
end
function Index4i.Zero(): { number }
	return { 0, 0, 0, 0 }
end
function Index4i.Max(): { number }
	return { MathUtil.MaxInt, MathUtil.MaxInt, MathUtil.MaxInt, MathUtil.MaxInt }
end
function Index4i.Contains(t: { number }, val: number): boolean
	return t[1] == val or t[2] == val or t[3] == val or t[4] == val
end
-- sorts in place (C# Sort() mutates the struct)
function Index4i.Sort(t: { number })
	local a, b, c, d = t[1], t[2], t[3], t[4]
	if d < c then
		c, d = d, c
	end
	if c < b then
		b, c = c, b
	end
	if b < a then
		a, b = b, a
	end -- now a is smallest value
	if b > c then
		b, c = c, b
	end
	if c > d then
		c, d = d, c
	end -- now d is largest value
	if b > c then
		b, c = c, b
	end -- now b,c are sorted
	t[1], t[2], t[3], t[4] = a, b, c, d
end
function Index4i.Equals(x: { number }, y: { number }): boolean
	return x[1] == y[1] and x[2] == y[2] and x[3] == y[3] and x[4] == y[4]
end
function Index4i.ToString(t: { number }): string
	return string.format("[%d,%d,%d,%d]", t[1], t[2], t[3], t[4])
end

return Index
