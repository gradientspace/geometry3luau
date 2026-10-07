--!native
-- Port of geometry3Sharp core/Util.cs (subset relevant to Luau) and the FailMode enum.
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt

local Util = {}

-- C# enum FailMode { DebugAssert, gDevAssert, Throw, ReturnOnly }
Util.FailMode = table.freeze({
	DebugAssert = 0,
	gDevAssert = 1,
	Throw = 2,
	ReturnOnly = 3,
})

-- Equivalent of the C# DEBUG configuration. When false, gDevAssert and debug checks are no-ops.
Util.DebugChecks = false

-- When true, Util.DebugPrint forwards to print (replacement for System.Console.WriteLine diagnostics).
Util.DebugPrintEnabled = false

-- [Conditional("DEBUG")] gDevAssert: errors when the condition fails and debug checks are enabled.
function Util.gDevAssert(bValue: boolean, message: string?)
	if Util.DebugChecks and not bValue then
		error(message or "gDevAssert", 2)
	end
end

function Util.DebugPrint(...)
	if Util.DebugPrintEnabled then
		print(...)
	end
end

function Util.IsBitSet(n: number, pos: number): boolean
	return bit32.band(n, bit32.lshift(1, pos)) ~= 0
end

-- C# Array.Fill / new T[n] helpers for 1-based Luau arrays
function Util.MakeArray(n: number, value: any): { any }
	return table.create(n, value)
end

-- Returns a 1-based Luau array copy of the values produced by an iterator function.
function Util.ToArray(iter: () -> any?): { any }
	local result = {}
	for v in iter do
		table.insert(result, v)
	end
	return result
end

-- Count elements produced by an iterator function (C# IEnumerable.Count()).
function Util.Count(iter: () -> any?): number
	local n = 0
	for _ in iter do
		n += 1
	end
	return n
end

-- Iterate over a 1-based Luau array as a value iterator (for passing arrays where C# takes IEnumerable<int>).
function Util.ArrayItr(arr: { any }): () -> any?
	local i = 0
	return function()
		i += 1
		return arr[i]
	end
end

-- Iterate over integers [a, b] inclusive (Interval1i enumeration).
function Util.RangeItr(a: number, b: number): () -> number?
	local i = a - 1
	return function()
		i += 1
		if i <= b then
			return i
		end
		return nil
	end
end

return Util
