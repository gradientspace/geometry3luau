-- Port of geometry3Sharp core/gParallel.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau has no shared-memory threads, so all "parallel" loops run sequentially, in order.
-- The functions keep the C# signatures so ported code reads the same.
-- `source` may be: an iterator function, a 1-based array, or an Interval1i (iterates a..b).

local gParallel = {}

local function forEachSequential(source: any, body: (any) -> ())
	if type(source) == "function" then
		for v in source do
			body(v)
		end
	elseif source.a ~= nil and source.b ~= nil and type(source.a) == "number" then
		-- Interval1i
		for i = source.a, source.b do
			body(i)
		end
	else
		for _, v in ipairs(source) do
			body(v)
		end
	end
end

function gParallel.ForEach_Sequential(source: any, body: (any) -> ())
	forEachSequential(source, body)
end

function gParallel.ForEach(source: any, body: (any) -> ())
	forEachSequential(source, body)
end

-- Evaluate(params Action[] funcs)
function gParallel.Evaluate(...: () -> ())
	for _, f in { ... } do
		f()
	end
end

-- Process indices [iStart, iEnd] in blocks: blockF(blockStart, blockEnd) (inclusive)
function gParallel.BlockStartEnd(
	iStart: number,
	iEnd: number,
	blockF: (number, number) -> (),
	iBlockSize: number?,
	bDisableParallel: boolean?
)
	local blockSize = iBlockSize or -1
	if blockSize == -1 then
		blockSize = 100 -- seems to work
	end
	local N = (iEnd - iStart + 1)
	local num_blocks = N // blockSize
	-- process main blocks
	for bi = 0, num_blocks - 1 do
		local k = iStart + blockSize * bi
		blockF(k, k + blockSize - 1)
	end
	-- process leftover elements
	local remaining = N - (num_blocks * blockSize)
	if remaining > 0 then
		local k = iStart + num_blocks * blockSize
		blockF(k, k + remaining - 1)
	end
end

return gParallel
