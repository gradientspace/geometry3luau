-- Port of geometry3Sharp core/ProgressCancel.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- ProgressCancel is passed to long-running computes (Remesher.Progress, Reducer.Progress, ...) and is
-- polled via Cancelled(). The source can be:
--   * a function returning true to cancel (C# CancelFunction / Func<bool>)
--   * an object with a :Cancelled() method (C# ICancelSource)
--
-- Roblox note: a long compute in a single frame can hit the script timeout. A cancel function is
-- polled regularly from inside the algorithms, so it is a natural place to yield:
--   ProgressCancel.new(function() if os.clock() - last > 0.1 then task.wait(); last = os.clock() end return false end)
-- ProgressCancel.Yielding(seconds) builds exactly that.

local ProgressCancel = {}
ProgressCancel.__index = ProgressCancel

-- C# CancelFunction : ICancelSource
local CancelFunction = {}
CancelFunction.__index = CancelFunction
function CancelFunction.new(cancelF: () -> boolean)
	return setmetatable({ CancelF = cancelF }, CancelFunction)
end
function CancelFunction:Cancelled(): boolean
	return self.CancelF()
end
ProgressCancel.CancelFunction = CancelFunction

function ProgressCancel.new(source: any)
	if type(source) == "function" then
		source = CancelFunction.new(source)
	end
	return setmetatable({ Source = source, WasCancelled = false }, ProgressCancel)
end

function ProgressCancel:Cancelled(): boolean
	if self.WasCancelled then
		return true
	end
	self.WasCancelled = self.Source:Cancelled()
	return self.WasCancelled
end

-- Creates a ProgressCancel that never cancels, but yields (task.wait) whenever more than
-- `seconds` (default 0.1) of wall-clock time has elapsed since the last yield. Optionally
-- also cancels when shouldCancel() returns true. Only usable where task.wait exists (Roblox).
function ProgressCancel.Yielding(seconds: number?, shouldCancel: (() -> boolean)?)
	local interval = seconds or 0.1
	local last = os.clock()
	return ProgressCancel.new(function()
		local now = os.clock()
		if now - last > interval then
			local t = task :: any
			if t and t.wait then
				t.wait()
			end
			last = os.clock()
		end
		if shouldCancel then
			return shouldCancel()
		end
		return false
	end)
end

return ProgressCancel
