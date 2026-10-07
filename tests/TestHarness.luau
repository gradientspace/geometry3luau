-- Minimal test harness shared by all suites (headless under Lune, or inside Roblox via RunAllTests).
--
-- A suite module returns `function(G3, h)` where h = TestHarness.new(options):
--   h:test(name, fn)       register and run a test case (fn errors to fail)
--   h:skip(name, reason)   record a skipped case
--   h.native               true when running inside Roblox (engine APIs available)
--   h:results()            return { results = { {name, passed, skipped, error} } }
-- and assertion helpers h.eq(a, b, msg), h.near(a, b, eps, msg), h.truthy(v, msg).

local TestHarness = {}
TestHarness.__index = TestHarness

function TestHarness.new(options)
	options = options or {}
	return setmetatable({ native = options.native == true, suite = options.suite or "?", list = {}, verbose = options.verbose }, TestHarness)
end

function TestHarness:test(name: string, fn: () -> ())
	local started = os.clock()
	local ok, err = xpcall(fn, function(e)
		return tostring(e) .. "\n" .. debug.traceback()
	end)
	table.insert(self.list, { name = name, passed = ok, error = (not ok) and err or nil, seconds = os.clock() - started })
	if self.verbose then
		print(string.format("  %s %s (%.2fs)", ok and "ok  " or "FAIL", name, os.clock() - started))
	end
end

function TestHarness:skip(name: string, reason: string?)
	table.insert(self.list, { name = name, passed = false, skipped = true, error = reason })
end

function TestHarness:results()
	return { results = self.list }
end

local function fmt(v)
	if type(v) == "number" then
		return string.format("%.17g", v)
	end
	return tostring(v)
end

function TestHarness.eq(a, b, msg: string?)
	if a ~= b then
		error(string.format("%sexpected %s, got %s", msg and (msg .. ": ") or "", fmt(b), fmt(a)), 2)
	end
end

function TestHarness.near(a: number, b: number, eps: number?, msg: string?)
	local e = eps or 1e-9
	if not (math.abs(a - b) <= e) then
		error(string.format("%sexpected %s +- %g, got %s", msg and (msg .. ": ") or "", fmt(b), e, fmt(a)), 2)
	end
end

function TestHarness.truthy(v, msg: string?)
	if not v then
		error(msg or "expected truthy value", 2)
	end
end

return TestHarness
