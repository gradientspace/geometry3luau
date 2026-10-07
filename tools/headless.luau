-- Lune adapter that runs the test suites outside Roblox.
--
-- Mirrors the package tree (root ModuleScript = src/init.luau with every src/*.luau as a child, plus a
-- Tests folder and an Examples folder) and gives each module a `script` object with Name/Parent/children,
-- so `require(script.Parent.X)` works exactly as in Roblox. Roblox data types (Vector3, Vector2, Color3,
-- CFrame) come from @lune/roblox. Engine services are not available: tests that need them should
-- check `harness.native` and skip.
--
-- Usage: lune run tools/headless.luau [--suite Name ...] [--out path] [--bench]
-- Without --suite, every tests/*Tests.luau suite runs.
local fs = require("@lune/fs")
local luau = require("@lune/luau")
local roblox = require("@lune/roblox")
local serde = require("@lune/serde")
local task = require("@lune/task")
local process = require("@lune/process")

local function node(name, parent, path)
	return { Name = name, Parent = parent, path = path, source = path and fs.readFile(path), children = {} }
end

local root = node("geometry3luau", nil, "src/init.luau")
local testsFolder = node("Tests", root, nil)
local examplesFolder = node("Examples", root, nil)
root.children.Tests = testsFolder
root.children.Examples = examplesFolder

local function mount(folder, parent)
	if not fs.isDir(folder) then
		return
	end
	for _, filename in fs.readDir(folder) do
		local name = string.match(filename, "^(.+)%.luau$")
		if name and name ~= "init" then
			parent.children[name] = node(name, parent, folder .. "/" .. filename)
		end
	end
end
mount("src", root)
mount("tests", testsFolder)
mount("examples", examplesFolder)

-- read-only Instance view of the tree
local instances = {}
local function wrap(n)
	if n == nil then
		return nil
	end
	if instances[n] == nil then
		instances[n] = setmetatable({}, {
			__index = function(_, key)
				if key == "Name" then
					return n.Name
				elseif key == "Parent" then
					return wrap(n.Parent)
				elseif key == "FindFirstChild" then
					return function(_, name)
						return wrap(n.children[name])
					end
				elseif key == "GetChildren" then
					return function()
						local list = {}
						for _, child in n.children do
							table.insert(list, wrap(child))
						end
						table.sort(list, function(a, b)
							return a.Name < b.Name
						end)
						return list
					end
				elseif key == "IsA" then
					return function(_, className)
						return (className == "ModuleScript" and n.source ~= nil) or (className == "Folder" and n.source == nil)
					end
				elseif key == "__node" then
					return n
				end
				local child = n.children[key]
				if child == nil then
					error(tostring(key) .. " is not a valid member of " .. n.Name, 2)
				end
				return wrap(child)
			end,
			__tostring = function()
				return n.Name
			end,
		})
	end
	return instances[n]
end

local NATIVE_SENTINEL = "Roblox engine API unavailable in headless tests"
local function native()
	error(NATIVE_SENTINEL, 2)
end
local nativeObject = setmetatable({}, { __index = native })
local game = setmetatable({
	GetService = function(_, name)
		return nativeObject
	end,
}, { __index = native })

local INJECT = "local script, require, game, workspace, task, Instance, Content, Vector2, Vector3, Color3, CFrame, Enum = ...; "
local function inject(source: string): string
	local pos = 1
	while string.sub(source, pos, pos + 2) == "--!" do
		local nl = string.find(source, "\n", pos, true)
		if nl == nil then
			break
		end
		pos = nl + 1
	end
	return string.sub(source, 1, pos - 1) .. INJECT .. string.sub(source, pos)
end

local cache = {}
local loading = {}
local function load(instance)
	assert(instance ~= nil, "Attempted to call require with invalid argument(s).")
	local target = instance.__node
	assert(target and target.source, "Not a ModuleScript: " .. tostring(instance))
	if cache[target] ~= nil then
		return cache[target]
	end
	if loading[target] then
		error("Requested module was required recursively: " .. target.path)
	end
	loading[target] = true
	-- Engine globals are injected as chunk arguments rather than through a custom `environment`: a custom
	-- environment disables Luau's builtin fast paths and native code generation, which makes headless
	-- timings ~10x slower than Roblox. The injected `local ... = ...;` is placed at the start of the first
	-- non-directive line, so line numbers in error messages are unchanged and --!native still applies.
	local value = luau.load(inject(target.source), { debugName = target.path, codegenEnabled = true })(
		wrap(target),
		load,
		game,
		nativeObject,
		task,
		nativeObject,
		nativeObject,
		roblox.Vector2,
		roblox.Vector3,
		roblox.Color3,
		roblox.CFrame,
		roblox.Enum
	)
	loading[target] = nil
	cache[target] = value
	return value
end

local selected, out, bench = {}, nil, false
local args = process.args
local i = 1
while i <= #args do
	if args[i] == "--suite" then
		table.insert(selected, args[i + 1])
		i += 1
	elseif args[i] == "--out" then
		out = args[i + 1]
		i += 1
	elseif args[i] == "--bench" then
		bench = true
	else
		error("Unknown argument " .. args[i])
	end
	i += 1
end

local names = selected
if #names == 0 then
	for name in testsFolder.children do
		if string.match(name, "Tests$") then
			table.insert(names, name)
		end
	end
	table.sort(names)
end

local G3 = load(wrap(root))
local harness = load(wrap(testsFolder.children.TestHarness))

local result = { passed = 0, failed = 0, skipped = 0, suites = {} }
for _, name in names do
	local started = os.clock()
	local ok, report = pcall(function()
		local suite = assert(testsFolder.children[name], "Unknown suite " .. name)
		return load(wrap(suite))(G3, harness.new({ native = false, suite = name }))
	end)
	local summary = { passed = 0, failed = 0, skipped = 0, failures = {}, seconds = 0 }
	if not ok then
		summary.failed = 1
		summary.failures = { { name = "<suite>", error = tostring(report) } }
	else
		for _, case in report.results do
			if case.skipped then
				summary.skipped += 1
			elseif case.passed then
				summary.passed += 1
			elseif string.find(tostring(case.error), NATIVE_SENTINEL, 1, true) then
				summary.skipped += 1
			else
				summary.failed += 1
				table.insert(summary.failures, { name = case.name, error = tostring(case.error) })
			end
		end
	end
	summary.seconds = math.floor((os.clock() - started) * 100) / 100
	result.suites[name] = summary
	result.passed += summary.passed
	result.failed += summary.failed
	result.skipped += summary.skipped
	print(
		string.format(
			"%-32s %4d passed %3d failed %3d skipped %7.2fs",
			name,
			summary.passed,
			summary.failed,
			summary.skipped,
			summary.seconds
		)
	)
	for _, failure in summary.failures do
		print("    FAIL " .. failure.name .. ": " .. string.sub(failure.error, 1, 2000))
	end
end
print(string.format("TOTAL: %d passed, %d failed, %d skipped", result.passed, result.failed, result.skipped))

if bench then
	local benchmarks = examplesFolder.children.Benchmarks
	if benchmarks then
		load(wrap(benchmarks))(G3)
	end
end

if out then
	fs.writeFile(out, serde.encode("json", result, true))
end
process.exit(result.failed == 0 and 0 or 1)
