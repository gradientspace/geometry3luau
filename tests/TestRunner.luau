-- Runs every *Tests suite in this folder inside Roblox (development package only).
--   local results = require(game.ReplicatedStorage.geometry3luau.Tests.TestRunner)()
--   require(...TestRunner)({ suites = { "DMesh3Tests" }, verbose = true })
-- Returns { passed, failed, skipped, failures = { {suite, name, error} } } and prints a summary.
-- Engine-dependent suites (RobloxMeshTests) run here; headless runs skip them.

return function(options)
	options = options or {}
	local testsFolder = script.Parent
	local G3 = require(testsFolder.Parent)
	local TestHarness = require(testsFolder.TestHarness)

	local names = {}
	if options.suites then
		names = options.suites
	else
		for _, child in testsFolder:GetChildren() do
			if child:IsA("ModuleScript") and string.match(child.Name, "Tests$") then
				table.insert(names, child.Name)
			end
		end
		table.sort(names)
	end

	local total = { passed = 0, failed = 0, skipped = 0, failures = {} }
	for _, name in names do
		local started = os.clock()
		local h = TestHarness.new({ native = true, suite = name, verbose = options.verbose })
		local ok, report = pcall(function()
			return require(testsFolder:FindFirstChild(name))(G3, h)
		end)
		local passed, failed, skipped = 0, 0, 0
		if not ok then
			failed = 1
			table.insert(total.failures, { suite = name, name = "<suite>", error = tostring(report) })
		else
			for _, case in report.results do
				if case.skipped then
					skipped += 1
				elseif case.passed then
					passed += 1
				else
					failed += 1
					table.insert(total.failures, { suite = name, name = case.name, error = tostring(case.error) })
				end
			end
		end
		total.passed += passed
		total.failed += failed
		total.skipped += skipped
		print(string.format("%-32s %4d passed %3d failed %3d skipped %7.2fs", name, passed, failed, skipped, os.clock() - started))
		if options.yield ~= false then
			task.wait()
		end
	end
	for _, f in total.failures do
		print("FAIL " .. f.suite .. "/" .. f.name .. ": " .. string.sub(f.error, 1, 1000))
	end
	print(string.format("TOTAL: %d passed, %d failed, %d skipped", total.passed, total.failed, total.skipped))
	return total
end
