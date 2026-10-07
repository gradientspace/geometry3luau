-- Verify the built packages: lune run tools/verify_package.luau
-- Deserializes dist/<profile>/geometry3luau.rbxmx (and geometry3luau.rbxm if built with Rojo), checks the
-- ModuleScript tree matches sources.json exactly, and that the production profile has no dev folders.
local fs = require("@lune/fs")
local serde = require("@lune/serde")
local roblox = require("@lune/roblox")

local failures = 0
for _, profile in { "production", "development" } do
	local directory = "dist/" .. profile
	if fs.isDir(directory) then
		local sources = serde.decode("json", fs.readFile(directory .. "/sources.json"))
		for _, filename in { "geometry3luau.rbxmx", "geometry3luau.rbxm" } do
			local path = directory .. "/" .. filename
			if fs.isFile(path) then
				local objects = roblox.deserializeModel(fs.readFile(path))
				assert(#objects == 1 and objects[1].ClassName == "ModuleScript", path .. ": expected one root ModuleScript")
				local root = objects[1]
				assert(root.Name == "geometry3luau", path .. ": root must be named geometry3luau")
				if profile == "production" then
					assert(
						root:FindFirstChild("Tests") == nil and root:FindFirstChild("Examples") == nil,
						path .. ": production includes dev folders"
					)
				end
				local count = 1
				for _, d in root:GetDescendants() do
					if d:IsA("ModuleScript") then
						count += 1
					end
				end
				if count ~= #sources then
					print(string.format("%s: %d ModuleScripts but %d sources", path, count, #sources))
					failures += 1
				end
				for _, source in sources do
					local module = root
					if source.path ~= "" then
						for _, name in string.split(source.path, "/") do
							module = module:FindFirstChild(name)
							if module == nil then
								break
							end
						end
					end
					if module == nil then
						print(path .. ": missing " .. source.path)
						failures += 1
					elseif module.Source ~= source.source then
						print(path .. ": source mismatch " .. source.path)
						failures += 1
					end
				end
				print(string.format("%s: %d ModuleScripts verified", path, count))
			end
		end
	end
end
if failures > 0 then
	error(failures .. " package verification failure(s)")
end
