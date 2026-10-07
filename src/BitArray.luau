-- Replacement for System.Collections.BitArray, used by several geometry3Sharp classes.
-- Indices are 0-based like C#. Stored as a Luau array of booleans.

local BitArray = {}
BitArray.__index = BitArray

function BitArray.new(count: number, defaultValue: boolean?)
	return setmetatable({ bits = table.create(count, defaultValue == true), Count = count }, BitArray)
end

function BitArray:Get(i: number): boolean
	return self.bits[i + 1]
end

function BitArray:Set(i: number, value: boolean)
	self.bits[i + 1] = value
end

function BitArray:SetAll(value: boolean)
	local bits = self.bits
	for i = 1, self.Count do
		bits[i] = value
	end
end

function BitArray:Length(): number
	return self.Count
end

return BitArray
