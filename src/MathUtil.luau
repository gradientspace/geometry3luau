--!native
-- Port of geometry3Sharp math/MathUtil.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Functions that take Vector3d arguments require the Vector3d module lazily to avoid a require cycle
-- (Vector3d uses MathUtil constants).

local MathUtil = {}

local abs, sqrt, atan2, floor, ceil = math.abs, math.sqrt, math.atan2, math.floor, math.ceil

MathUtil.Deg2Rad = math.pi / 180.0
MathUtil.Rad2Deg = 180.0 / math.pi
MathUtil.TwoPI = 2.0 * math.pi
MathUtil.FourPI = 4.0 * math.pi
MathUtil.HalfPI = 0.5 * math.pi
MathUtil.ZeroTolerance = 1e-08
MathUtil.Epsilon = 2.2204460492503131e-016
MathUtil.SqrtTwo = 1.41421356237309504880168872420969807
MathUtil.SqrtTwoInv = 1.0 / 1.41421356237309504880168872420969807
MathUtil.SqrtThree = 1.73205080756887729352744634150587236

MathUtil.Deg2Radf = math.pi / 180.0
MathUtil.Rad2Degf = 180.0 / math.pi
MathUtil.PIf = math.pi
MathUtil.TwoPIf = 2.0 * math.pi
MathUtil.HalfPIf = 0.5 * math.pi
MathUtil.SqrtTwof = 1.41421356237
MathUtil.ZeroTolerancef = 1e-06
MathUtil.Epsilonf = 1.192092896e-07

-- C# numeric limits. These are finite values; never replace them with math.huge.
MathUtil.MaxDouble = 1.7976931348623157e308 -- double.MaxValue
MathUtil.MinDouble = -1.7976931348623157e308 -- double.MinValue
MathUtil.MaxFloat = 3.4028234663852886e38 -- float.MaxValue
MathUtil.MinFloat = -3.4028234663852886e38 -- float.MinValue
MathUtil.MaxInt = 2147483647 -- int.MaxValue
MathUtil.MinInt = -2147483648 -- int.MinValue

---------------------------------------------------------------------------------------------------
-- integer helpers for C# semantics

-- C# (int)x : truncation toward zero
function MathUtil.TruncInt(x: number): number
	if x >= 0 then
		return floor(x)
	else
		return ceil(x)
	end
end

-- C# integer division a / b (truncates toward zero)
function MathUtil.IntDiv(a: number, b: number): number
	local q = a / b
	if q >= 0 then
		return floor(q)
	else
		return ceil(q)
	end
end

-- C# integer remainder a % b (result has the sign of a)
function MathUtil.IntMod(a: number, b: number): number
	return math.fmod(a, b)
end

-- C# Math.Sign
function MathUtil.Sign(f: number): number
	if f > 0 then
		return 1
	elseif f < 0 then
		return -1
	end
	return 0
end

-- C# Math.Round(x) uses banker's rounding (MidpointRounding.ToEven)
function MathUtil.RoundToEven(x: number): number
	local f = floor(x)
	local d = x - f
	if d > 0.5 then
		return f + 1
	elseif d < 0.5 then
		return f
	else
		return (f % 2 == 0) and f or f + 1
	end
end

-- C# Math.Round(x, nDecimals)
function MathUtil.Round(x: number, nDecimals: number?): number
	if nDecimals == nil or nDecimals == 0 then
		return MathUtil.RoundToEven(x)
	end
	local s = 10 ^ nDecimals
	return MathUtil.RoundToEven(x * s) / s
end

---------------------------------------------------------------------------------------------------

function MathUtil.IsFinite(d: number): boolean
	return d == d and d ~= math.huge and d ~= -math.huge
end

function MathUtil.EpsilonEqual(a: number, b: number, epsilon: number?): boolean
	return abs(a - b) <= (epsilon or MathUtil.Epsilon)
end

function MathUtil.Clamp(f: number, low: number, high: number): number
	if f < low then
		return low
	elseif f > high then
		return high
	end
	return f
end

function MathUtil.ModuloClamp(f: number, N: number): number
	while f < 0 do
		f += N
	end
	return f % N
end

function MathUtil.RangeClamp(fValue: number, fMinMaxValue: number): number
	return MathUtil.Clamp(fValue, -abs(fMinMaxValue), abs(fMinMaxValue))
end

-- SignedClamp(f, fMax) or SignedClamp(f, fMin, fMax)
function MathUtil.SignedClamp(f: number, a: number, b: number?): number
	if b == nil then
		return MathUtil.Clamp(abs(f), 0, a) * MathUtil.Sign(f)
	end
	return MathUtil.Clamp(abs(f), a, b) * MathUtil.Sign(f)
end

function MathUtil.InRange(f: number, low: number, high: number): boolean
	return f >= low and f <= high
end

function MathUtil.ClampAngleDeg(theta: number, min: number, max: number): number
	local c = (min + max) * 0.5
	local e = max - c
	theta = math.fmod(theta, 360)
	theta -= c
	if theta < -180 then
		theta += 360
	elseif theta > 180 then
		theta -= 360
	end
	if theta < -e then
		theta = -e
	elseif theta > e then
		theta = e
	end
	return theta + c
end

function MathUtil.ClampAngleRad(theta: number, min: number, max: number): number
	local c = (min + max) * 0.5
	local e = max - c
	theta = math.fmod(theta, MathUtil.TwoPI)
	theta -= c
	if theta < -math.pi then
		theta += MathUtil.TwoPI
	elseif theta > math.pi then
		theta -= MathUtil.TwoPI
	end
	if theta < -e then
		theta = -e
	elseif theta > e then
		theta = e
	end
	return theta + c
end

function MathUtil.WrapSignedIndex(val: number, mod: number): number
	while val < 0 do
		val += mod
	end
	return val % mod
end

-- C# MinMax(a, b, c, out min, out max) -> returns min, max
function MathUtil.MinMax(a: number, b: number, c: number): (number, number)
	if a < b then
		if a < c then
			return a, math.max(b, c)
		else
			return c, b
		end
	else
		if a > c then
			return math.min(b, c), a
		else
			return b, c
		end
	end
end

function MathUtil.Min(a: number, b: number, c: number): number
	return math.min(a, math.min(b, c))
end

function MathUtil.Max(a: number, b: number, c: number): number
	return math.max(a, math.max(b, c))
end

-- NOTE: matches C# implementation exactly (which returns f / sqrt(f), ie sqrt(f))
function MathUtil.InvSqrt(f: number): number
	return f / sqrt(f)
end

function MathUtil.Atan2Positive(y: number, x: number, bInRadians: boolean?): number
	local theta = atan2(y, x)
	if theta < 0 then
		theta = (2 * math.pi) + theta
	end
	if bInRadians == false then
		return MathUtil.Rad2Deg * theta
	end
	return theta
end

-- PlaneAngleD(Vector3d a, Vector3d b, int nPlaneNormalIdx = 1)
function MathUtil.PlaneAngleD(a, b, nPlaneNormalIdx: number?)
	local idx = nPlaneNormalIdx or 1
	a = a:With(idx, 0):Normalized()
	b = b:With(idx, 0):Normalized()
	return a:AngleD(b)
end

-- PlaneAngleSignedD(vFrom, vTo, nPlaneNormalIdx: int) or PlaneAngleSignedD(vFrom, vTo, planeN: Vector3d)
-- or the 2D variant PlaneAngleSignedD(Vector2d, Vector2d)
function MathUtil.PlaneAngleSignedD(vFrom, vTo, planeNormal)
	if vFrom.z == nil then
		-- Vector2d variant
		vFrom = vFrom:Normalized()
		vTo = vTo:Normalized()
		local fSign = MathUtil.Sign(vFrom:Cross(vTo))
		return fSign * vFrom:AngleD(vTo)
	end
	if planeNormal == nil or type(planeNormal) == "number" then
		local idx = planeNormal or 1
		vFrom = vFrom:With(idx, 0):Normalized()
		vTo = vTo:With(idx, 0):Normalized()
		local c = vFrom:Cross(vTo)
		if c:LengthSquared() < MathUtil.ZeroTolerance then -- vectors are parallel
			return (vFrom:Dot(vTo) < 0) and 180.0 or 0
		end
		local fSign = MathUtil.Sign(c:Get(idx))
		return fSign * vFrom:AngleD(vTo)
	else
		local planeN = planeNormal
		vFrom = vFrom - vFrom:Dot(planeN) * planeN
		vTo = vTo - vTo:Dot(planeN) * planeN
		vFrom = vFrom:Normalized()
		vTo = vTo:Normalized()
		local c = vFrom:Cross(vTo)
		if c:LengthSquared() < MathUtil.ZeroTolerance then -- vectors are parallel
			return (vFrom:Dot(vTo) < 0) and 180.0 or 0
		end
		local fSign = MathUtil.Sign(c:Dot(planeN))
		return fSign * vFrom:AngleD(vTo)
	end
end

-- MostParallelAxis(Frame3d f, Vector3d vDir)
function MathUtil.MostParallelAxis(f, vDir): number
	local dot0 = abs(f:X():Dot(vDir))
	local dot1 = abs(f:Y():Dot(vDir))
	local dot2 = abs(f:Z():Dot(vDir))
	local m = math.max(dot0, math.max(dot1, dot2))
	return (m == dot0) and 0 or ((m == dot1) and 1 or 2)
end

function MathUtil.Lerp(a: number, b: number, t: number): number
	return (1.0 - t) * a + t * b
end

function MathUtil.SmoothStep(a: number, b: number, t: number): number
	t = t * t * (3.0 - 2.0 * t)
	return (1.0 - t) * a + t * b
end

function MathUtil.WyvillRise01(fX: number): number
	local d = MathUtil.Clamp(1.0 - fX * fX, 0.0, 1.0)
	return 1 - (d * d * d)
end

function MathUtil.SmoothInterp(a: number, b: number, t: number): number
	local tt = MathUtil.WyvillRise01(t)
	return (1.0 - tt) * a + tt * b
end

function MathUtil.SmoothRise0To1(fX: number, yshift: number, xZero: number, speed: number): number
	local denom = (fX - (xZero - 1)) ^ speed
	local fY = (1 + yshift) + (1 / -denom)
	return MathUtil.Clamp(fY, 0, 1)
end

function MathUtil.WyvillFalloff01(fX: number): number
	local d = 1 - fX * fX
	return (d >= 0) and (d * d * d) or 0
end

function MathUtil.WyvillFalloff(fD: number, fInnerRad: number, fOuterRad: number): number
	if fD > fOuterRad then
		return 0
	elseif fD > fInnerRad then
		fD -= fInnerRad
		fD /= (fOuterRad - fInnerRad)
		fD = math.max(0, math.min(1, fD))
		local fVal = (1.0 - fD * fD)
		return fVal * fVal * fVal
	end
	return 1.0
end

function MathUtil.LinearRampT(R: number, deadzoneR: number, x: number): number
	local sign = MathUtil.Sign(x)
	x = abs(x)
	if x < deadzoneR then
		return 0.0
	elseif x > R then
		return sign * 1.0
	else
		x = math.min(x, R)
		local d = (x - deadzoneR) / (R - deadzoneR)
		return sign * d
	end
end

---------------------------------------------------------------------------------------------------
-- triangle functions (Vector3d arguments)

function MathUtil.Area(v1, v2, v3): number
	return 0.5 * (v2 - v1):Cross(v3 - v1):Length()
end

function MathUtil.Normal(v1, v2, v3)
	local edge1 = (v2 - v1):Normalized()
	local edge2 = (v3 - v2):Normalized()
	return edge1:Cross(edge2):Normalized()
end

function MathUtil.FastNormalDirection(v1, v2, v3)
	local edge1 = v2 - v1
	local edge2 = v3 - v1
	return edge1:Cross(edge2)
end

-- C# FastNormalArea(ref v1, ref v2, ref v3, out double area) -> returns normal, area
function MathUtil.FastNormalArea(v1, v2, v3)
	local edge1 = v2 - v1
	local edge2 = v3 - v1
	local vCross, len = edge1:Cross(edge2):NormalizedAndLength()
	return vCross, 0.5 * len
end

function MathUtil.AspectRatio(v1, v2, v3): number
	local a, b, c = v1:Distance(v2), v2:Distance(v3), v3:Distance(v1)
	local s = (a + b + c) / 2.0
	return (a * b * c) / (8.0 * (s - a) * (s - b) * (s - c))
end

function MathUtil.VectorCot(v1, v2): number
	local fDot = v1:Dot(v2)
	local lensqr1 = v1:LengthSquared()
	local lensqr2 = v2:LengthSquared()
	local d = MathUtil.Clamp(lensqr1 * lensqr2 - fDot * fDot, 0.0, MathUtil.MaxDouble)
	if d < MathUtil.ZeroTolerance then
		return 0
	else
		return fDot / sqrt(d)
	end
end

function MathUtil.VectorTan(v1, v2): number
	local fDot = v1:Dot(v2)
	local lensqr1 = v1:LengthSquared()
	local lensqr2 = v2:LengthSquared()
	local d = MathUtil.Clamp(lensqr1 * lensqr2 - fDot * fDot, 0.0, MathUtil.MaxDouble)
	if d == 0 then
		return 0
	end
	return sqrt(d) / fDot
end

function MathUtil.IsObtuse(v1, v2, v3): boolean
	local a2 = v1:DistanceSquared(v2)
	local b2 = v1:DistanceSquared(v3)
	local c2 = v2:DistanceSquared(v3)
	return (a2 + b2 < c2) or (b2 + c2 < a2) or (c2 + a2 < b2)
end

-- IsLeft(Vector2d P0, P1, P2)
function MathUtil.IsLeft(P0, P1, P2): number
	return MathUtil.Sign(((P1.x - P0.x) * (P2.y - P0.y) - (P2.x - P0.x) * (P1.y - P0.y)))
end

-- BarycentricCoords(vPoint, V0, V1, V2) for Vector3d or Vector2d arguments; returns Vector3d
function MathUtil.BarycentricCoords(vPoint, V0, V1, V2)
	local Vector3d = require(script.Parent.Vector3d)
	local kV02 = V0 - V2
	local kV12 = V1 - V2
	local kPV2 = vPoint - V2
	local fM00 = kV02:Dot(kV02)
	local fM01 = kV02:Dot(kV12)
	local fM11 = kV12:Dot(kV12)
	local fR0 = kV02:Dot(kPV2)
	local fR1 = kV12:Dot(kPV2)
	local fDet = fM00 * fM11 - fM01 * fM01
	local fInvDet = 1.0 / fDet
	local fBary1 = (fM11 * fR0 - fM01 * fR1) * fInvDet
	local fBary2 = (fM00 * fR1 - fM01 * fR0) * fInvDet
	local fBary3 = 1.0 - fBary1 - fBary2
	return Vector3d.new(fBary1, fBary2, fBary3)
end

-- TriSolidAngle(Vector3d a, b, c, ref Vector3d p)
function MathUtil.TriSolidAngle(a, b, c, p): number
	local ax, ay, az = a.x - p.x, a.y - p.y, a.z - p.z
	local bx, by, bz = b.x - p.x, b.y - p.y, b.z - p.z
	local cx, cy, cz = c.x - p.x, c.y - p.y, c.z - p.z
	local la = sqrt(ax * ax + ay * ay + az * az)
	local lb = sqrt(bx * bx + by * by + bz * bz)
	local lc = sqrt(cx * cx + cy * cy + cz * cz)
	local bottom = (la * lb * lc)
		+ (ax * bx + ay * by + az * bz) * lc
		+ (bx * cx + by * cy + bz * cz) * la
		+ (cx * ax + cy * ay + cz * az) * lb
	local top = ax * (by * cz - cy * bz) - ay * (bx * cz - cx * bz) + az * (bx * cy - cx * by)
	return 2.0 * atan2(top, bottom)
end

-- SolveQuadratic(a, b, c, out minT, out maxT) -> returns ok, minT, maxT
function MathUtil.SolveQuadratic(a: number, b: number, c: number): (boolean, number, number)
	local minT, maxT = 0, 0
	if a == 0 and b == 0 then -- function is constant...
		return true, minT, maxT
	end
	local discrim = b * b - 4.0 * a * c
	if discrim < 0 then
		return false, minT, maxT -- no solution
	end
	-- a bit odd but this is the "stable" way to do it (avoid cancellation)
	local t = -0.5 * (b + MathUtil.Sign(b) * sqrt(discrim))
	minT = t / a
	maxT = c / t
	if minT > maxT then
		minT, maxT = maxT, minT
	end
	return true, minT, maxT
end

local powers_of_10 = { 1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000 }
function MathUtil.PowerOf10(n: number): number
	return powers_of_10[n + 1]
end

-- ModuloIteration(nMaxExclusive, nPrime = 2147483647): iterate over [0, nMaxExclusive) in a
-- "random" order by stepping a prime modulo N.
function MathUtil.ModuloIteration(nMaxExclusive: number, nPrime: number?): () -> number?
	local prime = nPrime or 2147483647
	local i = 0
	local done = false
	return function()
		if done then
			return nil
		end
		local cur = i
		if nMaxExclusive <= 0 then
			-- C# yields 0 and then throws DivideByZeroException; without this check Luau would yield NaN forever
			if cur ~= 0 then
				error("MathUtil.ModuloIteration: nMaxExclusive must be positive (division by zero)")
			end
			i = -1
			return cur
		end
		i = (i + prime) % nMaxExclusive
		done = (i == 0)
		return cur
	end
end

return MathUtil
