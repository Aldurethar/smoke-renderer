#pragma once

namespace constants
{
	template<typename Real>
	constexpr auto pi = Real(3.1415926535897932384626433832795l);

	template<typename Real>
	constexpr auto half_pi = Real(1.5707963267948966192313216916398l);

	template<typename Real>
	constexpr auto two_pi = Real(6.283185307179586476925286766559l);

	template<typename Real>
	constexpr auto rad_to_deg = Real(57.295779513082320876798154814105l);

	template<typename Real>
	constexpr auto deg_to_rad = Real(0.01745329251994329576923690768489l);
}
