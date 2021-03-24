#ifndef PI_HPP
#define PI_HPP

#include <cmath>

template <typename T> constexpr T pi = T(3.14159265358979323846264338327950);

template <typename T> constexpr T wrap_to_pi(T rad) {
	return std::remainder(rad, 2 * pi<T>);
}

template <typename T> constexpr T rad_to_deg(T rad) {
	return rad * T(180) / pi<T>;
}

template <typename T> constexpr T deg_to_rad(T deg) {
	return deg * pi<T> / T(180);
}

// static float wrap_to_pi(float number)
// {
//   if (number > (float)M_PI) {
//     return (number - (float)(2 * M_PI));
//   } else if (number < (float)(-1 * M_PI)) {
//     return (number + (float)(2 * M_PI));
//   } else {
//     return (number);
//   }
// }

#endif /* PI_HPP */
