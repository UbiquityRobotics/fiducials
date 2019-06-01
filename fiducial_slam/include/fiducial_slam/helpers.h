#include <cmath>
#include <memory>

// Basic make_unique helper function, similar to C++14 one
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

// Degrees to radians
constexpr double deg2rad(double deg) { return deg * M_PI / 180.0; }

// Radians to degrees
constexpr double rad2deg(double rad) { return rad * 180.0 / M_PI; }