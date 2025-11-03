#ifndef UTILS_H
#define UTILS_H
#include <eigen3/Eigen/Dense>

namespace utils
{
    bool turns_left(const Eigen::Vector2d &pi, const Eigen::Vector2d &pj, const Eigen::Vector2d &pk);
}
#endif // UTILS_H