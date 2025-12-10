#ifndef UTILS_H
#define UTILS_H
#include <eigen3/Eigen/Dense>

namespace utils
{
bool turns_left(const Eigen::Vector2d& pi, const Eigen::Vector2d& pj, const Eigen::Vector2d& pk);
void save_convex_hull_to_json(const std::string& filename,
                              const std::vector<Eigen::Vector2d>& points,
                              const std::vector<int>& convex_hull_indices);
}  // namespace utils
#endif  // UTILS_H