#include "chull2.h"
#include "utils.h"
#include <algorithm>
#include <numeric>

namespace cg = computational_geometry;

cg::Chull2::Chull2(const std::vector<Eigen::Vector2d> &input_points)
    : input_points_{input_points},
      mol_(input_points.size()),
      nvlist_{},
      kccv_(input_points.size()),
      kcv_(input_points.size())
{
    std::iota(mol_.begin(), mol_.end(), 0);
    sort_points_by_x();
}

auto cg::Chull2::sort_points_by_x() -> void
{
    std::sort(mol_.begin(), mol_.end(),
              [this](int lhs, int rhs)
              {
                  const auto &lhs_point = input_points_.at(static_cast<std::size_t>(lhs));
                  const auto &rhs_point = input_points_.at(static_cast<std::size_t>(rhs));
                  if (lhs_point.x() == rhs_point.x())
                  {
                      return lhs_point.y() < rhs_point.y();
                  }
                  return lhs_point.x() < rhs_point.x();
              });
}

auto cg::Chull2::sorted_indices() const -> const std::vector<int> &
{
    return mol_;
}
auto cg::Chull2::generate_initial_triangle() -> void
{
    // Implementation for generating the initial triangle of the convex hull
    const auto p0_index = mol_.at(0);
    const auto p1_index = mol_.at(1);
    const auto p2_index = mol_.at(2);

    // この順にたどったとき反時計回りになる場合
    if (utils::turns_left(input_points_.at(static_cast<std::size_t>(p0_index)),
                          input_points_.at(static_cast<std::size_t>(p1_index)),
                          input_points_.at(static_cast<std::size_t>(p2_index))))
    {
        kccv_.at(0) = 1;
        kccv_.at(1) = 2;
        kccv_.at(2) = 0;
        kcv_.at(0) = 2;
        kcv_.at(1) = 0;
        kcv_.at(2) = 1;
    }

    // 時計回りになる場合
}