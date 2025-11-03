#include "chull2.h"
#include "utils.h"
#include <algorithm>
#include <numeric>

namespace cg = computational_geometry;

cg::Chull2::Chull2(const std::vector<Eigen::Vector2d> &input_points)
    : input_points_{input_points},
      mol_(input_points.size()),
      nvlist_(),
      kccv_(input_points.size()),
      kcv_(input_points.size()),
      kvert_(input_points.size()),
      kemp_{0}
{
    std::iota(mol_.begin(), mol_.end(), 0);
    sort_points_by_x_then_y();
}

auto cg::Chull2::sort_points_by_x_then_y() -> void
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

auto cg::Chull2::generate_initial_triangle() -> void
{
    // Implementation for generating the initial triangle of the convex hull
    const auto mol_0 = mol_.at(0);
    const auto mol_1 = mol_.at(1);
    const auto mol_2 = mol_.at(2);

    // この順にたどったとき反時計回りになる場合
    if (utils::turns_left(input_points_.at(static_cast<std::size_t>(mol_0)),
                          input_points_.at(static_cast<std::size_t>(mol_1)),
                          input_points_.at(static_cast<std::size_t>(mol_2))))
    {
        kvert_.at(0) = mol_0;
        kvert_.at(1) = mol_1;
        kvert_.at(2) = mol_2;
        kccv_.at(0) = 1;
        kccv_.at(1) = 2;
        kccv_.at(2) = 0;
        kcv_.at(0) = 2;
        kcv_.at(1) = 0;
        kcv_.at(2) = 1;
    }

    // 時計回りになる場合
    else
    {
        kvert_.at(0) = mol_0;
        kvert_.at(1) = mol_1;
        kvert_.at(2) = mol_2;
        kccv_.at(0) = 2;
        kccv_.at(1) = 0;
        kccv_.at(2) = 1;
        kcv_.at(0) = 1;
        kcv_.at(1) = 2;
        kcv_.at(2) = 0;
    }
}

auto cg::Chull2::set_initial_vertex_list_structure() -> void
{
    auto size = static_cast<int>(input_points_.size());
    for (auto i = 3; i < size - 1; ++i)
    {
        kvert_[i] = i + 1;
    }
    kvert_.back() = 0;
    kemp_ = 3;
}
