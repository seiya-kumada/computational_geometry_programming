#include "chull2.h"
#include "utils.h"
#include <algorithm>
#include <numeric>

namespace cg = computational_geometry;

cg::Chull2::Chull2(const std::vector<Eigen::Vector2d> &input_points)
    : input_points_{input_points},// この添え字がNAMEに相当
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

auto cg::Chull2::set_initial_values() -> void{
    mright_ = 2;// 最初の三角形の右端頂点のインデックス(mol_の添え字)
    inext_ = 3;// 凸包に追加される次の頂点のインデックス(mol_の添え字)
}


auto cg::Chull2::add_other_input_points_one_by_one() -> void{
    const auto size = static_cast<int>(input_points_.size());
    while (inext_ < size)
    {
        auto newv = mol_.at(static_cast<std::size_t>(inext_)); // 追加する入力点番号
        auto muppv = generate_upper_part_of_convex_hull(newv); // 上接線の凸包頂点番号
        auto mlowv = generate_lower_part_of_convex_hull(newv); // 下接線の凸包頂点番号
        auto mv = fetch_unused_vertex_index(); // 新しい凸包頂点番号を取得
        kvert_.at(static_cast<std::size_t>(mv)) = newv;
        // ここでkccv_とkcv_を更新する
        // 上接線の更新
        kccv_.at(static_cast<std::size_t>(mv)) = muppv;
        kcv_.at(static_cast<std::size_t>(mv)) = mlowv;
        kcv_.at(static_cast<std::size_t>(muppv)) = mv;
        kccv_.at(static_cast<std::size_t>(mlowv)) = mv;
        ++inext_;
    }
    // この後でoutputを生成
    // generate_output();

}
auto cg::Chull2::generate_upper_part_of_convex_hull(int newv) -> int{
    // newvは入力点番号
    auto mv1 = mright_; // 右端の凸法頂点番号
    auto jv1 = kvert_.at(static_cast<std::size_t>(mv1)); // 入力点番号
    return find_upper_tangent_point(newv, mv1, jv1); 
}


// 再帰を使わずに実装する
auto cg::Chull2::find_upper_tangent_point(int newv, int mv1, int jv1) -> int{
    while (true) {
        auto mv2 = kccv_.at(static_cast<std::size_t>(mv1)); // 次の凸包頂点番号
        auto jv2 = kvert_.at(static_cast<std::size_t>(mv2)); // 入力点番号
        if (utils::turns_left(
            input_points_.at(static_cast<std::size_t>(newv)), 
            input_points_.at(static_cast<std::size_t>(jv1)), 
            input_points_.at(static_cast<std::size_t>(jv2)))) {
            // 上接線が見つかった
            auto muppv = mv1;
            return muppv;
        } else { // TODO: utils::turns_leftが3値を返すようにする。1, 0, -1
            // 次の頂点へ移動（反時計回り）
            mv1 = mv2;
            jv1 = jv2;
        }
    }
}
auto cg::Chull2::generate_lower_part_of_convex_hull(int newv) -> int{
    auto mv1 = mright_; // 右端の凸法頂点番号
    auto jv1 = kvert_.at(static_cast<std::size_t>(mv1)); // 入力点番号
    return find_lower_tangent_point(newv, mv1, jv1);
}
auto cg::Chull2::find_lower_tangent_point(int newv, int mv1, int jv1) -> int{
    while (true) {
        auto mv2 = kcv_.at(static_cast<std::size_t>(mv1)); // 次の凸包頂点番号
        auto jv2 = kvert_.at(static_cast<std::size_t>(mv2)); // 入力点番号
        if (!utils::turns_left(
            input_points_.at(static_cast<std::size_t>(newv)), 
            input_points_.at(static_cast<std::size_t>(jv1)), 
            input_points_.at(static_cast<std::size_t>(jv2)))) {
            // 下接線が見つかった
            auto mlowv = mv1;
            return mlowv;
        } else { // TODO: utils::turns_leftが3値を返すようにする。1, 0, -1
            // 次の頂点へ移動（時計回り）
            mv1 = mv2;
            jv1 = jv2;
        }
    }
}
auto cg::Chull2::replace_vertices() -> void{}
auto cg::Chull2::add_next_point() -> void{}
auto cg::Chull2::generate_output() -> void{}


auto cg::Chull2::fetch_unused_vertex_index() -> int {
    auto unused_index = kemp_;
    kemp_ = kvert_.at(static_cast<std::size_t>(kemp_));
    return unused_index;
}

