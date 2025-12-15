#include "chull2.h"

#include "utils.h"

#include <algorithm>
#include <numeric>
namespace cg = computational_geometry;

cg::Chull2::Chull2(const std::vector<Eigen::Vector2d>& input_points)
    // clang-format off
    : input_points_{input_points},  // この添え字がテキストのNAMEに相当
      input_point_size_{static_cast<int>(input_points.size())}, 
      mol_(input_points.size()),
      kccv_(input_points.size()), 
      kcv_(input_points.size()), 
      kvert_(input_points.size()), 
      kemp_{0}
// clang-format on
{
    std::iota(mol_.begin(), mol_.end(), 0);
}

cg::Chull2::Chull2(std::vector<Eigen::Vector2d>&& input_points)
    // clang-format off
    : input_points_{std::move(input_points)},  // この添え字がテキストのNAMEに相当
      input_point_size_{static_cast<int>(input_points_.size())}, 
      mol_(input_points_.size()),
      kccv_(input_points_.size()), 
      kcv_(input_points_.size()), 
      kvert_(input_points_.size()),
      kemp_{0}
// clang-format on
{
    std::iota(mol_.begin(), mol_.end(), 0);
}

auto cg::Chull2::execute() -> std::expected<std::vector<int>, std::string>
{
    if (input_point_size_ < 3)
    {
        return std::unexpected<std::string>("入力点が3点未満です。凸包を計算できません。");
    }
    // 最初に入力点をx座標優先でソートする
    sort_points_by_x_then_y();

    // 最初の三角形を作成する
    generate_initial_triangle();

    // 頂点リスト構造を初期化する
    set_initial_vertex_list_structure();

    // 初期値を設定する
    set_initial_values();

    // 残りの入力点を1点ずつ凸包に追加していく
    add_other_input_points_one_by_one();

    // 凸包頂点リストを生成する
    return generate_output();
}

auto cg::Chull2::generate_output() -> std::vector<int>
{
    std::vector<int> output_points;
    output_points.reserve(input_points_.size());
    auto mv = 0;
    auto mstart = mv;
    do
    {
        const auto jv = kvert_[mv];
        output_points.push_back(jv);
        mv = kccv_[mv];
    } while (mv != mstart);

    return output_points;
}

auto cg::Chull2::sort_points_by_x_then_y() -> void
{
    std::sort(mol_.begin(), mol_.end(),
              [this](int lhs, int rhs)
              {
                  const auto& lhs_point = input_points_[lhs];
                  const auto& rhs_point = input_points_[rhs];
                  if (lhs_point.x() == rhs_point.x())
                  {
                      return lhs_point.y() < rhs_point.y();
                  }
                  return lhs_point.x() < rhs_point.x();
              });
}

auto cg::Chull2::generate_initial_triangle() -> void
{
    // 最初の三角形を生成する
    const auto& mol_0 = mol_[0];
    const auto& mol_1 = mol_[1];
    const auto& mol_2 = mol_[2];
    kvert_[0] = mol_0;
    kvert_[1] = mol_1;
    kvert_[2] = mol_2;

    // この順にたどったとき反時計回りになる場合
    if (utils::turns_left(input_points_[mol_0], input_points_[mol_1], input_points_[mol_2]))
    {
        kccv_[0] = 1;
        kccv_[1] = 2;
        kccv_[2] = 0;
        kcv_[0] = 2;
        kcv_[1] = 0;
        kcv_[2] = 1;
    }

    // 時計回りになる場合
    else
    {
        kccv_[0] = 2;
        kccv_[1] = 0;
        kccv_[2] = 1;
        kcv_[0] = 1;
        kcv_[1] = 2;
        kcv_[2] = 0;
    }
}

auto cg::Chull2::set_initial_vertex_list_structure() -> void
{
    for (auto i = 3; i < input_point_size_ - 1; ++i)
    {
        kvert_[i] = i + 1;
    }
    kvert_.back() = 0;
    kemp_ = 3;
}

auto cg::Chull2::set_initial_values() -> void
{
    mright_ = 2;  // 最初の三角形の右端の凸包頂点番号
    inext_ = 3;   // 凸包に追加される次の頂点のインデックス(mol_の添え字)
}

auto cg::Chull2::add_other_input_points_one_by_one() -> void
{
    while (inext_ < input_point_size_)
    {
        auto newv = mol_[inext_];                                      // 追加する入力点番号
        auto muppv = generate_upper_part_of_convex_hull(newv);         // 上接線の凸包頂点番号
        auto mlowv = generate_lower_part_of_convex_hull(newv, muppv);  // 下接線の凸包頂点番号

        replace_vertices(newv, muppv, mlowv);  // 凸包頂点の置換
        ++inext_;
    }
}

auto cg::Chull2::generate_upper_part_of_convex_hull(int newv) -> int
{
    // newvは入力点番号
    auto mv1 = mright_;      // 右端の凸法頂点番号
    auto jv1 = kvert_[mv1];  // 入力点番号
    return find_upper_tangent_point(newv, mv1, jv1);
}

auto cg::Chull2::find_upper_tangent_point(int newv, int mv1, int jv1) -> int
{
    // newv: 追加する入力点番号
    // mv1: 右端の凸包頂点番号
    // jv1: mv1の入力点番号
    while (true)
    {
        auto mv2 = kccv_[mv1];   // 次の凸包頂点番号
        auto jv2 = kvert_[mv2];  // 入力点番号
        if (utils::turns_left(input_points_[newv], input_points_[jv1], input_points_[jv2]))
        {
            // 上接線が見つかった
            return mv1;  // muppv
        }
        else
        {
            if (mv1 != mright_)
            {
                return_unused_vertex_index(mv1);
            }
            // 次の頂点へ移動（反時計回り）
            mv1 = mv2;
            jv1 = jv2;
        }
    }
}

auto cg::Chull2::return_unused_vertex_index(int mv) -> void
{
    // mv: 返却する凸包頂点番号
    kvert_[mv] = kemp_;
    kemp_ = mv;
}

auto cg::Chull2::generate_lower_part_of_convex_hull(int newv, int muppv) -> int
{
    auto mv1 = mright_;      // 右端の凸法頂点番号
    auto jv1 = kvert_[mv1];  // 入力点番号
    return find_lower_tangent_point(newv, mv1, jv1, muppv);
}

auto cg::Chull2::find_lower_tangent_point(int newv, int mv1, int jv1, int muppv) -> int
{
    // newv: 追加する入力点番号
    // mv1: 右端の凸包頂点番号
    // jv1: mv1の入力点番号
    while (true)
    {
        auto mv2 = kcv_[mv1];    // 次の凸包頂点番号
        auto jv2 = kvert_[mv2];  // 入力点番号
        if (!utils::turns_left(input_points_[newv], input_points_[jv1], input_points_[jv2]))
        {
            // 下接線が見つかった
            return mv1;  // mlowv
        }
        else if (mv1 != muppv)
        {
            return_unused_vertex_index(mv1);
        }
        // 次の頂点へ移動（時計回り）
        mv1 = mv2;
        jv1 = jv2;
    }
}

auto cg::Chull2::replace_vertices(int newv, int muppv, int mlowv) -> void
{
    auto mv = fetch_new_vertex_index();  // 新しい凸包頂点番号を取得
    kvert_[mv] = newv;
    // ここでkccv_とkcv_を更新する
    // 上接線の更新
    kccv_[mv] = muppv;
    kcv_[mv] = mlowv;
    kcv_[muppv] = mv;
    kccv_[mlowv] = mv;
    mright_ = mv;
}

auto cg::Chull2::fetch_new_vertex_index() -> int
{
    if (kemp_ == 0)
    {
        throw std::runtime_error("No more vertex list");
    }
    auto unused_index = kemp_;
    kemp_ = kvert_[kemp_];
    return unused_index;
}
