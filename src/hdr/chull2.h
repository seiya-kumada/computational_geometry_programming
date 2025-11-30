#ifndef CHULL2_H
#define CHULL2_H

#include <eigen3/Eigen/Dense>

#include <vector>
namespace computational_geometry
{
class Chull2
{
  public:
    Chull2(const std::vector<Eigen::Vector2d>& input_points);
    auto execute() -> std::vector<Eigen::Vector2d>;

  private:
    // 凸包計算対象の点群
    std::vector<Eigen::Vector2d> input_points_;

    // x座標ソート後の点を「元の並び番号」で表すためのリスト
    // 元の並び番号はテキストのNAMEに相当
    // 整列番号（NAMEを入れる）
    std::vector<int> mol_;

    // 凸包の頂点リスト
    std::vector<int> nvlist_;

    // 反時計回り頂点リスト（NAMEを入れる）
    std::vector<int> kccv_;

    // 時計回り頂点リスト（NAMEを入れる）
    std::vector<int> kcv_;

    // 凸包頂点番号（NAMEを入れる）
    std::vector<int> kvert_;

    auto sort_points_by_x_then_y() -> void;

    auto generate_initial_triangle() -> void;

    auto set_initial_vertex_list_structure() -> void;
    auto set_initial_values() -> void;
    auto add_other_input_points_one_by_one() -> void;
    auto generate_upper_part_of_convex_hull(int newv) -> int;
    auto find_upper_tangent_point(int newv, int mv1, int jv1) -> int;
    auto generate_lower_part_of_convex_hull(int newv) -> int;
    auto find_lower_tangent_point(int newv, int mv1, int jv1) -> int;
    auto replace_vertices(int newv, int muppv, int mlowv) -> void;
    auto generate_output() -> std::vector<Eigen::Vector2d>;
    auto fetch_unused_vertex_index() -> int;

    int kemp_;
    int mright_;
    int inext_;

    friend struct Chull2TestAccess;
};

struct Chull2TestAccess
{
    static void GenerateInitialTriangle(Chull2& instance)
    {
        instance.generate_initial_triangle();
    }

    static void SetInitialVertexListStructure(Chull2& instance)
    {
        instance.set_initial_vertex_list_structure();
    }

    static const std::vector<int>& kvert(const Chull2& instance)
    {
        return instance.kvert_;
    }

    static const std::vector<int>& kccv(const Chull2& instance)
    {
        return instance.kccv_;
    }

    static const std::vector<int>& kcv(const Chull2& instance)
    {
        return instance.kcv_;
    }

    static const std::vector<int>& mol(const Chull2& instance)
    {
        return instance.mol_;
    }

    static int kemp(const Chull2& instance)
    {
        return instance.kemp_;
    }

    static void SetInitialValues(Chull2& instance)
    {
        instance.set_initial_values();
    }

    static int mright(const Chull2& instance)
    {
        return instance.mright_;
    }

    static int inext(const Chull2& instance)
    {
        return instance.inext_;
    }

    static int FetchUnusedVertexIndex(Chull2& instance)
    {
        return instance.fetch_unused_vertex_index();
    }

    static void set_kemp(Chull2& instance, int value)
    {
        instance.kemp_ = value;
    }

    static void set_kvert_at(Chull2& instance, std::size_t index, int value)
    {
        instance.kvert_.at(index) = value;
    }

    static int FindUpperTangentPoint(Chull2& instance, int newv, int mv1, int jv1)
    {
        return instance.find_upper_tangent_point(newv, mv1, jv1);
    }

    static int FindLowerTangentPoint(Chull2& instance, int newv, int mv1, int jv1)
    {
        return instance.find_lower_tangent_point(newv, mv1, jv1);
    }

    static int GenerateUpperPartOfConvexHull(Chull2& instance, int newv)
    {
        return instance.generate_upper_part_of_convex_hull(newv);
    }

    static int GenerateLowerPartOfConvexHull(Chull2& instance, int newv)
    {
        return instance.generate_lower_part_of_convex_hull(newv);
    }

    static void ReplaceVertices(Chull2& instance, int newv, int muppv, int mlowv)
    {
        instance.replace_vertices(newv, muppv, mlowv);
    }

    static void set_mright(Chull2& instance, int value)
    {
        instance.mright_ = value;
    }

    static void set_kccv_at(Chull2& instance, std::size_t index, int value)
    {
        instance.kccv_.at(index) = value;
    }

    static void SortPointsByXThenY(Chull2& instance)
    {
        instance.sort_points_by_x_then_y();
    }
};
}  // namespace computational_geometry
#endif  // CHULL2_H
