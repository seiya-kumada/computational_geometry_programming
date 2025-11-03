#ifndef CHULL2_H
#define CHULL2_H

#include <eigen3/Eigen/Dense>
#include <vector>
namespace computational_geometry
{
    class Chull2
    {
    public:
        Chull2(const std::vector<Eigen::Vector2d> &input_points);
        auto sorted_indices() const -> const std::vector<int> &;

    private:
        // 凸包計算対象の点群
        std::vector<Eigen::Vector2d> input_points_;

        // x座標ソート後の点を「元の並び番号」で表すためのリスト
        // 元の並び番号はテキストのNAMEに相当
        std::vector<int> mol_;

        // 凸包の頂点リスト
        std::vector<int> nvlist_;

        // 反時計回り頂点リスト
        std::vector<int> kccv_;

        // 時計回り頂点リスト
        std::vector<int> kcv_;

        auto sort_points_by_x() -> void;

        auto generate_initial_triangle() -> void;
    };
}
#endif // CHULL2_H
