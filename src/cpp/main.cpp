#include "chull2.h"
#include "utils.h"

namespace cg = computational_geometry;

int main(int argc, char* argv[])
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0: p0
        {3.0, 0.0},  // index 1: p1
        {2.0, 3.0},  // index 2: p2
        {2.0, 1.0},  // index 3: p3 - 凸包内部に入り削除される点
        {4.0, 1.0}   // index 4: p4
    };

    cg::Chull2 chull(points);
    auto const& outputs = chull.execute();

    // outputsには凸包を構成する入力点番号が反時計回りで格納されている。
    // これを入力点座標に変換しJSON形式で保存する。
    const std::string json_filename =
        "/home/kumada/data/computational_geometry_programming/convex_hull_output.json";
    utils::save_convex_hull_to_json(points, outputs, json_filename);

    // 凸法を描画する
    const std::string jpg_filename =
        "/home/kumada/data/computational_geometry_programming/convex_hull_output.jpg";
    utils::draw_convex_hull(json_filename, jpg_filename);
    return 0;
}