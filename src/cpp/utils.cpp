#include "utils.h"

#include <fstream>
#include <nlohmann/json.hpp>

bool utils::turns_left(const Eigen::Vector2d& pi, const Eigen::Vector2d& pj,
                       const Eigen::Vector2d& pk)
{
    Eigen::Matrix2d m;
    m << pj.x() - pi.x(), pj.y() - pi.y(), pk.x() - pi.x(), pk.y() - pi.y();
    return m.determinant() > 0;
}

void utils::save_convex_hull_to_json(const std::string& filename,
                                     const std::vector<Eigen::Vector2d>& points,
                                     const std::vector<int>& convex_hull_indices)
{
    nlohmann::json j;
    j["convex_hull"] = nlohmann::json::array();

    for (const auto index : convex_hull_indices)
    {
        nlohmann::json point_json;
        point_json["index"] = index;
        point_json["x"] = points[index].x();
        point_json["y"] = points[index].y();
        j["convex_hull"].push_back(point_json);
    }

    std::ofstream file(filename);
    if (file.is_open())
    {
        file << j.dump(4);  // インデント幅4で整形して保存
        file.close();
    }
}