#include "utils.h"

#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <random>
#include <set>

bool utils::turns_left(const Eigen::Vector2d& pi, const Eigen::Vector2d& pj,
                       const Eigen::Vector2d& pk)
{
    Eigen::Matrix2d m;
    m << pj.x() - pi.x(), pj.y() - pi.y(), pk.x() - pi.x(), pk.y() - pi.y();
    return m.determinant() > 0;
}

void utils::save_convex_hull_to_json(const std::vector<Eigen::Vector2d>& points,
                                     const std::vector<int>& convex_hull_indices,
                                     const std::string& filename)
{
    nlohmann::json j;

    // 全入力点を保存
    j["input_points"] = nlohmann::json::array();
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        nlohmann::json point_json;
        point_json["index"] = static_cast<int>(i);
        point_json["x"] = points[i].x();
        point_json["y"] = points[i].y();
        j["input_points"].push_back(point_json);
    }

    // 凸包頂点のインデックスを保存
    j["convex_hull_indices"] = convex_hull_indices;

    std::ofstream file(filename);
    if (file.is_open())
    {
        file << j.dump(4);  // インデント幅4で整形して保存
        file.close();
    }
}

auto utils::make_input_points(int n_points, int seed) -> std::vector<Eigen::Vector2d>
{
    std::vector<Eigen::Vector2d> points;
    points.reserve(n_points);

    std::mt19937 gen(seed);
    std::uniform_real_distribution<double> dist(0.0, 100.0);

    for (int i = 0; i < n_points; ++i)
    {
        points.emplace_back(dist(gen), dist(gen));
    }
    return points;
}

namespace
{

// JSONファイルを読み込む
auto read_json(const std::string& input_file_path) -> std::optional<nlohmann::json>
{
    // JSONファイルを読み込む
    std::ifstream file(input_file_path);
    if (!file.is_open())
    {
        return std::nullopt;
    }
    nlohmann::json j;
    file >> j;
    file.close();
    return j;
}

// 全入力点を取得
auto extract_all_points(const nlohmann::json& j) -> std::vector<cv::Point2d>
{
    std::vector<cv::Point2d> all_points;
    for (const auto& point : j["input_points"])
    {
        all_points.emplace_back(point["x"].get<double>(), point["y"].get<double>());
    }
    return all_points;
}

// 凸包頂点のインデックスを取得
auto extract_convex_hull_indices(const nlohmann::json& value) -> std::set<int>
{
    // あとで凸包頂点のインデックスを高速検索するためにsetに格納
    std::set<int> convex_hull_indices;
    for (const auto& idx : value["convex_hull_indices"])
    {
        convex_hull_indices.insert(idx.get<int>());
    }
    return convex_hull_indices;
}

// 凸包頂点の座標を取得
auto make_convex_hull_points(const std::vector<cv::Point2d>& all_points,
                             const nlohmann::json& value) -> std::vector<cv::Point2d>
{
    std::vector<cv::Point2d> convex_hull_points;
    for (const auto& idx : value["convex_hull_indices"])
    {
        int i = idx.get<int>();
        convex_hull_points.push_back(all_points[i]);
    }
    return convex_hull_points;
}

// 座標の範囲を計算
auto decide_point_region(const std::vector<cv::Point2d>& all_points)
    -> std::tuple<double, double, double, double>
{
    double min_x = all_points[0].x;
    double max_x = all_points[0].x;
    double min_y = all_points[0].y;
    double max_y = all_points[0].y;
    for (const auto& p : all_points)
    {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }
    return std::make_tuple(min_x, max_x, min_y, max_y);
}

// 凸包の辺を描画（青）
auto draw_convex_hull_edges(cv::Mat& image, const std::vector<cv::Point2d>& convex_hull_points,
                            const std::function<cv::Point(const cv::Point2d&)>& to_image_coord)
    -> void
{
    for (std::size_t i = 0; i < convex_hull_points.size(); ++i)
    {
        auto p1 = to_image_coord(convex_hull_points[i]);
        auto p2 = to_image_coord(convex_hull_points[(i + 1) % convex_hull_points.size()]);
        cv::line(image, p1, p2, cv::Scalar(255, 0, 0), 2);
    }
}

// 全入力点を描画
auto draw_all_points(cv::Mat& image, const std::vector<cv::Point2d>& all_points,
                     const std::set<int>& convex_hull_indices,
                     const std::function<cv::Point(const cv::Point2d&)>& to_image_coord) -> void
{
    for (std::size_t i = 0; i < all_points.size(); ++i)
    {
        auto img_p = to_image_coord(all_points[i]);
        if (convex_hull_indices.count(static_cast<int>(i)) > 0)
        {
            // 凸包頂点は赤で描画
            cv::circle(image, img_p, 5, cv::Scalar(0, 0, 255), -1);
        }
        else
        {
            // 内部点は緑で描画
            cv::circle(image, img_p, 2, cv::Scalar(0, 200, 0), -1);
        }
    }
}
}  // namespace

void utils::draw_convex_hull(const std::string& input_file_path,
                             const std::string& output_file_path)
{
    // JSONファイルを読み込む
    const auto j = read_json(input_file_path);
    if (!j.has_value())
    {
        return;
    }

    // 全入力点を取得
    const auto& value = j.value();
    const auto all_points = extract_all_points(value);

    if (all_points.empty())
    {
        return;
    }

    // 凸包頂点のインデックスを取得
    const auto convex_hull_indices = extract_convex_hull_indices(value);

    // 凸包頂点の座標を取得
    const auto convex_hull_points = make_convex_hull_points(all_points, value);

    // 座標の範囲を計算
    double min_x, max_x, min_y, max_y;
    std::tie(min_x, max_x, min_y, max_y) = decide_point_region(all_points);

    // 画像サイズとマージンを設定
    const int margin = 50;
    const int image_size = 600;
    double scale = (image_size - 2 * margin) / std::max(max_x - min_x, max_y - min_y);

    // 座標を画像座標に変換するラムダ関数
    auto to_image_coord = [&](const cv::Point2d& p) -> cv::Point
    {
        int x = static_cast<int>((p.x - min_x) * scale) + margin;
        int y = image_size - (static_cast<int>((p.y - min_y) * scale) + margin);  // Y軸を反転
        return cv::Point(x, y);
    };

    // 画像を作成（白背景）
    cv::Mat image(image_size, image_size, CV_8UC3, cv::Scalar(255, 255, 255));

    // 凸包の辺を描画（青）
    draw_convex_hull_edges(image, convex_hull_points, to_image_coord);

    // 全入力点を描画
    draw_all_points(image, all_points, convex_hull_indices, to_image_coord);

    // 画像を保存
    cv::imwrite(output_file_path, image);
}