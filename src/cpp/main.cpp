#include "chull2.h"
#include "utils.h"

#include <boost/program_options.hpp>
#include <expected>
#include <iostream>
#include <optional>

namespace cg = computational_geometry;
namespace po = boost::program_options;

namespace
{
auto extract_args(int argc, char* argv[])
    -> std::expected<std::tuple<std::string, std::string, int, int>, std::string>
{
    po::options_description desc("Options");
    // clang-format off
    desc.add_options()
        ("help,h", "Show help message")
        ("json_path,j", po::value<std::string>()->required(), "json file path")
        ("image_path,i", po::value<std::string>()->required(), "image file path")
        ("n_points,n", po::value<int>()->default_value(100), "number of input points")
        ("seed,s", po::value<int>()->default_value(12345), "random seed");
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    if (vm.count("help"))
    {
        std::ostringstream oss;
        oss << desc;
        return std::unexpected<std::string>(oss.str());
    }
    try
    {
        po::notify(vm);
    }
    catch (const po::error& e)
    {
        return std::unexpected<std::string>(e.what());
    }
    const auto input = vm["json_path"].as<std::string>();
    const auto output = vm["image_path"].as<std::string>();
    const auto n_points = vm["n_points"].as<int>();
    const auto seed = vm["seed"].as<int>();
    return std::make_tuple(input, output, n_points, seed);
}

auto make_input_points() -> std::vector<Eigen::Vector2d>
{
    return std::vector<Eigen::Vector2d>{
        {0.0, 0.0},  // index 0: p0
        {3.0, 0.0},  // index 1: p1
        {2.0, 3.0},  // index 2: p2
        {2.0, 1.0},  // index 3: p3
        {4.0, 1.0}   // index 4: p4
    };
}
}  // namespace

int main(int argc, char* argv[])
{
    // コマンドライン引数を解析
    auto args_opt = extract_args(argc, argv);
    if (!args_opt)
    {
        std::cout << args_opt.error() << std::endl;
        return 1;
    }

    // 引数を取得
    const auto [json_file_path, image_file_path, n_points, seed] = *args_opt;

    // 入力点群を作成
    const auto points = utils::make_input_points(n_points, seed);

    // 凸包を計算
    cg::Chull2 chull(points);
    auto result_0 = chull.execute();
    if (!result_0)
    {
        std::cerr << result_0.error() << std::endl;
        return 1;
    }
    const auto outputs = *result_0;

    // 入力点の座標と凸包頂点の座標をJSONファイルに保存
    auto result_1 = utils::save_convex_hull_to_json(points, outputs, json_file_path);
    if (!result_1)
    {
        std::cerr << result_1.error() << std::endl;
        return 1;
    }

    // 凸法を描画する
    auto result_2 = utils::draw_convex_hull(json_file_path, image_file_path);
    if (!result_2)
    {
        std::cerr << result_2.error() << std::endl;
        return 1;
    }
    return 0;
}