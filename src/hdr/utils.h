#ifndef UTILS_H
#define UTILS_H
#include <eigen3/Eigen/Dense>

namespace utils
{
/**
 * @brief 3点が左折（反時計回り）かどうかを判定する
 *
 * 点pi, pj, pkをこの順にたどったとき、左に曲がるかどうかを判定する。
 * 外積の符号で判定を行う。
 *
 * @param pi 1番目の点
 * @param pj 2番目の点
 * @param pk 3番目の点
 * @return 左折（反時計回り）ならtrue、右折または直進ならfalse
 */
bool turns_left(const Eigen::Vector2d& pi, const Eigen::Vector2d& pj, const Eigen::Vector2d& pk);

/**
 * @brief 凸包の計算結果をJSONファイルに保存する
 *
 * 凸包を構成する頂点の座標をJSON形式で出力する。
 *
 * @param points 入力点群
 * @param convex_hull_indices 凸包を構成する入力点のインデックス
 * @param filename 出力ファイルのパス
 */
void save_convex_hull_to_json(const std::vector<Eigen::Vector2d>& points,
                              const std::vector<int>& convex_hull_indices,
                              const std::string& filename);

/**
 * @brief 凸法を描画する
 *
 * 凸法を2次元平面に描画する。
 *
 * @param input_file_path 入力ファイル（JSON形式）のパス
 * @param output_file_path 出力ファイル（画像）のパス
 */
void draw_convex_hull(const std::string& input_file_path, const std::string& output_file_path);
}  // namespace utils
#endif  // UTILS_H