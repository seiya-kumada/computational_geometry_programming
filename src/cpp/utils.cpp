#include "utils.h"
bool utils::turns_left(const Eigen::Vector2d &pi, const Eigen::Vector2d &pj, const Eigen::Vector2d &pk)
{
    Eigen::Matrix2d m;
    m << pj.x() - pi.x(), pj.y() - pi.y(),
        pk.x() - pi.x(), pk.y() - pi.y();
    return m.determinant() > 0;
}