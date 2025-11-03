#include <gtest/gtest.h>

#include "utils.h"

namespace {

TEST(TurnsLeftTest, ReturnsTrueForCounterClockwiseTurn) {
    const Eigen::Vector2d pi(0.0, 0.0);
    const Eigen::Vector2d pj(1.0, 0.0);
    const Eigen::Vector2d pk(1.0, 1.0);

    EXPECT_TRUE(utils::turns_left(pi, pj, pk));
}

TEST(TurnsLeftTest, ReturnsFalseForClockwiseTurn) {
    const Eigen::Vector2d pi(0.0, 0.0);
    const Eigen::Vector2d pj(1.0, 0.0);
    const Eigen::Vector2d pk(1.0, -1.0);

    EXPECT_FALSE(utils::turns_left(pi, pj, pk));
}

TEST(TurnsLeftTest, ReturnsFalseForCollinearPoints) {
    const Eigen::Vector2d pi(0.0, 0.0);
    const Eigen::Vector2d pj(2.0, 2.0);
    const Eigen::Vector2d pk(4.0, 4.0);

    EXPECT_FALSE(utils::turns_left(pi, pj, pk));
}

}  // namespace
