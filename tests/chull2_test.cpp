#include <gtest/gtest.h>

#include "chull2.h"

namespace {

TEST(Chull2Test, SortsIndicesByXThenY) {
    const std::vector<Eigen::Vector2d> points{
        {2.0, 1.0},  // index 0
        {1.0, 3.0},  // index 1
        {2.0, 0.0},  // index 2
        {0.0, 5.0}   // index 3
    };

    const computational_geometry::Chull2 chull(points);
    const auto& sorted = chull.sorted_indices();

    ASSERT_EQ(sorted.size(), points.size());
    const std::vector<int> expected{3, 1, 2, 0};
    EXPECT_EQ(sorted, expected);
}

TEST(Chull2Test, HandlesDuplicateCoordinates) {
    const std::vector<Eigen::Vector2d> points{
        {1.0, 1.0},  // index 0
        {1.0, -1.0}, // index 1
        {0.0, 2.0},  // index 2
        {1.0, 0.5}   // index 3
    };

    const computational_geometry::Chull2 chull(points);
    const auto& sorted = chull.sorted_indices();

    const std::vector<int> expected{2, 1, 3, 0};
    EXPECT_EQ(sorted, expected);
}

}  // namespace
