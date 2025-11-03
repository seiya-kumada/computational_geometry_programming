#include <gtest/gtest.h>

#include "chull2.h"

namespace cg = computational_geometry;
namespace
{

    TEST(Chull2Test, SortIndicesByXThenY)
    {
        const std::vector<Eigen::Vector2d> points{
            {2.0, 1.0}, // index 0
            {1.0, 3.0}, // index 1
            {2.0, 0.0}, // index 2
            {0.0, 5.0}  // index 3
        };

        const cg::Chull2 chull(points);
        const auto &sorted = cg::Chull2TestAccess::mol(chull);

        ASSERT_EQ(sorted.size(), points.size());
        const std::vector<int> expected{3, 1, 2, 0};
        EXPECT_EQ(sorted, expected);
    }

    TEST(Chull2Test, SortIndicesByXThenY_WithDuplicateCoordinates)
    {
        const std::vector<Eigen::Vector2d> points{
            {1.0, 1.0},  // index 0
            {1.0, -1.0}, // index 1
            {0.0, 2.0},  // index 2
            {1.0, 0.5}   // index 3
        };

        const cg::Chull2 chull(points);
        const auto &sorted = cg::Chull2TestAccess::mol(chull);

        const std::vector<int> expected{2, 1, 3, 0};
        EXPECT_EQ(sorted, expected);
    }

    TEST(Chull2Test, GenerateInitialTriangle_WithCounterClockwiseTriangle)
    {
        const std::vector<Eigen::Vector2d> points{
            {0.0, 0.0}, // index 0
            {1.0, 0.0}, // index 1
            {2.0, 1.0}  // index 2
        };

        cg::Chull2 chull(points);
        cg::Chull2TestAccess::GenerateInitialTriangle(chull);

        const auto &kvert = cg::Chull2TestAccess::kvert(chull);
        const auto &kccv = cg::Chull2TestAccess::kccv(chull);
        const auto &kcv = cg::Chull2TestAccess::kcv(chull);
        const auto &mol = cg::Chull2TestAccess::mol(chull);

        EXPECT_EQ(mol, (std::vector<int>{0, 1, 2}));
        EXPECT_EQ(kvert, (std::vector<int>{0, 1, 2}));
        EXPECT_EQ(kccv, (std::vector<int>{1, 2, 0}));
        EXPECT_EQ(kcv, (std::vector<int>{2, 0, 1}));
    }

    TEST(Chull2Test, GenerateInitialTriangle_WithClockwiseTriangle)
    {
        const std::vector<Eigen::Vector2d> points{
            {0.0, 0.0}, // index 0
            {1.0, 1.0}, // index 1
            {2.0, 0.0}  // index 2
        };

        cg::Chull2 chull(points);
        cg::Chull2TestAccess::GenerateInitialTriangle(chull);

        const auto &kvert = cg::Chull2TestAccess::kvert(chull);
        const auto &kccv = cg::Chull2TestAccess::kccv(chull);
        const auto &kcv = cg::Chull2TestAccess::kcv(chull);

        ASSERT_GE(kvert.size(), 3);
        ASSERT_GE(kccv.size(), 3);
        ASSERT_GE(kcv.size(), 3);

        EXPECT_EQ(kvert, (std::vector<int>{0, 1, 2}));
        EXPECT_EQ(kccv, (std::vector<int>{2, 0, 1}));
        EXPECT_EQ(kcv, (std::vector<int>{1, 2, 0}));
    }

    TEST(Chull2Test, SetInitialVertexListStructure)
    {
        const std::vector<Eigen::Vector2d> points{
            {0.0, 0.0}, // index 0
            {1.0, 1.0}, // index 1
            {2.0, 0.0}, // index 2
            {3.0, 4.0}, // index 3
            {4.0, 2.0}  // index 4
        };

        cg::Chull2 chull(points);
        cg::Chull2TestAccess::GenerateInitialTriangle(chull);

        const auto &kvert = cg::Chull2TestAccess::kvert(chull);
        const auto &kccv = cg::Chull2TestAccess::kccv(chull);
        const auto &kcv = cg::Chull2TestAccess::kcv(chull);

        ASSERT_GE(kvert.size(), 5);
        ASSERT_GE(kccv.size(), 5);
        ASSERT_GE(kcv.size(), 5);

        cg::Chull2TestAccess::GenerateInitialTriangle(chull);
        EXPECT_EQ(kvert, (std::vector<int>{0, 1, 2, 0, 0}));
        EXPECT_EQ(kccv, (std::vector<int>{2, 0, 1, 0, 0}));
        EXPECT_EQ(kcv, (std::vector<int>{1, 2, 0, 0, 0}));

        cg::Chull2TestAccess::SetInitialVertexListStructure(chull);
        EXPECT_EQ(kvert, (std::vector<int>{0, 1, 2, 4, 0}));
        EXPECT_EQ(3, cg::Chull2TestAccess::kemp(chull));
    }
} // namespace
