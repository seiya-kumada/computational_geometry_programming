#include "chull2.h"

#include <gtest/gtest.h>

namespace cg = computational_geometry;
namespace
{

TEST(Chull2Test, SortIndicesByXThenY)
{
    const std::vector<Eigen::Vector2d> points{
        {2.0, 1.0},  // index 0
        {1.0, 3.0},  // index 1
        {2.0, 0.0},  // index 2
        {0.0, 5.0}   // index 3
    };

    const cg::Chull2 chull(points);
    const auto& sorted = cg::Chull2TestAccess::mol(chull);

    ASSERT_EQ(sorted.size(), points.size());
    const std::vector<int> expected{3, 1, 2, 0};
    EXPECT_EQ(sorted, expected);
}

TEST(Chull2Test, SortIndicesByXThenY_WithDuplicateCoordinates)
{
    const std::vector<Eigen::Vector2d> points{
        {1.0, 1.0},   // index 0
        {1.0, -1.0},  // index 1
        {0.0, 2.0},   // index 2
        {1.0, 0.5}    // index 3
    };

    const cg::Chull2 chull(points);
    const auto& sorted = cg::Chull2TestAccess::mol(chull);

    const std::vector<int> expected{2, 1, 3, 0};
    EXPECT_EQ(sorted, expected);
}

TEST(Chull2Test, GenerateInitialTriangle_WithCounterClockwiseTriangle)
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {1.0, 0.0},  // index 1
        {2.0, 1.0}   // index 2
    };

    cg::Chull2 chull(points);
    cg::Chull2TestAccess::GenerateInitialTriangle(chull);

    const auto& kvert = cg::Chull2TestAccess::kvert(chull);
    const auto& kccv = cg::Chull2TestAccess::kccv(chull);
    const auto& kcv = cg::Chull2TestAccess::kcv(chull);
    const auto& mol = cg::Chull2TestAccess::mol(chull);

    EXPECT_EQ(mol, (std::vector<int>{0, 1, 2}));
    EXPECT_EQ(kvert, (std::vector<int>{0, 1, 2}));
    EXPECT_EQ(kccv, (std::vector<int>{1, 2, 0}));
    EXPECT_EQ(kcv, (std::vector<int>{2, 0, 1}));
}

TEST(Chull2Test, GenerateInitialTriangle_WithClockwiseTriangle)
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {1.0, 1.0},  // index 1
        {2.0, 0.0}   // index 2
    };

    cg::Chull2 chull(points);
    cg::Chull2TestAccess::GenerateInitialTriangle(chull);

    const auto& kvert = cg::Chull2TestAccess::kvert(chull);
    const auto& kccv = cg::Chull2TestAccess::kccv(chull);
    const auto& kcv = cg::Chull2TestAccess::kcv(chull);

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
        {0.0, 0.0},  // index 0
        {1.0, 1.0},  // index 1
        {2.0, 0.0},  // index 2
        {3.0, 4.0},  // index 3
        {4.0, 2.0}   // index 4
    };

    cg::Chull2 chull(points);
    cg::Chull2TestAccess::GenerateInitialTriangle(chull);

    const auto& kvert = cg::Chull2TestAccess::kvert(chull);
    const auto& kccv = cg::Chull2TestAccess::kccv(chull);
    const auto& kcv = cg::Chull2TestAccess::kcv(chull);

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

TEST(Chull2Test, SetInitialValues)
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {1.0, 1.0},  // index 1
        {2.0, 0.0},  // index 2
        {3.0, 4.0},  // index 3
        {4.0, 2.0}   // index 4
    };

    cg::Chull2 chull(points);
    cg::Chull2TestAccess::SetInitialValues(chull);
    EXPECT_EQ(2, cg::Chull2TestAccess::mright(chull));
    EXPECT_EQ(3, cg::Chull2TestAccess::inext(chull));
}

TEST(Chull2Test, FetchUnusedVertexIndex_ReturnsCurrentKempAndUpdates)
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {1.0, 0.0},  // index 1
        {2.0, 0.0},  // index 2
        {3.0, 0.0},  // index 3
        {4.0, 0.0}   // index 4
    };

    cg::Chull2 chull(points);
    cg::Chull2TestAccess::SetInitialVertexListStructure(chull);

    // fetch_unused_vertex_index を呼び出し
    const auto result = cg::Chull2TestAccess::FetchUnusedVertexIndex(chull);

    // 戻り値は元の kemp_ の値（3）であるべき
    EXPECT_EQ(3, result);

    // kemp_ は kvert_[3] の値（4）に更新されているべき
    EXPECT_EQ(4, cg::Chull2TestAccess::kemp(chull));
}

TEST(Chull2Test, FetchUnusedVertexIndex_FollowsLinkedList)
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {1.0, 0.0},  // index 1
        {2.0, 0.0},  // index 2
        {3.0, 0.0},  // index 3
        {4.0, 0.0},  // index 4
        {5.0, 0.0}   // index 5
    };

    cg::Chull2 chull(points);

    // リンクリスト構造を設定: 3 -> 4 -> 5 -> 0
    cg::Chull2TestAccess::SetInitialVertexListStructure(chull);

    // 1回目の呼び出し
    auto result1 = cg::Chull2TestAccess::FetchUnusedVertexIndex(chull);
    EXPECT_EQ(3, result1);
    EXPECT_EQ(4, cg::Chull2TestAccess::kemp(chull));

    // 2回目の呼び出し
    auto result2 = cg::Chull2TestAccess::FetchUnusedVertexIndex(chull);
    EXPECT_EQ(4, result2);
    EXPECT_EQ(5, cg::Chull2TestAccess::kemp(chull));

    // 3回目の呼び出し
    auto result3 = cg::Chull2TestAccess::FetchUnusedVertexIndex(chull);
    EXPECT_EQ(5, result3);
    EXPECT_EQ(0, cg::Chull2TestAccess::kemp(chull));
}

TEST(Chull2Test, FindUpperTangentPoint_ImmediatelyFindsFirstPoint)
{
    // 凸包: (0,0) -> (2,0) -> (1,2) (反時計回り)
    // 新しい点: (3,1) - 右側に追加
    // (3,1)から(2,0)への接線が既に上接線
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {2.0, 0.0},  // index 1
        {1.0, 2.0},  // index 2
        {3.0, 1.0}   // index 3 - 追加する点
    };

    cg::Chull2 chull(points);

    // 凸包の構造を設定（三角形: 0 -> 1 -> 2 -> 0）
    // kvert_: 凸包頂点の実際の点のインデックス
    cg::Chull2TestAccess::SortPointsByXThenY(chull);
    const auto& mol = cg::Chull2TestAccess::mol(chull);
    EXPECT_EQ(mol, (std::vector<int>{0, 2, 1, 3}));
    cg::Chull2TestAccess::GenerateInitialTriangle(chull);
    cg::Chull2TestAccess::SetInitialVertexListStructure(chull);
    cg::Chull2TestAccess::SetInitialValues(chull);
    const auto& kvert = cg::Chull2TestAccess::kvert(chull);
    const auto& kccv = cg::Chull2TestAccess::kccv(chull);
    const auto& kcv = cg::Chull2TestAccess::kcv(chull);

    EXPECT_EQ(kvert, (std::vector<int>{0, 2, 1, 0}));
    EXPECT_EQ(kccv, (std::vector<int>{2, 0, 1, 0}));
    EXPECT_EQ(kcv, (std::vector<int>{1, 2, 0, 0}));
    EXPECT_EQ(2, cg::Chull2TestAccess::mright(chull));
    EXPECT_EQ(3, cg::Chull2TestAccess::inext(chull));

    const auto& result = cg::Chull2TestAccess::FindUpperTangentPoint(chull, 3, 2, 1);

    // (3,1), (2,0), (1,2) で左折するので、凸包頂点1が上接線点
    EXPECT_EQ(1, result);
}
/*
TEST(Chull2Test, FindUpperTangentPoint_SearchesMultipleVertices)
{
    // 凸包: (0,0) -> (2,0) -> (3,2) -> (1,3) (反時計回り)
    // 新しい点: (4,1.5)
    // (4,1.5)から開始して上接線を探す
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {2.0, 0.0},  // index 1
        {3.0, 2.0},  // index 2
        {1.0, 3.0},  // index 3
        {4.0, 1.5}   // index 4 - 追加する点
    };

    cg::Chull2 chull(points);

    // 凸包の構造を設定（四角形: 0 -> 1 -> 2 -> 3 -> 0）
    cg::Chull2TestAccess::set_kvert_at(chull, 0, 0);
    cg::Chull2TestAccess::set_kvert_at(chull, 1, 1);
    cg::Chull2TestAccess::set_kvert_at(chull, 2, 2);
    cg::Chull2TestAccess::set_kvert_at(chull, 3, 3);

    // kccv_: 反時計回り
    cg::Chull2TestAccess::set_kccv_at(chull, 0, 1);
    cg::Chull2TestAccess::set_kccv_at(chull, 1, 2);
    cg::Chull2TestAccess::set_kccv_at(chull, 2, 3);
    cg::Chull2TestAccess::set_kccv_at(chull, 3, 0);

    // 点1(2,0)が右端
    cg::Chull2TestAccess::set_mright(chull, 1);

    // find_upper_tangent_point(newv=4, mv1=1, jv1=1)
    auto result = cg::Chull2TestAccess::FindUpperTangentPoint(chull, 4, 1, 1);

    // (4,1.5), (2,0), (3,2): 右折 -> 続行
    // (4,1.5), (3,2), (1,3): 左折 -> 凸包頂点2が上接線点
    EXPECT_EQ(2, result);
}

TEST(Chull2Test, FindUpperTangentPoint_WithSquareConvexHull)
{
    // 凸包: (0,0) -> (2,0) -> (2,2) -> (0,2) (反時計回り)
    // 新しい点: (3,1) - 右側に追加
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {2.0, 0.0},  // index 1
        {2.0, 2.0},  // index 2
        {0.0, 2.0},  // index 3
        {3.0, 1.0}   // index 4 - 追加する点
    };

    cg::Chull2 chull(points);

    // 凸包の構造を設定
    cg::Chull2TestAccess::set_kvert_at(chull, 0, 0);
    cg::Chull2TestAccess::set_kvert_at(chull, 1, 1);
    cg::Chull2TestAccess::set_kvert_at(chull, 2, 2);
    cg::Chull2TestAccess::set_kvert_at(chull, 3, 3);

    cg::Chull2TestAccess::set_kccv_at(chull, 0, 1);
    cg::Chull2TestAccess::set_kccv_at(chull, 1, 2);
    cg::Chull2TestAccess::set_kccv_at(chull, 2, 3);
    cg::Chull2TestAccess::set_kccv_at(chull, 3, 0);

    // 点1(2,0)が右端
    cg::Chull2TestAccess::set_mright(chull, 1);

    auto result = cg::Chull2TestAccess::FindUpperTangentPoint(chull, 4, 1, 1);

    // (3,1), (2,0), (2,2): 左折 -> 凸包頂点1が上接線点
    EXPECT_EQ(1, result);
}
*/
}  // namespace
