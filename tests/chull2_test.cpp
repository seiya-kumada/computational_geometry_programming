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
    const auto result = cg::Chull2TestAccess::FetchNewVertexIndex(chull);

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
    auto result1 = cg::Chull2TestAccess::FetchNewVertexIndex(chull);
    EXPECT_EQ(3, result1);
    EXPECT_EQ(4, cg::Chull2TestAccess::kemp(chull));

    // 2回目の呼び出し
    auto result2 = cg::Chull2TestAccess::FetchNewVertexIndex(chull);
    EXPECT_EQ(4, result2);
    EXPECT_EQ(5, cg::Chull2TestAccess::kemp(chull));

    // 3回目の呼び出し
    auto result3 = cg::Chull2TestAccess::FetchNewVertexIndex(chull);
    EXPECT_EQ(5, result3);
    EXPECT_EQ(0, cg::Chull2TestAccess::kemp(chull));
}

TEST(Chull2Test, FindUpperTangentPoint_1)
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {2.0, 0.0},  // index 1
        {1.0, 2.0},  // index 2
        {3.0, 1.0}   // index 3 - 追加する点
    };

    cg::Chull2 chull(points);

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

TEST(Chull2Test, GenerateUpperPartOfConvexHull_1)
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {2.0, 0.0},  // index 1
        {1.0, 2.0},  // index 2
        {3.0, 1.0}   // index 3 - 追加する点
    };

    cg::Chull2 chull(points);

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

    const auto& result = cg::Chull2TestAccess::GenerateUpperPartOfConvexHull(chull, 3);

    // (3,1), (2,0), (1,2) で左折するので、凸包頂点1が上接線点
    EXPECT_EQ(1, result);
}

TEST(Chull2Test, FindUpperTangentPoint_2)
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {2.0, 0.0},  // index 1
        {3.0, 2.0},  // index 2
        {4.0, 1.5}   // index 3 - 追加する点
    };

    cg::Chull2 chull(points);

    // 凸包の構造を設定（四角形: 0 -> 1 -> 2 -> 3 -> 0）
    cg::Chull2TestAccess::SortPointsByXThenY(chull);
    const auto& mol = cg::Chull2TestAccess::mol(chull);
    // mol_: ソート後の点の元のインデックス（入力点番号）
    EXPECT_EQ(mol, (std::vector<int>{0, 1, 2, 3}));

    cg::Chull2TestAccess::GenerateInitialTriangle(chull);
    cg::Chull2TestAccess::SetInitialVertexListStructure(chull);
    cg::Chull2TestAccess::SetInitialValues(chull);
    const auto& kvert = cg::Chull2TestAccess::kvert(chull);
    const auto& kccv = cg::Chull2TestAccess::kccv(chull);
    const auto& kcv = cg::Chull2TestAccess::kcv(chull);

    EXPECT_EQ(kvert, (std::vector<int>{0, 1, 2, 0}));
    EXPECT_EQ(kccv, (std::vector<int>{1, 2, 0, 0}));
    EXPECT_EQ(kcv, (std::vector<int>{2, 0, 1, 0}));
    EXPECT_EQ(2, cg::Chull2TestAccess::mright(chull));
    EXPECT_EQ(3, cg::Chull2TestAccess::inext(chull));

    const auto& result = cg::Chull2TestAccess::FindUpperTangentPoint(chull, 3, 2, 2);
    EXPECT_EQ(2, result);
}

TEST(Chull2Test, GenerateUpperPartOfConvexHull_2)
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {2.0, 0.0},  // index 1
        {3.0, 2.0},  // index 2
        {4.0, 1.5}   // index 3 - 追加する点
    };

    cg::Chull2 chull(points);

    // 凸包の構造を設定（四角形: 0 -> 1 -> 2 -> 3 -> 0）
    cg::Chull2TestAccess::SortPointsByXThenY(chull);
    const auto& mol = cg::Chull2TestAccess::mol(chull);
    // mol_: ソート後の点の元のインデックス（入力点番号）
    EXPECT_EQ(mol, (std::vector<int>{0, 1, 2, 3}));

    cg::Chull2TestAccess::GenerateInitialTriangle(chull);
    cg::Chull2TestAccess::SetInitialVertexListStructure(chull);
    cg::Chull2TestAccess::SetInitialValues(chull);
    const auto& kvert = cg::Chull2TestAccess::kvert(chull);
    const auto& kccv = cg::Chull2TestAccess::kccv(chull);
    const auto& kcv = cg::Chull2TestAccess::kcv(chull);

    EXPECT_EQ(kvert, (std::vector<int>{0, 1, 2, 0}));
    EXPECT_EQ(kccv, (std::vector<int>{1, 2, 0, 0}));
    EXPECT_EQ(kcv, (std::vector<int>{2, 0, 1, 0}));
    EXPECT_EQ(2, cg::Chull2TestAccess::mright(chull));
    EXPECT_EQ(3, cg::Chull2TestAccess::inext(chull));

    const auto& result = cg::Chull2TestAccess::GenerateUpperPartOfConvexHull(chull, 3);
    EXPECT_EQ(2, result);
}

TEST(Chull2Test, FindUpperTangentPoint_3)
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {2.0, 0.0},  // index 1
        {2.0, 2.0},  // index 2
        {3.0, 1.0}   // index 3 - 追加する点
    };

    cg::Chull2 chull(points);
    cg::Chull2TestAccess::SortPointsByXThenY(chull);
    const auto& mol = cg::Chull2TestAccess::mol(chull);
    EXPECT_EQ(mol, (std::vector<int>{0, 1, 2, 3}));
    cg::Chull2TestAccess::GenerateInitialTriangle(chull);
    cg::Chull2TestAccess::SetInitialVertexListStructure(chull);
    cg::Chull2TestAccess::SetInitialValues(chull);
    const auto& kvert = cg::Chull2TestAccess::kvert(chull);
    const auto& kccv = cg::Chull2TestAccess::kccv(chull);
    const auto& kcv = cg::Chull2TestAccess::kcv(chull);
    EXPECT_EQ(kvert, (std::vector<int>{0, 1, 2, 0}));
    EXPECT_EQ(kccv, (std::vector<int>{1, 2, 0, 0}));
    EXPECT_EQ(kcv, (std::vector<int>{2, 0, 1, 0}));
    EXPECT_EQ(2, cg::Chull2TestAccess::mright(chull));
    EXPECT_EQ(3, cg::Chull2TestAccess::inext(chull));

    auto result = cg::Chull2TestAccess::FindUpperTangentPoint(chull, 3, 1, 1);

    EXPECT_EQ(2, result);
}

TEST(Chull2Test, GenerateUpperPartOfConvexHull_3)
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {2.0, 0.0},  // index 1
        {2.0, 2.0},  // index 2
        {3.0, 1.0}   // index 3 - 追加する点
    };

    cg::Chull2 chull(points);
    cg::Chull2TestAccess::SortPointsByXThenY(chull);
    const auto& mol = cg::Chull2TestAccess::mol(chull);
    EXPECT_EQ(mol, (std::vector<int>{0, 1, 2, 3}));
    cg::Chull2TestAccess::GenerateInitialTriangle(chull);
    cg::Chull2TestAccess::SetInitialVertexListStructure(chull);
    cg::Chull2TestAccess::SetInitialValues(chull);
    const auto& kvert = cg::Chull2TestAccess::kvert(chull);
    const auto& kccv = cg::Chull2TestAccess::kccv(chull);
    const auto& kcv = cg::Chull2TestAccess::kcv(chull);
    EXPECT_EQ(kvert, (std::vector<int>{0, 1, 2, 0}));
    EXPECT_EQ(kccv, (std::vector<int>{1, 2, 0, 0}));
    EXPECT_EQ(kcv, (std::vector<int>{2, 0, 1, 0}));
    EXPECT_EQ(2, cg::Chull2TestAccess::mright(chull));
    EXPECT_EQ(3, cg::Chull2TestAccess::inext(chull));

    auto result = cg::Chull2TestAccess::GenerateUpperPartOfConvexHull(chull, 3);

    EXPECT_EQ(2, result);
}

TEST(Chull2Test, FindUpperTangentPoint_4)
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 1.0},  // index 0
        {1.0, 0.0},  // index 1
        {1.0, 2.0},  // index 2
        {2.0, 1.0},  // index 3
    };

    cg::Chull2 chull(points);
    cg::Chull2TestAccess::SortPointsByXThenY(chull);
    const auto& mol = cg::Chull2TestAccess::mol(chull);
    EXPECT_EQ(mol, (std::vector<int>{0, 1, 2, 3}));
    cg::Chull2TestAccess::GenerateInitialTriangle(chull);
    cg::Chull2TestAccess::SetInitialVertexListStructure(chull);
    cg::Chull2TestAccess::SetInitialValues(chull);
    const auto& kvert = cg::Chull2TestAccess::kvert(chull);
    const auto& kccv = cg::Chull2TestAccess::kccv(chull);
    const auto& kcv = cg::Chull2TestAccess::kcv(chull);
    EXPECT_EQ(kvert, (std::vector<int>{0, 1, 2, 0}));
    EXPECT_EQ(kccv, (std::vector<int>{1, 2, 0, 0}));
    EXPECT_EQ(kcv, (std::vector<int>{2, 0, 1, 0}));
    EXPECT_EQ(2, cg::Chull2TestAccess::mright(chull));
    EXPECT_EQ(3, cg::Chull2TestAccess::inext(chull));

    auto result = cg::Chull2TestAccess::FindUpperTangentPoint(chull, 3, 2, 2);
    EXPECT_EQ(kvert, (std::vector<int>{0, 1, 2, 0}));

    EXPECT_EQ(2, result);
}

TEST(Chull2Test, GenerateUpperPartOfConvexHull_4)
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 1.0},  // index 0
        {1.0, 0.0},  // index 1
        {1.0, 2.0},  // index 2
        {2.0, 1.0},  // index 3
    };

    cg::Chull2 chull(points);

    cg::Chull2TestAccess::SortPointsByXThenY(chull);
    const auto& mol = cg::Chull2TestAccess::mol(chull);
    EXPECT_EQ(mol, (std::vector<int>{0, 1, 2, 3}));
    cg::Chull2TestAccess::GenerateInitialTriangle(chull);
    cg::Chull2TestAccess::SetInitialVertexListStructure(chull);
    cg::Chull2TestAccess::SetInitialValues(chull);
    const auto& kvert = cg::Chull2TestAccess::kvert(chull);
    const auto& kccv = cg::Chull2TestAccess::kccv(chull);
    const auto& kcv = cg::Chull2TestAccess::kcv(chull);

    EXPECT_EQ(kvert, (std::vector<int>{0, 1, 2, 0}));
    EXPECT_EQ(kccv, (std::vector<int>{1, 2, 0, 0}));
    EXPECT_EQ(kcv, (std::vector<int>{2, 0, 1, 0}));
    EXPECT_EQ(2, cg::Chull2TestAccess::mright(chull));
    EXPECT_EQ(3, cg::Chull2TestAccess::inext(chull));

    const auto& result = cg::Chull2TestAccess::GenerateUpperPartOfConvexHull(chull, 3);

    EXPECT_EQ(2, result);
}

TEST(Chull2Test, FindLowerTangentPoint_1)
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {2.0, 0.0},  // index 1
        {1.0, 2.0},  // index 2
        {3.0, 1.0}   // index 3 - 追加する点
    };

    cg::Chull2 chull(points);

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

    auto dummy = 1;  // 凸法頂点番号
    const auto& result = cg::Chull2TestAccess::FindLowerTangentPoint(chull, 3, 2, 1, dummy);

    EXPECT_EQ(2, result);
}

TEST(Chull2Test, GenerateLowerPartOfConvexHull_1)
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {2.0, 0.0},  // index 1
        {1.0, 2.0},  // index 2
        {3.0, 1.0}   // index 3 - 追加する点
    };

    cg::Chull2 chull(points);

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

    auto dummy = 1;  // 凸法頂点番号
    const auto& result = cg::Chull2TestAccess::GenerateLowerPartOfConvexHull(chull, 3, dummy);

    EXPECT_EQ(2, result);
}

TEST(Chull2Test, FindLowerTangentPoint_2)
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {2.0, 0.0},  // index 1
        {3.0, 2.0},  // index 2
        {4.0, 1.5}   // index 3 - 追加する点
    };

    cg::Chull2 chull(points);

    cg::Chull2TestAccess::SortPointsByXThenY(chull);
    const auto& mol = cg::Chull2TestAccess::mol(chull);
    EXPECT_EQ(mol, (std::vector<int>{0, 1, 2, 3}));

    cg::Chull2TestAccess::GenerateInitialTriangle(chull);
    cg::Chull2TestAccess::SetInitialVertexListStructure(chull);
    cg::Chull2TestAccess::SetInitialValues(chull);
    const auto& kvert = cg::Chull2TestAccess::kvert(chull);
    const auto& kccv = cg::Chull2TestAccess::kccv(chull);
    const auto& kcv = cg::Chull2TestAccess::kcv(chull);

    EXPECT_EQ(kvert, (std::vector<int>{0, 1, 2, 0}));
    EXPECT_EQ(kccv, (std::vector<int>{1, 2, 0, 0}));
    EXPECT_EQ(kcv, (std::vector<int>{2, 0, 1, 0}));
    EXPECT_EQ(2, cg::Chull2TestAccess::mright(chull));
    EXPECT_EQ(3, cg::Chull2TestAccess::inext(chull));
    auto dummy = 2;  // 凸法頂点番号
    const auto& result = cg::Chull2TestAccess::FindLowerTangentPoint(chull, 3, 2, 2, dummy);
    EXPECT_EQ(1, result);
}

TEST(Chull2Test, GenerateLowerPartOfConvecHull_2)
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {2.0, 0.0},  // index 1
        {3.0, 2.0},  // index 2
        {4.0, 1.5}   // index 3 - 追加する点
    };

    cg::Chull2 chull(points);

    cg::Chull2TestAccess::SortPointsByXThenY(chull);
    const auto& mol = cg::Chull2TestAccess::mol(chull);
    EXPECT_EQ(mol, (std::vector<int>{0, 1, 2, 3}));

    cg::Chull2TestAccess::GenerateInitialTriangle(chull);
    cg::Chull2TestAccess::SetInitialVertexListStructure(chull);
    cg::Chull2TestAccess::SetInitialValues(chull);
    const auto& kvert = cg::Chull2TestAccess::kvert(chull);
    const auto& kccv = cg::Chull2TestAccess::kccv(chull);
    const auto& kcv = cg::Chull2TestAccess::kcv(chull);

    EXPECT_EQ(kvert, (std::vector<int>{0, 1, 2, 0}));
    EXPECT_EQ(kccv, (std::vector<int>{1, 2, 0, 0}));
    EXPECT_EQ(kcv, (std::vector<int>{2, 0, 1, 0}));
    EXPECT_EQ(2, cg::Chull2TestAccess::mright(chull));
    EXPECT_EQ(3, cg::Chull2TestAccess::inext(chull));
    auto dummy = 2;  // 凸法頂点番号
    const auto& result = cg::Chull2TestAccess::GenerateLowerPartOfConvexHull(chull, 3, dummy);
    EXPECT_EQ(1, result);
}

TEST(Chull2Test, FindLowerTangentPoint_3)
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {2.0, 0.0},  // index 1
        {2.0, 2.0},  // index 2
        {3.0, 1.0}   // index 3 - 追加する点
    };

    cg::Chull2 chull(points);
    cg::Chull2TestAccess::SortPointsByXThenY(chull);
    const auto& mol = cg::Chull2TestAccess::mol(chull);
    EXPECT_EQ(mol, (std::vector<int>{0, 1, 2, 3}));
    cg::Chull2TestAccess::GenerateInitialTriangle(chull);
    cg::Chull2TestAccess::SetInitialVertexListStructure(chull);
    cg::Chull2TestAccess::SetInitialValues(chull);
    const auto& kvert = cg::Chull2TestAccess::kvert(chull);
    const auto& kccv = cg::Chull2TestAccess::kccv(chull);
    const auto& kcv = cg::Chull2TestAccess::kcv(chull);
    EXPECT_EQ(kvert, (std::vector<int>{0, 1, 2, 0}));
    EXPECT_EQ(kccv, (std::vector<int>{1, 2, 0, 0}));
    EXPECT_EQ(kcv, (std::vector<int>{2, 0, 1, 0}));
    EXPECT_EQ(2, cg::Chull2TestAccess::mright(chull));
    EXPECT_EQ(3, cg::Chull2TestAccess::inext(chull));

    auto dummy = 2;  // 凸法頂点番号
    auto result = cg::Chull2TestAccess::FindLowerTangentPoint(chull, 3, 1, 1, dummy);

    EXPECT_EQ(1, result);
}

TEST(Chull2Test, GenerateLowerPartOfConvexHull_3)
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {2.0, 0.0},  // index 1
        {2.0, 2.0},  // index 2
        {3.0, 1.0}   // index 3 - 追加する点
    };

    cg::Chull2 chull(points);
    cg::Chull2TestAccess::SortPointsByXThenY(chull);
    const auto& mol = cg::Chull2TestAccess::mol(chull);
    EXPECT_EQ(mol, (std::vector<int>{0, 1, 2, 3}));
    cg::Chull2TestAccess::GenerateInitialTriangle(chull);
    cg::Chull2TestAccess::SetInitialVertexListStructure(chull);
    cg::Chull2TestAccess::SetInitialValues(chull);
    const auto& kvert = cg::Chull2TestAccess::kvert(chull);
    const auto& kccv = cg::Chull2TestAccess::kccv(chull);
    const auto& kcv = cg::Chull2TestAccess::kcv(chull);
    EXPECT_EQ(kvert, (std::vector<int>{0, 1, 2, 0}));
    EXPECT_EQ(kccv, (std::vector<int>{1, 2, 0, 0}));
    EXPECT_EQ(kcv, (std::vector<int>{2, 0, 1, 0}));
    EXPECT_EQ(2, cg::Chull2TestAccess::mright(chull));
    EXPECT_EQ(3, cg::Chull2TestAccess::inext(chull));

    auto dummy = 2;  // 凸法頂点番号
    auto result = cg::Chull2TestAccess::GenerateLowerPartOfConvexHull(chull, 3, dummy);

    EXPECT_EQ(1, result);
}

TEST(Chull2Test, ReplaceVertices_1)
{
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0
        {2.0, 0.0},  // index 1
        {1.0, 2.0},  // index 2
        {3.0, 1.0}   // index 3 - 追加する点
    };

    cg::Chull2 chull(points);

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

    // upper, lowerは凸法頂点番号
    const auto& upper = cg::Chull2TestAccess::GenerateUpperPartOfConvexHull(chull, 3);
    const auto& lower = cg::Chull2TestAccess::GenerateLowerPartOfConvexHull(chull, 3, upper);
    EXPECT_EQ(1, upper);
    EXPECT_EQ(2, lower);
    const auto& kemp = cg::Chull2TestAccess::kemp(chull);
    EXPECT_EQ(3, kemp);
    // const auto& unsed_index = cg::Chull2TestAccess::FetchNewVertexIndex(chull);
    // EXPECT_EQ(3, unsed_index);
    EXPECT_EQ(kvert, (std::vector<int>{0, 2, 1, 0}));
    cg::Chull2TestAccess::ReplaceVertices(chull, 3, upper, lower);
    EXPECT_EQ(kvert, (std::vector<int>{0, 2, 1, 3}));
    EXPECT_EQ(kccv, (std::vector<int>{2, 0, 3, 1}));
    EXPECT_EQ(kcv, (std::vector<int>{1, 3, 0, 2}));
}

TEST(Chull2Test, ReturnUnusedVertexIndex_1)
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

    // 初期状態: kemp_ = 3, kvert_ = {0, 1, 2, 4, 0}
    EXPECT_EQ(3, cg::Chull2TestAccess::kemp(chull));

    // 凸包頂点番号1をフリーリストに返却
    cg::Chull2TestAccess::ReturnUnusedVertexIndex(chull, 1);

    // kemp_ は 1 に更新される（返却した頂点番号がフリーリストの先頭になる）
    EXPECT_EQ(1, cg::Chull2TestAccess::kemp(chull));

    // kvert_[1] は元の kemp_ の値（3）に設定される
    const auto& kvert = cg::Chull2TestAccess::kvert(chull);
    EXPECT_EQ(3, kvert[1]);

    // フリーリストの構造: 1 -> 3 -> 4 -> 0
    // 次に fetch すると 1 が返り、kemp_ は 3 になる
    auto fetched = cg::Chull2TestAccess::FetchNewVertexIndex(chull);
    EXPECT_EQ(1, fetched);
    EXPECT_EQ(3, cg::Chull2TestAccess::kemp(chull));
}

TEST(Chull2Test, AddOtherInputPointsOneByOne_1)
{
    // 5点の凸包を構築するテスト（点の削除を伴う）
    // 初期三角形の右端頂点が、新しい点の追加により削除されるケース
    //
    //        (2,3) p2
    //         /\
    //        /  \
    //       /    \
    //      /  p3  \      p4
    //     / (2,1)  \   (4,1)
    //    /          \
    //   p0----------p1
    // (0,0)       (3,0)
    //
    // ソート後: p0(0,0) -> p3(2,1) -> p2(2,3) -> p1(3,0) -> p4(4,1)
    // 初期三角形: p0, p3, p2
    // p1追加時: p3が凸包から削除される
    // p4追加時: 削除なし
    const std::vector<Eigen::Vector2d> points{
        {0.0, 0.0},  // index 0: p0
        {3.0, 0.0},  // index 1: p1
        {2.0, 3.0},  // index 2: p2
        {2.0, 1.0},  // index 3: p3 - 凸包内部に入り削除される点
        {4.0, 1.0}   // index 4: p4
    };

    cg::Chull2 chull(points);

    // 初期化処理
    cg::Chull2TestAccess::SortPointsByXThenY(chull);
    const auto& mol = cg::Chull2TestAccess::mol(chull);
    // x座標でソート: p0(0,0), p3(2,1), p2(2,3), p1(3,0), p4(4,1)
    EXPECT_EQ(mol, (std::vector<int>{0, 3, 2, 1, 4}));

    cg::Chull2TestAccess::GenerateInitialTriangle(chull);
    cg::Chull2TestAccess::SetInitialVertexListStructure(chull);
    cg::Chull2TestAccess::SetInitialValues(chull);

    // 初期状態の確認
    // 初期三角形: 凸包頂点0=p0, 凸包頂点1=p3, 凸包頂点2=p2
    const auto& kvert = cg::Chull2TestAccess::kvert(chull);
    const auto& kccv = cg::Chull2TestAccess::kccv(chull);
    const auto& kcv = cg::Chull2TestAccess::kcv(chull);
    EXPECT_EQ(kvert, (std::vector<int>{0, 3, 2, 4, 0}));
    EXPECT_EQ(2, cg::Chull2TestAccess::mright(chull));
    EXPECT_EQ(3, cg::Chull2TestAccess::inext(chull));

    // 残りの点を追加
    cg::Chull2TestAccess::AddOtherInputPointsOneByOne(chull);

    // 最終的な凸包: p0, p1, p4, p2 が反時計回りに並ぶ
    // p3は凸包から削除され、フリーリストに返却される
    EXPECT_EQ(4, cg::Chull2TestAccess::mright(chull));
    EXPECT_EQ(5, cg::Chull2TestAccess::inext(chull));

    // p3(凸包頂点番号1)がフリーリストに返却されているはず
    // kemp_が1を指しているか、kvert_[1]がフリーリストの次を指している
    const auto kemp = cg::Chull2TestAccess::kemp(chull);
    EXPECT_EQ(1, kemp);  // フリーリストの先頭が凸包頂点番号1(元のp3)
}

}  // namespace
