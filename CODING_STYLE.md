# Coding Style Guide

このプロジェクトのコーディング規約ドキュメントです。

## 目次

1. [概要](#概要)
2. [命名規則](#命名規則)
3. [インデントとスペース](#インデントとスペース)
4. [ブレースの配置](#ブレースの配置)
5. [関数とメソッド](#関数とメソッド)
6. [クラス設計](#クラス設計)
7. [ファイル構成](#ファイル構成)
8. [コメント](#コメント)
9. [C++機能の使用](#c機能の使用)
10. [自動フォーマット](#自動フォーマット)

---

## 概要

このプロジェクトは以下の規約に従います：

- **言語**: C++20
- **スタイル**: Allman (BSD) スタイル
- **インデント**: 4スペース（タブ不使用）
- **行長**: 最大100文字を推奨

---

## 命名規則

### クラス名

**PascalCase**を使用します。

```cpp
class Chull2
{
    // ...
};

struct Chull2TestAccess
{
    // ...
};
```

### メンバ変数

**snake_case + アンダースコアサフィックス（`_`）**を使用します。

```cpp
class Chull2
{
private:
    std::vector<Eigen::Vector2d> input_points_;
    std::vector<int> mol_;
    int kemp_;
    int mright_;
};
```

### メンバ関数・関数

**snake_case**を使用します。

```cpp
auto sort_points_by_x_then_y() -> void;
auto generate_initial_triangle() -> void;
auto find_upper_tangent_point(int newv, int mv1, int jv1) -> int;
```

### ローカル変数・引数

**snake_case**を使用します。

```cpp
auto find_upper_tangent_point(int newv, int mv1, int jv1) -> int
{
    const auto mol_0 = mol_.at(0);
    const auto mol_1 = mol_.at(1);
    auto upper_vertex = kvert_.at(0);
}
```

### 名前空間

**snake_case**を使用します。

```cpp
namespace computational_geometry
{
    // ...
}

namespace utils
{
    // ...
}
```

### 定数・マクロ

- **定数**: `snake_case`または`UPPER_SNAKE_CASE`
- **マクロ**: `UPPER_SNAKE_CASE`

```cpp
constexpr int max_vertices = 1000;
constexpr int MAX_ITERATIONS = 10000;

#define CHULL2_VERSION_MAJOR 1
```

---

## インデントとスペース

### インデント

- **4スペース**を使用
- **タブ文字は使用しない**

```cpp
class Chull2
{
public:
    Chull2(const std::vector<Eigen::Vector2d>& input_points)
        : input_points_{input_points},
          mol_(input_points.size()),
          kemp_{0}
    {
        std::iota(mol_.begin(), mol_.end(), 0);
        sort_points_by_x_then_y();
    }
};
```

### スペースの配置

#### 演算子の周り

```cpp
int a = b + c;
if (x == y)
bool result = (a && b) || c;
```

#### 制御構造のキーワード後

```cpp
if (condition)
for (int i = 0; i < n; ++i)
while (running)
```

#### カンマの後

```cpp
void function(int a, int b, int c);
std::vector<int> vec{1, 2, 3};
```

#### テンプレート

```cpp
std::vector<std::vector<int>> matrix;
template <typename T>
```

---

## ブレースの配置

### Allman (BSD) スタイル

**すべての開き括弧は改行後に配置します。**

#### 関数定義

```cpp
auto sort_points_by_x_then_y() -> void
{
    // 実装
}

void function_name(int arg1, int arg2)
{
    // 実装
}
```

#### クラス定義

```cpp
class Chull2
{
public:
    Chull2(const std::vector<Eigen::Vector2d>& input_points);

private:
    std::vector<Eigen::Vector2d> input_points_;
};
```

#### 制御構造

```cpp
if (condition)
{
    // true の場合
}
else
{
    // false の場合
}

for (int i = 0; i < n; ++i)
{
    // ループ本体
}

while (running)
{
    // ループ本体
}

switch (value)
{
case 1:
{
    // ケース1
    break;
}
case 2:
{
    // ケース2
    break;
}
default:
{
    // デフォルト
    break;
}
}
```

#### 名前空間

名前空間は例外的に**同じ行に開き括弧**を配置します。

```cpp
namespace computational_geometry {

class Chull2
{
    // ...
};

} // namespace computational_geometry
```

#### ラムダ式

短いラムダ式は1行でも可：

```cpp
auto pred = [](int x) { return x > 0; };
```

複雑なラムダ式は改行：

```cpp
std::sort(mol_.begin(), mol_.end(),
          [this](int lhs, int rhs)
          {
              const auto& lhs_point = input_points_.at(static_cast<std::size_t>(lhs));
              const auto& rhs_point = input_points_.at(static_cast<std::size_t>(rhs));
              if (lhs_point.x() == rhs_point.x())
              {
                  return lhs_point.y() < rhs_point.y();
              }
              return lhs_point.x() < rhs_point.x();
          });
```

---

## 関数とメソッド

### 戻り値型の宣言

**Trailing return type**を使用します。

```cpp
auto sort_points_by_x_then_y() -> void;
auto generate_initial_triangle() -> void;
auto find_upper_tangent_point(int newv, int mv1, int jv1) -> int;
auto fetch_unused_vertex_index() -> int;
```

### 引数の渡し方

- **プリミティブ型**: 値渡し
- **大きなオブジェクト**: const参照渡し
- **変更する場合**: 非const参照渡しまたはポインタ

```cpp
void process_value(int n);                                     // プリミティブ
void process_vector(const std::vector<int>& vec);             // const参照
void modify_vector(std::vector<int>& vec);                     // 非const参照
auto calculate(const Eigen::Vector2d& point) -> double;        // const参照
```

---

## クラス設計

### アクセス指定子の順序

1. `public`
2. `protected`
3. `private`

```cpp
class Chull2
{
public:
    // コンストラクタ・デストラクタ
    Chull2(const std::vector<Eigen::Vector2d>& input_points);
    ~Chull2() = default;

    // パブリックメソッド
    auto compute() -> void;

protected:
    // プロテクテッドメンバ

private:
    // プライベートメンバ変数
    std::vector<Eigen::Vector2d> input_points_;
    int kemp_;

    // プライベートメソッド
    auto sort_points_by_x_then_y() -> void;

    // フレンド宣言
    friend struct Chull2TestAccess;
};
```

### メンバ初期化

**初期化リスト**を使用します。

```cpp
Chull2::Chull2(const std::vector<Eigen::Vector2d>& input_points)
    : input_points_{input_points},
      mol_(input_points.size()),
      nvlist_(),
      kemp_{0},
      mright_{0}
{
    // コンストラクタ本体
}
```

### テストアクセス用構造体

プライベートメンバをテストする場合、`friend`構造体を使用します。

```cpp
// ヘッダーファイル
class Chull2
{
private:
    std::vector<int> kvert_;
    auto generate_initial_triangle() -> void;

    friend struct Chull2TestAccess;
};

struct Chull2TestAccess
{
    static void GenerateInitialTriangle(Chull2& instance)
    {
        instance.generate_initial_triangle();
    }

    static const std::vector<int>& kvert(const Chull2& instance)
    {
        return instance.kvert_;
    }
};
```

---

## ファイル構成

### ディレクトリ構造

```
project_root/
├── src/
│   ├── hdr/          # ヘッダーファイル (.h)
│   └── cpp/          # ソースファイル (.cpp)
├── tests/            # テストファイル (*_test.cpp)
├── CMakeLists.txt
├── .clang-format
├── .editorconfig
└── CODING_STYLE.md
```

### ヘッダーファイル (.h)

#### インクルードガード

`#ifndef`/`#define`形式を使用します。

```cpp
#ifndef CHULL2_H
#define CHULL2_H

// ヘッダー内容

#endif // CHULL2_H
```

#### インクルード順序

1. 対応するヘッダー（実装ファイルの場合）
2. プロジェクト内ヘッダー（`"header.h"`）
3. サードパーティライブラリ（`<eigen3/Eigen/Dense>`）
4. C++標準ライブラリ（`<vector>`, `<algorithm>`など）

```cpp
// chull2.cpp
#include "chull2.h"        // 1. 対応するヘッダー
#include "utils.h"         // 2. プロジェクト内ヘッダー
#include <eigen3/Eigen/Dense>  // 3. サードパーティ
#include <algorithm>       // 4. 標準ライブラリ
#include <numeric>
#include <vector>
```

### ソースファイル (.cpp)

#### 名前空間エイリアス

長い名前空間にはエイリアスを使用できます。

```cpp
namespace cg = computational_geometry;

cg::Chull2::Chull2(const std::vector<Eigen::Vector2d>& input_points)
    : input_points_{input_points}
{
    // ...
}
```

---

## コメント

### スタイル

**C++スタイル（`//`）**を使用します。

```cpp
// これは1行コメントです

// 複数行のコメントは
// このように書きます
```

### 言語

**日本語**を使用します。

```cpp
// x座標ソート後の点を「元の並び番号」で表すためのリスト
std::vector<int> mol_;

// 反時計回り頂点リスト（NAMEを入れる）
std::vector<int> kccv_;
```

### TODOコメント

```cpp
// TODO: utils::turns_leftが3値を返すようにする。1, 0, -1
if (condition)
{
    // ...
}
```

### クラス・関数の説明

```cpp
// 2次元凸包を計算するクラス
// Graham's scanアルゴリズムの変種を使用
class Chull2
{
    // ...
};

// 上側の接線点を探索する
// newv: 新しい頂点のインデックス
// mv1: 探索開始頂点
// jv1: 探索開始頂点の座標インデックス
// 戻り値: 接線点のインデックス
auto find_upper_tangent_point(int newv, int mv1, int jv1) -> int;
```

---

## C++機能の使用

### C++20機能

このプロジェクトは**C++20**を使用します。

### auto キーワード

型が明白な場合は`auto`を積極的に使用します。

```cpp
auto mol_0 = mol_.at(0);
const auto mol_1 = mol_.at(1);
const auto& lhs_point = input_points_.at(static_cast<std::size_t>(lhs));

// ただし、型が不明瞭な場合は明示的に書く
std::vector<int> indices;  // auto ではなく明示的に
```

### 型キャスト

**`static_cast`**を使用します。Cスタイルキャストは避けます。

```cpp
// 良い例
auto size = static_cast<int>(input_points_.size());
const auto& point = input_points_.at(static_cast<std::size_t>(index));

// 悪い例
auto size = (int)input_points_.size();  // Cスタイルキャスト
```

### 初期化

**中括弧初期化**を優先します。

```cpp
std::vector<int> vec{1, 2, 3};
Eigen::Vector2d point{1.0, 2.0};

Chull2::Chull2(const std::vector<Eigen::Vector2d>& input_points)
    : input_points_{input_points},
      kemp_{0}
{
    // ...
}
```

### ポインタと参照

左寄せで記述します。

```cpp
int* ptr;
int& ref;
const std::vector<int>& vec;
```

### Range-based for

可能な限り使用します。

```cpp
for (const auto& point : input_points_)
{
    // pointを使用
}

for (auto& value : vec)
{
    value *= 2;  // 変更する場合
}
```

### nullptr

`NULL`や`0`の代わりに**`nullptr`**を使用します。

```cpp
int* ptr = nullptr;

if (ptr == nullptr)
{
    // ...
}
```

### enum class

スコープ付き列挙型を使用します。

```cpp
enum class Orientation
{
    Clockwise,
    CounterClockwise,
    Collinear
};

auto orient = Orientation::CounterClockwise;
```

---

## 自動フォーマット

### clang-format

プロジェクトルートに`.clang-format`ファイルがあります。

#### コマンドラインでフォーマット

```bash
# 単一ファイル
clang-format -i src/cpp/chull2.cpp

# ディレクトリ全体
find src/ -name "*.cpp" -o -name "*.h" | xargs clang-format -i
```

#### VSCodeでの設定

`.vscode/settings.json`に以下を追加：

```json
{
    "editor.formatOnSave": true,
    "C_Cpp.clang_format_style": "file"
}
```

#### CMakeからの実行

```bash
cmake --build build --target format
```

### EditorConfig

プロジェクトルートに`.editorconfig`ファイルがあります。
ほとんどのエディタが自動的に設定を読み込みます。

---

## テストコード

### Google Test規約

テスト名は**PascalCase**を使用します。

```cpp
TEST(Chull2Test, SortIndicesByXThenY)
{
    // テストコード
}

TEST(Chull2Test, GenerateInitialTriangle_WithCounterClockwiseTriangle)
{
    // テストコード
}

TEST(TurnsLeftTest, ReturnsTrueForCounterClockwiseTurn)
{
    // テストコード
}
```

### テストファイル名

`*_test.cpp`の形式を使用します。

```
tests/
├── chull2_test.cpp
├── utils_test.cpp
└── ...
```

---

## まとめ

- **Allman (BSD) スタイル**でブレースを配置
- **4スペース**インデント（タブ不使用）
- **snake_case**関数名、**PascalCase**クラス名
- **メンバ変数は末尾にアンダースコア**（`member_`）
- **Trailing return type**（`auto func() -> int`）
- **日本語コメント**を使用
- **C++20機能**を積極活用
- **`.clang-format`**で自動フォーマット

---

## 参考資料

- [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines)
- [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- [Clang-Format Documentation](https://clang.llvm.org/docs/ClangFormat.html)
