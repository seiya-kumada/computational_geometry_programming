# Computational Geometry Programming

2次元凸包（Convex Hull）を計算するC++プログラムです。

## 概要

増分法（Incremental Algorithm）を用いて2次元点群の凸包を計算します。計算量は O(n log n) です。

## 機能

- 2次元凸包の計算
- 結果のJSON形式での出力
- OpenCVを用いた凸包の可視化
- 乱数による入力点群の生成

## 依存ライブラリ

- **C++23** - std::expected を使用
- **Eigen3** - 線形代数計算
- **OpenCV** - 画像描画・出力
- **nlohmann/json** - JSON入出力
- **Boost.Program_options** - コマンドライン引数解析
- **Google Test** - 単体テスト

## インストール

### 依存ライブラリのインストール (Ubuntu/Debian)

```bash
sudo apt update
sudo apt install libeigen3-dev libopencv-dev nlohmann-json3-dev libboost-program-options-dev
```

### ビルド

```bash
mkdir build
cd build
cmake ..
make
```

## 使い方

```bash
./computational_geometry_programming -j output.json -i output.png [-n 点数] [-s シード]
```

### オプション

| オプション | 説明 | デフォルト値 |
|------------|------|--------------|
| `-j, --json_path` | 出力JSONファイルのパス | (必須) |
| `-i, --image_path` | 出力画像ファイルのパス | (必須) |
| `-n, --n_points` | 入力点の数 | 100 |
| `-s, --seed` | 乱数シード | 12345 |
| `-h, --help` | ヘルプを表示 | - |

### 実行例

```bash
# 100点の凸包を計算し、結果を保存
./computational_geometry_programming -j result.json -i result.png

# 500点、シード42で計算
./computational_geometry_programming -j result.json -i result.png -n 500 -s 42
```

## 出力

### JSONファイル

入力点の座標と凸包を構成する頂点のインデックスが保存されます。

```json
{
    "input_points": [
        {"index": 0, "x": 1.23, "y": 4.56},
        {"index": 1, "x": 7.89, "y": 0.12},
        ...
    ],
    "convex_hull_indices": [0, 3, 7, 2]
}
```

### 画像ファイル

- 凸包の辺: 青色
- 凸包の頂点: 赤色
- 内部点: 緑色

## テスト

```bash
cd build
ctest
```

または直接実行:

```bash
./utils_tests
```

## プロジェクト構成

```
.
├── CMakeLists.txt
├── README.md
├── src/
│   ├── cpp/
│   │   ├── main.cpp       # メインプログラム
│   │   ├── chull2.cpp     # 凸包計算の実装
│   │   └── utils.cpp      # ユーティリティ関数
│   └── hdr/
│       ├── chull2.h       # Chull2クラスの定義
│       └── utils.h        # ユーティリティ関数の宣言
└── tests/
    ├── chull2_test.cpp    # Chull2クラスのテスト
    └── utils_test.cpp     # ユーティリティ関数のテスト
```

## アルゴリズム

増分法による2次元凸包計算の手順:

1. 入力点をx座標（同値ならy座標）でソート
2. 最初の3点で初期三角形を作成
3. 残りの点を1点ずつ追加し、凸包を更新
   - 上接線（upper tangent）を探索
   - 下接線（lower tangent）を探索
   - 凸包頂点リストを更新

## ドキュメント生成

Doxygenによるドキュメント生成:

```bash
sudo apt install doxygen graphviz
doxygen Doxyfile
```

## TODO
入力点座標$(x,y)$の範囲を$-1000.0\lt x,y\lt 1000.0$に限定し、SCALE=1e6で整数化(std::int64_t)し、凸包計算を行う。

## ライセンス

MIT License
