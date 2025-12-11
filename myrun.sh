#!/bin/bash

set -euo pipefail

cmake --build build
#ctest --output-on-failure --test-dir build

#cmake --build build を実行し全ターゲットがビルドする。
#ctest --output-on-failure --test-dir buildでテストを実行。

# HTML作成
# doxygen DoxyFile

# PDF作成
# cd doc/latex
# make

JSON_PATH="/home/kumada/data/computational_geometry_programming/convex_hull_output.json"
IMAGE_PATH="/home/kumada/data/computational_geometry_programming/convex_hull_output.jpg"
N_POINTS=100
./build/computational_geometry_programming \
    --json_path "${JSON_PATH}" \
    --image_path "${IMAGE_PATH}" \
    --n_points "${N_POINTS}" \
    --seed 12345