#!/bin/bash

set -euo pipefail

cmake --build build
ctest --output-on-failure --test-dir build

#cmake --build build を実行し全ターゲットがビルドされることを確認。
#ctest --output-on-failure を実行し 3 件のテストがすべて成功。
#必要に応じて build/utils_tests でテストを個別実行。
#他のユーティリティが増えたら同様に utils_lib に追加し、テストを拡充。
