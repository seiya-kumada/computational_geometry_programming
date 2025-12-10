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