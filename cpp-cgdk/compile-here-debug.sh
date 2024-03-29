set -ex

find -type f -name "*.cpp" | xargs g++ -std=c++17 -static -fno-optimize-sibling-calls -fno-strict-aliasing -fno-omit-frame-pointer -Og -g3 -D_LINUX -lm -x c++ -Wall -Wtype-limits -Wno-unknown-pragmas -o MyStrategyDebug

