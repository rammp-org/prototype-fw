#!/usr/bin/env bash
# Build and run the STEM host-side tests (no ESP-IDF / hardware needed).
#
#   ./run_host_tests.sh                 # uses ../managed_components/espp__stream_frame (after an idf.py build)
#   ./run_host_tests.sh ~/esp-cpp/espp  # or point at an espp checkout for the stream_frame header
set -euo pipefail
cd "$(dirname "$0")"

if [[ $# -ge 1 ]]; then
  stream_frame_inc="$1/components/stream_frame/include"
else
  stream_frame_inc="../managed_components/espp__stream_frame/include"
fi
if [[ ! -f "$stream_frame_inc/stream_frame.hpp" ]]; then
  echo "stream_frame.hpp not found at $stream_frame_inc (run 'idf.py build' once, or pass an espp checkout)" >&2
  exit 1
fi

out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT
CXX="${CXX:-c++}"
flags=(-std=c++20 -Wall -Wextra -I ../main)

"$CXX" "${flags[@]}" -o "$out/kinematics" kinematics_host_test.cpp ../main/ik_5bar.cpp
"$out/kinematics"
"$CXX" "${flags[@]}" -I "$stream_frame_inc" -o "$out/protocol" protocol_host_test.cpp
"$out/protocol"
