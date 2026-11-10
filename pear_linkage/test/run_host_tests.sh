#!/usr/bin/env bash
# Build and run the PEAR linkage host-side tests (no ESP-IDF / hardware needed):
# the 5-bar kinematics (against the STEM edge poses and ik/fk round trips) and
# the wire protocol's payload round trips. Standard-library only.
#
#   ./run_host_tests.sh
set -euo pipefail
cd "$(dirname "$0")"

out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT
CXX="${CXX:-c++}"
flags=(-std=c++20 -Wall -Wextra -I ../main)

"$CXX" "${flags[@]}" -o "$out/kinematics" kinematics_host_test.cpp ../main/five_bar.cpp
"$out/kinematics"
"$CXX" "${flags[@]}" -o "$out/protocol" protocol_host_test.cpp ../main/five_bar.cpp
"$out/protocol"
