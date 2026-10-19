#!/usr/bin/env bash
# Fetch the stock espp web consoles the STEM firmware advertises (Device Hub,
# OTA, core dump) into this directory so they sit next to stem_console.html.
# The Device Hub links each discovered module's app as a same-directory page,
# so serving this one directory gives working links for all four.
#
# Usage: ./fetch_espp_consoles.sh [espp-checkout-dir]
#   With a local espp checkout the files are copied from it; otherwise they are
#   downloaded from the hosted apps at https://esp-cpp.github.io/espp/apps/.
set -euo pipefail
cd "$(dirname "$0")"

consoles=(dispatcher_hub.html ota_console.html coredump_console.html)
src_dirs=(components/dispatcher/web components/ota/web components/coredump/web)

if [[ $# -ge 1 ]]; then
  espp="$1"
  for i in "${!consoles[@]}"; do
    cp -L "$espp/${src_dirs[$i]}/${consoles[$i]}" .
    echo "copied ${consoles[$i]} from $espp"
  done
else
  for name in "${consoles[@]}"; do
    curl -fsSL "https://esp-cpp.github.io/espp/apps/$name" -o "$name"
    echo "downloaded $name"
  done
fi
