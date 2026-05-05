#!/usr/bin/env bash
# Fetch vendored ST CMSIS + STM32F1xx LL drivers as git submodules.
# Run once after cloning the repo.
set -euo pipefail
cd "$(dirname "$0")/.."
git submodule update --init --depth 1 --recursive
echo "Vendor setup complete."
echo "Submodules:"
git submodule status
