#!/bin/bash
#
# Script to build and test everything
#
# Demonstrates build system
#

set -e

echo "Building..."
cmake -S . -B build-can
cmake --build build-can

echo "Building tests..."
cmake -S . -B build-tests -DPICO_PLATFORM=host
cmake --build build-tests

echo "Testing..."
ctest --test-dir build-tests/test

echo "OK."
