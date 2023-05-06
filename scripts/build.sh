#!/bin/bash
echo "Building..."
rm -r build/
mkdir build && cd build
cmake .. && make
echo "Finished"