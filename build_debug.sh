#!/usr/bin/env bash

mkdir -p build_debug
cd build_debug
cmake -S ../ -B . -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug
make
mv debug__route_finder ../
cd ..