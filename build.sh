#!/usr/bin/env bash

mkdir -p build
cd build
cmake -S ../ -B . -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
make
mv route_finder ../
cd ..