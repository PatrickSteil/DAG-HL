#!/usr/bin/bash

rm -rf build-release build-debug build-san build-benchmarks
mkdir -p build-release build-debug build-san build-benchmarks

cd build-release
cmake -DCMAKE_BUILD_TYPE=Release ..
make

cd ../build-debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
make

cd ../build-san
cmake -DCMAKE_BUILD_TYPE=Sanitize ..
make

cd ../build-benchmarks
cmake -DENABLE_BENCHMARKS=ON -DCMAKE_BUILD_TYPE=Release ..
make

cd ..
