#!/usr/bin/bash

mkdir -p build-release build-debug build-san

cd build-release
cmake -DCMAKE_BUILD_TYPE=Release ..
make

cd ../build-debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
make

cd ../build-san
cmake -DCMAKE_BUILD_TYPE=Sanitize ..
make

cd ..
