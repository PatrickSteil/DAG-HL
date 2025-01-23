#!/usr/bin/bash

rm -rf build-release build-debug build-san
mkdir -p build-release build-debug build-san

cd build-debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
cd ..

cd build-san
cmake -DCMAKE_BUILD_TYPE=Sanitize ..
cd ..

cd build-release
cmake -DCMAKE_BUILD_TYPE=Release ..
cd ..

cd build-debug
make
cd ..

cd build-release
make
cd ..

cd build-san
make
cd ..
