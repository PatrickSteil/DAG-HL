#!/usr/bin/bash

mkdir -p build-release build-debug build-san

cd build-release
cmake -DCMAKE_BUILD_TYPE=Release ..

cd ../build-debug
cmake -DCMAKE_BUILD_TYPE=Debug ..

cd ../build-san
cmake -DCMAKE_BUILD_TYPE=Sanitize ..

cd ..
