#!/usr/bin/bash

rm -rf build-release
mkdir build-release

cd build-release
cmake -DCMAKE_BUILD_TYPE=Release ..
make
cd ..

timestamp=$(date +"%Y%m%d_%H%M%S")

for file in data/*.dimacs; do
    if [ -f "$file" ]; then
        base_name=$(basename "$file")
        echo "Starting Hub Label for $file!";
        ./build-release/DAGHL -i "$file" -s -b > "result/${base_name%.txt}_${timestamp}.log"
        echo "Finished Hub Label for $file!";
    fi
done; 

