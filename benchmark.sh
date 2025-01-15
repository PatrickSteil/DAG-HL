#!/usr/bin/bash
#
max_cores=$(nproc)

for dimacs_file in data/*.dimacs; do
    echo "Processing file: $dimacs_file"

    threads=1
    while [ $threads -le $max_cores ]; do
        echo "Running with $threads threads"

        ./build-release/DAGHL -i "$dimacs_file" -t "$threads"

        threads=$((threads * 2))
    done
done
