#!/usr/bin/bash
#
max_cores=$(nproc)

for file in data/snap/*; do
    echo "Processing file: $file"

    threads=1
    while [ $threads -le $max_cores ]; do
        echo "Running with $threads threads"

        ./build-release/DAGHL -s -i "$file" -f SNAP -t "$threads"

        threads=$((threads * 2))
    done
done
