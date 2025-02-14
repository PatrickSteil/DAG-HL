#!/bin/bash

cd ../build-release/
make
./edgetree_bench --benchmark_format=csv > ../benchmark/benchmark_edge_tree.csv
./compress_vector_bench --benchmark_format=csv > ../benchmark/benchmark_compress_vector.csv
./bit_vector_bench --benchmark_format=csv > ../benchmark/benchmark_bit_vector.csv

cd ../benchmark/
python3 plot.py
python3 plot_compressed_vector.py
python3 plot_bit_vector.py
