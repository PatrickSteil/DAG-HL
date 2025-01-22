#!/bin/bash

cd ../build-release/
./edgetree_bench --benchmark_format=csv > ../benchmark/benchmark_results.csv

cd ../benchmark/
python3 plot.py
