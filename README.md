
# Hub Labeling

Author: Patrick Steil

This repository implements a hub labeling algorithm for reachability queries on directed acyclic graphs (DAGs). It does **not** compute or store shortest path distances.

## Build Instructions

To build the project, run the provided `build.sh` script:

```bash
./build.sh
```

Alternatively, to build only the release version:

```bash
mkdir -p build-release
cd build-release
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

The project supports three build configurations:

-   **Release:** Optimized for performance.
-   **Debug:** Includes debug symbols and error checks.
-   **Sanitize:** Uses sanitizers for runtime error detection.

To build and run the release version:

```bash
cd build-release
make
./DAGHL -h
```

## Output File Format

The output file contains hub labels for all vertices in the graph. Each vertex has exactly two lines:

-   **`o` line** – outgoing hubs (forward labels).
-   **`i` line** – incoming hubs (backward labels).

### File Structure
1.  **Outgoing Hubs (`o` line)**
    -   Starts with `o`.
    -   Followed by the vertex ID.
    -   Followed by hubs reachable in the forward direction.
    -   Example: `o 0 1 3` → Vertex `0` has outgoing hubs `1` and `3`.
2.  **Incoming Hubs (`i` line)**
    -   Starts with `i`.
    -   Followed by the vertex ID.
    -   Followed by hubs reachable in the backward direction.
    -   Example: `i 0 2 4` → Vertex `0` has incoming hubs `2` and `4`.
3.  Each vertex contributes **exactly two lines** to the file (`o` and `i`).
4.  The total number of lines is `2 × number of vertices`.

### Example Output
For a graph with the following labels:
-   Vertex `0`: Outgoing: `[1, 3]`, Incoming: `[2, 4]`
-   Vertex `1`: Outgoing: `[2]`, Incoming: `[0, 3]`

The output file:
```
o 0 1 3
i 0 2 4
o 1 2
i 1 0 3
```

## Example Execution
### Parallel Version
Using the **bitset scheme** and **tail parallelism** with 4 threads:
```bash
./DAGHL -s -i ../data/kvv.dimacs -t 4 -c -b
```

**Example Output:**
```
Reading graph from DIMACS ... done [2627ms]
Forward Graph Statistics:
  Number of vertices: 2527390
  Number of edges:    7955735
  Min degree:         0
  Max degree:         15
  Average degree:     3.14781
Reversing Graph ... done [103ms]
Initializing data structures ... done [191ms]
Computing HLs ... done [39661ms]
Computing hub permutation ... done [2450ms]
Sorting labels ... done [592ms]
Forward Labels:
  Min Size:     1
  Max Size:     240
  Avg Size:     78.5583
Backward Labels:
  Min Size:     1
  Max Size:     227
  Avg Size:     69.6271
FWD # count:    198547362
BWD # count:    175974860
Both # count:   374522222
Total memory usage: 2188.12 MB
10000 random queries: total time 180436 ms, avg time 18.0436 ns
```

### Sequential PLL Version (Single Threaded)

```bash
./PLL -s -i ../data/kvv.dimacs -c -b
```

**Example Output:**
```
Reading graph from DIMACS ... done [2627ms]
Forward Graph Statistics:
  Number of vertices: 2527390
  Number of edges:    7955735
  Min degree:         0
  Max degree:         15
  Average degree:     3.14781
Reversing Graph ... done [106ms]
Computing HLs ... done [105986ms]
Computing hub permutation ... done [2444ms]
Sorting labels ... done [407ms]
Forward Labels:
  Min Size:     1
  Max Size:     240
  Avg Size:     78.5563
Backward Labels:
  Min Size:     1
  Max Size:     227
  Avg Size:     69.6233
FWD # count:    198542389
BWD # count:    175965342
Both # count:   374507731
Total memory usage: 2149.45 MB
10000 random queries: total time 180408 ms, avg time 18.0408 ns
```

## Reference

This project is based on:
- **"Fast Exact Shortest-Path Distance Queries on Large Networks by Pruned Landmark Labeling"** – Akiba et al. [ArXiv](https://arxiv.org/pdf/1304.4661)
- **"Robust Exact Distance Queries on Massive Networks"** – Delling et al. [Microsoft](https://www.microsoft.com/en-us/research/wp-content/uploads/2014/07/complexTR-rev2.pdf)
