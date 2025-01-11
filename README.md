# Hub Labeling
Author: Patrick Steil

This repository implements a hub labeling algorithm for reachability requests on DAGs. No shortest path distances are computed or stored.

## Build Instructions
To build the project, run the provided build.sh script:

```bash
./build.sh
```

Or if you only want to build the release version, call:

```bash
mkdir -p build-release 
cd build-release
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

The project supports three build configurations:

- Release: Optimized for performance.
- Debug: Includes debug symbols and error checks.
- Sanitize: Configured with sanitizers for runtime error detection.

For example, to build and run the release build:
```bash
cd build-release
make
./DAGHL -h
```

## Output File Format
The file contains the labels for all vertices in the graph. Each vertex contributes two lines:
-   An **`o` line** for the outgoing hubs (forward labels).
-   An **`i` line** for the incoming hubs (backward labels).

#### File Format
1.  **`o` Line (Outgoing Hubs)**:
    -   Starts with the character `o`.
    -   Followed by the vertex ID.
    -   Followed by the hubs that are reachable in the forward direction.
    -   Example: `o 0 1 3`, indicates the outgoing hubs for vertex 0 are: 1 and 3
2.  **`i` Line (Incoming Hubs)**:
    -   Starts with the character `i`.
    -   Followed by the vertex ID.
    -   Followed by the hubs that are reachable in the backward direction.
    -   Example: `i 0 2 4`, indicates the incoming hubs for vertex 0 are: 2 and 4
3.  Each vertex contributes exactly **two lines** to the file, one starting with `o` and one starting with `i`.
4. The number of lines is 2 * number of vertices.

#### Example File
For a graph with the following labels:
-   Vertex 0: Outgoing: [1, 3], Incoming: [2, 4]
-   Vertex 1: Outgoing: [2], Incoming: [0, 3]

The output file will be:
```
o 0 1 3
i 0 2 4
o 1 2
i 1 0 3
```

## Example

```bash
>>> ./DAGHL -s -i ../data/icice.dimacs -t 6 -b
Reading graph from dimacs ... done [188ms]
Forward Graph Statistics:
  Number of vertices: 186068
  Number of edges:    586679
  Min degree:         0
  Max degree:         19
  Average degree:     3.15304
Reversing Graph ... done [6ms]
Init the datastructures ... done [52ms]
Computing HLs ... done [1118ms]
Sort all labels ... done [86ms]
Forward Labels Statistics:
  Min Size:     1
  Max Size:     222
  Avg Size:     64.8756
Backward Labels Statistics:
  Min Size:     1
  Max Size:     133
  Avg Size:     42.7924
FWD # count:    12071266
BWD # count:    7962297
Both # count:   20033563
Total memory consumption [megabytes]:
  131.711
The 10000 random queries took in total 197383 [ms] and on average 19.7383 [ns]!
```

## Reference

Based on "Fast Exact Shortest-Path Distance Queries on Large Networks by Pruned Landmark Labeling" by Akiba et al [ArXiv](https://arxiv.org/pdf/1304.4661).
