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
```

The project supports three build configurations:

- Release: Optimized for performance.
- Debug: Includes debug symbols and error checks.
- Sanitize: Configured with sanitizers for runtime error detection.

For example, to build and run the release build:
```bash
cd build-release
make
./HCL -h
```

## Output File Format
The file contains the labels for all vertices in the graph. Each vertex contributes two lines:
-   An **`o` line** for the outgoing hubs (forward labels).
-   An **`i` line** for the incoming hubs (backward labels).

#### File Format
1.  **`o` Line (Outgoing Hubs)**:
    -   Starts with the character `o`.
    -   Followed by the hubs that are reachable in the forward direction.
    -   Example: `o 1 3`.
2.  **`i` Line (Incoming Hubs)**:
    -   Starts with the character `i`.
    -   Followed by the hubs that are reachable in the backward direction.
    -   Example: `i 2 4`.
3.  Each vertex contributes exactly **two lines** to the file, one starting with `o` and one starting with `i`.
4. The number of lines is 2 * number of vertices.
5. The hubs for vertex v starts at line number 2*v. 

#### Example File
For a graph with the following labels:
-   Vertex 0: Outgoing: [1, 3], Incoming: [2, 4]
-   Vertex 1: Outgoing: [2], Incoming: [0, 3]
The output file will be:
```
o 1 3
i 2 4
o 2
i 0 3
```

#### **Notes**
-   The file format is space-separated.
-   Hubs are listed in arbitrary order but reflect the graph's hub labeling.

#### **Usage**
-   The format is compact and easy to parse programmatically, with the leading `o` or `i` distinguishing the direction of the labels.

## Example

```bash
>> ./HCL -i icice.dimacs.gr -s -c -b
Reading graph from dimacs ... done [189ms]
Forward Graph Statistics:
  Number of vertices: 186068
  Number of edges:    586679
  Min degree:         0
  Max degree:         19
  Average degree:     3.15304
Reversing Graph ... done [3ms]
Computing a topological ordering ... done [6ms]
Computing HLs ... done [3180ms]
Compute Hub permutation ... done [56ms]
Sort all labels ... done [24ms]
Forward Labels Statistics:
  Min Size: 1
  Max Size: 232
  Avg Size: 65.1842
Backward Labels Statistics:
  Min Size: 1
  Max Size: 131
  Avg Size: 42.7483
Total memory consumption [megabytes]:
  117.881
The 10000 random queries took in total 200523 [ms] and on average 20.0523 [ns]!
```
