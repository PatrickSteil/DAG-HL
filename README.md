# Hub Labeling
Author: Patrick Steil

## Build Instructions
```bash
mkdir -p build-debug build
```
Generate Debug Build:
```bash
cd build-debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake --build .
```
Generate Release Build:
```bash
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
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
