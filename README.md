# RXL

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
