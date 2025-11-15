/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <cstdint>
#include <limits>
#include <vector>

typedef uint32_t Vertex;
typedef std::size_t Index;
typedef uint8_t Weight;

constexpr Vertex noVertex = uint32_t(-1);
constexpr Index noIndex = std::size_t(-1);
constexpr Weight noWeight = uint8_t(-1);

enum DIRECTION : bool { FWD, BWD };
