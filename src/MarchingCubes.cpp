#include "MarchingCubes.h"
#include "Octree.h"
#include "OctreeNode.h"

#include <glm/glm.hpp>
#include <vector>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <unordered_map>
#include <functional>
#include <limits>

// ============================================================================
// Lorensen & Cline  ?? Marching Cubes lookup tables
// ============================================================================

// Corner layout (standard MC convention ??Paul Bourke):
//
//        7 -------- 6          y
//       /|         /|          |
//      4 -------- 5 |         |
//      | 3 -------| 2         +--- x
//      |/         |/          /
//      0 -------- 1          z
//
// corner offset (dx, dy, dz) where each is 0 or 1:
//   0:(0,0,0) 1:(1,0,0) 2:(1,1,0) 3:(0,1,0)
//   4:(0,0,1) 5:(1,0,1) 6:(1,1,1) 7:(0,1,1)
//
// Edge indices (pair of corners):
//   0:(0,1) 1:(1,2) 2:(2,3) 3:(3,0)   -- bottom face
//   4:(4,5) 5:(5,6) 6:(6,7) 7:(7,4)   -- top face
//   8:(0,4) 9:(1,5) 10:(2,6) 11:(3,7)  -- vertical
// ============================================================================

static const int edgeConn[12][2] = {
    {0,1},{1,2},{2,3},{3,0},
    {4,5},{5,6},{6,7},{7,4},
    {0,4},{1,5},{2,6},{3,7}
};

static const float cornerOff[8][3] = {
    {0,0,0},{1,0,0},{1,1,0},{0,1,0},
    {0,0,1},{1,0,1},{1,1,1},{0,1,1}
};

// edgeTable[256]: bitmask of intersected edges for each cube configuration
static const int edgeTable[256] = {
0x0  ,0x109,0x203,0x30a,0x406,0x50f,0x605,0x70c,
0x80c,0x905,0xa0f,0xb06,0xc0a,0xd03,0xe09,0xf00,
0x190,0x99 ,0x393,0x29a,0x596,0x49f,0x795,0x69c,
0x99c,0x895,0xb9f,0xa96,0xd9a,0xc93,0xf99,0xe90,
0x230,0x339,0x33 ,0x13a,0x636,0x73f,0x435,0x53c,
0xa3c,0xb35,0x83f,0x936,0xe3a,0xf33,0xc39,0xd30,
0x3a0,0x2a9,0x1a3,0xaa ,0x7a6,0x6af,0x5a5,0x4ac,
0xbac,0xaa5,0x9af,0x8a6,0xfaa,0xea3,0xda9,0xca0,
0x460,0x569,0x663,0x76a,0x66 ,0x16f,0x265,0x36c,
0xc6c,0xd65,0xe6f,0xf66,0x86a,0x963,0xa69,0xb60,
0x5f0,0x4f9,0x7f3,0x6fa,0x1f6,0xff ,0x3f5,0x2fc,
0xdfc,0xcf5,0xfff,0xef6,0x9fa,0x8f3,0xbf9,0xaf0,
0x650,0x759,0x453,0x55a,0x256,0x35f,0x55 ,0x15c,
0xe5c,0xf55,0xc5f,0xd56,0xa5a,0xb53,0x859,0x950,
0x7c0,0x6c9,0x5c3,0x4ca,0x3c6,0x2cf,0x1c5,0xcc ,
0xfcc,0xec5,0xdcf,0xcc6,0xbca,0xac3,0x9c9,0x8c0,
0x8c0,0x9c9,0xac3,0xbca,0xcc6,0xdcf,0xec5,0xfcc,
0xcc ,0x1c5,0x2cf,0x3c6,0x4ca,0x5c3,0x6c9,0x7c0,
0x950,0x859,0xb53,0xa5a,0xd56,0xc5f,0xf55,0xe5c,
0x15c,0x55 ,0x35f,0x256,0x55a,0x453,0x759,0x650,
0xaf0,0xbf9,0x8f3,0x9fa,0xef6,0xfff,0xcf5,0xdfc,
0x2fc,0x3f5,0xff ,0x1f6,0x6fa,0x7f3,0x4f9,0x5f0,
0xb60,0xa69,0x963,0x86a,0xf66,0xe6f,0xd65,0xc6c,
0x36c,0x265,0x16f,0x66 ,0x76a,0x663,0x569,0x460,
0xca0,0xda9,0xea3,0xfaa,0x8a6,0x9af,0xaa5,0xbac,
0x4ac,0x5a5,0x6af,0x7a6,0xaa ,0x1a3,0x2a9,0x3a0,
0xd30,0xc39,0xf33,0xe3a,0x936,0x83f,0xb35,0xa3c,
0x53c,0x435,0x73f,0x636,0x13a,0x33 ,0x339,0x230,
0xe90,0xf99,0xc93,0xd9a,0xa96,0xb9f,0x895,0x99c,
0x69c,0x795,0x49f,0x596,0x29a,0x393,0x99 ,0x190,
0xf00,0xe09,0xd03,0xc0a,0xb06,0xa0f,0x905,0x80c,
0x70c,0x605,0x50f,0x406,0x30a,0x203,0x109,0x0
};

// triTable[256][16]: Paul Bourke standard table
// Matches cornerOff {0,0,0},{1,0,0},{1,1,0},{0,1,0},{0,0,1},{1,0,1},{1,1,1},{0,1,1}
// and edgeConn {0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}
static const int triTable[256][16] =
{{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
	{3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
	{3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
	{3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
	{9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
	{9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
	{2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
	{8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
	{9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
	{4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
	{3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
	{1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
	{4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
	{4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
	{9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
	{5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
	{2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
	{9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
	{0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
	{2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
	{10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
	{4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
	{5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
	{5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
	{9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
	{0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
	{1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
	{10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
	{8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
	{2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
	{7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
	{9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
	{2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
	{11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
	{9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
	{5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
	{11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
	{11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
	{1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
	{9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
	{5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
	{2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
	{0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
	{5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
	{6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
	{3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
	{6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
	{5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
	{1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
	{10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
	{6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
	{8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
	{7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
	{3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
	{5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
	{0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
	{9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
	{8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
	{5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
	{0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
	{6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
	{10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
	{10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
	{8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
	{1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
	{3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
	{0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
	{10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
	{3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
	{6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
	{9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
	{8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
	{3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
	{6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
	{0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
	{10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
	{10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
	{2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
	{7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
	{7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
	{2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
	{1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
	{11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
	{8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
	{0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
	{7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
	{10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
	{2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
	{6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
	{7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
	{2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
	{1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
	{10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
	{10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
	{0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
	{7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
	{6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
	{8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
	{9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
	{6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
	{4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
	{10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
	{8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
	{0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
	{1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
	{8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
	{10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
	{4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
	{10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
	{5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
	{11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
	{9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
	{6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
	{7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
	{3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
	{7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
	{9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
	{3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
	{6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
	{9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
	{1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
	{4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
	{7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
	{6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
	{3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
	{0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
	{6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
	{0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
	{11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
	{6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
	{5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
	{9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
	{1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
	{1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
	{10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
	{0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
	{5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
	{10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
	{11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
	{9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
	{7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
	{2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
	{8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
	{9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
	{9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
	{1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
	{9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
	{9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
	{5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
	{0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
	{10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
	{2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
	{0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
	{0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
	{9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
	{5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
	{3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
	{5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
	{8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
	{0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
	{9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
	{1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
	{3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
	{4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
	{9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
	{11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
	{11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
	{2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
	{9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
	{3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
	{1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
	{4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
	{4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
	{0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
	{3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
	{3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
	{0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
	{9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
	{1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}};

// ============================================================================
// dB/dt  (quadratic B-spline 誘몃텇, PoissonSolver.cpp 怨??숈씪)
// ============================================================================
static float dBSpline1D(float t)
{
    float ta = std::fabsf(t);
    if (ta < 0.5f)  return -2.0f * t;
    if (ta < 1.5f)  return (t > 0.0f ? 1.0f : -1.0f) * (ta - 1.5f);
    return 0.0f;
}

// ============================================================================
// ?대? ?⑥닔: 誘몃━ 媛?몄삩 ?몃뱶 由ъ뒪?몃? ?ъ슜?섏뿬 ?(p) ?됯?
// ============================================================================
static float evalChi(const std::vector<OctreeNode *> &nodes, const glm::vec3 &p)
{
    float chi = 0.0f;
    for (const OctreeNode *n : nodes)
    {
        float w = n->width();
        float tx = (p.x - n->center.x) / w;
        float ty = (p.y - n->center.y) / w;
        float tz = (p.z - n->center.z) / w;

        if (std::fabsf(tx) >= 1.5f || std::fabsf(ty) >= 1.5f || std::fabsf(tz) >= 1.5f)
            continue;

        float Fo = (OctreeNode::BSpline1D(tx) * OctreeNode::BSpline1D(ty) * OctreeNode::BSpline1D(tz)) / (w * w * w);
        chi += n->scalarValue * Fo;
    }
    return chi;
}

// ============================================================================
// ?대? ?⑥닔: 誘몃━ 媛?몄삩 ?몃뱶 由ъ뒪?몃? ?ъ슜?섏뿬 ?눹?p) ?됯?
// ============================================================================
static glm::vec3 evalGradChi(const std::vector<OctreeNode *> &nodes, const glm::vec3 &p)
{
    glm::vec3 grad(0.0f);
    for (const OctreeNode *n : nodes)
    {
        float w = n->width();
        float tx = (p.x - n->center.x) / w;
        float ty = (p.y - n->center.y) / w;
        float tz = (p.z - n->center.z) / w;

        if (std::fabsf(tx) >= 1.5f || std::fabsf(ty) >= 1.5f || std::fabsf(tz) >= 1.5f)
            continue;

        float Bx  = OctreeNode::BSpline1D(tx);
        float By  = OctreeNode::BSpline1D(ty);
        float Bz  = OctreeNode::BSpline1D(tz);
        float invW4 = 1.0f / (w * w * w * w);

        float xo = n->scalarValue;
        grad.x += xo * dBSpline1D(tx) * By  * Bz * invW4;
        grad.y += xo * Bx  * dBSpline1D(ty) * Bz * invW4;
        grad.z += xo * Bx  * By  * dBSpline1D(tz) * invW4;
    }
    return grad;
}

// Keep only the largest connected triangle component.
// This removes tiny floating shells that are common when extracting from noisy fields.
static void keepLargestComponent(MCMesh &mesh)
{
    const size_t triCount = mesh.indices.size() / 3;
    if (triCount == 0 || mesh.positions.empty())
        return;

    std::vector<std::vector<unsigned int>> vertToTri(mesh.positions.size());
    for (unsigned int t = 0; t < (unsigned int)triCount; ++t)
    {
        unsigned int i0 = mesh.indices[t * 3 + 0];
        unsigned int i1 = mesh.indices[t * 3 + 1];
        unsigned int i2 = mesh.indices[t * 3 + 2];
        if (i0 >= vertToTri.size() || i1 >= vertToTri.size() || i2 >= vertToTri.size())
            continue;
        vertToTri[i0].push_back(t);
        vertToTri[i1].push_back(t);
        vertToTri[i2].push_back(t);
    }

    std::vector<int> triComp(triCount, -1);
    int compCount = 0;
    size_t bestSize = 0;
    int bestComp = -1;

    for (unsigned int t0 = 0; t0 < (unsigned int)triCount; ++t0)
    {
        if (triComp[t0] != -1)
            continue;

        size_t compSize = 0;
        std::vector<unsigned int> stack;
        stack.push_back(t0);
        triComp[t0] = compCount;

        while (!stack.empty())
        {
            unsigned int t = stack.back();
            stack.pop_back();
            compSize++;

            for (int k = 0; k < 3; ++k)
            {
                unsigned int v = mesh.indices[t * 3 + (size_t)k];
                if (v >= vertToTri.size())
                    continue;
                for (unsigned int nt : vertToTri[v])
                {
                    if (triComp[nt] == -1)
                    {
                        triComp[nt] = compCount;
                        stack.push_back(nt);
                    }
                }
            }
        }

        if (compSize > bestSize)
        {
            bestSize = compSize;
            bestComp = compCount;
        }
        compCount++;
    }

    if (compCount <= 1 || bestComp < 0)
        return;

    std::vector<int> remap(mesh.positions.size(), -1);
    std::vector<glm::vec3> newPos;
    std::vector<glm::vec3> newNrm;
    std::vector<unsigned int> newIdx;
    newPos.reserve(bestSize * 3 / 2);
    newNrm.reserve(bestSize * 3 / 2);
    newIdx.reserve(bestSize * 3);

    for (unsigned int t = 0; t < (unsigned int)triCount; ++t)
    {
        if (triComp[t] != bestComp)
            continue;

        for (int k = 0; k < 3; ++k)
        {
            unsigned int oldV = mesh.indices[t * 3 + (size_t)k];
            if (oldV >= mesh.positions.size())
                continue;
            if (remap[oldV] < 0)
            {
                remap[oldV] = (int)newPos.size();
                newPos.push_back(mesh.positions[oldV]);
                newNrm.push_back(mesh.normals[oldV]);
            }
            newIdx.push_back((unsigned int)remap[oldV]);
        }
    }

    mesh.positions.swap(newPos);
    mesh.normals.swap(newNrm);
    mesh.indices.swap(newIdx);
    printf("[MC] component filter: kept largest component (%zu / %zu triangles, %d components)\n",
           mesh.indices.size() / 3, triCount, compCount);
}

// ============================================================================
// Public wrappers (?몄쓽???좎?)
// ============================================================================
float MarchingCubes::evaluateChi(const Octree *octree, const glm::vec3 &p)
{
    if (octree && octree->hasRegularGridField())
        return octree->sampleRegularGrid(p);
    auto nodes = octree->getAllNodes();
    return evalChi(nodes, p);
}

glm::vec3 MarchingCubes::gradChi(const Octree *octree, const glm::vec3 &p)
{
    if (octree && octree->hasRegularGridField())
        return octree->gradientRegularGrid(p);
    auto nodes = octree->getAllNodes();
    return evalGradChi(nodes, p);
}

float MarchingCubes::computeIsovalue(const Octree *octree,
                                     const std::vector<glm::vec3> &samples)
{
    if (samples.empty()) return 0.0f;
    double weightedSum = 0.0;
    double weightSum = 0.0;
    for (const auto &s : samples)
    {
        float W = octree->evaluateW(s);
        if (W <= 1e-12f)
            continue;
        double w = 1.0 / (double)W;
        weightedSum += w * (double)evaluateChi(octree, s);
        weightSum += w;
    }
    if (weightSum <= 0.0)
        return 0.0f;
    return (float)(weightedSum / weightSum);
}

// ============================================================================
// extract  ?? Marching Cubes on leaf nodes (罹먯떆???몃뱶 由ъ뒪???ъ슜)
// ============================================================================
MCMesh MarchingCubes::extract(const Octree *octree,
                              const std::vector<glm::vec3> &samplePositions,
                              int extractionDepth)
{
    MCMesh mesh;

    if (!octree || !octree->root)
    {
        printf("[MC] invalid octree.\n");
        return mesh;
    }

    std::vector<OctreeNode *> allNodes = octree->getAllNodes();
    if (allNodes.empty())
    {
        printf("[MC] no octree nodes.\n");
        return mesh;
    }

    printf("[MC] total nodes = %zu\n", allNodes.size());

    // PSR isovalue sigma = mean chi(sample)
    float sigma = 0.0f;
    if (!samplePositions.empty())
    {
        printf("[MC] computing weighted isovalue over %zu samples...\n", samplePositions.size());
        sigma = computeIsovalue(octree, samplePositions);
    }
    printf("[MC] isovalue sigma = %f\n", sigma);

    std::vector<OctreeNode *> leaves = octree->getAllLeaves();
    printf("[MC] leaf count = %zu\n", leaves.size());
    if (leaves.empty())
        return mesh;

    const int maxDepth = octree->maxDepth;
    const int fieldDepth = maxDepth;
    const bool useRegularGrid = octree->hasRegularGridField();
    if (extractionDepth < 0)
        extractionDepth = maxDepth + 1;
    extractionDepth = std::max(fieldDepth, extractionDepth);
    const int gridRes = 1 << extractionDepth; // number of extraction cells on each axis
    const float rootHalf = octree->root->halfSize;
    const glm::vec3 domainMin = octree->root->center - glm::vec3(rootHalf);
    const float finestStep = (2.0f * rootHalf) / (float)gridRes;
    printf("[MC] extractionDepth = %d (fieldDepth=%d, gridRes=%d)\n", extractionDepth, fieldDepth, gridRes);

    struct NodeKey
    {
        int x, y, z;
        bool operator==(const NodeKey &o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct NodeKeyHash
    {
        size_t operator()(const NodeKey &k) const
        {
            size_t h = 1469598103934665603ull;
            h ^= (size_t)k.x; h *= 1099511628211ull;
            h ^= (size_t)k.y; h *= 1099511628211ull;
            h ^= (size_t)k.z; h *= 1099511628211ull;
            return h;
        }
    };

    std::vector<std::unordered_map<NodeKey, const OctreeNode *, NodeKeyHash>> nodesByDepth(fieldDepth + 1);
    if (!useRegularGrid)
    {
        for (int d = 0; d <= fieldDepth; ++d)
            nodesByDepth[d].reserve(allNodes.size() / (fieldDepth + 1) + 8);

        for (const OctreeNode *n : allNodes)
        {
            if (!n) continue;
            if (n->depth < 0 || n->depth > fieldDepth) continue;
            const float step = 2.0f * n->halfSize;
            const int ix = (int)std::lround((n->center.x - domainMin.x) / step - 0.5f);
            const int iy = (int)std::lround((n->center.y - domainMin.y) / step - 0.5f);
            const int iz = (int)std::lround((n->center.z - domainMin.z) / step - 0.5f);
            nodesByDepth[n->depth][NodeKey{ix, iy, iz}] = n;
        }
    }

    auto evalChiFast = [&](const glm::vec3 &p) -> float
    {
        if (useRegularGrid)
            return octree->sampleRegularGrid(p);
        float chi = 0.0f;
        for (int d = 0; d <= fieldDepth; ++d)
        {
            const auto &level = nodesByDepth[d];
            if (level.empty()) continue;

            const float step = (2.0f * rootHalf) / (float)(1 << d);
            const float ux = (p.x - domainMin.x) / step - 0.5f;
            const float uy = (p.y - domainMin.y) / step - 0.5f;
            const float uz = (p.z - domainMin.z) / step - 0.5f;
            const int bx = (int)std::floor(ux);
            const int by = (int)std::floor(uy);
            const int bz = (int)std::floor(uz);

            for (int dz = -2; dz <= 2; ++dz)
            for (int dy = -2; dy <= 2; ++dy)
            for (int dx = -2; dx <= 2; ++dx)
            {
                auto it = level.find(NodeKey{bx + dx, by + dy, bz + dz});
                if (it == level.end()) continue;
                const OctreeNode *n = it->second;
                const float w = n->width();
                const float tx = (p.x - n->center.x) / w;
                const float ty = (p.y - n->center.y) / w;
                const float tz = (p.z - n->center.z) / w;
                if (std::fabsf(tx) >= 1.5f || std::fabsf(ty) >= 1.5f || std::fabsf(tz) >= 1.5f)
                    continue;
                chi += n->scalarValue * OctreeNode::BSpline1D(tx) * OctreeNode::BSpline1D(ty) * OctreeNode::BSpline1D(tz) / (w * w * w);
            }
        }
        return chi;
    };

    auto evalGradFast = [&](const glm::vec3 &p) -> glm::vec3
    {
        if (useRegularGrid)
            return octree->gradientRegularGrid(p);
        glm::vec3 grad(0.0f);
        for (int d = 0; d <= fieldDepth; ++d)
        {
            const auto &level = nodesByDepth[d];
            if (level.empty()) continue;

            const float step = (2.0f * rootHalf) / (float)(1 << d);
            const float ux = (p.x - domainMin.x) / step - 0.5f;
            const float uy = (p.y - domainMin.y) / step - 0.5f;
            const float uz = (p.z - domainMin.z) / step - 0.5f;
            const int bx = (int)std::floor(ux);
            const int by = (int)std::floor(uy);
            const int bz = (int)std::floor(uz);

            for (int dz = -2; dz <= 2; ++dz)
            for (int dy = -2; dy <= 2; ++dy)
            for (int dx = -2; dx <= 2; ++dx)
            {
                auto it = level.find(NodeKey{bx + dx, by + dy, bz + dz});
                if (it == level.end()) continue;
                const OctreeNode *n = it->second;
                const float w = n->width();
                const float tx = (p.x - n->center.x) / w;
                const float ty = (p.y - n->center.y) / w;
                const float tz = (p.z - n->center.z) / w;
                if (std::fabsf(tx) >= 1.5f || std::fabsf(ty) >= 1.5f || std::fabsf(tz) >= 1.5f)
                    continue;

                const float Bx  = OctreeNode::BSpline1D(tx);
                const float By  = OctreeNode::BSpline1D(ty);
                const float Bz  = OctreeNode::BSpline1D(tz);
                const float invW4 = 1.0f / (w * w * w * w);
                const float xo  = n->scalarValue;

                grad.x += xo * dBSpline1D(tx) * By  * Bz * invW4;
                grad.y += xo * Bx  * dBSpline1D(ty) * Bz * invW4;
                grad.z += xo * Bx  * By  * dBSpline1D(tz) * invW4;
            }
        }
        return grad;
    };

    struct GridKey
    {
        int x, y, z;
        bool operator==(const GridKey &o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct GridKeyHash
    {
        size_t operator()(const GridKey &k) const
        {
            size_t h = 1469598103934665603ull;
            h ^= (size_t)k.x; h *= 1099511628211ull;
            h ^= (size_t)k.y; h *= 1099511628211ull;
            h ^= (size_t)k.z; h *= 1099511628211ull;
            return h;
        }
    };

    std::unordered_map<GridKey, float, GridKeyHash> chiCache;
    chiCache.reserve((size_t)gridRes * (size_t)gridRes * 4);

    auto clampGrid = [&](int v) { return std::max(0, std::min(v, gridRes)); };

    auto gridToWorld = [&](int gx, int gy, int gz) -> glm::vec3
    {
        return domainMin + glm::vec3((float)gx, (float)gy, (float)gz) * finestStep;
    };

    auto sampleField = [&](int gx, int gy, int gz) -> float
    {
        gx = clampGrid(gx);
        gy = clampGrid(gy);
        gz = clampGrid(gz);
        GridKey key{gx, gy, gz};
        auto it = chiCache.find(key);
        if (it != chiCache.end())
            return it->second;
        float v = evalChiFast(gridToWorld(gx, gy, gz)) - sigma;
        chiCache.emplace(key, v);
        return v;
    };

    float fMin = std::numeric_limits<float>::max();
    float fMax = -std::numeric_limits<float>::max();
    int cellsVisited = 0;
    int cellsUniform = 0;
    int cellsMeshed = 0;

    struct EdgeKey
    {
        int ax, ay, az;
        int bx, by, bz;
        bool operator==(const EdgeKey &o) const
        {
            return ax == o.ax && ay == o.ay && az == o.az &&
                   bx == o.bx && by == o.by && bz == o.bz;
        }
    };
    struct EdgeKeyHash
    {
        size_t operator()(const EdgeKey &k) const
        {
            size_t h = 1469598103934665603ull;
            h ^= (size_t)k.ax; h *= 1099511628211ull;
            h ^= (size_t)k.ay; h *= 1099511628211ull;
            h ^= (size_t)k.az; h *= 1099511628211ull;
            h ^= (size_t)k.bx; h *= 1099511628211ull;
            h ^= (size_t)k.by; h *= 1099511628211ull;
            h ^= (size_t)k.bz; h *= 1099511628211ull;
            return h;
        }
    };

    auto makeEdgeKey = [](int x0, int y0, int z0, int x1, int y1, int z1) -> EdgeKey
    {
        if (x0 > x1 || (x0 == x1 && (y0 > y1 || (y0 == y1 && z0 > z1))))
        {
            std::swap(x0, x1);
            std::swap(y0, y1);
            std::swap(z0, z1);
        }
        return EdgeKey{x0, y0, z0, x1, y1, z1};
    };

    std::unordered_map<EdgeKey, unsigned int, EdgeKeyHash> edgeVertexCache;
    edgeVertexCache.reserve((size_t)gridRes * (size_t)gridRes * 8);

    std::function<void(int, int, int, int, int)> processCell;
    processCell = [&](int ix, int iy, int iz, int span, int depth)
    {
        float f[8];
        glm::vec3 cp[8];
        int gx[8], gy[8], gz[8];
        bool hasNeg = false;
        bool hasPos = false;

        for (int c = 0; c < 8; ++c)
        {
            gx[c] = ix + (int)cornerOff[c][0] * span;
            gy[c] = iy + (int)cornerOff[c][1] * span;
            gz[c] = iz + (int)cornerOff[c][2] * span;
            cp[c] = gridToWorld(gx[c], gy[c], gz[c]);
            f[c] = sampleField(gx[c], gy[c], gz[c]);
            fMin = std::min(fMin, f[c]);
            fMax = std::max(fMax, f[c]);
            if (f[c] < 0.0f) hasNeg = true;
            else             hasPos = true;
        }

        ++cellsVisited;
        // Always descend to maxDepth inside each leaf block.
        // Early culling at coarse levels can miss fine-scale zero-crossings because
        // chi is not trilinear over coarse cells.
        if (depth < maxDepth)
        {
            const int childSpan = span >> 1;
            if (childSpan <= 0)
                return;

            for (int dz = 0; dz < 2; ++dz)
            for (int dy = 0; dy < 2; ++dy)
            for (int dx = 0; dx < 2; ++dx)
                processCell(ix + dx * childSpan,
                            iy + dy * childSpan,
                            iz + dz * childSpan,
                            childSpan, depth + 1);
            return;
        }

        if (!(hasNeg && hasPos))
        {
            ++cellsUniform;
            return;
        }

        int cubeIdx = 0;
        for (int c = 0; c < 8; ++c)
            if (f[c] < 0.0f) cubeIdx |= (1 << c);

        if (edgeTable[cubeIdx] == 0)
            return;

        int edgeVert[12];
        for (int e = 0; e < 12; ++e) edgeVert[e] = -1;

        for (int e = 0; e < 12; ++e)
        {
            if (!(edgeTable[cubeIdx] & (1 << e)))
                continue;

            const int c0 = edgeConn[e][0];
            const int c1 = edgeConn[e][1];
            const EdgeKey ek = makeEdgeKey(gx[c0], gy[c0], gz[c0], gx[c1], gy[c1], gz[c1]);
            auto it = edgeVertexCache.find(ek);
            if (it != edgeVertexCache.end())
            {
                edgeVert[e] = (int)it->second;
                continue;
            }

            const float denom = (f[c0] - f[c1]);
            float t = (std::fabsf(denom) > 1e-12f) ? (f[c0] / denom) : 0.5f;
            t = glm::clamp(t, 0.0f, 1.0f);
            glm::vec3 v = cp[c0] + t * (cp[c1] - cp[c0]);

            glm::vec3 g = evalGradFast(v);
            float len = glm::length(g);
            glm::vec3 n = (len > 1e-12f) ? g / len : glm::vec3(0, 1, 0);

            unsigned int vid = (unsigned int)mesh.positions.size();
            mesh.positions.push_back(v);
            mesh.normals.push_back(n);
            edgeVertexCache.emplace(ek, vid);
            edgeVert[e] = (int)vid;
        }

        for (int i = 0; triTable[cubeIdx][i] != -1; i += 3)
        {
            const int i0 = edgeVert[triTable[cubeIdx][i    ]];
            const int i1 = edgeVert[triTable[cubeIdx][i + 1]];
            const int i2 = edgeVert[triTable[cubeIdx][i + 2]];
            if (i0 < 0 || i1 < 0 || i2 < 0)
                continue;

            mesh.indices.push_back((unsigned int)i0);
            mesh.indices.push_back((unsigned int)i1);
            mesh.indices.push_back((unsigned int)i2);
        }

        ++cellsMeshed;
    };

    // Fast field evaluator sanity check against exact summation.
    if (!samplePositions.empty() && !useRegularGrid)
    {
        size_t chkN = std::min<size_t>(samplePositions.size(), 512);
        double errSum = 0.0, errMax = 0.0;
        for (size_t i = 0; i < chkN; ++i)
        {
            float exact = evalChi(allNodes, samplePositions[i]);
            float fast  = evalChiFast(samplePositions[i]);
            double e = std::fabs((double)exact - (double)fast);
            errSum += e;
            errMax = std::max(errMax, e);
        }
        printf("[MC] evalChiFast sanity: samples=%zu avgAbsErr=%.3e maxAbsErr=%.3e\n",
               chkN, errSum / (double)chkN, errMax);
    }

    // Extract on a uniform finest grid.
    // Adaptive octree is used to evaluate chi/grad efficiently, while topology
    // is generated on a conforming regular grid to avoid adaptive MC cracks.
    for (int iz = 0; iz < gridRes; ++iz)
    for (int iy = 0; iy < gridRes; ++iy)
    for (int ix = 0; ix < gridRes; ++ix)
        processCell(ix, iy, iz, 1, maxDepth);

    const bool keepOnlyLargestComponent = false;
    if (keepOnlyLargestComponent)
        keepLargestComponent(mesh);

    printf("[MC] corner f range: [%e, %e]\n", fMin, fMax);
    printf("[MC] visited cells=%d uniform=%d meshed=%d cachedCorners=%zu\n",
           cellsVisited, cellsUniform, cellsMeshed, chiCache.size());
    printf("[MC] triangles = %zu, vertices = %zu\n",
           mesh.indices.size() / 3, mesh.positions.size());
    return mesh;
}

// ============================================================================
// exportOBJ  ?? Wavefront OBJ ?뚯씪 異쒕젰
//   v  x y z
//   f  v1 v2 v3
// ============================================================================
bool MarchingCubes::exportOBJ(const MCMesh &mesh, const std::string &path)
{
    FILE *fp = fopen(path.c_str(), "w");
    if (!fp) { printf("[OBJ] Failed to open %s\n", path.c_str()); return false; }

    size_t nv = mesh.positions.size();
    size_t nt = mesh.indices.empty() ? (nv / 3) : (mesh.indices.size() / 3);
    fprintf(fp, "# Poisson Surface Reconstruction - OBJ export\n");
    fprintf(fp, "# vertices: %zu  triangles: %zu\n\n", nv, nt);

    // vertices
    for (size_t i = 0; i < nv; i++)
        fprintf(fp, "v %f %f %f\n", mesh.positions[i].x, mesh.positions[i].y, mesh.positions[i].z);

    // faces (1-indexed)
    if (!mesh.indices.empty())
    {
        for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
        {
            size_t a = (size_t)mesh.indices[i] + 1;
            size_t b = (size_t)mesh.indices[i + 1] + 1;
            size_t c = (size_t)mesh.indices[i + 2] + 1;
            fprintf(fp, "f %zu %zu %zu\n", a, b, c);
        }
    }
    else
    {
        for (size_t i = 0; i + 2 < nv; i += 3)
            fprintf(fp, "f %zu %zu %zu\n",
                    i + 1, i + 2, i + 3);
    }

    fclose(fp);
    printf("[OBJ] Exported %zu triangles to %s\n", nt, path.c_str());
    return true;
}

