/*
 * Poisson Surface Reconstruction - Console Pipeline
 *
 * XYZ 점군 → Octree → Poisson Solve → Marching Cubes → OBJ 내보내기
 */

#include <stdio.h>
#include <glm/glm.hpp>
#include <vector>
#include <algorithm>
#include <string>
#include "common.h"
#include "Octree.h"
#include "MarchingCubes.h"

// =============================================================================
// Main  –  Console Pipeline (no GL window)
//
// 사용법:  PoissonSurfaceReconstruction.exe [input.xyz] [output.obj] [maxDepth] [densityDepth]
//          기본값: ../../resource/points.xyz  output.obj  8  6
// =============================================================================
int main(int argc, char *argv[])
{
    const char *inputPath  = (argc >= 2) ? argv[1] : "../../resource/points.xyz";
    const char *outputPath = (argc >= 3) ? argv[2] : "output.obj";
    int maxDepth           = (argc >= 4) ? atoi(argv[3]) : 6;
    int densityDepth       = (argc >= 5) ? atoi(argv[4]) : std::max(maxDepth - 2, 1);
    int extractionDepth    = (argc >= 6) ? atoi(argv[5]) : (maxDepth + 1);

    printf("=== Poisson Surface Reconstruction ===\n");
    printf("  input       : %s\n", inputPath);
    printf("  output      : %s\n", outputPath);
    printf("  maxDepth    : %d\n", maxDepth);
    printf("  densityDepth: %d\n", densityDepth);
    printf("  extractionDepth: %d\n", extractionDepth);
    printf("======================================\n\n");

    // 1. Load point cloud
    std::vector<glm::vec3> positions, normals;
    bool res = loadPointCloud(inputPath, positions, normals);
    if (!res)
    {
        printf("Failed to load point cloud: %s\n", inputPath);
        return -1;
    }
    printf("Loaded %zu points from %s\n", positions.size(), inputPath);

    // 2. Bounding box
    glm::vec3 bbMin = positions[0], bbMax = positions[0];
    for (const auto &p : positions)
    {
        bbMin = glm::min(bbMin, p);
        bbMax = glm::max(bbMax, p);
    }
    printf("BBox: [%.3f,%.3f,%.3f] ~ [%.3f,%.3f,%.3f]\n",
           bbMin.x, bbMin.y, bbMin.z, bbMax.x, bbMax.y, bbMax.z);

    // 3. Build Octree
    Octree octree(maxDepth, densityDepth, 1);
    octree.build(positions, normals, bbMin, bbMax);
    octree.prepareEvaluationTree(positions);
    octree.printStats();

    // 4. Density field + splat
    octree.computeDensityField(positions);
    octree.splat(positions, normals);

    // 5. Poisson solve
    printf("\n=== Poisson Solve ===\n");
    octree.poissonSolve(2000, 1e-6f);

    // 디버그: scalarValue 통계
    {
        auto allNodes = octree.getAllNodes();
        float sMin = 1e30f, sMax = -1e30f;
        int nonZero = 0;
        for (auto *n : allNodes) {
            if (n->scalarValue != 0.0f) nonZero++;
            sMin = std::min(sMin, n->scalarValue);
            sMax = std::max(sMax, n->scalarValue);
        }
        printf("[Debug] scalarValue range: [%e, %e]  nonZero: %d / %zu\n",
               sMin, sMax, nonZero, allNodes.size());
    }

    // 6. Marching Cubes
    printf("\n=== Marching Cubes ===\n");
    MCMesh mesh = MarchingCubes::extract(&octree, positions, extractionDepth);

    // 7. Export OBJ
    if (mesh.positions.empty())
    {
        printf("No mesh generated!\n");
        return -1;
    }
    MarchingCubes::exportOBJ(mesh, outputPath);

    printf("\nDone. Output: %s\n", outputPath);
    return 0;
}
