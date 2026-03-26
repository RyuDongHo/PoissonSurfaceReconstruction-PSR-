#ifndef MARCHINGCUBES_H
#define MARCHINGCUBES_H

#include <glm/glm.hpp>
#include <vector>
#include <string>

class Octree;

// ============================================================================
// MCMesh  -- Marching Cubes 출력 메시
//   positions[3k], positions[3k+1], positions[3k+2]  : k번째 삼각형 꼭짓점
//   normals[i]                                        : positions[i]의 법선
// ============================================================================
struct MCMesh
{
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    // Triangle indices (0-based). Every consecutive 3 entries form one face.
    std::vector<unsigned int> indices;
};

// ============================================================================
// MarchingCubes
//
// 논문 방식:
//   - 각 leaf 노드 = MC 큐브  (8 모서리에서 χ 평가)
//   - σ = (1/|S|) Σ_s χ(s.p)  를 isovalue 로 사용
//   - non-conforming edge: 한 edge 에 인접한 가장 fine 한 노드 쌍이
//     zero-crossing 위치를 결정 (T-junction 크랙 방지)
//   - 법선 = ∇χ 의 반대 방향  (내향 법선 → 외향 법선으로 flip)
//
// 호출 순서:
//   octree.poissonSolve();
//   MCMesh mesh = MarchingCubes::extract(octree, samplePositions);
// ============================================================================
class MarchingCubes
{
public:
    // poissonSolve() 이후 호출. samplePositions 는 isovalue 계산에 사용.
    static MCMesh   extract         (const Octree *octree,
                                     const std::vector<glm::vec3> &samplePositions,
                                     int extractionDepth = -1);

    // χ(p) = Σ_o  scalarValue_o · F_o(p)
    static float    evaluateChi     (const Octree *octree, const glm::vec3 &p);

    // ∇χ(p)  (법선 계산에 사용)
    static glm::vec3 gradChi        (const Octree *octree, const glm::vec3 &p);

    // σ = (1/|S|) Σ_s χ(s.p)
    static float    computeIsovalue (const Octree *octree,
                                     const std::vector<glm::vec3> &samples);

    // OBJ 파일로 내보내기 (positions + normals → v/vn/f)
    static bool     exportOBJ       (const MCMesh &mesh, const std::string &path);
};

#endif // MARCHINGCUBES_H
