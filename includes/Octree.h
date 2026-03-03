#ifndef OCTREE_H
#define OCTREE_H

#include "OctreeNode.h"
#include <vector>
#include <glm/glm.hpp>

// ============================================================================
// Octree
//
// Adaptive Octree for Poisson Surface Reconstruction.
//
// 사용 흐름:
//   Octree tree(maxDepth, minPointsToSplit);
//   tree.build(positions, normals);
//   tree.printStats();
//
// 이후 PSR 단계:
//   splat(positions, normals)        → vectorCoeff/splatWeight 채움
//   computeDivergence()              → RHS 구성 (추후)
//   poissonSolve()                   → scalarValue 채움 (추후)
//   marchingCubes()                  → mesh 추출 (추후)
// ============================================================================
class Octree
{
public:
    OctreeNode* root;

    int maxDepth;          // 최대 허용 깊이 (보통 6~10)
    int minPointsToSplit;  // 이 수 초과 시 분할 (보통 1)

    // 통계
    int totalNodeCount;
    int leafNodeCount;

    // ─────────────────────────────────────────────────────────────────────
    Octree(int maxDepth = 8, int minPointsToSplit = 1);
    ~Octree();

    // ── 트리 구축 ─────────────────────────────────────────────────────────
    // 포인트 리스트를 받아 경계 박스 계산 후 adaptive하게 분할
    void build(const std::vector<glm::vec3>& positions,
               const std::vector<glm::vec3>& normals);

    // ── 조회 ──────────────────────────────────────────────────────────────
    // 위치 p를 포함하는 가장 깊은 leaf 노드 반환
    OctreeNode* findLeaf(const glm::vec3& p) const;

    // node와 같은 depth 기준으로 3×3×3 이웃 leaf 노드 반환
    // (B-spline support가 겹치는 노드들 - splatting에 사용)
    std::vector<OctreeNode*> getLeafNeighborhood(OctreeNode* node) const;

    // 모든 leaf 노드 반환
    std::vector<OctreeNode*> getAllLeaves() const;

    // 모든 노드 반환 (내부 + leaf)
    std::vector<OctreeNode*> getAllNodes() const;

    // ── PSR Step 2: Splatting ─────────────────────────────────────────────
    // 각 포인트의 법선을 주변 leaf 노드들에 B-spline 가중치로 분배
    // 결과: 각 노드의 vectorCoeff 에 Σ w_i * N_i, splatWeight 에 Σ w_i
    void splat(const std::vector<glm::vec3>& positions,
               const std::vector<glm::vec3>& normals);

    // ── 출력 ──────────────────────────────────────────────────────────────
    void printStats() const;

private:
    // 노드를 8개 자식으로 분할 (자식만 생성, 포인트 재배분은 외부에서)
    void subdivide(OctreeNode* node);

    // 포인트를 내려보낼 자식 인덱스 계산 (bit-flag 방식)
    static int childIndexOf(const glm::vec3& nodeCenter, const glm::vec3& p);

    // 재귀 수집
    void collectLeaves(OctreeNode* node, std::vector<OctreeNode*>& out) const;
    void collectAll(OctreeNode* node,    std::vector<OctreeNode*>& out) const;

    // 특정 월드 좌표를 포함하는 leaf 탐색
    OctreeNode* findLeafImpl(OctreeNode* node, const glm::vec3& p) const;

    // 메모리 해제 (재귀)
    void deleteSubtree(OctreeNode* node);
};

#endif // OCTREE_H
