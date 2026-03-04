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
    OctreeNode *root;

    int maxDepth;         // D: 재구성 최대 깊이 (Depth(s.p) 상한)
    int densityDepth;     // D_hat: 밀도 필드 추정 깊이 (W 계산 기준, 독립 파라미터)
    int minPointsToSplit; // 이 수 초과 시 분할 (보통 1)

    // 통계
    int totalNodeCount;
    int leafNodeCount;

    // ─────────────────────────────────────────────────────────────────────
    // maxDepth    : 재구성 해상도 D
    // densityDepth: W 계산에 쓰이는 D_hat  (보통 maxDepth - 2 정도로 설정하면
    //               주변 영역에서 스무스하게 밀도 평균함)
    Octree(int maxDepth = 8, int densityDepth = 6, int minPointsToSplit = 1);
    ~Octree();

    // ── 트리 구축 ─────────────────────────────────────────────────────────
    // 포인트 리스트를 받아 경계 박스 계산 후 adaptive하게 분할
    void build(const std::vector<glm::vec3> &positions,
               const std::vector<glm::vec3> &normals,
               glm::vec3 bbMin, glm::vec3 bbMax);

    // ── 조회 ──────────────────────────────────────────────────────────────
    // 위치 p를 포함하는 가장 깊은 leaf 노드 반환
    OctreeNode *findLeaf(const glm::vec3 &p) const;

    // 모든 leaf 노드 반환
    std::vector<OctreeNode *> getAllLeaves() const;

    // 모든 노드 반환 (내부 + leaf)
    std::vector<OctreeNode *> getAllNodes() const;

    // ── PSR Step 2: splatting ───────────────────────────────────────────
    // α_{o,s} 계산: trilinear (linear B-spline), 8 이웃
    //   논문: "trilinear interpolation weights to the eight nodes"
    void getTrilinear8(const glm::vec3 &pos, int depth,
                       OctreeNode *outNodes[8], float outWeights[8]) const;

    // F_o(q) 평가: quadratic B-spline (n=3), 27 이웃
    //   논문: F supported on [-1.5,1.5]^3
    void getNeighbors27(const glm::vec3 &pos, int depth,
                        OctreeNode *outNodes[27], float outWeights[27]) const;

    // Phase 1: 모든 샘플로 각 노드의 densityCoeff(c_o) 적립
    // splat() 전에 반드시 호출
    void computeDensityField(const std::vector<glm::vec3> &positions);

    // Phase 2: 위치 q에서 W_D(q) = Σ_o α_{o,q} * c_o 평가
    float evaluateW(const glm::vec3 &q) const;

    // Phase 3~4: 법선 splatting (논문 수식 그대로)
    // computeDensityField() 이후 호출
    void splat(const std::vector<glm::vec3> &positions,
               const std::vector<glm::vec3> &normals);

    // ── PSR Step 3: Poisson 풀이 ─────────────────────────────────────────
    // splat() 이후 호출.
    // Galerkin 이산화: L x = b  (L_{ij} = ⟨∇F_i,∇F_j⟩, b_i = -⟨∇F_i,V⟩)
    // CG 풀이 후 각 노드의 scalarValue ← x_o
    void poissonSolve(int maxIter = 2000, float tol = 1e-6f);

    // ── 출력 ──────────────────────────────────────────────────────────────
    void printStats() const;

private:
    // 노드를 8개 자식으로 분할 (자식만 생성, 포인트 재배분은 외부에서)
    void subdivide(OctreeNode *node);

    // 포인트를 내려보낼 자식 인덱스 계산 (bit-flag 방식)
    static int childIndexOf(const glm::vec3 &nodeCenter, const glm::vec3 &p);

    // 재귀 수집
    void collectLeaves(OctreeNode *node, std::vector<OctreeNode *> &out) const;
    void collectAll(OctreeNode *node, std::vector<OctreeNode *> &out) const;

    // 특정 월드 좌표를 포함하는 leaf 탐색
    OctreeNode *findLeafImpl(OctreeNode *node, const glm::vec3 &p) const;

    // targetCenter 위치에서 targetDepth까지 내려간 노드 반환
    // (해당 depth에 노드 없으면 가장 깊은 조상 반환)
    OctreeNode *findNodeNearestDepth(const glm::vec3 &targetCenter, int targetDepth) const;

    // 메모리 해제 (재귀)
    void deleteSubtree(OctreeNode *node);
};

#endif // OCTREE_H
