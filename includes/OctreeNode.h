#ifndef OCTREENODE_H
#define OCTREENODE_H

#include <glm/glm.hpp>
#include <vector>
#include <cmath>
#include <cstdlib>

// ============================================================================
// OctreeNode
//
// 노드 하나를 표현. 자식 인덱스 규칙:
//   bit0 = x > center.x
//   bit1 = y > center.y
//   bit2 = z > center.z
//   → 0:(---) 1:(+--) 2:(-+-) 3:(++-) 4:(--+) 5:(+-+) 6:(-++) 7:(+++)
// ============================================================================
class OctreeNode
{
public:
    // ── 공간 정보 ──────────────────────────────────────────────────────────
    glm::vec3 center;
    float     halfSize;   // 노드 한 변 절반 길이
    int       depth;      // 루트=0, 최대 maxDepth까지
    int       childIndex; // 부모 기준 자식 번호 (0~7), 루트는 -1

    // ── 트리 구조 ─────────────────────────────────────────────────────────
    OctreeNode* children[8];
    OctreeNode* parent;

    // ── 담긴 포인트 인덱스 (leaf 노드만 유효) ──────────────────────────────
    std::vector<int> pointIndices;

    // ── PSR Splatting 필드 ────────────────────────────────────────────────
    // densityCoeff : c_o = Σ_s α_{o,s}          (Phase1: W 계산용 밀도 계수)
    // vectorCoeff  : Σ (α_{o,s}/W_s) * N_s      (Phase4: V의 basis 계수 α_o)
    // splatWeight  : Σ α_{o,s}                   (진단용)
    // scalarValue  : χ(o)                        (Poisson 풀이 결과)
    // nodeIndex    : poissonSolve 에서 행렬 인덱스로 사용
    float     densityCoeff;
    glm::vec3 vectorCoeff;
    float     splatWeight;
    float     scalarValue;
    int       nodeIndex;

    // ─────────────────────────────────────────────────────────────────────
    OctreeNode(glm::vec3 center_, float halfSize_, int depth_,
               OctreeNode* parent_, int childIndex_)
        : center(center_), halfSize(halfSize_), depth(depth_),
          childIndex(childIndex_), parent(parent_),
          densityCoeff(0.0f), vectorCoeff(0.0f), splatWeight(0.0f), scalarValue(0.0f),
        nodeIndex(-1)
    {
        for (int i = 0; i < 8; i++) children[i] = nullptr;
    }

    // ── 유틸 ──────────────────────────────────────────────────────────────
    bool isLeaf() const
    {
        for (int i = 0; i < 8; i++)
            if (children[i] != nullptr) return false;
        return true;
    }

    // ── B-spline basis ────────────────────────────────────────────────────
    // F(x,y,z) ≡ (B(x)B(y)B(z))^{*n}  (논문 수식, n=3)
    //
    // n=3 → 1D quadratic B-spline (B*3):
    //   |t| < 0.5  :  3/4 - t²
    //   0.5 ≤ |t| < 1.5  :  (1/2)(1.5 - |t|)²
    //   |t| ≥ 1.5  :  0
    // Support: t ∈ (-1.5, 1.5)  (기존 tent의 [-1,1]에서 확장)
    // Partition of unity: Σ_j B(t-j) = 1 ✓
    static float BSpline1D(float t)
    {
        t = std::fabsf(t);
        if (t < 0.5f)  return 0.75f - t * t;
        if (t < 1.5f)  { float u = 1.5f - t; return 0.5f * u * u; }
        return 0.0f;
    }

    // 3D tensor-product basis: F_o(p) = B(tx) * B(ty) * B(tz)
    // (getNeighbors27 이 이를 격자 기준으로 계산하므로 직접 호출은 드묾)
    float F(const glm::vec3& p) const
    {
        float tx = (p.x - center.x) / halfSize;
        float ty = (p.y - center.y) / halfSize;
        float tz = (p.z - center.z) / halfSize;
        return BSpline1D(tx) * BSpline1D(ty) * BSpline1D(tz);
    }

    // 포인트가 노드 AABB 내부에 있는지 (경계 포함)
    bool contains(const glm::vec3& p) const
    {
        return (std::fabsf(p.x - center.x) <= halfSize &&
                std::fabsf(p.y - center.y) <= halfSize &&
                std::fabsf(p.z - center.z) <= halfSize);
    }
};

#endif // OCTREENODE_H