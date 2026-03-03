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
    // vectorCoeff : Σ w_i * N_i  (법선 가중합, 정규화 전)
    // splatWeight : Σ w_i        (가중치 합, 정규화에 사용)
    // scalarValue : χ(o)         (Poisson 풀이 결과)
    glm::vec3 vectorCoeff;
    float     splatWeight;
    float     scalarValue;

    // ─────────────────────────────────────────────────────────────────────
    OctreeNode(glm::vec3 center_, float halfSize_, int depth_,
               OctreeNode* parent_, int childIndex_)
        : center(center_), halfSize(halfSize_), depth(depth_),
          childIndex(childIndex_), parent(parent_),
          vectorCoeff(0.0f), splatWeight(0.0f), scalarValue(0.0f)
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
    // Linear (tent) B-spline: B(t) = max(0, 1 - |t|)
    // Support: t ∈ [-1, 1]
    // 노드에서 t = (p_axis - center_axis) / halfSize 로 정규화
    static float BSpline1D(float t)
    {
        t = std::fabsf(t);
        return (t < 1.0f) ? (1.0f - t) : 0.0f;
    }

    // 3D tensor-product basis: F_o(p) = B(tx) * B(ty) * B(tz)
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