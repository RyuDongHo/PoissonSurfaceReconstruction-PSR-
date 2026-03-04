#include "Octree.h"
#include <queue>
#include <cmath>
#include <cassert>
#include <cstdio>
#include <algorithm>
#include <unordered_set>

// ============================================================================
// 내부 헬퍼 매크로
// ============================================================================
// 부호 비트 → {-1, +1} 방향 오프셋
static inline float sign1(bool positive) { return positive ? 1.0f : -1.0f; }

// ============================================================================
// 생성자 / 소멸자
// ============================================================================
Octree::Octree(int maxDepth_, int densityDepth_, int minPointsToSplit_)
    : root(nullptr),
      maxDepth(maxDepth_),
      densityDepth(densityDepth_),
      minPointsToSplit(minPointsToSplit_),
      totalNodeCount(0),
      leafNodeCount(0)
{
}

Octree::~Octree()
{
    deleteSubtree(root);
}

void Octree::deleteSubtree(OctreeNode *node)
{
    if (!node)
        return;
    for (int i = 0; i < 8; i++)
        deleteSubtree(node->children[i]);
    delete node;
}

// ============================================================================
// childIndexOf
//   포인트 p가 nodeCenter를 기준으로 어느 자식 옥탄트에 속하는지 계산
//   bit0 = x > center, bit1 = y > center, bit2 = z > center
// ============================================================================
int Octree::childIndexOf(const glm::vec3 &nodeCenter, const glm::vec3 &p)
{
    int idx = 0;
    if (p.x > nodeCenter.x)
        idx |= 1;
    if (p.y > nodeCenter.y)
        idx |= 2;
    if (p.z > nodeCenter.z)
        idx |= 4;
    return idx;
}

// ============================================================================
// subdivide
//   node를 8개 자식으로 분할 (포인트 재배분은 build() 에서 담당)
// ============================================================================
void Octree::subdivide(OctreeNode *node)
{
    assert(node->isLeaf() && "이미 자식이 있는 노드를 다시 분할할 수 없습니다");

    float ch = node->halfSize * 0.5f; // child half-size
    int cd = node->depth + 1;

    for (int i = 0; i < 8; i++)
    {
        // 자식 중심: 각 비트에 따라 ± 방향
        glm::vec3 offset(
            sign1(i & 1) * ch,
            sign1(i & 2) * ch,
            sign1(i & 4) * ch);
        node->children[i] = new OctreeNode(
            node->center + offset, ch, cd, node, i);
        totalNodeCount++;
    }
}

// ============================================================================
// build
//   1) AABB 계산 → 정육면체 루트 생성
//   2) 모든 포인트를 루트에 넣고 BFS로 분할
// ============================================================================
void Octree::build(const std::vector<glm::vec3> &positions,
                   const std::vector<glm::vec3> &normals, 
                   glm::vec3 bbMin, glm::vec3 bbMax)
{
    if (positions.empty())
        return;

    // 정육면체 + 약간의 여백 (경계 포인트가 밖으로 나가지 않도록)
    glm::vec3 bbCenter = (bbMin + bbMax) * 0.5f;
    glm::vec3 bbExtent = bbMax - bbMin;
    float extent = std::max({bbExtent.x, bbExtent.y, bbExtent.z}) * 0.5f * 1.001f;

    printf("[Octree] BBox center: (%.3f, %.3f, %.3f)  halfSize: %.3f\n",
           bbCenter.x, bbCenter.y, bbCenter.z, extent);
    printf("[Octree] Points: %zu  maxDepth: %d  minSplit: %d\n",
           positions.size(), maxDepth, minPointsToSplit);

    // ── 2. 루트 생성 ─────────────────────────────────────────────────────
    deleteSubtree(root);
    root = new OctreeNode(bbCenter, extent, 0, nullptr, -1);
    totalNodeCount = 1;
    leafNodeCount = 0;

    // 모든 포인트를 루트에 할당
    root->pointIndices.resize(positions.size());
    for (int i = 0; i < (int)positions.size(); i++)
        root->pointIndices[i] = i;

    // ── 3. BFS로 adaptive 분할 ───────────────────────────────────────────
    std::queue<OctreeNode *> q;
    q.push(root);

    while (!q.empty())
    {
        OctreeNode *node = q.front();
        q.pop();

        bool shouldSplit = ((int)node->pointIndices.size() > minPointsToSplit) && (node->depth < maxDepth);

        if (!shouldSplit)
        {
            // Leaf 확정
            leafNodeCount++;
            continue;
        }

        // ── 분할 ─────────────────────────────────────────────────────────
        subdivide(node);

        // 포인트를 8개 자식에 배분
        for (int idx : node->pointIndices)
        {
            int ci = childIndexOf(node->center, positions[idx]);
            node->children[ci]->pointIndices.push_back(idx);
        }
        node->pointIndices.clear(); // 내부 노드는 포인트 직접 보관 안 함
        node->pointIndices.shrink_to_fit();

        for (int i = 0; i < 8; i++)
        {
            if (!node->children[i]->pointIndices.empty())
                q.push(node->children[i]);
            else
                leafNodeCount++; // 빈 leaf도 leaf로 카운트
        }
    }

    printf("[Octree] Build done.  totalNodes: %d  leaves: %d\n",
           totalNodeCount, leafNodeCount);
}

// ============================================================================
// findLeafImpl / findLeaf
//   루트부터 재귀적으로 p를 포함하는 leaf 탐색
// ============================================================================
OctreeNode *Octree::findLeafImpl(OctreeNode *node, const glm::vec3 &p) const
{
    if (!node || !node->contains(p))
        return nullptr;
    if (node->isLeaf())
        return node;

    int ci = childIndexOf(node->center, p);
    OctreeNode *res = findLeafImpl(node->children[ci], p);
    if (res)
        return res;

    // 안전망: 경계 근처 부동소수 오차 대비 나머지 자식도 확인
    for (int i = 0; i < 8; i++)
    {
        if (i == ci)
            continue;
        res = findLeafImpl(node->children[i], p);
        if (res)
            return res;
    }
    return nullptr;
}

OctreeNode *Octree::findLeaf(const glm::vec3 &p) const
{
    return findLeafImpl(root, p);
}

// ============================================================================
// findNodeNearestDepth
//   루트에서 targetDepth까지 targetCenter 방향으로 내려가다가
//   leaf에 도달하거나 targetDepth에 도달하면 해당 노드 반환.
//   (adaptive tree에서 빈 영역은 더 얕은 노드로 표현됨)
// ============================================================================
OctreeNode *Octree::findNodeNearestDepth(const glm::vec3 &targetCenter, int targetDepth) const
{
    OctreeNode *node = root;
    while (node->depth < targetDepth && !node->isLeaf())
    {
        int ci = childIndexOf(node->center, targetCenter);
        node = node->children[ci];
    }
    return node;
}

// ============================================================================
// getTrilinear8  —  α_{o,s} 계산
//   논문: "trilinear interpolation weights to the eight depth-D nodes closest"
//   → linear B-spline (tent), 8 이웃, 합=1
// ============================================================================
void Octree::getTrilinear8(const glm::vec3 &pos, int depth,
                            OctreeNode *outNodes[8], float outWeights[8]) const
{
    float h    = root->halfSize / (float)(1 << depth);
    float step = h * 2.0f;
    glm::vec3 origin = root->center - glm::vec3(root->halfSize) + glm::vec3(h);

    float fx = (pos.x - origin.x) / step;
    float fy = (pos.y - origin.y) / step;
    float fz = (pos.z - origin.z) / step;

    int ix = (int)std::floor(fx);
    int iy = (int)std::floor(fy);
    int iz = (int)std::floor(fz);

    int maxIdx = (1 << depth) - 1;
    ix = std::max(0, std::min(ix, maxIdx - 1));
    iy = std::max(0, std::min(iy, maxIdx - 1));
    iz = std::max(0, std::min(iz, maxIdx - 1));

    float lx = std::max(0.0f, std::min(1.0f, fx - ix));
    float ly = std::max(0.0f, std::min(1.0f, fy - iy));
    float lz = std::max(0.0f, std::min(1.0f, fz - iz));

    for (int dz = 0; dz < 2; dz++)
    for (int dy = 0; dy < 2; dy++)
    for (int dx = 0; dx < 2; dx++)
    {
        int i = dx | (dy << 1) | (dz << 2);
        // linear B-spline 가중치 = α_{o,s}
        outWeights[i] = (dx ? lx : 1.0f-lx) * (dy ? ly : 1.0f-ly) * (dz ? lz : 1.0f-lz);
        glm::vec3 nc = origin + glm::vec3((ix+dx)*step, (iy+dy)*step, (iz+dz)*step);
        outNodes[i] = findNodeNearestDepth(nc, depth);
    }
}

// ============================================================================
// getNeighbors27  — F_o(q) 평가용
//   quadratic B-spline (n=3), support [-1.5,1.5]^3, 27 이웃
// ============================================================================
void Octree::getNeighbors27(const glm::vec3 &pos, int depth,
                             OctreeNode *outNodes[27], float outWeights[27]) const
{
    float h    = root->halfSize / (float)(1 << depth);
    float step = h * 2.0f;

    glm::vec3 origin = root->center - glm::vec3(root->halfSize) + glm::vec3(h);

    float fx = (pos.x - origin.x) / step;
    float fy = (pos.y - origin.y) / step;
    float fz = (pos.z - origin.z) / step;

    // quadratic B-spline stencil 하한 인덱스: floor(f - 0.5)
    // → f 주변 [j_lo, j_lo+1, j_lo+2] 세 노드가 지지(support) 내에 있음
    int ix_lo = (int)std::floor(fx - 0.5f);
    int iy_lo = (int)std::floor(fy - 0.5f);
    int iz_lo = (int)std::floor(fz - 0.5f);

    // 경계 클램프: j_lo+2 ≤ maxIdx 보장
    int maxIdx = (1 << depth) - 1;
    ix_lo = std::max(0, std::min(ix_lo, maxIdx - 2));
    iy_lo = std::max(0, std::min(iy_lo, maxIdx - 2));
    iz_lo = std::max(0, std::min(iz_lo, maxIdx - 2));

    // 각 축 3개 가중치: B^{*3}(fx - (ix_lo + d))
    float wx[3], wy[3], wz[3];
    for (int d = 0; d < 3; d++)
    {
        wx[d] = OctreeNode::BSpline1D(fx - (float)(ix_lo + d));
        wy[d] = OctreeNode::BSpline1D(fy - (float)(iy_lo + d));
        wz[d] = OctreeNode::BSpline1D(fz - (float)(iz_lo + d));
    }

    // 27개 조합
    for (int dz = 0; dz < 3; dz++)
    for (int dy = 0; dy < 3; dy++)
    for (int dx = 0; dx < 3; dx++)
    {
        int i = dx + dy * 3 + dz * 9;
        outWeights[i] = wx[dx] * wy[dy] * wz[dz];

        glm::vec3 nodeCenter = origin + glm::vec3(
            (float)(ix_lo + dx) * step,
            (float)(iy_lo + dy) * step,
            (float)(iz_lo + dz) * step);
        outNodes[i] = findNodeNearestDepth(nodeCenter, depth);
    }
}

// ============================================================================
// computeDensityField  —  논문 W_D̂ 사전 계산 (Phase 1)
//
// 논문 수식:
//   W_{D̂}(q) = Σ_{s∈S} Σ_{o∈Ngbr_{D̂}(s)} α_{o,s} · F_o(q)
//
// 구현 전략 (동치 변환):
//   순서를 바꾸면   = Σ_o F_o(q) · ( Σ_s α_{o,s} )
//                  = Σ_o F_o(q) · c_o
//
//   여기서 c_o = Σ_s α_{o,s}  ← 이 함수에서 미리 적립
//
//   evaluateW(q) 는 나중에 Σ_o F_o(q)·c_o 만 계산하면 됨
//   (모든 샘플을 다시 순회할 필요 없음)
//
//   α_{o,s} : sample s 에서 D̂ 깊이 격자의 이웃 노드 o 로의
//             trilinear 보간 가중치  ( Σ_o α_{o,s} = 1 )
// ============================================================================
void Octree::computeDensityField(const std::vector<glm::vec3> &positions)
{
    auto allNodes = getAllNodes();
    for (auto *n : allNodes)
        n->densityCoeff = 0.0f;

    OctreeNode *nbrs8[8];
    float       wgts8[8];

    // 모든 샘플 s 순회 → c_o = Σ_s α_{o,s}  적립 (at D̂ = densityDepth)
    // α_{o,s}: trilinear (linear B-spline), 8 이웃
    for (const auto &p : positions)
    {
        getTrilinear8(p, densityDepth, nbrs8, wgts8);
        for (int k = 0; k < 8; k++)
            if (nbrs8[k]) nbrs8[k]->densityCoeff += wgts8[k];
    }

    printf("[Octree] DensityField done (D_hat=%d). nodes with c_o>0: ", densityDepth);
    int cnt = 0;
    for (auto *n : allNodes) if (n->densityCoeff > 0.0f) cnt++;
    printf("%d / %d\n", cnt, (int)allNodes.size());
}

// ============================================================================
// evaluateW  —  논문 W_{D̂}(q) 평가 (Phase 2)
//
// 논문 수식:
//   W_{D̂}(q) = Σ_{s∈S} Σ_{o∈Ngbr_{D̂}(s)} α_{o,s} · F_o(q)
//
// 구현 (computeDensityField 에서 c_o = Σ_s α_{o,s} 적립 후):
//   = Σ_o F_o(q) · c_o
//
// F_o(q) 는 tent B-spline 기저 함수로, support 가 노드 o 의 인접 셀까지만
// 이므로 q 의 D̂ 깊이 인접 8개 노드에서만  F_o(q) ≠ 0.
//   → wgts[k] = F_{nbrs[k]}(q)  (getTrilinear8 이 이를 계산)
// ============================================================================
float Octree::evaluateW(const glm::vec3 &q) const
{
    OctreeNode *nbrs27[27];
    float       wgts27[27];
    // q 의 D̂ 깊이 quadratic B-spline 27 이웃: wgts[k] = F_{nbrs[k]}(q)
    getNeighbors27(q, densityDepth, nbrs27, wgts27);

    // W_{D̂}(q) = Σ_o F_o(q) · c_o
    float W = 0.0f;
    for (int k = 0; k < 27; k++)
        if (nbrs27[k]) W += wgts27[k] * nbrs27[k]->densityCoeff;
    return W;
}

// ============================================================================
// getAllLeaves / getAllNodes
// ============================================================================
void Octree::collectLeaves(OctreeNode *node, std::vector<OctreeNode *> &out) const
{
    if (!node)
        return;
    if (node->isLeaf())
    {
        out.push_back(node);
        return;
    }
    for (int i = 0; i < 8; i++)
        collectLeaves(node->children[i], out);
}

void Octree::collectAll(OctreeNode *node, std::vector<OctreeNode *> &out) const
{
    if (!node)
        return;
    out.push_back(node);
    for (int i = 0; i < 8; i++)
        collectAll(node->children[i], out);
}

std::vector<OctreeNode *> Octree::getAllLeaves() const
{
    std::vector<OctreeNode *> out;
    collectLeaves(root, out);
    return out;
}

std::vector<OctreeNode *> Octree::getAllNodes() const
{
    std::vector<OctreeNode *> out;
    collectAll(root, out);
    return out;
}

// ============================================================================
// splat  —  논문 V(q) basis 계수 계산 (Phase 3 + 4)
//
// 논문 수식:
//   V(q) = Σ_{s∈S} (1/W_{D̂}(s.p)) Σ_{o∈Ngbr_{Depth(s.p)}(s)} α_{o,s} F_o(q)
//
//   Depth(s.p) = min(D,  D + log₄(W_{D̂}(s.p) / W))
//     W = (1/|S|) Σ_s W_{D̂}(s.p)   ← 전체 샘플 평균 밀도
//
// 구현:
//   V(q) = Σ_o vectorCoeff_o · F_o(q)  이므로
//   vectorCoeff_o = Σ_s (α_{o,s} / W_{D̂}(s.p)) · N_s
//                  (o ∈ Ngbr_{Depth(s.p)}(s) 인 경우에만 α_{o,s} ≠ 0)
//
//   α_{o,s} : sample s 에서 Depth(s.p) 깊이 격자의 이웃 o 까지의
//             trilinear 가중치  (D̂ 가 아닌 Depth(s.p) 기준)
//
// 사전 조건: computeDensityField(positions) 가 먼저 호출되어야 함
// ============================================================================
void Octree::splat(const std::vector<glm::vec3> &positions,
                   const std::vector<glm::vec3> &normals)
{
    assert(positions.size() == normals.size());
    const int N = (int)positions.size();

    // ── 모든 노드 splatting 필드 초기화 ──────────────────────────────────
    auto allNodes = getAllNodes();
    for (auto *n : allNodes)
    {
        n->vectorCoeff = glm::vec3(0.0f);
        n->splatWeight  = 0.0f;
    }

    // ── Phase 3: 평균 밀도 W 계산 ────────────────────────────────────────
    double wSum = 0.0;
    for (int i = 0; i < N; i++)
        wSum += (double)evaluateW(positions[i]);
    float wAvg = (N > 0) ? (float)(wSum / N) : 1.0f;
    if (wAvg < 1e-8f) wAvg = 1.0f;
    printf("[Octree] W_avg = %.6f  (D=%d, D_hat=%d)\n", wAvg, maxDepth, densityDepth);

    // ── Phase 4: splatting ───────────────────────────────────────────────
    OctreeNode *nbrs8[8];
    float       wgts8[8];
    int         splatted = 0;

    for (int i = 0; i < N; i++)
    {
        float W_s = evaluateW(positions[i]);
        if (W_s < 1e-8f) continue;

        float logRatio = std::log2(W_s / wAvg) * 0.5f;
        int sampleDepth = std::min(maxDepth,
                          maxDepth + (int)std::floor(logRatio));
        sampleDepth = std::max(0, sampleDepth);

        // α_{o,s}: trilinear (linear B-spline), 8 이웃
        getTrilinear8(positions[i], sampleDepth, nbrs8, wgts8);

        bool contributed = false;
        for (int k = 0; k < 8; k++)
        {
            if (!nbrs8[k] || wgts8[k] <= 0.0f) continue;
            float scale = wgts8[k] / W_s;
            nbrs8[k]->vectorCoeff += scale * normals[i];
            nbrs8[k]->splatWeight  += wgts8[k];
            contributed = true;
        }
        if (contributed) splatted++;
    }

    int filled = 0;
    for (auto *n : allNodes) if (n->splatWeight > 1e-8f) filled++;
    printf("[Octree] Splat done. samples contributed: %d / %d  "
           "nodes filled: %d / %d\n",
           splatted, N, filled, (int)allNodes.size());
}

// ============================================================================
// printStats
// ============================================================================
void Octree::printStats() const
{
    auto leaves = getAllLeaves();
    auto all = getAllNodes();

    // 깊이별 노드 수
    int depthCount[32] = {};
    int leafDepthCount[32] = {};
    int maxD = 0;
    for (auto *n : all)
    {
        int d = n->depth;
        if (d > maxD)
            maxD = d;
        depthCount[d]++;
        if (n->isLeaf())
            leafDepthCount[d]++;
    }

    // 포인트 분포
    size_t maxPts = 0, totalPts = 0;
    for (auto *lf : leaves)
    {
        totalPts += lf->pointIndices.size();
        maxPts = std::max(maxPts, lf->pointIndices.size());
    }

    printf("\n=== Octree Stats ===\n");
    printf("  Total nodes   : %d\n", (int)all.size());
    printf("  Leaf nodes    : %d\n", (int)leaves.size());
    printf("  Max depth     : %d\n", maxD);
    printf("  Points in tree: %zu (max per leaf: %zu)\n", totalPts, maxPts);
    printf("  --- depth breakdown ---\n");
    for (int d = 0; d <= maxD; d++)
        printf("    depth %2d : %4d nodes  (%d leaves)\n",
               d, depthCount[d], leafDepthCount[d]);
    printf("====================\n\n");
}
