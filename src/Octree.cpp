#include "Octree.h"
#include "PoissonSolver.h"
#include <queue>
#include <cmath>
#include <cassert>
#include <cstdio>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <utility>

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
      leafNodeCount(0),
      regularGridResolution(0),
      regularGridMin(0.0f),
      regularGridStep(0.0f)
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
    // Add a generous padding so reconstructed isosurface does not get clipped
    // at the root boundary (which creates open boundaries in the final mesh).
    float extent = std::max({bbExtent.x, bbExtent.y, bbExtent.z}) * 0.5f * 1.10f;

    printf("[Octree] BBox center: (%.3f, %.3f, %.3f)  halfSize: %.3f\n",
           bbCenter.x, bbCenter.y, bbCenter.z, extent);
    printf("[Octree] Points: %zu  maxDepth: %d  minSplit: %d\n",
           positions.size(), maxDepth, minPointsToSplit);

    // ── 2. 루트 생성 ─────────────────────────────────────────────────────
    deleteSubtree(root);
    root = new OctreeNode(bbCenter, extent, 0, nullptr, -1);
    totalNodeCount = 1;
    leafNodeCount = 0;
    clearRegularGridField();

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

        // Paper-style discretization: every sample should lie in a depth-D leaf.
        bool shouldSplit = (!node->pointIndices.empty()) && (node->depth < maxDepth);

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

    // Validate point-to-node correspondence after build.
    // Each input point index should appear in exactly one leaf node.
    {
        std::vector<OctreeNode *> leaves = getAllLeaves();
        std::vector<int> freq(positions.size(), 0);

        int badIndexRefs = 0;
        int containMismatches = 0;
        int maxLeafDepth = 0;
        int leafAtMaxDepth = 0;
        int leafShallower = 0;
        int pointsAtMaxDepthLeaves = 0;
        int pointsAtShallowerLeaves = 0;

        for (OctreeNode *lf : leaves)
        {
            if (!lf) continue;
            maxLeafDepth = std::max(maxLeafDepth, lf->depth);
            if (lf->depth == maxDepth) leafAtMaxDepth++;
            else if (lf->depth < maxDepth) leafShallower++;

            for (int pi : lf->pointIndices)
            {
                if (pi < 0 || pi >= (int)positions.size())
                {
                    badIndexRefs++;
                    continue;
                }
                freq[(size_t)pi]++;
                if (!lf->contains(positions[(size_t)pi]))
                    containMismatches++;
                if (lf->depth == maxDepth) pointsAtMaxDepthLeaves++;
                else if (lf->depth < maxDepth) pointsAtShallowerLeaves++;
            }
        }

        int missing = 0;
        int duplicated = 0;
        for (int c : freq)
        {
            if (c == 0) missing++;
            else if (c > 1) duplicated++;
        }

        printf("[Octree][Validate] points=%zu missing=%d duplicated=%d badRefs=%d containMismatch=%d\n",
               positions.size(), missing, duplicated, badIndexRefs, containMismatches);
        printf("[Octree][Validate] leafDepth max=%d  leaves@maxDepth=%d  leaves<maxDepth=%d\n",
               maxLeafDepth, leafAtMaxDepth, leafShallower);
        printf("[Octree][Validate] pointMembership: @maxDepthLeaves=%d  @shallowerLeaves=%d\n",
               pointsAtMaxDepthLeaves, pointsAtShallowerLeaves);
    }
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
    if (!root) return nullptr;
    OctreeNode *node = root;
    while (node->depth < targetDepth && !node->isLeaf())
    {
        int ci = childIndexOf(node->center, targetCenter);
        node = node->children[ci];
    }
    return node;
}

OctreeNode *Octree::findNodeAtDepth(const glm::vec3 &targetCenter, int targetDepth) const
{
    OctreeNode *node = findNodeNearestDepth(targetCenter, targetDepth);
    return (node && node->depth == targetDepth) ? node : nullptr;
}

OctreeNode *Octree::ensureNodeAtDepth(const glm::vec3 &targetCenter, int targetDepth)
{
    if (!root) return nullptr;
    targetDepth = std::max(0, std::min(targetDepth, maxDepth));

    OctreeNode *node = root;
    while (node->depth < targetDepth)
    {
        if (node->isLeaf())
        {
            subdivide(node);
            // splitting one leaf into 8 leaves: net +7 leaves
            leafNodeCount += 7;
        }
        int ci = childIndexOf(node->center, targetCenter);
        node = node->children[ci];
    }
    return node;
}

void Octree::prepareEvaluationTree(const std::vector<glm::vec3> &positions)
{
    if (!root || positions.empty())
        return;

    auto ensureTrilinear8AtDepth = [&](const glm::vec3 &pos, int depth)
    {
        float h = root->halfSize / (float)(1 << depth);
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

        for (int dz = 0; dz < 2; ++dz)
        for (int dy = 0; dy < 2; ++dy)
        for (int dx = 0; dx < 2; ++dx)
        {
            glm::vec3 nc = origin + glm::vec3((ix + dx) * step, (iy + dy) * step, (iz + dz) * step);
            ensureNodeAtDepth(nc, depth);
        }
    };

    auto ensureNeighbors27AtDepth = [&](const glm::vec3 &pos, int depth)
    {
        float h = root->halfSize / (float)(1 << depth);
        float step = h * 2.0f;
        glm::vec3 origin = root->center - glm::vec3(root->halfSize) + glm::vec3(h);

        float fx = (pos.x - origin.x) / step;
        float fy = (pos.y - origin.y) / step;
        float fz = (pos.z - origin.z) / step;

        int ix_lo = (int)std::floor(fx - 0.5f);
        int iy_lo = (int)std::floor(fy - 0.5f);
        int iz_lo = (int)std::floor(fz - 0.5f);

        int maxIdx = (1 << depth) - 1;
        ix_lo = std::max(0, std::min(ix_lo, maxIdx - 2));
        iy_lo = std::max(0, std::min(iy_lo, maxIdx - 2));
        iz_lo = std::max(0, std::min(iz_lo, maxIdx - 2));

        for (int dz = 0; dz < 3; ++dz)
        for (int dy = 0; dy < 3; ++dy)
        for (int dx = 0; dx < 3; ++dx)
        {
            glm::vec3 nc = origin + glm::vec3(
                (float)(ix_lo + dx) * step,
                (float)(iy_lo + dy) * step,
                (float)(iz_lo + dz) * step);
            ensureNodeAtDepth(nc, depth);
        }
    };

    int beforeNodes = totalNodeCount;
    int beforeLeaves = leafNodeCount;

    for (const glm::vec3 &p : positions)
    {
        ensureTrilinear8AtDepth(p, densityDepth);
        ensureNeighbors27AtDepth(p, densityDepth);
        for (int depth = 1; depth <= maxDepth; ++depth)
            ensureTrilinear8AtDepth(p, depth);
    }

    printf("[Octree] Evaluation tree fixed. nodes: %d -> %d  leaves: %d -> %d\n",
           beforeNodes, totalNodeCount, beforeLeaves, leafNodeCount);
}

// ============================================================================
// getTrilinear8  —  α_{o,s} 계산
//   논문: "trilinear interpolation weights to the eight depth-D nodes closest"
//   → linear B-spline (tent), 8 이웃, 합=1
// ============================================================================
void Octree::getTrilinear8(const glm::vec3 &pos, int depth,
                            OctreeNode *outNodes[8], float outWeights[8]) const
{
    if (depth <= 0)
    {
        for (int i = 0; i < 8; ++i)
        {
            outNodes[i] = root;
            outWeights[i] = (i == 0) ? 1.0f : 0.0f;
        }
        return;
    }

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
        outNodes[i] = findNodeAtDepth(nc, depth);
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
    float invStep3 = 1.0f / (step * step * step);

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
        outWeights[i] = wx[dx] * wy[dy] * wz[dz] * invStep3;

        glm::vec3 nodeCenter = origin + glm::vec3(
            (float)(ix_lo + dx) * step,
            (float)(iy_lo + dy) * step,
            (float)(iz_lo + dz) * step);
        outNodes[i] = findNodeAtDepth(nodeCenter, depth);
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
    int totalRefs = 0;
    int fallbackRefs = 0;
    int collapsedSamples = 0;
    int fallbackSamples = 0;

    // Accumulate c_o = Σ_s α_{o,s} at depth D̂
    // α_{o,s}: trilinear interpolation weights (8-neighbor)
    // Per paper: "the eight depth-D nodes closest to s.p"
    for (const auto &p : positions)
    {
        getTrilinear8(p, densityDepth, nbrs8, wgts8);
        int uniq = 0;
        bool hasFallback = false;
        for (int k = 0; k < 8; k++)
        {
            if (!nbrs8[k]) continue;
            totalRefs++;
            if (nbrs8[k]->depth < densityDepth)
            {
                fallbackRefs++;
                hasFallback = true;
            }
            bool seen = false;
            for (int u = 0; u < k; ++u)
            {
                if (nbrs8[u] == nbrs8[k]) { seen = true; break; }
            }
            if (!seen) uniq++;
        }
        if (uniq < 8) collapsedSamples++;
        if (hasFallback) fallbackSamples++;

        for (int k = 0; k < 8; k++)
            if (nbrs8[k]) nbrs8[k]->densityCoeff += wgts8[k];
    }

    auto allNodesNow = getAllNodes();
    printf("[Octree] DensityField done (D_hat=%d). nodes with c_o>0: ", densityDepth);
    int cnt = 0;
    for (auto *n : allNodesNow) if (n->densityCoeff > 0.0f) cnt++;
    printf("%d / %d\n", cnt, (int)allNodesNow.size());
    printf("[Octree] DensityField trilinear8: refs=%d fallbackRefs=%d (%.2f%%) collapsedSamples=%d/%zu fallbackSamples=%d/%zu\n",
           totalRefs, fallbackRefs, totalRefs > 0 ? 100.0 * (double)fallbackRefs / (double)totalRefs : 0.0,
           collapsedSamples, positions.size(), fallbackSamples, positions.size());
}

// ============================================================================
// evaluateW  —  논문 W_{D̂}(q) 평가
//
// 논문 수식:
//   W_{D̂}(q) = Σ_{s∈S} Σ_{o∈Ngbr_{D̂}(s)} α_{o,s} · F_o(q)
//
// 구현:
//   computeDensityField()에서 c_o = Σ_s α_{o,s}를 미리 쌓았으므로
//   여기서는 q 주변 27개 노드만 순회해
//   W(q) = Σ_o c_o F_o(q) 를 평가한다.
// ============================================================================
float Octree::evaluateW(const glm::vec3 &q) const
{
    OctreeNode *nbrs27_q[27];
    float       wgts27_q[27];
    
    // q의 27개 이웃 노드와 F_o(q) 계산
    getNeighbors27(q, densityDepth, nbrs27_q, wgts27_q);

    float W = 0.0f;

    // W(q) = Σ_o c_o F_o(q)
    for (int i = 0; i < 27; i++)
    {
        if (!nbrs27_q[i] || wgts27_q[i] <= 0.0f) continue;
        W += nbrs27_q[i]->densityCoeff * wgts27_q[i];
    }

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
    int totalRefs = 0;
    int fallbackRefs = 0;
    int collapsedSamples = 0;
    int fallbackSamples = 0;

    for (int i = 0; i < N; i++)
    {
        float W_s = evaluateW(positions[i]);
        if (W_s < 1e-8f) continue;

        // Paper Eq. (Section 4.5):
        //   Depth(s.p) = min(D, D + log4(W_Dhat(s.p) / W))
        // Use adaptive basis width so sparse regions contribute with wider
        // kernels and dense regions retain higher-frequency detail.
        float logRatio = std::log2(W_s / wAvg) * 0.5f;
        int sampleDepth = std::min(maxDepth,
                          maxDepth + (int)std::floor(logRatio));
        sampleDepth = std::max(0, sampleDepth);

        // α_{o,s}: trilinear (8-neighbor, 2x2x2 grid)
        getTrilinear8(positions[i], sampleDepth, nbrs8, wgts8);
        int uniq = 0;
        bool hasFallback = false;
        for (int k = 0; k < 8; ++k)
        {
            if (!nbrs8[k]) continue;
            totalRefs++;
            if (nbrs8[k]->depth < sampleDepth)
            {
                fallbackRefs++;
                hasFallback = true;
            }
            bool seen = false;
            for (int u = 0; u < k; ++u)
            {
                if (nbrs8[u] == nbrs8[k]) { seen = true; break; }
            }
            if (!seen) uniq++;
        }
        if (uniq < 8) collapsedSamples++;
        if (hasFallback) fallbackSamples++;

        bool contributed = false;
        for (int k = 0; k < 8; k++)
        {
            if (!nbrs8[k] || wgts8[k] <= 0.0f) continue;
            float scale = wgts8[k] / W_s;
            // The paper assumes inward-facing sample normals.
            // Callers must provide normals in that convention.
            nbrs8[k]->vectorCoeff += scale * normals[i];
            nbrs8[k]->splatWeight  += wgts8[k];
            contributed = true;
        }
        if (contributed) splatted++;
    }

    auto allNodesNow = getAllNodes();
    int filled = 0;
    for (auto *n : allNodesNow)
        if (n->splatWeight > 1e-8f) filled++;

    // Diffuse the finest-level vector field locally to bridge sparse holes.
    // This keeps the solve basis fixed at maxDepth while approximating the
    // wider kernels that PSR uses in under-sampled regions.
    {
        struct GridKey
        {
            int x, y, z;
            bool operator==(const GridKey &o) const
            {
                return x == o.x && y == o.y && z == o.z;
            }
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

        std::vector<OctreeNode *> finestNodes;
        finestNodes.reserve(allNodesNow.size());
        std::unordered_map<GridKey, OctreeNode *, GridKeyHash> finestMap;
        finestMap.reserve(allNodesNow.size());

        const float step = (2.0f * root->halfSize) / (float)(1 << maxDepth);
        const glm::vec3 domainMin = root->center - glm::vec3(root->halfSize);

        for (OctreeNode *n : allNodesNow)
        {
            if (!n || n->depth != maxDepth)
                continue;
            finestNodes.push_back(n);
            int ix = (int)std::lround((n->center.x - domainMin.x) / step - 0.5f);
            int iy = (int)std::lround((n->center.y - domainMin.y) / step - 0.5f);
            int iz = (int)std::lround((n->center.z - domainMin.z) / step - 0.5f);
            finestMap[GridKey{ix, iy, iz}] = n;
        }

        const int smoothPasses = 0;
        for (int pass = 0; pass < smoothPasses; ++pass)
        {
            std::vector<glm::vec3> nextCoeff(finestNodes.size(), glm::vec3(0.0f));
            std::vector<float> nextWeight(finestNodes.size(), 0.0f);

            for (size_t idx = 0; idx < finestNodes.size(); ++idx)
            {
                OctreeNode *n = finestNodes[idx];
                int ix = (int)std::lround((n->center.x - domainMin.x) / step - 0.5f);
                int iy = (int)std::lround((n->center.y - domainMin.y) / step - 0.5f);
                int iz = (int)std::lround((n->center.z - domainMin.z) / step - 0.5f);

                glm::vec3 sumVec(0.0f);
                float sumW = 0.0f;
                float nbrWeightSum = 0.0f;
                int nbrCount = 0;

                for (int dz = -1; dz <= 1; ++dz)
                for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx)
                {
                    auto it = finestMap.find(GridKey{ix + dx, iy + dy, iz + dz});
                    if (it == finestMap.end())
                        continue;

                    OctreeNode *nbr = it->second;
                    float spatialW = (dx == 0 && dy == 0 && dz == 0) ? 4.0f : 1.0f;
                    float dataW = std::max(nbr->splatWeight, 1e-6f);
                    float w = spatialW * dataW;
                    sumVec += nbr->vectorCoeff * w;
                    sumW += w;
                    if (!(dx == 0 && dy == 0 && dz == 0))
                    {
                        nbrWeightSum += nbr->splatWeight;
                        nbrCount++;
                    }
                }

                if (sumW <= 0.0f)
                {
                    nextCoeff[idx] = n->vectorCoeff;
                    nextWeight[idx] = n->splatWeight;
                    continue;
                }

                glm::vec3 avg = sumVec / sumW;
                float nbrAvgWeight = (nbrCount > 0) ? (nbrWeightSum / (float)nbrCount) : 0.0f;
                float blend = 0.0f;
                if (n->splatWeight <= 1e-8f)
                    blend = 1.0f;
                else if (nbrAvgWeight > 0.0f)
                    blend = glm::clamp(1.0f - (n->splatWeight / (nbrAvgWeight + 1e-6f)), 0.0f, 0.5f);

                nextCoeff[idx] = n->vectorCoeff * (1.0f - blend) + avg * blend;
                nextWeight[idx] = std::max(n->splatWeight, nbrAvgWeight * blend);
            }

            for (size_t idx = 0; idx < finestNodes.size(); ++idx)
            {
                finestNodes[idx]->vectorCoeff = nextCoeff[idx];
                finestNodes[idx]->splatWeight = nextWeight[idx];
            }
        }
    }

    printf("[Octree] Splatting done. samples: %d / %d  nodes: %d / %d\n",
           splatted, N, filled, (int)allNodesNow.size());
    printf("[Octree] Splat trilinear8@adaptiveDepth: refs=%d fallbackRefs=%d (%.2f%%) collapsedSamples=%d/%d fallbackSamples=%d/%d\n",
           totalRefs, fallbackRefs, totalRefs > 0 ? 100.0 * (double)fallbackRefs / (double)totalRefs : 0.0,
           collapsedSamples, N, fallbackSamples, N);
}

void Octree::clearRegularGridField()
{
    regularGridResolution = 0;
    regularGridMin = glm::vec3(0.0f);
    regularGridStep = 0.0f;
    regularGridChi.clear();
}

void Octree::storeRegularGridField(int resolution,
                                   const glm::vec3 &domainMin,
                                   float step,
                                   std::vector<float> values)
{
    regularGridResolution = resolution;
    regularGridMin = domainMin;
    regularGridStep = step;
    regularGridChi = std::move(values);
}

bool Octree::hasRegularGridField() const
{
    if (regularGridResolution <= 0 || regularGridStep <= 0.0f)
        return false;
    const size_t side = (size_t)regularGridResolution + 1;
    return regularGridChi.size() == side * side * side;
}

float Octree::sampleRegularGrid(const glm::vec3 &p) const
{
    if (!hasRegularGridField())
        return 0.0f;

    const int res = regularGridResolution;
    const auto idx = [res](int x, int y, int z) -> size_t
    {
        return (size_t)x + ((size_t)res + 1) * ((size_t)y + ((size_t)res + 1) * (size_t)z);
    };
    const auto clampCoord = [res](int v) { return std::max(0, std::min(res, v)); };

    const glm::vec3 g = (p - regularGridMin) / regularGridStep;
    int ix = (int)std::floor(g.x);
    int iy = (int)std::floor(g.y);
    int iz = (int)std::floor(g.z);
    float fx = g.x - (float)ix;
    float fy = g.y - (float)iy;
    float fz = g.z - (float)iz;

    ix = std::max(0, std::min(ix, res - 1));
    iy = std::max(0, std::min(iy, res - 1));
    iz = std::max(0, std::min(iz, res - 1));
    fx = std::max(0.0f, std::min(1.0f, fx));
    fy = std::max(0.0f, std::min(1.0f, fy));
    fz = std::max(0.0f, std::min(1.0f, fz));

    float out = 0.0f;
    for (int dz = 0; dz < 2; ++dz)
    for (int dy = 0; dy < 2; ++dy)
    for (int dx = 0; dx < 2; ++dx)
    {
        const float wx = dx ? fx : (1.0f - fx);
        const float wy = dy ? fy : (1.0f - fy);
        const float wz = dz ? fz : (1.0f - fz);
        const int sx = clampCoord(ix + dx);
        const int sy = clampCoord(iy + dy);
        const int sz = clampCoord(iz + dz);
        out += wx * wy * wz * regularGridChi[idx(sx, sy, sz)];
    }
    return out;
}

glm::vec3 Octree::gradientRegularGrid(const glm::vec3 &p) const
{
    if (!hasRegularGridField())
        return glm::vec3(0.0f);

    const float h = regularGridStep;
    const glm::vec3 ex(h, 0.0f, 0.0f);
    const glm::vec3 ey(0.0f, h, 0.0f);
    const glm::vec3 ez(0.0f, 0.0f, h);

    return glm::vec3(
        (sampleRegularGrid(p + ex) - sampleRegularGrid(p - ex)) / (2.0f * h),
        (sampleRegularGrid(p + ey) - sampleRegularGrid(p - ey)) / (2.0f * h),
        (sampleRegularGrid(p + ez) - sampleRegularGrid(p - ez)) / (2.0f * h));
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

// ============================================================================
// Octree::poissonSolve  (PoissonSolver 에 위임)
// ============================================================================
void Octree::poissonSolve(int maxIter, float tol)
{
    PoissonSolver::solve(this, maxIter, tol);
}
