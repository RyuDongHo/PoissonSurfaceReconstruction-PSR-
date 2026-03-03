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
Octree::Octree(int maxDepth_, int minPointsToSplit_)
    : root(nullptr),
      maxDepth(maxDepth_),
      minPointsToSplit(minPointsToSplit_),
      totalNodeCount(0),
      leafNodeCount(0)
{
}

Octree::~Octree()
{
    deleteSubtree(root);
}

void Octree::deleteSubtree(OctreeNode* node)
{
    if (!node) return;
    for (int i = 0; i < 8; i++) deleteSubtree(node->children[i]);
    delete node;
}

// ============================================================================
// childIndexOf
//   포인트 p가 nodeCenter를 기준으로 어느 자식 옥탄트에 속하는지 계산
//   bit0 = x > center, bit1 = y > center, bit2 = z > center
// ============================================================================
int Octree::childIndexOf(const glm::vec3& nodeCenter, const glm::vec3& p)
{
    int idx = 0;
    if (p.x > nodeCenter.x) idx |= 1;
    if (p.y > nodeCenter.y) idx |= 2;
    if (p.z > nodeCenter.z) idx |= 4;
    return idx;
}

// ============================================================================
// subdivide
//   node를 8개 자식으로 분할 (포인트 재배분은 build() 에서 담당)
// ============================================================================
void Octree::subdivide(OctreeNode* node)
{
    assert(node->isLeaf() && "이미 자식이 있는 노드를 다시 분할할 수 없습니다");

    float ch = node->halfSize * 0.5f;   // child half-size
    int   cd = node->depth + 1;

    for (int i = 0; i < 8; i++)
    {
        // 자식 중심: 각 비트에 따라 ± 방향
        glm::vec3 offset(
            sign1(i & 1) * ch,
            sign1(i & 2) * ch,
            sign1(i & 4) * ch
        );
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
void Octree::build(const std::vector<glm::vec3>& positions,
                   const std::vector<glm::vec3>& normals)
{
    if (positions.empty()) return;

    // ── 1. 경계 박스 ─────────────────────────────────────────────────────
    glm::vec3 bbMin = positions[0], bbMax = positions[0];
    for (const auto& p : positions)
    {
        bbMin = glm::min(bbMin, p);
        bbMax = glm::max(bbMax, p);
    }

    // 정육면체 + 약간의 여백 (경계 포인트가 밖으로 나가지 않도록)
    glm::vec3 bbCenter = (bbMin + bbMax) * 0.5f;
    glm::vec3 bbExtent = bbMax - bbMin;
    float     extent   = std::max({bbExtent.x, bbExtent.y, bbExtent.z}) * 0.5f * 1.001f;

    printf("[Octree] BBox center: (%.3f, %.3f, %.3f)  halfSize: %.3f\n",
           bbCenter.x, bbCenter.y, bbCenter.z, extent);
    printf("[Octree] Points: %zu  maxDepth: %d  minSplit: %d\n",
           positions.size(), maxDepth, minPointsToSplit);

    // ── 2. 루트 생성 ─────────────────────────────────────────────────────
    deleteSubtree(root);
    root = new OctreeNode(bbCenter, extent, 0, nullptr, -1);
    totalNodeCount = 1;
    leafNodeCount  = 0;

    // 모든 포인트를 루트에 할당
    root->pointIndices.resize(positions.size());
    for (int i = 0; i < (int)positions.size(); i++)
        root->pointIndices[i] = i;

    // ── 3. BFS로 adaptive 분할 ───────────────────────────────────────────
    std::queue<OctreeNode*> q;
    q.push(root);

    while (!q.empty())
    {
        OctreeNode* node = q.front(); q.pop();

        bool shouldSplit = ((int)node->pointIndices.size() > minPointsToSplit)
                           && (node->depth < maxDepth);

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
        node->pointIndices.clear();   // 내부 노드는 포인트 직접 보관 안 함
        node->pointIndices.shrink_to_fit();

        for (int i = 0; i < 8; i++)
        {
            if (!node->children[i]->pointIndices.empty())
                q.push(node->children[i]);
            else
                leafNodeCount++;  // 빈 leaf도 leaf로 카운트
        }
    }

    printf("[Octree] Build done.  totalNodes: %d  leaves: %d\n",
           totalNodeCount, leafNodeCount);
}

// ============================================================================
// findLeafImpl / findLeaf
//   루트부터 재귀적으로 p를 포함하는 leaf 탐색
// ============================================================================
OctreeNode* Octree::findLeafImpl(OctreeNode* node, const glm::vec3& p) const
{
    if (!node || !node->contains(p)) return nullptr;
    if (node->isLeaf()) return node;

    int ci = childIndexOf(node->center, p);
    OctreeNode* res = findLeafImpl(node->children[ci], p);
    if (res) return res;

    // 안전망: 경계 근처 부동소수 오차 대비 나머지 자식도 확인
    for (int i = 0; i < 8; i++)
    {
        if (i == ci) continue;
        res = findLeafImpl(node->children[i], p);
        if (res) return res;
    }
    return nullptr;
}

OctreeNode* Octree::findLeaf(const glm::vec3& p) const
{
    return findLeafImpl(root, p);
}

// ============================================================================
// getLeafNeighborhood
//   node와 같은 depth 수준으로 ±1 격자 오프셋에 있는 leaf 노드들을 반환.
//   Linear B-spline support = ±halfSize 이므로 인접 노드(2*halfSize 거리)만
//   겹침 가능 → 3×3×3 = 27 후보
// ============================================================================
std::vector<OctreeNode*> Octree::getLeafNeighborhood(OctreeNode* node) const
{
    std::vector<OctreeNode*> result;
    if (!node) return result;

    float step = node->halfSize * 2.0f;  // 이웃 노드까지의 중심 거리

    for (int di = -1; di <= 1; di++)
    for (int dj = -1; dj <= 1; dj++)
    for (int dk = -1; dk <= 1; dk++)
    {
        // 후보 노드의 중심 (같은 depth라면 이 위치에 노드가 있을 수도 있음)
        glm::vec3 candidatePos = node->center + glm::vec3(di, dj, dk) * step;

        // 루트 범위 밖이면 스킵
        if (!root->contains(candidatePos)) continue;

        // 해당 위치의 leaf 탐색
        OctreeNode* neighbor = findLeafImpl(root, candidatePos);

        if (neighbor)
        {
            // 중복 제거 (다른 오프셋이 같은 leaf를 가리킬 수 있음)
            bool dup = false;
            for (auto* n : result) if (n == neighbor) { dup = true; break; }
            if (!dup) result.push_back(neighbor);
        }
    }
    return result;
}

// ============================================================================
// getAllLeaves / getAllNodes
// ============================================================================
void Octree::collectLeaves(OctreeNode* node, std::vector<OctreeNode*>& out) const
{
    if (!node) return;
    if (node->isLeaf()) { out.push_back(node); return; }
    for (int i = 0; i < 8; i++) collectLeaves(node->children[i], out);
}

void Octree::collectAll(OctreeNode* node, std::vector<OctreeNode*>& out) const
{
    if (!node) return;
    out.push_back(node);
    for (int i = 0; i < 8; i++) collectAll(node->children[i], out);
}

std::vector<OctreeNode*> Octree::getAllLeaves() const
{
    std::vector<OctreeNode*> out;
    collectLeaves(root, out);
    return out;
}

std::vector<OctreeNode*> Octree::getAllNodes() const
{
    std::vector<OctreeNode*> out;
    collectAll(root, out);
    return out;
}

// ============================================================================
// splat
//   PSR Step 2: 각 포인트의 법선을 주변 leaf 노드들에 B-spline 가중치로 분배
//
//   for each point p_i with normal N_i:
//     leaf = findLeaf(p_i)
//     neighbors = getLeafNeighborhood(leaf)   ← B-spline support가 겹치는 노드
//     for each neighbor o:
//         w = o.F(p_i)                        ← 3D tensor-product B-spline
//         o.vectorCoeff += w * N_i
//         o.splatWeight  += w
//
//   정규화: vectorCoeff /= splatWeight (0이 아닌 경우)
// ============================================================================
void Octree::splat(const std::vector<glm::vec3>& positions,
                   const std::vector<glm::vec3>& normals)
{
    assert(positions.size() == normals.size());

    // 모든 leaf 초기화
    auto leaves = getAllLeaves();
    for (auto* leaf : leaves)
    {
        leaf->vectorCoeff = glm::vec3(0.0f);
        leaf->splatWeight = 0.0f;
    }

    // Splatting
    for (int i = 0; i < (int)positions.size(); i++)
    {
        OctreeNode* leaf = findLeaf(positions[i]);
        if (!leaf) continue;

        auto neighbors = getLeafNeighborhood(leaf);
        for (OctreeNode* o : neighbors)
        {
            float w = o->F(positions[i]);
            if (w <= 0.0f) continue;

            o->vectorCoeff += w * normals[i];
            o->splatWeight  += w;
        }
    }

    // 정규화
    int splatted = 0;
    for (auto* leaf : leaves)
    {
        if (leaf->splatWeight > 1e-8f)
        {
            leaf->vectorCoeff /= leaf->splatWeight;
            splatted++;
        }
    }
    printf("[Octree] Splat done. %d / %d leaves received normal contribution\n",
           splatted, (int)leaves.size());
}

// ============================================================================
// printStats
// ============================================================================
void Octree::printStats() const
{
    auto leaves = getAllLeaves();
    auto all    = getAllNodes();

    // 깊이별 노드 수
    int depthCount[32] = {};
    int leafDepthCount[32] = {};
    int maxD = 0;
    for (auto* n : all)
    {
        int d = n->depth;
        if (d > maxD) maxD = d;
        depthCount[d]++;
        if (n->isLeaf()) leafDepthCount[d]++;
    }

    // 포인트 분포
    size_t maxPts = 0, totalPts = 0;
    for (auto* lf : leaves)
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
