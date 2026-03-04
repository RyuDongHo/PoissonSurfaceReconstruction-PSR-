#include "PoissonSolver.h"
#include "Octree.h"
#include "OctreeNode.h"

#include <glm/glm.hpp>
#include <vector>
#include <cmath>
#include <cstdio>
#include <utility>
#include <algorithm>

// ============================================================================
// 5-point Gauss-Legendre 구적  ([-1,1] -> [lo,hi] 변환 포함)
// ============================================================================
static const float gl_x[5] = { -0.906180f, -0.538469f,  0.0f,  0.538469f,  0.906180f };
static const float gl_w[5] = {  0.236927f,  0.478629f,  0.568889f,  0.478629f,  0.236927f };

template<typename Func>
static float gauss5(float lo, float hi, Func f)
{
    if (hi <= lo) return 0.0f;
    float mid  = 0.5f * (lo + hi);
    float half = 0.5f * (hi - lo);
    float sum  = 0.0f;
    for (int k = 0; k < 5; k++)
        sum += gl_w[k] * f(mid + half * gl_x[k]);
    return half * sum;
}

// ============================================================================
// dB(t)/dt  (quadratic B-spline, n=3)
// ============================================================================
static float dBSpline1D(float t)
{
    float ta = std::fabsf(t);
    if (ta < 0.5f)  return -2.0f * t;
    if (ta < 1.5f)  return (t > 0.0f ? 1.0f : -1.0f) * (ta - 1.5f);
    return 0.0f;
}

// ============================================================================
// 1D inner product:  integral fi(x) fj(x) dx
//   deriv = true  -> B'((x-c)/h) / h   (공간 미분)
//   deriv = false -> B((x-c)/h)
// ============================================================================
static float inner1D(float ci, float h_i, float cj, float h_j,
                     bool deriv_i, bool deriv_j)
{
    float lo = std::max(ci - 1.5f * h_i, cj - 1.5f * h_j);
    float hi = std::min(ci + 1.5f * h_i, cj + 1.5f * h_j);
    if (lo >= hi) return 0.0f;

    return gauss5(lo, hi, [&](float x) {
        float ti = (x - ci) / h_i;
        float tj = (x - cj) / h_j;
        float fi = deriv_i ? (dBSpline1D(ti) / h_i) : OctreeNode::BSpline1D(ti);
        float fj = deriv_j ? (dBSpline1D(tj) / h_j) : OctreeNode::BSpline1D(tj);
        return fi * fj;
    });
}

// ============================================================================
// 두 노드의 3D 지지역이 겹치는가
// ============================================================================
static bool supportsOverlap(const OctreeNode *a, const OctreeNode *b)
{
    float thresh = 1.5f * (a->halfSize + b->halfSize);
    return (std::fabsf(a->center.x - b->center.x) < thresh &&
            std::fabsf(a->center.y - b->center.y) < thresh &&
            std::fabsf(a->center.z - b->center.z) < thresh);
}

// ============================================================================
// L_{ij} = <nabla F_i, nabla F_j>  (3D separable)
//   = dBdB_x * BB_y * BB_z
//   + BB_x  * dBdB_y * BB_z
//   + BB_x  * BB_y  * dBdB_z
// ============================================================================
static float computeLij(const OctreeNode *a, const OctreeNode *b)
{
    const float ca[3] = { a->center.x, a->center.y, a->center.z };
    const float cb[3] = { b->center.x, b->center.y, b->center.z };

    float BB[3], dBdB[3];
    for (int ax = 0; ax < 3; ax++)
    {
        BB[ax]   = inner1D(ca[ax], a->halfSize, cb[ax], b->halfSize, false, false);
        dBdB[ax] = inner1D(ca[ax], a->halfSize, cb[ax], b->halfSize, true,  true );
    }

    float Lij = 0.0f;
    if (BB[1] != 0.0f && BB[2] != 0.0f) Lij += dBdB[0] * BB[1]   * BB[2];
    if (BB[0] != 0.0f && BB[2] != 0.0f) Lij += BB[0]   * dBdB[1] * BB[2];
    if (BB[0] != 0.0f && BB[1] != 0.0f) Lij += BB[0]   * BB[1]   * dBdB[2];
    return Lij;
}

// ============================================================================
// integral nabla F_a · F_b dV  (벡터, 컴포넌트별)
//   x: dBB_x * BB_y  * BB_z
//   y: BB_x  * dBB_y * BB_z
//   z: BB_x  * BB_y  * dBB_z
// ============================================================================
static glm::vec3 computeGradFdotF(const OctreeNode *a, const OctreeNode *b)
{
    const float ca[3] = { a->center.x, a->center.y, a->center.z };
    const float cb[3] = { b->center.x, b->center.y, b->center.z };

    float BB[3], dBB[3];
    for (int ax = 0; ax < 3; ax++)
    {
        BB[ax]  = inner1D(ca[ax], a->halfSize, cb[ax], b->halfSize, false, false);
        dBB[ax] = inner1D(ca[ax], a->halfSize, cb[ax], b->halfSize, true,  false);
    }

    return glm::vec3(
        dBB[0] * BB[1]  * BB[2],
        BB[0]  * dBB[1] * BB[2],
        BB[0]  * BB[1]  * dBB[2]
    );
}

// ============================================================================
// Conjugate Gradient  (L x = b, L symmetric positive semi-definite)
// ============================================================================
using SparseRow = std::vector<std::pair<int, float>>;

static float cgDot(const std::vector<float> &a, const std::vector<float> &b)
{
    float s = 0.0f;
    for (int i = 0; i < (int)a.size(); i++) s += a[i] * b[i];
    return s;
}

static std::vector<float> cgMul(const std::vector<SparseRow> &L,
                                 const std::vector<float> &v)
{
    int N = (int)L.size();
    std::vector<float> out(N, 0.0f);
    for (int i = 0; i < N; i++)
        for (auto &[j, val] : L[i])
            out[i] += val * v[j];
    return out;
}

static std::vector<float> conjugateGradient(
    const std::vector<SparseRow> &L,
    const std::vector<float> &b,
    int maxIter, float tol)
{
    int N = (int)b.size();
    std::vector<float> x(N, 0.0f);
    std::vector<float> r = b;
    std::vector<float> p = r;
    float rr = cgDot(r, r);

    printf("[CG] N=%d, r0=%.4e\n", N, std::sqrt(rr));

    for (int iter = 0; iter < maxIter; iter++)
    {
        auto Lp   = cgMul(L, p);
        float pLp = cgDot(p, Lp);
        if (std::fabsf(pLp) < 1e-20f) break;

        float alpha = rr / pLp;
        for (int i = 0; i < N; i++) x[i] += alpha * p[i];
        for (int i = 0; i < N; i++) r[i] -= alpha * Lp[i];

        float rr_new = cgDot(r, r);
        float res    = std::sqrt(rr_new);

        if (iter % 100 == 0)
            printf("[CG] iter=%4d  residual=%.4e\n", iter, res);

        if (res < tol) {
            printf("[CG] converged at iter=%d  residual=%.4e\n", iter, res);
            break;
        }

        float beta = rr_new / rr;
        for (int i = 0; i < N; i++) p[i] = r[i] + beta * p[i];
        rr = rr_new;
    }
    return x;
}

// ============================================================================
// PoissonSolver::solve
// ============================================================================
void PoissonSolver::solve(Octree *octree, int maxIter, float tol)
{
    // Step 1: 노드 인덱싱
    auto allNodes = octree->getAllNodes();
    int N = (int)allNodes.size();
    for (int i = 0; i < N; i++)
        allNodes[i]->nodeIndex = i;

    printf("[Poisson] nodes=%d  building system...\n", N);

    // Step 2: sparse L, dense b 구축
    std::vector<SparseRow> Lrows(N);
    std::vector<float>     bvec(N, 0.0f);

    for (int i = 0; i < N; i++)
    {
        OctreeNode *ni = allNodes[i];

        for (int j = 0; j < N; j++)
        {
            OctreeNode *nj = allNodes[j];
            if (!supportsOverlap(ni, nj)) continue;

            // L_{ij}
            float Lij = computeLij(ni, nj);
            if (std::fabsf(Lij) > 1e-14f)
                Lrows[i].push_back({ j, Lij });

            // b_i += v_j · integral nabla F_i · F_j   (b = L^{-1} * <nablaF_i, V>)
            // 유도: Green IBP 양변에 적용 → -<nablaF_i,nablaX> = -<nablaF_i,V> → 부호 상쇄
            if (glm::dot(nj->vectorCoeff, nj->vectorCoeff) > 1e-20f)
            {
                glm::vec3 gFF = computeGradFdotF(ni, nj);
                bvec[i] += glm::dot(nj->vectorCoeff, gFF);
            }
        }
    }

    printf("[Poisson] system built, solving CG...\n");

    // Step 3: CG 풀기
    auto x = conjugateGradient(Lrows, bvec, maxIter, tol);

    // Step 4: scalarValue 저장
    for (int i = 0; i < N; i++)
        allNodes[i]->scalarValue = x[i];

    printf("[Poisson] done.\n");
}
