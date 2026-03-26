#include "PoissonSolver.h"
#include "Octree.h"
#include "OctreeNode.h"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_map>
#include <utility>
#include <vector>

// ============================================================================
// 5-point Gauss-Legendre integration
// ============================================================================
static const float gl_x[5] = { -0.906180f, -0.538469f, 0.0f, 0.538469f, 0.906180f };
static const float gl_w[5] = { 0.236927f, 0.478629f, 0.568889f, 0.478629f, 0.236927f };

template <typename Func>
static float gauss5(float lo, float hi, Func f)
{
    if (hi <= lo) return 0.0f;
    float mid = 0.5f * (lo + hi);
    float half = 0.5f * (hi - lo);
    float sum = 0.0f;
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
    if (ta < 0.5f) return -2.0f * t;
    if (ta < 1.5f) return (t > 0.0f ? 1.0f : -1.0f) * (ta - 1.5f);
    return 0.0f;
}

// ============================================================================
// 1D inner product: integral fi(x) * fj(x) dx
// ============================================================================
static float inner1D(float ci, float w_i, float cj, float w_j,
                     bool deriv_i, bool deriv_j)
{
    float lo = std::max(ci - 1.5f * w_i, cj - 1.5f * w_j);
    float hi = std::min(ci + 1.5f * w_i, cj + 1.5f * w_j);
    if (lo >= hi) return 0.0f;

    return gauss5(lo, hi, [&](float x) {
        float ti = (x - ci) / w_i;
        float tj = (x - cj) / w_j;
        float fi = deriv_i ? (dBSpline1D(ti) / (w_i * w_i))
                           : (OctreeNode::BSpline1D(ti) / w_i);
        float fj = deriv_j ? (dBSpline1D(tj) / (w_j * w_j))
                           : (OctreeNode::BSpline1D(tj) / w_j);
        return fi * fj;
    });
}

// ============================================================================
// Support overlap quick test
// ============================================================================
static bool supportsOverlap(const OctreeNode *a, const OctreeNode *b)
{
    float thresh = 1.5f * (a->width() + b->width());
    return (std::fabsf(a->center.x - b->center.x) < thresh &&
            std::fabsf(a->center.y - b->center.y) < thresh &&
            std::fabsf(a->center.z - b->center.z) < thresh);
}

// ============================================================================
// Exact pair integrals
// ============================================================================
static float computeLijExact(const OctreeNode *a, const OctreeNode *b)
{
    const float ca[3] = { a->center.x, a->center.y, a->center.z };
    const float cb[3] = { b->center.x, b->center.y, b->center.z };
    const float wa[3] = { a->width(), a->width(), a->width() };
    const float wb[3] = { b->width(), b->width(), b->width() };

    float BB[3], dBdB[3];
    for (int ax = 0; ax < 3; ax++)
    {
        BB[ax] = inner1D(ca[ax], wa[ax], cb[ax], wb[ax], false, false);
        dBdB[ax] = inner1D(ca[ax], wa[ax], cb[ax], wb[ax], true, true);
    }

    float Lij = 0.0f;
    if (BB[1] != 0.0f && BB[2] != 0.0f) Lij += dBdB[0] * BB[1] * BB[2];
    if (BB[0] != 0.0f && BB[2] != 0.0f) Lij += BB[0] * dBdB[1] * BB[2];
    if (BB[0] != 0.0f && BB[1] != 0.0f) Lij += BB[0] * BB[1] * dBdB[2];
    return Lij;
}

static glm::vec3 computeGradFdotFExact(const OctreeNode *a, const OctreeNode *b)
{
    const float ca[3] = { a->center.x, a->center.y, a->center.z };
    const float cb[3] = { b->center.x, b->center.y, b->center.z };
    const float wa[3] = { a->width(), a->width(), a->width() };
    const float wb[3] = { b->width(), b->width(), b->width() };

    float BB[3], dBB[3];
    for (int ax = 0; ax < 3; ax++)
    {
        BB[ax] = inner1D(ca[ax], wa[ax], cb[ax], wb[ax], false, false);
        dBB[ax] = inner1D(ca[ax], wa[ax], cb[ax], wb[ax], true, false);
    }

    return glm::vec3(
        dBB[0] * BB[1] * BB[2],
        BB[0] * dBB[1] * BB[2],
        BB[0] * BB[1] * dBB[2]);
}

// ============================================================================
// Relative-offset table key/value
// ============================================================================
struct RelativeOffsetKey
{
    int depth_i, depth_j;
    int dx, dy, dz; // quantized by finest half-step

    bool operator==(const RelativeOffsetKey &o) const
    {
        return depth_i == o.depth_i && depth_j == o.depth_j &&
               dx == o.dx && dy == o.dy && dz == o.dz;
    }
};

struct RelativeOffsetKeyHash
{
    size_t operator()(const RelativeOffsetKey &k) const
    {
        size_t h = 1469598103934665603ull;
        h ^= (size_t)k.depth_i; h *= 1099511628211ull;
        h ^= (size_t)k.depth_j; h *= 1099511628211ull;
        h ^= (size_t)(k.dx + 1000003); h *= 1099511628211ull;
        h ^= (size_t)(k.dy + 1000033); h *= 1099511628211ull;
        h ^= (size_t)(k.dz + 1000037); h *= 1099511628211ull;
        return h;
    }
};

struct PairIntegralValue
{
    float Lij;
    glm::vec3 gradFdotF;
};

using PairIntegralCache = std::unordered_map<RelativeOffsetKey, PairIntegralValue, RelativeOffsetKeyHash>;

static int quantizeOffset(float delta, float quantum)
{
    return (int)std::llround((double)delta / (double)quantum);
}

static const PairIntegralValue &getOrCreatePairIntegral(
    const OctreeNode *ni,
    const OctreeNode *nj,
    float coordQuantum,
    PairIntegralCache &cache)
{
    RelativeOffsetKey key{
        ni->depth, nj->depth,
        quantizeOffset(nj->center.x - ni->center.x, coordQuantum),
        quantizeOffset(nj->center.y - ni->center.y, coordQuantum),
        quantizeOffset(nj->center.z - ni->center.z, coordQuantum)
    };

    auto it = cache.find(key);
    if (it != cache.end())
        return it->second;

    PairIntegralValue val;
    val.Lij = computeLijExact(ni, nj);
    val.gradFdotF = computeGradFdotFExact(ni, nj);
    return cache.emplace(key, val).first->second;
}

// ============================================================================
// Sparse matrix formats
// ============================================================================
struct CSRMatrix
{
    int n = 0;
    std::vector<int> rowPtr;
    std::vector<int> colIdx;
    std::vector<float> values;
};

// ============================================================================
// Conjugate Gradient on CSR
// ============================================================================
static float cgDot(const std::vector<float> &a, const std::vector<float> &b)
{
    float s = 0.0f;
    for (int i = 0; i < (int)a.size(); i++) s += a[i] * b[i];
    return s;
}

static std::vector<float> csrMul(const CSRMatrix &A, const std::vector<float> &x)
{
    std::vector<float> out(A.n, 0.0f);
    for (int i = 0; i < A.n; ++i)
    {
        for (int p = A.rowPtr[i]; p < A.rowPtr[i + 1]; ++p)
            out[i] += A.values[p] * x[A.colIdx[p]];
    }
    return out;
}

static std::vector<float> conjugateGradientCSR(
    const CSRMatrix &A,
    const std::vector<float> &b,
    int maxIter, float tol)
{
    int N = A.n;
    std::vector<float> x(N, 0.0f);
    std::vector<float> r = b;
    std::vector<float> z(N, 0.0f);
    std::vector<float> p(N, 0.0f);
    std::vector<float> invDiag(N, 1.0f);

    for (int i = 0; i < N; ++i)
    {
        float diag = 0.0f;
        for (int k = A.rowPtr[i]; k < A.rowPtr[i + 1]; ++k)
        {
            if (A.colIdx[k] == i)
            {
                diag = A.values[k];
                break;
            }
        }
        if (std::fabsf(diag) > 1e-20f)
            invDiag[i] = 1.0f / diag;
    }

    for (int i = 0; i < N; ++i)
    {
        z[i] = invDiag[i] * r[i];
        p[i] = z[i];
    }
    float rz = cgDot(r, z);

    printf("[CG] N=%d, r0=%.4e\n", N, std::sqrt(cgDot(r, r)));

    for (int iter = 0; iter < maxIter; iter++)
    {
        auto Ap = csrMul(A, p);
        float pAp = cgDot(p, Ap);
        if (std::fabsf(pAp) < 1e-20f) break;

        float alpha = rz / pAp;
        for (int i = 0; i < N; i++) x[i] += alpha * p[i];
        for (int i = 0; i < N; i++) r[i] -= alpha * Ap[i];

        float rr_new = cgDot(r, r);
        float res = std::sqrt(rr_new);

        if (iter % 100 == 0)
            printf("[CG] iter=%4d  residual=%.4e\n", iter, res);

        if (res < tol)
        {
            printf("[CG] converged at iter=%d  residual=%.4e\n", iter, res);
            break;
        }

        for (int i = 0; i < N; ++i)
            z[i] = invDiag[i] * r[i];

        float rz_new = cgDot(r, z);
        float beta = rz_new / rz;
        for (int i = 0; i < N; i++) p[i] = z[i] + beta * p[i];
        rz = rz_new;
    }
    return x;
}

static int interiorIndex3D(int x, int y, int z, int interiorRes)
{
    return (x - 1) + interiorRes * ((y - 1) + interiorRes * (z - 1));
}

static void applyRegularGridLaplacian(const std::vector<float> &x,
                                      std::vector<float> &Ax,
                                      int interiorRes)
{
    const int N = interiorRes * interiorRes * interiorRes;
    Ax.assign((size_t)N, 0.0f);

    for (int z = 1; z <= interiorRes; ++z)
    for (int y = 1; y <= interiorRes; ++y)
    for (int x0 = 1; x0 <= interiorRes; ++x0)
    {
        const int row = interiorIndex3D(x0, y, z, interiorRes);
        float v = 6.0f * x[(size_t)row];

        if (x0 > 1)           v -= x[(size_t)interiorIndex3D(x0 - 1, y, z, interiorRes)];
        if (x0 < interiorRes) v -= x[(size_t)interiorIndex3D(x0 + 1, y, z, interiorRes)];
        if (y > 1)            v -= x[(size_t)interiorIndex3D(x0, y - 1, z, interiorRes)];
        if (y < interiorRes)  v -= x[(size_t)interiorIndex3D(x0, y + 1, z, interiorRes)];
        if (z > 1)            v -= x[(size_t)interiorIndex3D(x0, y, z - 1, interiorRes)];
        if (z < interiorRes)  v -= x[(size_t)interiorIndex3D(x0, y, z + 1, interiorRes)];

        Ax[(size_t)row] = v;
    }
}

static std::vector<float> conjugateGradientRegularGrid(const std::vector<float> &b,
                                                       int interiorRes,
                                                       int maxIter,
                                                       float tol)
{
    const int N = interiorRes * interiorRes * interiorRes;
    std::vector<float> x((size_t)N, 0.0f);
    std::vector<float> r = b;
    std::vector<float> z((size_t)N, 0.0f);
    std::vector<float> p((size_t)N, 0.0f);
    std::vector<float> Ap((size_t)N, 0.0f);

    const float invDiag = 1.0f / 6.0f;
    for (int i = 0; i < N; ++i)
    {
        z[(size_t)i] = invDiag * r[(size_t)i];
        p[(size_t)i] = z[(size_t)i];
    }

    float rz = cgDot(r, z);
    printf("[PCG] N=%d, r0=%.4e\n", N, std::sqrt(cgDot(r, r)));

    for (int iter = 0; iter < maxIter; ++iter)
    {
        applyRegularGridLaplacian(p, Ap, interiorRes);
        float pAp = cgDot(p, Ap);
        if (std::fabsf(pAp) < 1e-20f)
            break;

        float alpha = rz / pAp;
        for (int i = 0; i < N; ++i)
        {
            x[(size_t)i] += alpha * p[(size_t)i];
            r[(size_t)i] -= alpha * Ap[(size_t)i];
        }

        float rr = cgDot(r, r);
        float res = std::sqrt(rr);
        if (iter % 100 == 0)
            printf("[PCG] iter=%4d residual=%.4e\n", iter, res);
        if (res < tol)
        {
            printf("[PCG] converged at iter=%d residual=%.4e\n", iter, res);
            break;
        }

        for (int i = 0; i < N; ++i)
            z[(size_t)i] = invDiag * r[(size_t)i];

        float rzNew = cgDot(r, z);
        float beta = rzNew / rz;
        for (int i = 0; i < N; ++i)
            p[(size_t)i] = z[(size_t)i] + beta * p[(size_t)i];
        rz = rzNew;
    }

    return x;
}

// ============================================================================
// PoissonSolver::solve
// ============================================================================
void PoissonSolver::solve(Octree *octree, int maxIter, float tol)
{
    auto allNodes = octree->getAllNodes();
    const int N = (int)allNodes.size();
    for (int i = 0; i < N; ++i)
    {
        allNodes[(size_t)i]->scalarValue = 0.0f;
        allNodes[(size_t)i]->nodeIndex = i;
    }

    octree->clearRegularGridField();

    if (N == 0)
        return;

    const float coordQuantum =
        octree->root->halfSize / (float)(1 << octree->maxDepth);
    PairIntegralCache pairCache;

    std::vector<std::vector<std::pair<int, float>>> rowEntries((size_t)N);
    std::vector<float> bvec((size_t)N, 0.0f);

    size_t overlaps = 0;
    size_t nnz = 0;

    printf("[Poisson] nodes=%d  building system (CSR + relative-offset cache)...\n", N);

    for (int i = 0; i < N; ++i)
    {
        OctreeNode *ni = allNodes[(size_t)i];

        for (int j = i; j < N; ++j)
        {
            OctreeNode *nj = allNodes[(size_t)j];
            if (!supportsOverlap(ni, nj))
                continue;

            overlaps++;
            const PairIntegralValue &pair =
                getOrCreatePairIntegral(ni, nj, coordQuantum, pairCache);

            if (std::fabsf(pair.Lij) > 1e-14f)
            {
                rowEntries[(size_t)i].push_back({ j, pair.Lij });
                nnz++;
                if (j != i)
                {
                    rowEntries[(size_t)j].push_back({ i, pair.Lij });
                    nnz++;
                }
            }

            if (glm::dot(nj->vectorCoeff, nj->vectorCoeff) > 1e-20f)
                bvec[(size_t)i] += glm::dot(nj->vectorCoeff, pair.gradFdotF);

            if (j != i && glm::dot(ni->vectorCoeff, ni->vectorCoeff) > 1e-20f)
                bvec[(size_t)j] -= glm::dot(ni->vectorCoeff, pair.gradFdotF);
        }
    }

    CSRMatrix A;
    A.n = N;
    A.rowPtr.resize((size_t)N + 1, 0);
    A.colIdx.reserve(nnz);
    A.values.reserve(nnz);

    for (int i = 0; i < N; ++i)
    {
        auto &row = rowEntries[(size_t)i];
        std::sort(row.begin(), row.end(),
                  [](const auto &lhs, const auto &rhs) { return lhs.first < rhs.first; });
        A.rowPtr[(size_t)i + 1] = A.rowPtr[(size_t)i] + (int)row.size();
        for (const auto &entry : row)
        {
            A.colIdx.push_back(entry.first);
            A.values.push_back(entry.second);
        }
    }

    double avgRowNnz = (N > 0) ? (double)nnz / (double)N : 0.0;
    printf("[Poisson] overlaps=%zu  cacheEntries=%zu  nnz=%zu  avgRowNNZ=%.2f\n",
           overlaps, pairCache.size(), nnz, avgRowNnz);
    printf("[Poisson] system built, solving CG (CSR SpMV)...\n");

    auto x = conjugateGradientCSR(A, bvec, maxIter, tol);
    for (int i = 0; i < N; ++i)
        allNodes[(size_t)i]->scalarValue = x[(size_t)i];

    printf("[Poisson] done.\n");
}
