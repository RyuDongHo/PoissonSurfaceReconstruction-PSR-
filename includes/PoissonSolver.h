#ifndef POISSON_SOLVER_H
#define POISSON_SOLVER_H

// Forward declaration
class Octree;

// ============================================================================
// PoissonSolver
//
// Octree 기반 Poisson Surface Reconstruction 의 선형 시스템을 풀어
// 각 노드에 scalarValue (χ_o) 를 채운다.
//
// 이론:
//   ∇²χ = ∇·V
//   Galerkin 이산화: L x = b
//     L_{ij} = <∇F_i, ∇F_j>  (stiffness matrix)
//     b_i    = -<∇F_i, V>    (RHS from splatted vector field)
//   Conjugate Gradient 풀이 후 x_o → node.scalarValue
//
// 사용:
//   octree.build(...);
//   octree.computeDensityField(...);
//   octree.splat(...);
//   PoissonSolver::solve(&octree);   // ← scalarValue 채워짐
// ============================================================================
class PoissonSolver
{
public:
    // octree 의 모든 노드에 scalarValue 를 채운다.
    // splat() 이후에 호출해야 함.
    static void solve(Octree *octree, int maxIter = 2000, float tol = 1e-6f);
};

#endif // POISSON_SOLVER_H
