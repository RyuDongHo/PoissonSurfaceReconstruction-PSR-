# OctreeNode & Octree 설계 분석 보고서

## 목차
1. [전체 설계 철학](#1-전체-설계-철학)
2. [OctreeNode 클래스 상세 분석](#2-octreenode-클래스-상세-분석)
   - 2.1 공간 정보 멤버
   - 2.2 트리 구조 멤버
   - 2.3 포인트 데이터 멤버
   - 2.4 PSR Splatting 멤버
   - 2.5 생성자
   - 2.6 메소드 분석
3. [Octree 클래스 상세 분석](#3-octree-클래스-상세-분석)
   - 3.1 멤버 변수
   - 3.2 public 메소드 분석
   - 3.3 private 메소드 분석
4. [핵심 알고리즘 상세 분석](#4-핵심-알고리즘-상세-분석)
   - 4.1 Adaptive 분할 전략 (BFS)
   - 4.2 자식 인덱스 비트 플래그 체계
   - 4.3 B-spline Basis Function
   - 4.4 Splatting 파이프라인
   - 4.5 이웃 노드 탐색 (3×3×3 Neighborhood)
5. [PSR 전체 파이프라인과의 연결](#5-psr-전체-파이프라인과의-연결)
6. [복잡도 분석](#6-복잡도-분석)
7. [현재 설계의 한계 및 개선 방향](#7-현재-설계의-한계-및-개선-방향)

---

## 1. 전체 설계 철학

이 구현은 Kazhdan et al. (2006) "Poisson Surface Reconstruction" 논문의 알고리즘을 따른다.  
전체적인 책임 분리 원칙은 다음과 같다.

```
OctreeNode  →  공간 분할의 단위 셀 + PSR 필드 값 저장소
Octree      →  트리 전체의 생명주기 관리 + 알고리즘 실행 주체
```

`OctreeNode`는 자신이 어느 공간을 차지하는지, 그 공간에 어떤 포인트가 있는지, 그리고 PSR 계산에 필요한 값들을 **직접 보유**한다.  
`Octree`는 노드들을 어떻게 구성하고, 어떤 순서로 알고리즘을 수행할지를 **관장**한다.

---

## 2. OctreeNode 클래스 상세 분석

### 2.1 공간 정보 멤버

```cpp
glm::vec3 center;
float     halfSize;
int       depth;
int       childIndex;
```

#### `center` — 노드의 공간적 위치
- 노드가 담당하는 정육면체(AABB)의 **기하학적 중심점** (월드 좌표)
- 루트 노드의 center는 전체 포인트 클라우드 AABB의 중심
- 자식 노드 생성 시 부모 center에서 `±halfSize_child` 방향으로 오프셋

#### `halfSize` — 노드의 크기 척도
- 노드 AABB 한 변 길이의 **절반**
- 루트의 halfSize = `max(AABB 세 축 길이) / 2 * 1.001`
- 깊이가 1 깊어질 때마다 `halfSize /= 2` (octree 성질)
- **B-spline basis의 support 범위도 이 halfSize가 결정**한다 (정규화 기준)

깊이 `d`에서의 halfSize:
$$\text{halfSize}_d = \frac{\text{halfSize}_{\text{root}}}{2^d}$$

#### `depth` — 깊이 레벨
- 루트 = 0, 자식은 부모 + 1
- `Octree::build()` 에서 splitCondition: `depth < maxDepth` 판단에 사용
- `printStats()` 에서 깊이별 분포 통계에 사용

#### `childIndex` — 부모 기준 옥탄트 번호 (0~7, 루트=-1)
- 비트 플래그로 옥탄트 방향을 인코딩 (아래 4.2절 참고)
- 주로 디버깅과 향후 neighbor lookup 최적화에 활용 가능

---

### 2.2 트리 구조 멤버

```cpp
OctreeNode* children[8];
OctreeNode* parent;
```

#### `children[8]` — 8개 자식 노드 포인터
- 크기 고정 배열: octree는 항상 8분할이므로 동적 배열 불필요
- 생성 시 전부 `nullptr` 초기화
- **`isLeaf()` 판단 기준**: 8개 포인터 모두 nullptr이면 leaf
- leaf라면 자식 생성 없이 `pointIndices`에 포인트만 보유
- `subdivide()` 호출 시 8개 전부 new로 생성 (포인터 중 일부만 생성하지 않음 — 항상 8개 전부)

#### `parent` — 부모 노드 포인터
- 루트는 `nullptr`
- 현재 구현에서는 직접 사용하지 않음
- 향후 **bottom-up traversal** (Poisson 방정식 풀이에서 coarser 레벨로 올라가는 연산), 이웃 노드 탐색 최적화 등에 활용 가능

---

### 2.3 포인트 데이터 멤버

```cpp
std::vector<int> pointIndices;
```

#### `pointIndices` — 이 노드에 속한 포인트들의 인덱스
- 원본 `positions[]`, `normals[]` 벡터에 대한 **인덱스만 저장** (데이터 복사 없음)
- **Leaf 노드만 유효**: `build()` 과정에서 내부 노드는 자식에 포인트를 넘긴 후 `clear() + shrink_to_fit()`으로 메모리 해제
- Leaf 한 개에 최대 `minPointsToSplit`개(기본=1)의 포인트가 남음
- 빈 leaf(포인트 없음)도 존재 가능 — 공간적으로 분할은 됐지만 포인트가 없는 영역

**메모리 절약 설계**: `glm::vec3` 자체를 복사하지 않고 `int` 인덱스만 저장하므로, N개 포인트에 대해 추가 공간 O(N)만 사용.

---

### 2.4 PSR Splatting 멤버

```cpp
glm::vec3 vectorCoeff;
float     splatWeight;
float     scalarValue;
```

이 세 필드가 노드를 단순 공간 분할 구조가 아닌 **PSR 연산의 저장소**로 만든다.

#### `vectorCoeff` — 벡터장 V의 계수 α_o
PSR에서 입력 포인트의 법선을 basis function의 linear combination으로 표현:

$$\vec{V}(\mathbf{x}) = \sum_o \vec{\alpha}_o F_o(\mathbf{x})$$

`splat()` 수행 후 이 노드의 `vectorCoeff`는:
$$\vec{\alpha}_o = \frac{\sum_i w_i \cdot \vec{N}_i}{\sum_i w_i}$$

여기서 $w_i = F_o(\mathbf{p}_i)$ (B-spline basis를 포인트 위치에서 평가한 값).

- 정규화 전: $\sum_i w_i \cdot \vec{N}_i$
- 정규화 후: 위를 `splatWeight`로 나눈 값

#### `splatWeight` — 정규화를 위한 가중치 합
$$\text{splatWeight} = \sum_i F_o(\mathbf{p}_i)$$

- 이 값이 0이면 해당 노드의 B-spline support 안에 포인트가 하나도 없음
- `splat()` 정규화 단계에서 0 나누기 방지용으로 `> 1e-8` 조건 체크

#### `scalarValue` — Indicator function χ(o)의 값
- 현재는 0.0f로 초기화만 된 상태
- 향후 Poisson 방정식을 풀어 얻은 $\chi$값이 저장될 필드
- Marching Cubes 단계에서 이 값을 격자점 값으로 사용

---

### 2.5 생성자

```cpp
OctreeNode(glm::vec3 center_, float halfSize_, int depth_,
           OctreeNode* parent_, int childIndex_)
```

- 멤버 초기화 리스트 사용으로 불필요한 기본 초기화 방지
- PSR 필드(`vectorCoeff`, `splatWeight`, `scalarValue`)는 전부 0으로 초기화
- `children[8]` 전부 nullptr 루프 초기화

---

### 2.6 메소드 분석

#### `isLeaf() const`
```cpp
bool isLeaf() const {
    for (int i = 0; i < 8; i++)
        if (children[i] != nullptr) return false;
    return true;
}
```
자식 포인터 중 하나라도 존재하면 내부 노드. 설계상 `subdivide()`는 항상 8개 전부 생성하므로, 실제로는 `children[0] == nullptr`만 확인해도 동일하나 **방어적 설계**로 전부 확인.

---

#### `static BSpline1D(float t)`
```cpp
static float BSpline1D(float t) {
    t = std::fabsf(t);
    return (t < 1.0f) ? (1.0f - t) : 0.0f;
}
```

**Linear (order-1) B-spline**, 일명 tent function 또는 hat function:

$$B(t) = \max(0,\ 1 - |t|)$$

| 입력 t | 출력 |
|---|---|
| 0.0 | 1.0 (중심, 최대) |
| 0.5 | 0.5 |
| 0.9 | 0.1 |
| 1.0 | 0.0 (support 경계) |
| 1.5 | 0.0 (support 밖) |

- **static** 선언: 노드 인스턴스 없이 호출 가능, 순수 수학 함수
- support가 `[-1, 1]`로 제한됨 → t = `(p_axis - center_axis) / halfSize`로 정규화

---

#### `F(const glm::vec3& p) const`
```cpp
float F(const glm::vec3& p) const {
    float tx = (p.x - center.x) / halfSize;
    float ty = (p.y - center.y) / halfSize;
    float tz = (p.z - center.z) / halfSize;
    return BSpline1D(tx) * BSpline1D(ty) * BSpline1D(tz);
}
```

**3D tensor-product B-spline basis**:

$$F_o(\mathbf{p}) = B\!\left(\frac{p_x - c_x}{h}\right) \cdot B\!\left(\frac{p_y - c_y}{h}\right) \cdot B\!\left(\frac{p_z - c_z}{h}\right)$$

- 각 축을 독립적으로 정규화한 뒤 1D B-spline을 계산하고 곱셈
- 결과는 `[0, 1]` 범위
- p가 노드 중심에 가까울수록 1에 근접, support 경계(`halfSize` 거리)에서 0
- **support 범위**: `center ± halfSize` (각 축)
- 이 함수의 값이 곧 splatting 가중치 $w_i$

---

#### `contains(const glm::vec3& p) const`
```cpp
bool contains(const glm::vec3& p) const {
    return (std::fabsf(p.x - center.x) <= halfSize &&
            std::fabsf(p.y - center.y) <= halfSize &&
            std::fabsf(p.z - center.z) <= halfSize);
}
```

- AABB(Axis-Aligned Bounding Box) 포함 검사
- `<=` 를 사용하여 경계(surface)도 포함
- `findLeafImpl()` 탐색의 첫 번째 필터로 사용
- 부동소수점 오차로 인해 경계 포인트가 contains()를 통과하지 못할 수 있음 → `findLeafImpl()`에 안전망 코드(fallback 루프)가 별도 존재

---

## 3. Octree 클래스 상세 분석

### 3.1 멤버 변수

```cpp
OctreeNode* root;
int maxDepth;
int minPointsToSplit;
int totalNodeCount;
int leafNodeCount;
```

#### `root`
- 전체 트리의 진입점
- AABB 전체를 커버하는 단일 정육면체 노드
- `build()` 호출마다 기존 트리를 `deleteSubtree()`로 해제하고 새로 생성

#### `maxDepth`
- 트리가 세분화될 최대 깊이
- 깊이가 깊을수록 해상도 증가, 메모리/시간 지수적 증가
- 기본값 8 → 최대 $8^8 \approx 16$백만 개 노드 (실제로는 adaptive하므로 훨씬 적음)
- PSR 논문 권장: 해상도 요구에 따라 6~12

#### `minPointsToSplit`
- 노드 내 포인트 수가 이 값을 **초과**해야 분할
- 기본값 1 → 포인트가 2개 이상이면 분할 → leaf 하나에 최대 포인트 1개
- 값을 높이면 leaf 하나에 더 많은 포인트, 트리 깊이 감소 (빠른 빌드, 낮은 해상도)

#### `totalNodeCount` / `leafNodeCount`
- `subdivide()` 호출마다 totalNodeCount += 8
- BFS에서 분할하지 않기로 결정된 노드마다 leafNodeCount++
- `printStats()`와 디버깅에 활용

---

### 3.2 public 메소드 분석

#### `Octree(int maxDepth, int minPointsToSplit)` — 생성자
단순 멤버 초기화. 트리는 `build()` 호출 전까지 비어있음.

---

#### `~Octree()` — 소멸자
```cpp
Octree::~Octree() { deleteSubtree(root); }
```
`root`부터 재귀적으로 모든 노드 메모리 해제. 트리 생성에 `new`를 사용했으므로 반드시 필요.

---

#### `build(positions, normals)` — 핵심: 트리 구축

전체 파이프라인:

```
1. AABB 계산      → bbMin, bbMax
2. 정육면체화      → extent = max(dx, dy, dz) / 2 * 1.001
3. root 생성       → 모든 포인트 인덱스를 root.pointIndices에 적재
4. BFS 루프:
   ├─ shouldSplit 판단 (포인트 수 > minPointsToSplit && depth < maxDepth)
   ├─ true  → subdivide() → 포인트를 8자식에 배분 → 자식들을 queue에 push
   └─ false → leaf 확정, leafNodeCount++
```

**정육면체화** 이유: octree는 정육면체여야 `halfSize *= 0.5` 관계가 모든 깊이에서 성립. 직육면체이면 축마다 다른 halfSize를 관리해야 해서 복잡도 증가.

**`* 1.001` 여백**: 경계면 포인트가 AABB 표면에 정확히 위치할 경우 `contains()` 판단에서 누락될 수 있으므로 0.1% 여백 추가.

---

#### `findLeaf(const glm::vec3& p) const` — 포인트 위치로 leaf 탐색
```
root
 └→  childIndexOf()로 올바른 자식 선택
      └→  재귀 repeat until isLeaf()
```
O(depth) 시간. `splat()`에서 각 포인트마다 호출되므로, N개 포인트 × O(depth).

---

#### `getLeafNeighborhood(OctreeNode* node) const` — 3×3×3 이웃 탐색

splatting에서 linear B-spline이 노드 밖으로 영향을 미칠 수 있기 때문에 현재 leaf만이 아닌 인접 leaf들도 법선 기여를 받아야 한다. 이를 위해 설계된 함수.

```
step = node->halfSize * 2.0f   // 같은 depth 이웃까지의 중심 간 거리

for di in {-1, 0, +1}:
  for dj in {-1, 0, +1}:
    for dk in {-1, 0, +1}:
      candidatePos = node->center + (di, dj, dk) * step
      if root->contains(candidatePos):
        neighbor = findLeafImpl(root, candidatePos)
        if neighbor not in result:
          result.push_back(neighbor)
```

**27개 후보** → 실제 반환 개수는 그보다 적을 수 있음 (트리 경계, 빈 보셀 없음, 다른 resolution의 leaf가 여러 후보 중심을 커버하는 경우).

---

#### `getAllLeaves() / getAllNodes()` — 전수 수집
- `collectLeaves()` / `collectAll()`의 public wrapper
- DFS 후위 순회로 리스트 반환
- `splat()` 초기화, `printStats()` 등에서 사용

---

#### `splat(positions, normals)` — PSR Step 2: 법선 분배

상세 파이프라인:

```
1. 모든 leaf의 vectorCoeff, splatWeight을 0으로 초기화

2. for each point p_i (with normal N_i):
   a. leaf = findLeaf(p_i)              // p_i가 속한 leaf 탐색
   b. neighbors = getLeafNeighborhood(leaf)   // B-spline support 겹치는 이웃들
   c. for each node o in neighbors:
        w = o->F(p_i)                   // B-spline basis 평가 = 가중치
        if w > 0:
          o->vectorCoeff += w * N_i     // 법선 가중합
          o->splatWeight  += w          // 가중치 합

3. 정규화:
   for each leaf:
     if splatWeight > 1e-8:
       vectorCoeff /= splatWeight
```

이 결과로 각 leaf의 `vectorCoeff`는 **해당 노드 영역의 평균 법선 방향 벡터**가 된다.  
이것이 벡터장 $\vec{V}$의 계수 $\vec{\alpha}_o$이다.

---

#### `printStats() const` — 트리 통계 출력

출력 항목:
- 전체 노드 수 / leaf 노드 수
- 최대 깊이
- 트리 안에 배분된 총 포인트 수 / leaf 당 최대 포인트 수
- 깊이별 노드 수 / leaf 수

---

### 3.3 private 메소드 분석

#### `subdivide(OctreeNode* node)` — 노드를 8자식으로 분할

```cpp
float ch = node->halfSize * 0.5f;   // 자식 halfSize
int   cd = node->depth + 1;         // 자식 depth

for (int i = 0; i < 8; i++) {
    glm::vec3 offset(
        sign1(i & 1) * ch,   // bit0: x 방향
        sign1(i & 2) * ch,   // bit1: y 방향
        sign1(i & 4) * ch    // bit2: z 방향
    );
    node->children[i] = new OctreeNode(node->center + offset, ch, cd, node, i);
    totalNodeCount++;
}
```

`assert(node->isLeaf())` 로 이중 분할 방지. 포인트 배분은 `build()`가 담당하여 **관심사 분리**.

---

#### `childIndexOf(nodeCenter, p)` — 비트 플래그로 옥탄트 결정

```cpp
int idx = 0;
if (p.x > nodeCenter.x) idx |= 1;  // bit0
if (p.y > nodeCenter.y) idx |= 2;  // bit1
if (p.z > nodeCenter.z) idx |= 4;  // bit2
```

3번의 비교 연산만으로 8개 자식 중 어느 옥탄트인지 O(1)로 결정. (아래 4.2절 참고)

---

#### `findLeafImpl(node, p)` — 재귀 leaf 탐색 (내부 구현)

```
findLeafImpl(node, p):
  if node == null or !node->contains(p): return null
  if node->isLeaf(): return node       // 탐색 완료
  
  ci = childIndexOf(node->center, p)   // 올바른 자식 방향
  res = findLeafImpl(node->children[ci], p)
  if res: return res                   // 정상 경로
  
  // 안전망: 부동소수점 오차 대비 나머지 7개 자식도 시도
  for i in 0..7 (i != ci):
    res = findLeafImpl(node->children[i], p)
    if res: return res
  
  return null
```

**안전망 fallback**: `p`가 두 cell의 정확히 경계에 있을 때 `childIndexOf()`가 이상적인 방향을 반환해도 `contains()` 판단에서 부동소수점 오차로 실패할 수 있다. 이 경우 나머지 자식을 순차 탐색.

---

#### `collectLeaves(node, out)` / `collectAll(node, out)` — DFS 수집

```
collectLeaves(node, out):
  if node == null: return
  if node->isLeaf(): out.push_back(node); return
  for i in 0..7: collectLeaves(children[i], out)

collectAll(node, out):
  if node == null: return
  out.push_back(node)                  // 현재 노드 먼저 추가 (pre-order)
  for i in 0..7: collectAll(children[i], out)
```

---

#### `deleteSubtree(node)` — 재귀적 메모리 해제

```
deleteSubtree(node):
  if node == null: return
  for i in 0..7: deleteSubtree(children[i])   // 자식 먼저 (post-order)
  delete node                                   // 그 다음 본인
```

post-order traversal로 자식을 먼저 해제한 후 부모를 해제. dangling pointer 방지.

---

## 4. 핵심 알고리즘 상세 분석

### 4.1 Adaptive 분할 전략 (BFS)

**왜 BFS인가?**  
DFS로도 구현 가능하지만, BFS는 레벨 순서대로 처리되어 **같은 깊이의 노드들이 연속적으로 처리**된다. 직관적인 디버깅이 가능하고, 향후 레벨별 처리를 추가하기 쉽다.

**Adaptive의 의미:**

```
포인트 밀도 높은 영역:
  많은 포인트 → shouldSplit=true → 계속 분할 → 깊은 leaf

포인트 없는 영역:
  0개 포인트 → shouldSplit=false → 얕은 leaf
```

동일한 maxDepth를 설정해도 모든 노드가 maxDepth까지 분할되지 않는다. 포인트가 있는 곳만 깊어지므로 메모리를 효율적으로 사용.

**빈 자식 처리:**  
`subdivide()` 후 포인트가 0개인 자식도 생성된다. 이 자식들은 queue에 push하지 않고 즉시 `leafNodeCount++`. 빈 leaf이지만 공간적으로 존재하며, `getLeafNeighborhood()`에서 탐색될 수 있다.

---

### 4.2 자식 인덱스 비트 플래그 체계

```
childIndex: bit2(z) | bit1(y) | bit0(x)

idx 0 = 000 → ( x<c, y<c, z<c ) → 8번째 옥탄트 (-,-,-)
idx 1 = 001 → ( x>c, y<c, z<c ) → (+,-,-)
idx 2 = 010 → ( x<c, y>c, z<c ) → (-,+,-)
idx 3 = 011 → ( x>c, y>c, z<c ) → (+,+,-)
idx 4 = 100 → ( x<c, y<c, z>c ) → (-,-,+)
idx 5 = 101 → ( x>c, y<c, z>c ) → (+,-,+)
idx 6 = 110 → ( x<c, y>c, z>c ) → (-,+,+)
idx 7 = 111 → ( x>c, y>c, z>c ) → (+,+,+)
```

`subdivide()`의 offset 계산이 이와 정확히 일치:
```cpp
sign1(i & 1)  // bit0 → x 방향
sign1(i & 2)  // bit1 → y 방향  (i & 2 는 0 or 2, sign1(2)=+1 이므로 OK)
sign1(i & 4)  // bit2 → z 방향
```

`sign1(bool)`은 0→-1, nonzero→+1 이므로 bit masking 결과를 bool로 자동 변환하여 올바르게 동작.

---

### 4.3 B-spline Basis Function

**왜 Linear B-spline인가?**  
PSR 원 논문에서는 tri-linear B-spline (order 1)을 사용. 다음 이유로 선택:
- 연산이 단순 (곱셈 3회)
- support가 `[-1, 1]`으로 좁아 이웃 탐색 범위 최소화 (3×3×3)
- 연속적이지만 미분 불연속 (기울기 단절) → Poisson 풀이에서는 미분 필요 없음

더 높은 차수 (Quadratic, Cubic B-spline)를 사용하면:
- support 범위 증가 → 5×5×5, 7×7×7 이웃 탐색 필요
- 더 부드러운 벡터장 → 더 나은 표면 품질
- 구현 및 계산 복잡도 증가

**정규화 방식 설명:**

포인트 $p_i$에서 노드 $o$의 B-spline 값:
$$t_x = \frac{p_x - c_x}{h}, \quad t_y = \frac{p_y - c_y}{h}, \quad t_z = \frac{p_z - c_z}{h}$$

$$F_o(p_i) = B(t_x) \cdot B(t_y) \cdot B(t_z) \in [0, 1]$$

- $p_i = c_o$ (정확히 노드 중심)이면 $F_o = 1.0$
- $p_i$가 `halfSize` 이상 떨어지면 $F_o = 0.0$

---

### 4.4 Splatting 파이프라인

전체 흐름을 수식으로 표현:

$$\vec{V}(x) = \sum_{o} \vec{\alpha}_o F_o(x)$$

여기서 $\vec{\alpha}_o$는 splatting으로 결정:

$$\vec{\alpha}_o = \frac{\displaystyle\sum_{p_i \in \text{support}(o)} F_o(p_i) \cdot \vec{N}_i}{\displaystyle\sum_{p_i \in \text{support}(o)} F_o(p_i)}$$

**물리적 해석**: 노드 $o$의 영역을 커버하는 포인트들의 법선을 B-spline으로 가중 평균한 값. 포인트가 노드 중심 가까울수록 더 많이 기여.

**splatting의 목적**: 이산 포인트 샘플 $\{(p_i, N_i)\}$를 연속 벡터장 $\vec{V}$로 변환하는 과정. 이 $\vec{V}$의 divergence를 구하면 Poisson 방정식의 RHS가 된다.

---

### 4.5 이웃 노드 탐색 (3×3×3 Neighborhood)

**왜 이웃이 필요한가?**  
포인트 $p_i$가 leaf 노드 $o$의 경계 근처에 있으면, $p_i$는 인접 노드 $o'$의 support 안에도 들어갈 수 있다 ($F_{o'}(p_i) > 0$). 따라서 $o'$도 $p_i$의 법선 기여를 받아야 한다.

```
예:
노드 o (center=0, halfSize=1): support [-1, 1]
포인트 p_i = 0.8 (노드 경계 근처)

이웃 노드 o' (center=2, halfSize=1): support [1, 3]
F_{o'}(p_i) = B((0.8 - 2) / 1) = B(-1.2) = 0   → 이 경우엔 0

이웃 노드 o'' (center=1.5, halfSize=1): support [0.5, 2.5]
F_{o''}(p_i) = B((0.8 - 1.5) / 1) = B(-0.7) = 0.3 > 0  → 기여해야 함
```

**step = halfSize * 2.0**: 같은 깊이의 이웃 노드 중심까지의 거리. 이 오프셋으로 27개 후보 위치를 생성하고, 각 위치에서 실제 존재하는 leaf를 `findLeafImpl()`로 탐색.

**중복 제거**: adaptive tree이므로, 포인트 밀도가 낮은 영역에서는 더 큰 leaf가 여러 후보 위치를 커버할 수 있다. 중복 체크로 같은 노드가 여러 번 추가되는 것을 방지.

---

## 5. PSR 전체 파이프라인과의 연결

```
[완료] Step 1: Adaptive Octree 구축
        build(positions, normals)
        → 각 leaf에 pointIndices 저장

[완료] Step 2: Splatting  
        splat(positions, normals)
        → 각 leaf의 vectorCoeff (= α_o) 결정

[미구현] Step 3: 발산 계산 (RHS 구성)
        ∇·V(x) = Σ_o α_o · ∇F_o(x)
        → 각 노드에서 basis function의 gradient 계산 후 내적

[미구현] Step 4: Poisson 방정식 풀기
        ΔX = ∇·V
        → Sparse linear system 구성, Conjugate Gradient 등으로 풀기
        → 각 노드의 scalarValue에 χ 저장

[미구현] Step 5: Marching Cubes
        χ의 isovalue 추출 → 삼각 메시 생성
        → isovalue = octree leaf들에서 χ의 평균값
```

---

## 6. 복잡도 분석

| 연산 | 시간 복잡도 | 비고 |
|---|---|---|
| `build()` | $O(N \cdot D)$ | N=포인트 수, D=maxDepth |
| `findLeaf()` | $O(D)$ | |
| `getLeafNeighborhood()` | $O(27 \cdot D)$ | 27번의 findLeaf |
| `splat()` | $O(N \cdot 27 \cdot D)$ | 각 포인트마다 findLeaf + neighborhood |
| `getAllLeaves()` | $O(L)$ | L=leaf 노드 수 |
| 메모리 (노드) | $O(N)$ | adaptive이므로 포인트 수에 비례 |

minPointsToSplit=1 기준:
- 최악(모든 포인트가 같은 경로): leaf $\approx N$개, depth $\approx \log_8 N$
- maxDepth=8 제한 시: leaf $\leq 8^8 \approx 16$M개이나 실제로는 $\approx N$

---

## 7. 현재 설계의 한계 및 개선 방향

### 한계 1: `getLeafNeighborhood()`의 성능
27번의 `findLeafImpl()` 호출은 각각 O(depth) → 전체 splatting이 $O(N \cdot 27 \cdot D)$.  
**개선**: 각 노드에 이웃 노드 포인터를 precompute하여 캐싱 (`neighbor링크 테이블`)

### 한계 2: Adaptive depth 불일치 시 이웃 탐색 오차
`getLeafNeighborhood()`는 **현재 노드의 depth 기준** step으로 후보를 탐색한다. 이웃 영역의 leaf가 현재 노드보다 더 깊거나 얕을 수 있어 B-spline support가 정확히 일치하지 않을 수 있음.  
**개선**: support 겹침을 depth에 무관하게 AABB 교차 검사로 판단

### 한계 3: Linear B-spline의 미분 불연속
표면 법선 방향이 급격히 변하는 영역에서 아티팩트 발생 가능.  
**개선**: Quadratic 또는 Cubic B-spline으로 교체 (support 범위 확장 필요)

### 한계 4: `parent` 포인터 미활용
현재는 저장만 하고 사용하지 않음.  
**활용**: Poisson 방정식의 multigrid solver에서 coarser level로의 restriction 연산, 이웃 탐색 시 upward traversal 최적화

### 한계 5: `scalarValue` 미구현
PSR의 핵심 결과물인 $\chi$가 아직 계산되지 않음.  
**다음 단계**: `computeDivergence()` → `poissonSolve()` 구현 필요
