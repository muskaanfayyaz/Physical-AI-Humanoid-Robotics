# Appendix D: Mathematical Foundations

This appendix covers essential mathematical concepts for robotics, including linear algebra, rotation representations, transformations, and dynamics. These foundations underpin kinematics, control, and motion planning algorithms.

## D.1 Linear Algebra for Robotics

### D.1.1 Vectors and Matrices

**Vector Notation:**

A vector in n-dimensional space:
```
v = [v₁, v₂, ..., vₙ]ᵀ
```

For robotics, commonly 2D or 3D:
```
v₂ = [x, y]ᵀ
v₃ = [x, y, z]ᵀ
```

**Vector Operations:**

| Operation | Notation | Result | Dimension |
|-----------|----------|--------|-----------|
| Addition | **a** + **b** | [a₁+b₁, a₂+b₂, a₃+b₃]ᵀ | Same as inputs |
| Subtraction | **a** - **b** | [a₁-b₁, a₂-b₂, a₃-b₃]ᵀ | Same as inputs |
| Scalar multiplication | c**a** | [ca₁, ca₂, ca₃]ᵀ | Same as **a** |
| Dot product | **a** · **b** | a₁b₁ + a₂b₂ + a₃b₃ | Scalar |
| Cross product | **a** × **b** | [a₂b₃-a₃b₂, a₃b₁-a₁b₃, a₁b₂-a₂b₁]ᵀ | 3D vector |
| Magnitude | \|\|**a**\|\| | √(a₁² + a₂² + a₃²) | Scalar |

**Vector Magnitude and Normalization:**

```
Magnitude: ||v|| = √(v₁² + v₂² + ... + vₙ²)

Normalized (unit) vector: v̂ = v / ||v||

Property: ||v̂|| = 1
```

**Cross Product (3D vectors only):**

```
a × b = |i    j    k  |
        |a₁   a₂   a₃ |
        |b₁   b₂   b₃ |

      = i(a₂b₃ - a₃b₂) - j(a₁b₃ - a₃b₁) + k(a₁b₂ - a₂b₁)
```

Properties:
- Perpendicular to both **a** and **b**
- Magnitude: ||**a** × **b**|| = ||**a**|| ||**b**|| sin(θ)
- Anti-commutative: **a** × **b** = -(**b** × **a**)

**Matrix Notation:**

An m × n matrix:
```
A = [a₁₁  a₁₂  ...  a₁ₙ]
    [a₂₁  a₂₂  ...  a₂ₙ]
    [... ... ... ...]
    [aₘ₁  aₘ₂  ...  aₘₙ]
```

Common robot matrices:
- Rotation matrix: 3 × 3
- Transformation matrix: 4 × 4
- Jacobian: m × n (depends on robot)

**Matrix Transpose:**

```
If A is m × n, then Aᵀ is n × m

Aᵀ[i,j] = A[j,i]

Example:
A = [1  2  3]    Aᵀ = [1  4]
    [4  5  6]         [2  5]
                      [3  6]
```

**Matrix Inverse:**

For square matrix A (n × n), if it exists:
```
A⁻¹ · A = A · A⁻¹ = I

Where I is the identity matrix
```

Properties:
- Only square matrices can have inverses
- Inverse exists if det(A) ≠ 0 (non-singular)
- (AB)⁻¹ = B⁻¹A⁻¹
- (Aᵀ)⁻¹ = (A⁻¹)ᵀ

**2×2 Matrix Inverse:**

```
A = [a  b]    A⁻¹ = 1/(ad-bc) · [ d  -b]
    [c  d]                      [-c   a]
```

**3×3 Matrix Inverse (rotation matrices):**

For orthogonal matrices (rotation matrices):
```
R⁻¹ = Rᵀ
```

This is computationally efficient and numerically stable.

### D.1.2 Matrix Operations

**Matrix Multiplication:**

If A is m × n and B is n × p, then C = AB is m × p:

```
C[i,j] = Σ(k=1 to n) A[i,k] · B[k,j]
```

Properties:
- Not commutative: AB ≠ BA (in general)
- Associative: (AB)C = A(BC)
- Distributive: A(B+C) = AB + AC

**Matrix-Vector Multiplication:**

```
y = Ax

Where A is m × n, x is n × 1, y is m × 1

y[i] = Σ(j=1 to n) A[i,j] · x[j]
```

**Determinant:**

For 2×2 matrix:
```
det(A) = ad - bc

Where A = [a  b]
          [c  d]
```

For 3×3 matrix:
```
det(A) = a(ei-fh) - b(di-fg) + c(dh-eg)

Where A = [a  b  c]
          [d  e  f]
          [g  h  i]
```

Properties:
- det(AB) = det(A) · det(B)
- det(Aᵀ) = det(A)
- det(A⁻¹) = 1/det(A)
- For rotation matrices: det(R) = 1

**Trace:**

```
tr(A) = Σ(i=1 to n) A[i,i]

(sum of diagonal elements)
```

Properties:
- tr(A + B) = tr(A) + tr(B)
- tr(cA) = c · tr(A)
- tr(AB) = tr(BA)

**Block Matrices:**

Useful for transformation matrices:
```
T = [R  p]
    [0  1]

Where:
  R is 3×3 rotation matrix
  p is 3×1 position vector
  0 is 1×3 zero vector
  1 is scalar
```

### D.1.3 Eigenvalues and Eigenvectors

**Definition:**

For square matrix A and vector v:
```
Av = λv

Where:
  v is eigenvector (v ≠ 0)
  λ is eigenvalue (scalar)
```

**Characteristic Equation:**

```
det(A - λI) = 0

Solving gives eigenvalues λ₁, λ₂, ..., λₙ
```

**Example (2×2 matrix):**

```
A = [4  1]
    [2  3]

Characteristic equation:
det([4-λ    1  ]) = 0
   [2    3-λ ]

(4-λ)(3-λ) - 2 = 0
λ² - 7λ + 10 = 0
(λ-5)(λ-2) = 0

Eigenvalues: λ₁ = 5, λ₂ = 2

For λ₁ = 5:
[4-5  1 ][v₁] = 0
[2  3-5][v₂]

[-1  1][v₁] = 0
[2  -2][v₂]

Eigenvector: v₁ = [1, 1]ᵀ
```

**Properties:**

- Sum of eigenvalues = trace of matrix
- Product of eigenvalues = determinant of matrix
- Real symmetric matrices have real eigenvalues
- Eigenvectors corresponding to different eigenvalues are orthogonal

**Applications in Robotics:**

1. **Stability Analysis:** System stable if all eigenvalues have negative real parts
2. **Principal Component Analysis:** Eigenvectors define principal axes
3. **Dynamics:** Natural frequencies from eigenvalues of system matrix
4. **Optimal Control:** Solution involves eigenvalue decomposition

### D.1.4 Singular Value Decomposition (SVD)

**Definition:**

Any m × n matrix A can be decomposed as:
```
A = UΣVᵀ

Where:
  U is m × m orthogonal matrix (left singular vectors)
  Σ is m × n diagonal matrix (singular values)
  V is n × n orthogonal matrix (right singular vectors)
```

**Singular Values:**

```
Σ = [σ₁  0   0  ...  0 ]
    [0   σ₂  0  ...  0 ]
    [0   0   σ₃ ...  0 ]
    [... ... ... ... ...]

Where σ₁ ≥ σ₂ ≥ σ₃ ≥ ... ≥ 0
```

**Properties:**

- Singular values are always non-negative
- Number of non-zero singular values = rank of matrix
- ||A|| = σ₁ (largest singular value)
- Condition number = σ₁/σₙ (ratio of largest to smallest non-zero singular value)

**Moore-Penrose Pseudoinverse:**

For non-square or singular matrices:
```
A⁺ = VΣ⁺Uᵀ

Where Σ⁺[i,i] = 1/σᵢ if σᵢ ≠ 0, else 0
```

**Applications in Robotics:**

1. **Inverse Kinematics:** Pseudoinverse of Jacobian for redundant robots
2. **Least Squares:** Solve overdetermined systems (Ax = b)
3. **Dimensionality Reduction:** Keep only largest singular values
4. **Singularity Analysis:** Detect when matrix becomes singular (σₙ → 0)

**Example:**

```python
import numpy as np

# Define matrix (Jacobian example)
A = np.array([[1, 2, 3],
              [4, 5, 6]])

# SVD decomposition
U, sigma, Vt = np.linalg.svd(A)

# Pseudoinverse
A_pinv = np.linalg.pinv(A)

# Verify: A · A⁺ · A = A
print(np.allclose(A @ A_pinv @ A, A))  # True
```

---

## D.2 Rotation Representations

Multiple ways to represent 3D rotations exist, each with advantages and disadvantages.

### D.2.1 Euler Angles

**Definition:**

Three angles representing rotations about three axes in specific order.

**Common Conventions:**

| Convention | Order | Use Case |
|------------|-------|----------|
| XYZ (Roll-Pitch-Yaw) | Rotate about X, then Y, then Z | Aircraft, mobile robots |
| ZYX (Yaw-Pitch-Roll) | Rotate about Z, then Y, then X | Alternative convention |
| ZYZ | Rotate about Z, then Y, then Z | Robot arms |
| XYZ (Fixed axes) | All rotations about fixed world frame | Mathematical analysis |

**Roll-Pitch-Yaw (RPY) - XYZ Convention:**

```
Roll (φ): Rotation about X-axis
Pitch (θ): Rotation about Y-axis
Yaw (ψ): Rotation about Z-axis
```

Rotation matrices for elementary rotations:

```
Rx(φ) = [1    0      0   ]
        [0  cos(φ) -sin(φ)]
        [0  sin(φ)  cos(φ)]

Ry(θ) = [ cos(θ)  0  sin(θ)]
        [   0     1    0   ]
        [-sin(θ)  0  cos(θ)]

Rz(ψ) = [cos(ψ) -sin(ψ)  0]
        [sin(ψ)  cos(ψ)  0]
        [  0       0     1]
```

**Combined Rotation (XYZ intrinsic):**

```
R(φ, θ, ψ) = Rz(ψ) · Ry(θ) · Rx(φ)

           = [c(ψ)c(θ)   c(ψ)s(θ)s(φ)-s(ψ)c(φ)   c(ψ)s(θ)c(φ)+s(ψ)s(φ)]
             [s(ψ)c(θ)   s(ψ)s(θ)s(φ)+c(ψ)c(φ)   s(ψ)s(θ)c(φ)-c(ψ)s(φ)]
             [-s(θ)      c(θ)s(φ)                  c(θ)c(φ)              ]

Where c(·) = cos(·), s(·) = sin(·)
```

**Extracting Euler Angles from Rotation Matrix:**

For ZYX convention (given R):

```
θ = atan2(-R[2,0], √(R[0,0]² + R[1,0]²))
φ = atan2(R[2,1], R[2,2])
ψ = atan2(R[1,0], R[0,0])

Note: Check for gimbal lock when θ = ±π/2
```

**Advantages:**
- Intuitive for humans
- Three numbers (compact)
- Easy to interpolate individual angles

**Disadvantages:**
- Gimbal lock (loss of one degree of freedom at certain orientations)
- Discontinuous (multiple representations for same orientation)
- Order-dependent
- Difficult to interpolate smoothly

**Gimbal Lock:**

Occurs when θ = ±π/2, causing first and third rotations to align, losing one DOF.

Example: θ = 90°
```
R = [0     sin(φ-ψ)    cos(φ-ψ)]
    [0     cos(φ-ψ)   -sin(φ-ψ)]
    [-1      0            0     ]
```

Only (φ-ψ) is determinable, not φ and ψ independently.

### D.2.2 Rotation Matrices

**Definition:**

3×3 orthogonal matrix representing rotation in 3D space.

**Properties:**

1. Orthogonal: RᵀR = RRᵀ = I
2. Determinant: det(R) = 1 (proper rotation)
3. Inverse: R⁻¹ = Rᵀ
4. Preserves length: ||Rv|| = ||v||
5. Columns (and rows) are orthonormal vectors

**Standard Form:**

```
R = [r₁₁  r₁₂  r₁₃]
    [r₂₁  r₂₂  r₂₃]
    [r₃₁  r₃₂  r₃₃]

Where columns are unit vectors:
  x̂ = [r₁₁, r₂₁, r₃₁]ᵀ
  ŷ = [r₁₂, r₂₂, r₃₂]ᵀ
  ẑ = [r₁₃, r₂₃, r₃₃]ᵀ
```

**Composing Rotations:**

```
R₃ = R₂ · R₁

(Apply R₁ first, then R₂)
```

**Rotating a Vector:**

```
v' = R · v

Where v is vector in original frame,
      v' is vector in rotated frame
```

**Advantages:**
- No singularities
- Direct composition (matrix multiplication)
- Efficient computation with optimized libraries
- Represents full SO(3) group

**Disadvantages:**
- 9 numbers with 6 constraints (redundant)
- No compact interpolation method
- Difficult to ensure orthogonality after numerical operations

**Orthonormalization (Gram-Schmidt):**

If R becomes non-orthogonal due to numerical errors:

```python
import numpy as np

def orthonormalize(R):
    """Orthonormalize rotation matrix using Gram-Schmidt."""
    x = R[:, 0]
    y = R[:, 1]
    z = R[:, 2]

    # Orthonormalize
    x = x / np.linalg.norm(x)
    y = y - np.dot(x, y) * x
    y = y / np.linalg.norm(y)
    z = np.cross(x, y)

    return np.column_stack([x, y, z])
```

### D.2.3 Quaternions

**Definition:**

Four-dimensional representation of rotation:
```
q = [qw, qx, qy, qz]ᵀ = [qw, qv]

Where:
  qw is scalar part
  qv = [qx, qy, qz]ᵀ is vector part

Constraint: ||q|| = √(qw² + qx² + qy² + qz²) = 1 (unit quaternion)
```

**Relationship to Axis-Angle:**

```
For rotation by angle θ about unit axis n = [nx, ny, nz]:

qw = cos(θ/2)
qx = nx · sin(θ/2)
qy = ny · sin(θ/2)
qz = nz · sin(θ/2)
```

**Quaternion Multiplication:**

```
q₁ ⊗ q₂ = [q₁w·q₂w - q₁v·q₂v]
          [q₁w·q₂v + q₂w·q₁v + q₁v × q₂v]

Where · is dot product, × is cross product
```

Expanded form:
```
q₁ ⊗ q₂ = [q₁w·q₂w - q₁x·q₂x - q₁y·q₂y - q₁z·q₂z]
          [q₁w·q₂x + q₁x·q₂w + q₁y·q₂z - q₁z·q₂y]
          [q₁w·q₂y - q₁x·q₂z + q₁y·q₂w + q₁z·q₂x]
          [q₁w·q₂z + q₁x·q₂y - q₁y·q₂x + q₁z·q₂w]
```

**Conjugate:**

```
q* = [qw, -qx, -qy, -qz]ᵀ
```

**Inverse:**

```
q⁻¹ = q* / ||q||²

For unit quaternions: q⁻¹ = q*
```

**Rotating a Vector:**

```
v' = q ⊗ [0, v] ⊗ q*

Where v is 3D vector, represented as [0, vx, vy, vz]
```

**Quaternion to Rotation Matrix:**

```
R = [1-2(qy²+qz²)    2(qxqy-qwqz)    2(qxqz+qwqy)]
    [2(qxqy+qwqz)    1-2(qx²+qz²)    2(qyqz-qwqx)]
    [2(qxqz-qwqy)    2(qyqz+qwqx)    1-2(qx²+qy²)]
```

**Rotation Matrix to Quaternion:**

```python
def matrix_to_quaternion(R):
    """Convert rotation matrix to quaternion."""
    trace = np.trace(R)

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2,1] - R[1,2]) * s
        qy = (R[0,2] - R[2,0]) * s
        qz = (R[1,0] - R[0,1]) * s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        qw = (R[2,1] - R[1,2]) / s
        qx = 0.25 * s
        qy = (R[0,1] + R[1,0]) / s
        qz = (R[0,2] + R[2,0]) / s
    elif R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        qw = (R[0,2] - R[2,0]) / s
        qx = (R[0,1] + R[1,0]) / s
        qy = 0.25 * s
        qz = (R[1,2] + R[2,1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        qw = (R[1,0] - R[0,1]) / s
        qx = (R[0,2] + R[2,0]) / s
        qy = (R[1,2] + R[2,1]) / s
        qz = 0.25 * s

    return np.array([qw, qx, qy, qz])
```

**Spherical Linear Interpolation (SLERP):**

Smooth interpolation between quaternions:

```
slerp(q₁, q₂, t) = (sin((1-t)θ)/sin(θ))·q₁ + (sin(tθ)/sin(θ))·q₂

Where:
  θ = arccos(q₁·q₂)
  t ∈ [0, 1]
```

**Advantages:**
- No gimbal lock
- Compact (4 numbers with 1 constraint)
- Efficient composition (quaternion multiplication)
- Smooth interpolation (SLERP)
- Numerically stable

**Disadvantages:**
- Less intuitive than Euler angles
- Double cover (q and -q represent same rotation)
- Requires normalization to maintain unit constraint

### D.2.4 Axis-Angle Representation

**Definition:**

Rotation by angle θ about unit axis n:
```
r = θ · n = [θnx, θny, θnz]ᵀ

Where:
  n = [nx, ny, nz]ᵀ is unit axis (||n|| = 1)
  θ is rotation angle (radians)
```

**Rodrigues' Formula (Axis-Angle to Rotation Matrix):**

```
R = I + sin(θ)[n]× + (1-cos(θ))[n]×²

Where [n]× is skew-symmetric matrix:

[n]× = [ 0   -nz   ny]
       [ nz   0   -nx]
       [-ny   nx   0 ]
```

**Expanded Form:**

```
R = [nx²(1-c)+c      nxny(1-c)-nzs   nxnz(1-c)+nys]
    [nxny(1-c)+nzs   ny²(1-c)+c      nynz(1-c)-nxs]
    [nxnz(1-c)-nys   nynz(1-c)+nxs   nz²(1-c)+c   ]

Where c = cos(θ), s = sin(θ)
```

**Rotation Matrix to Axis-Angle:**

```
θ = arccos((trace(R) - 1) / 2)

If θ ≠ 0:
  n = 1/(2sin(θ)) · [R[2,1] - R[1,2]]
                    [R[0,2] - R[2,0]]
                    [R[1,0] - R[0,1]]

If θ = 0: any axis (no rotation)
```

**Advantages:**
- Intuitive (axis and angle)
- Compact (3 numbers)
- Direct geometric meaning

**Disadvantages:**
- Discontinuous at θ = 0, 2π
- Multiple representations (θ, n) and (-θ, -n) are same rotation
- Non-unique for θ = 0

### D.2.5 Conversion Summary

**Conversion Table:**

| From → To | Formula |
|-----------|---------|
| Euler → Rotation Matrix | Sequential multiplication of elementary rotations |
| Rotation Matrix → Euler | Extract using atan2 (watch for gimbal lock) |
| Axis-Angle → Rotation Matrix | Rodrigues' formula |
| Rotation Matrix → Axis-Angle | θ = arccos((tr(R)-1)/2), extract axis |
| Quaternion → Rotation Matrix | See section D.2.3 |
| Rotation Matrix → Quaternion | See section D.2.3 (careful with branches) |
| Axis-Angle → Quaternion | q = [cos(θ/2), n·sin(θ/2)] |
| Quaternion → Axis-Angle | θ = 2·arccos(qw), n = qv/||qv|| |
| Euler → Quaternion | Convert to matrix, then to quaternion |
| Quaternion → Euler | Convert to matrix, then extract Euler |

**Python Example (Using scipy):**

```python
from scipy.spatial.transform import Rotation as R
import numpy as np

# Create rotation from Euler angles (XYZ convention)
r_euler = R.from_euler('xyz', [30, 45, 60], degrees=True)

# Convert to different representations
matrix = r_euler.as_matrix()
quat = r_euler.as_quat()  # [qx, qy, qz, qw] format
rotvec = r_euler.as_rotvec()  # axis-angle (θ·n)

# Create rotation from quaternion
r_quat = R.from_quat([0, 0, 0.707, 0.707])

# Compose rotations
r_combined = r_euler * r_quat

# Apply rotation to vector
v = np.array([1, 0, 0])
v_rotated = r_euler.apply(v)
```

---

## D.3 Transformation Matrices

### D.3.1 Homogeneous Coordinates

**Motivation:**

Combine rotation and translation in single matrix operation.

**2D Homogeneous Coordinates:**

Point (x, y) represented as:
```
[x]       [x/w]
[y]   or  [y/w]  (if w ≠ 0)
[w]       [ 1 ]
```

Typical: w = 1, giving [x, y, 1]ᵀ

**3D Homogeneous Coordinates:**

Point (x, y, z) represented as:
```
[x]
[y]
[z]
[1]
```

**Advantages:**
- Unifies rotation and translation
- Enables matrix composition
- Simplifies perspective projection
- Represents points at infinity (w = 0)

### D.3.2 4×4 Transformation Matrices

**General Form:**

```
T = [R  p]
    [0  1]

  = [r₁₁  r₁₂  r₁₃  px]
    [r₂₁  r₂₂  r₂₃  py]
    [r₃₁  r₃₂  r₃₃  pz]
    [ 0    0    0   1 ]

Where:
  R is 3×3 rotation matrix
  p is 3×1 position vector [px, py, pz]ᵀ
  0 is 1×3 zero row
  1 is scalar
```

**Transforming a Point:**

```
p' = T · p

[x']   [r₁₁  r₁₂  r₁₃  px]   [x]
[y'] = [r₂₁  r₂₂  r₂₃  py] · [y]
[z']   [r₃₁  r₃₂  r₃₃  pz]   [z]
[1 ]   [ 0    0    0   1 ]   [1]

Expanded:
x' = r₁₁x + r₁₂y + r₁₃z + px
y' = r₂₁x + r₂₂y + r₂₃z + py
z' = r₃₁x + r₃₂y + r₃₃z + pz
```

**Elementary Transformations:**

**Translation:**
```
Trans(dx, dy, dz) = [1  0  0  dx]
                    [0  1  0  dy]
                    [0  0  1  dz]
                    [0  0  0  1 ]
```

**Rotation about X-axis:**
```
Rot(X, θ) = [1    0      0    0]
            [0  cos(θ) -sin(θ) 0]
            [0  sin(θ)  cos(θ) 0]
            [0    0      0    1]
```

**Rotation about Y-axis:**
```
Rot(Y, θ) = [ cos(θ)  0  sin(θ)  0]
            [   0     1    0     0]
            [-sin(θ)  0  cos(θ)  0]
            [   0     0    0     1]
```

**Rotation about Z-axis:**
```
Rot(Z, θ) = [cos(θ) -sin(θ)  0  0]
            [sin(θ)  cos(θ)  0  0]
            [  0       0     1  0]
            [  0       0     0  1]
```

**Scaling (not rigid-body):**
```
Scale(sx, sy, sz) = [sx  0   0   0]
                    [0   sy  0   0]
                    [0   0   sz  0]
                    [0   0   0   1]
```

### D.3.3 Composition of Transformations

**Matrix Multiplication:**

```
T₃ = T₂ · T₁

Applies T₁ first, then T₂
```

Order matters: T₂·T₁ ≠ T₁·T₂ (in general)

**Example:**

Rotate 90° about Z, then translate [1, 0, 0]:

```
T_rot = Rot(Z, π/2) = [0  -1  0  0]
                      [1   0  0  0]
                      [0   0  1  0]
                      [0   0  0  1]

T_trans = Trans(1, 0, 0) = [1  0  0  1]
                           [0  1  0  0]
                           [0  0  1  0]
                           [0  0  0  1]

T_combined = T_trans · T_rot = [0  -1  0  1]
                               [1   0  0  0]
                               [0   0  1  0]
                               [0   0  0  1]
```

Point [1, 0, 0]ᵀ after transformation:
```
[0  -1  0  1]   [1]   [1]
[1   0  0  0] · [0] = [1]
[0   0  1  0]   [0]   [0]
[0   0  0  1]   [1]   [1]

Result: [1, 1, 0]ᵀ
```

**Relative vs. Fixed Frame:**

- **Fixed frame (pre-multiplication):** T_new = T_op · T_old
- **Relative frame (post-multiplication):** T_new = T_old · T_op

### D.3.4 Inverse Transformations

**General Inverse:**

```
T⁻¹ = [R  p]⁻¹ = [Rᵀ  -Rᵀp]
      [0  1]     [0    1  ]
```

Derivation:
```
T · T⁻¹ = [R  p] · [Rᵀ  -Rᵀp]
          [0  1]   [0    1  ]

        = [RRᵀ  R(-Rᵀp)+p]
          [0        1     ]

        = [I  -p+p]
          [0   1  ]

        = [I  0]
          [0  1] = Identity
```

**Computational Note:**

Never compute R⁻¹ directly; use Rᵀ for efficiency and numerical stability.

**Example:**

```python
import numpy as np

def transform_inverse(T):
    """Compute inverse of homogeneous transformation matrix."""
    R = T[:3, :3]
    p = T[:3, 3]

    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ p

    return T_inv
```

**Chain of Transformations:**

```
T_total = T_n · ... · T_2 · T_1

T_total⁻¹ = T_1⁻¹ · T_2⁻¹ · ... · T_n⁻¹

(Reverse order)
```

---

## D.4 Differential Equations for Dynamics

### D.4.1 Newton-Euler Equations

**Newton's Second Law:**

```
F = ma

Where:
  F is net force (vector)
  m is mass (scalar)
  a is acceleration (vector)
```

For 3D:
```
[Fx]   [max]
[Fy] = [may]
[Fz]   [maz]
```

**Euler's Equation (Rotational Dynamics):**

```
τ = I·ω̇ + ω × (I·ω)

Where:
  τ is net torque (vector)
  I is inertia tensor (3×3 matrix)
  ω is angular velocity (vector)
  ω̇ is angular acceleration (vector)
```

**Inertia Tensor:**

```
I = [Ixx  -Ixy  -Ixz]
    [-Iyx  Iyy  -Iyz]
    [-Izx  -Izy  Izz]

Where:
  Ixx = ∫(y² + z²)dm
  Ixy = ∫(xy)dm
  ... (symmetric: Ixy = Iyx)
```

**Parallel Axis Theorem:**

Inertia about parallel axis offset by d:
```
I_parallel = I_cm + m·d²

For general offset [dx, dy, dz]:
Ixx' = Ixx + m(dy² + dz²)
Ixy' = Ixy - m·dx·dy
...
```

**Newton-Euler Equations for Rigid Body:**

```
F = m·ac          (Linear motion)
τ = I·ω̇ + ω×(I·ω)  (Angular motion)

Where ac is center of mass acceleration
```

### D.4.2 Lagrangian Mechanics

**Lagrangian:**

```
L = T - V

Where:
  T is kinetic energy
  V is potential energy
```

**Euler-Lagrange Equation:**

```
d/dt(∂L/∂q̇ᵢ) - ∂L/∂qᵢ = τᵢ

Where:
  qᵢ is generalized coordinate
  q̇ᵢ is generalized velocity
  τᵢ is generalized force/torque
```

**Kinetic Energy:**

For single rigid body:
```
T = (1/2)m·vᵀv + (1/2)ωᵀIω

Where:
  v is linear velocity
  ω is angular velocity
```

For robot with n joints:
```
T = (1/2)q̇ᵀM(q)q̇

Where:
  q is joint position vector
  q̇ is joint velocity vector
  M(q) is mass/inertia matrix (configuration-dependent)
```

**Potential Energy:**

Gravitational:
```
V = mgh

Where h is height of center of mass
```

**Equations of Motion:**

For robot manipulator:
```
M(q)q̈ + C(q,q̇)q̇ + G(q) = τ

Where:
  M(q) is mass matrix
  C(q,q̇) is Coriolis/centrifugal matrix
  G(q) is gravity vector
  τ is joint torque vector
```

**Example: Simple Pendulum**

```
q = θ (angle from vertical)
L = (1/2)ml²θ̇² - mgl(1 - cos(θ))

Euler-Lagrange:
d/dt(ml²θ̇) + mgl·sin(θ) = 0
ml²θ̈ + mgl·sin(θ) = 0
θ̈ + (g/l)sin(θ) = 0
```

**Advantages of Lagrangian Formulation:**
- Systematic approach for complex systems
- Automatic constraint handling
- Energy-based (physically intuitive)
- Easier for systems with many DOF

### D.4.3 State-Space Representations

**State-Space Form:**

```
ẋ = f(x, u, t)
y = h(x, u, t)

Where:
  x is state vector
  u is input vector
  y is output vector
  ẋ is state derivative
```

**Linear Time-Invariant (LTI) Systems:**

```
ẋ = Ax + Bu
y = Cx + Du

Where A, B, C, D are constant matrices
```

**Example: Mass-Spring-Damper**

```
mẍ + bẋ + kx = F

State variables: x₁ = x, x₂ = ẋ

State-space:
[ẋ₁]   [ 0     1  ] [x₁]   [0  ]
[ẋ₂] = [-k/m  -b/m] [x₂] + [1/m] F

y = [1  0] [x₁]
            [x₂]
```

**Robot Manipulator State-Space:**

From M(q)q̈ + C(q,q̇)q̇ + G(q) = τ:

```
State: x = [q, q̇]ᵀ

ẋ = [      q̇        ]
    [M⁻¹(τ - Cq̇ - G)]
```

**Linearization:**

For nonlinear system around equilibrium (x₀, u₀):

```
A = ∂f/∂x|(x₀,u₀)
B = ∂f/∂u|(x₀,u₀)
C = ∂h/∂x|(x₀,u₀)
D = ∂h/∂u|(x₀,u₀)
```

**Stability Analysis:**

System stable if all eigenvalues of A have negative real parts.

**Discrete-Time State-Space:**

```
x[k+1] = Ax[k] + Bu[k]
y[k] = Cx[k] + Du[k]

Where k is time step index
```

**Conversion from Continuous to Discrete:**

```
Ad = e^(A·Δt)
Bd = A⁻¹(Ad - I)B

Where Δt is sampling period
```

Approximations:
- Forward Euler: Ad ≈ I + A·Δt, Bd ≈ B·Δt
- Zero-order hold: Exact for piecewise constant inputs

---

## Summary

This appendix covered essential mathematical foundations for robotics:

- **Linear Algebra**: Vectors, matrices, operations, eigenvalues, and SVD for analysis and computation
- **Rotation Representations**: Euler angles, rotation matrices, quaternions, and axis-angle with conversions
- **Transformation Matrices**: Homogeneous coordinates and 4×4 transformations for spatial relationships
- **Differential Equations**: Newton-Euler, Lagrangian, and state-space formulations for robot dynamics

These mathematical tools underpin kinematics, dynamics, control, and motion planning algorithms throughout robotics. Refer to this appendix when implementing low-level robot controllers or analyzing system behavior.

**Key Takeaways:**

1. Use quaternions for orientation representation in code (avoid gimbal lock)
2. Rotation matrices provide direct composition but are redundant
3. Transformation matrices unify rotation and translation
4. Lagrangian mechanics simplifies deriving equations of motion
5. State-space form enables modern control theory application

**Recommended Practice:**

Implement these conversions and operations in Python/C++ to build intuition:
- Euler angles ↔ Quaternions ↔ Rotation matrices
- Forward/inverse kinematics using transformation matrices
- Simple dynamics simulation using Euler integration
- Eigenvalue analysis for stability

Understanding these foundations enables effective use of robotics libraries (ROS, Isaac, etc.) and debugging of unexpected robot behaviors.
