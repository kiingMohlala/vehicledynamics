# Phase 8.0 – Beam Element Derivation

## DOF ordering

Each node has 6 DOF:

```
[ux, uy, uz, rx, ry, rz]
```

Element local DOF vector (12):

```
[ux_i, uy_i, uz_i, rx_i, ry_i, rz_i, ux_j, uy_j, uz_j, rx_j, ry_j, rz_j]
```

## Local stiffness (Euler-Bernoulli)

### Axial

```
k_axial = (EA/L) * [ 1  -1 ;  -1  1 ]  on (ux_i, ux_j)
```

### Torsion

```
k_torsion = (GJ/L) * [ 1  -1 ;  -1  1 ]  on (rx_i, rx_j)
```

### Bending about local z (deflection in y)

```
12 EIz/L³ ,  6 EIz/L² ,  4 EIz/L ,  2 EIz/L
```

Standard Hermitian beam matrix on `(uy_i, rz_i, uy_j, rz_j)`.

### Bending about local y (deflection in z)

Same form on `(uz_i, ry_i, uz_j, ry_j)` with sign convention such that
positive `My` produces curvature consistent with the right-hand rule.

## Coordinate transformation

Local x-axis: unit vector from node i → node j.

Local y, z: constructed from a reference vector (prefer global Z, else Y)
to avoid singularity for vertical members.

```
R = [x_axis; y_axis; z_axis]   (3×3)
T = blkdiag(R, R, R, R)        (12×12)
k_global = Tᵀ k_local T
```

## Assembly

Scatter each `k_global` into the structural `K` using the 12 global DOF
indices of the element endpoints.

## Boundary conditions

Essential BCs: constrained DOFs removed (partitioned solve).

```
K_ff u_f = F_f
R = K u − F     (reactions on constrained DOFs)
```

## Analytical benchmarks

| Case | Formula |
|------|---------|
| Cantilever tip load | δ = P L³ / (3 EI) |
| Simply supported mid load | δ = P L³ / (48 EI) |
| Axial bar | δ = P L / (EA) |
| Cantilever tip moment | θ = M L / (EI), δ = M L² / (2 EI) |
