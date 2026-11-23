"""Phase 9: STEP export and a quick finite element check of the parts.

This is a *cross-check*, not the final study. The final study is done in
Fusion and Ansys (see ``docs/phase9-fea-guide.md``); this one runs in a few
seconds per part so you know roughly what to expect before you open them,
and it gives a second, independent answer to compare against.

Needs the optional extras: ``pip install -e ".[fea]"`` (gmsh, scikit-fem,
pyamg). Nothing else in suspkin depends on them.

The method, step by step
------------------------
1. **Geometry** (gmsh's OpenCASCADE kernel, the same CAD kernel family Fusion
   and Ansys read STEP from): every cylinder and box from ``parts.py`` is made,
   they're all fused into one solid, then the holes are cut. That solid is
   written as STEP.
2. **Mesh**: tetrahedra (pyramids with 4 faces), ``mesh_size`` mm across.
3. **Elements**: quadratic tetrahedra (10 nodes, "P2"). Linear tets are far too
   stiff in bending; quadratic ones are what Ansys uses by default too.
4. **Supports**: the hole named ``part.support`` (bearing bore, rocker pivot)
   or the weld faces (pushrod bracket) are fixed: no movement at all.
5. **Loads**: each joint force is spread over its hole like a pin pressing on
   it: the part of the force across the hole is shared out with a cosine
   weighting over the half of the hole it pushes on (the "bearing load" that
   Ansys also offers); any part along the hole's axis is spread evenly.
6. **Solve** ``K u = f`` (stiffness x displacement = force) with conjugate
   gradients and an algebraic multigrid preconditioner (fast for big meshes).
7. **Stress**: von Mises stress per element. Fixed supports give infinite
   stress at their edges in theory (a "singularity"), so we also report the
   peak stress *away* from the supports, which is the number that converges.

Mesh convergence: solve the worst case on 3 meshes, each finer than the last.
If the peak stress settles (changes by a few % from the second to the third
mesh), the mesh is fine enough.
"""

from __future__ import annotations

import os
import sys
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .parts import Box, Cylinder, Part


def _require():
    try:
        import gmsh  # noqa: F401
        import skfem  # noqa: F401
    except ImportError as e:  # pragma: no cover - depends on the environment
        raise SystemExit(f"{e}. The FEA check needs the extras: pip install -e \".[fea]\"")


# Up to this many unknowns, use a direct solver (fast for several load cases,
# a few GB of memory at the limit); above it, iterative.
DIRECT_LIMIT = 150_000


@contextmanager
def _quiet():
    """Hide the CAD kernel's chatter (it prints straight to the terminal)."""
    sys.stdout.flush()
    saved = os.dup(1)
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, 1)
    try:
        yield
    finally:
        os.dup2(saved, 1)
        os.close(devnull)
        os.close(saved)


# ---------------------------------------------------------------------------
# Geometry and mesh (gmsh)
# ---------------------------------------------------------------------------

def _add_solid(occ, s) -> int:
    if isinstance(s, Cylinder):
        d = s.p2 - s.p1
        return occ.addCylinder(*s.p1, *d, s.radius)
    if isinstance(s, Box):
        tag = occ.addBox(-s.length / 2, -s.width / 2, -s.height / 2, s.length, s.width, s.height)
        r = np.column_stack([s.length_dir, s.width_dir, s.height_dir])
        m = np.hstack([r, s.center.reshape(3, 1)])
        occ.affineTransform([(3, tag)], list(m.ravel()))
        return tag
    raise TypeError(type(s))


def build_occ(part: Part):
    """Make the part in gmsh's current model; returns its volume tags."""
    import gmsh
    occ = gmsh.model.occ
    tags = [(3, _add_solid(occ, s)) for s in part.solids]
    if len(tags) > 1:
        fused, _ = occ.fuse(tags[:1], tags[1:])
    else:
        fused = tags
    cuts = [(3, _add_solid(occ, c)) for c in part.cuts]
    if cuts:
        fused, _ = occ.cut(fused, cuts)
    occ.synchronize()
    if len(fused) != 1 and not part.flat_supports:   # welded tabs may be separate pieces
        raise RuntimeError(f"{part.name}: the solids don't join into one piece ({len(fused)} pieces)")
    return [t for _, t in fused]


def write_step(part: Part, path: str | Path) -> float:
    """Write the part as STEP (mm). Returns its volume in mm^3."""
    _require()
    import gmsh
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add(part.name)
        tags = build_occ(part)
        vol = sum(gmsh.model.occ.getMass(3, t) for t in tags)
        with _quiet():
            gmsh.write(str(path))
        return float(vol)
    finally:
        gmsh.finalize()


def mesh_part(part: Part, size: float):
    """Tet mesh of the part: (node coordinates N x 3, tets M x 4 corner node indices)."""
    import gmsh
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add(part.name)
        build_occ(part)
        gmsh.option.setNumber("Mesh.MeshSizeMax", size)
        gmsh.option.setNumber("Mesh.MeshSizeMin", size / 4)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 12)  # ~12 elements round each hole
        gmsh.model.mesh.generate(3)
        node_tags, coords, _ = gmsh.model.mesh.getNodes()
        coords = coords.reshape(-1, 3)
        index = np.zeros(int(node_tags.max()) + 1, dtype=int)
        index[node_tags.astype(int)] = np.arange(len(node_tags))
        _, elem_nodes = gmsh.model.mesh.getElementsByType(4)  # 4-node tets
        tets = index[elem_nodes.astype(int)].reshape(-1, 4)
    finally:
        gmsh.finalize()
    used = np.unique(tets)
    remap = -np.ones(len(coords), dtype=int)
    remap[used] = np.arange(len(used))
    return coords[used], remap[tets]


# ---------------------------------------------------------------------------
# Solve (scikit-fem)
# ---------------------------------------------------------------------------

@dataclass
class FEAResult:
    case: str
    mesh_size: float
    n_elements: int
    n_dofs: int
    max_vm_mpa: float              # anywhere (includes support-edge singularities)
    max_vm_away_mpa: float         # more than `away_mm` from the supports
    where_away: np.ndarray         # location of that peak
    max_disp_mm: float


def _hole_facets(mesh, hole, tol: float) -> np.ndarray:
    """Boundary facets lying on a hole's cylindrical surface."""
    p = mesh.p[:, mesh.facets[:, mesh.boundary_facets()]]       # 3 x 3 x F
    a = hole.p2 - hole.p1
    L = np.linalg.norm(a)
    a = a / L
    rel = p - hole.p1.reshape(3, 1, 1)
    s = np.einsum("i,ijk->jk", a, rel)
    radial = np.linalg.norm(rel - a.reshape(3, 1, 1) * s, axis=0)
    ok = (np.abs(radial - hole.radius) < tol).all(axis=0) & (s > -tol).all(axis=0) & (s < L + tol).all(axis=0)
    return mesh.boundary_facets()[ok]


def _plane_facets(mesh, point, normal, tol: float) -> np.ndarray:
    bf = mesh.boundary_facets()
    p = mesh.p[:, mesh.facets[:, bf]]
    d = np.einsum("i,ijk->jk", np.asarray(normal), p - np.asarray(point).reshape(3, 1, 1))
    return bf[(np.abs(d) < tol).all(axis=0)]


def solve_part(part: Part, load_cases: dict[str, dict[str, np.ndarray]], material: dict,
               mesh_size: float, away_mm: float = 4.0) -> list[FEAResult]:
    """Linear static FEA of one part for several load cases (one mesh, one stiffness matrix)."""
    _require()
    import pyamg
    from scipy.sparse.linalg import cg
    from skfem import (Basis, ElementTetP2, ElementVector, FacetBasis, Functional, LinearForm,
                       MeshTet, asm, condense)
    from skfem.models.elasticity import lame_parameters, linear_elasticity

    pts, tets = mesh_part(part, mesh_size)
    mesh = MeshTet(pts.T.copy(), tets.T.copy())
    e = ElementVector(ElementTetP2())
    basis = Basis(mesh, e, intorder=2)
    E, nu = material["e_gpa"] * 1000.0, material["poisson"]  # MPa = N/mm^2
    lam, mu = lame_parameters(E, nu)
    K = asm(linear_elasticity(lam, mu), basis)

    tol = max(0.05 * mesh_size, 0.05)
    if part.flat_supports:
        fixed_f = np.concatenate([_plane_facets(mesh, f["point"], f["normal"], tol)
                                  for f in part.flat_supports])
    else:
        fixed_f = _hole_facets(mesh, part.holes[part.support], tol)
    if len(fixed_f) == 0:
        raise RuntimeError(f"{part.name}: found no mesh faces on the support")
    fixed = basis.get_dofs(facets=fixed_f).all()

    # Rigid body modes (3 shifts + 3 rotations) help the multigrid with elasticity.
    xyz = basis.doflocs
    n = basis.N
    B = np.zeros((n, 6))
    for k in range(3):
        B[k::3, k] = 1.0
    x, y, z = xyz[0, 0::3], xyz[1, 0::3], xyz[2, 0::3]
    B[0::3, 3], B[1::3, 3] = -y, x
    B[1::3, 4], B[2::3, 4] = -z, y
    B[0::3, 5], B[2::3, 5] = z, -x

    hole_bases = {}
    for name in {h for lc in load_cases.values() for h in lc}:
        fac = _hole_facets(mesh, part.holes[name], tol)
        if len(fac) == 0:
            raise RuntimeError(f"{part.name}: found no mesh faces on hole {name}")
        hole_bases[name] = FacetBasis(mesh, e, facets=fac, intorder=3)

    def load_vector(forces: dict[str, np.ndarray]) -> np.ndarray:
        f = np.zeros(n)
        for name, F in forces.items():
            F = np.asarray(F, dtype=float)
            if np.linalg.norm(F) < 1e-9:
                continue
            h = part.holes[name]
            a = (h.p2 - h.p1) / np.linalg.norm(h.p2 - h.p1)
            f_ax = (F @ a) * a
            f_rad = F - f_ax
            fb = hole_bases[name]
            area = asm(Functional(lambda w: 1.0 + 0 * w.x[0]), fb)
            if np.linalg.norm(f_rad) > 1e-9:
                d = f_rad / np.linalg.norm(f_rad)
                # w.n points out of the part (into the hole). The pin pushes on
                # the side of the hole whose wall faces against the force.
                weight = lambda w: np.maximum(0.0, -(d[0] * w.n[0] + d[1] * w.n[1] + d[2] * w.n[2]))  # noqa: E731
                wsum = asm(Functional(lambda w: weight(w)), fb)
                fr = f_rad / wsum

                @LinearForm
                def rad(v, w):
                    return weight(w) * (fr[0] * v[0] + fr[1] * v[1] + fr[2] * v[2])
                f += asm(rad, fb)
            if np.linalg.norm(f_ax) > 1e-9:
                fa = f_ax / area

                @LinearForm
                def ax(v, w):
                    return fa[0] * v[0] + fa[1] * v[1] + fa[2] * v[2]
                f += asm(ax, fb)
        return f

    Kc, _, _, I = condense(K, np.zeros(n), D=fixed)
    Kc = Kc.tocsr()
    if len(I) <= DIRECT_LIMIT:
        # Small enough: factorise once (LU), then each load case is nearly free.
        from scipy.sparse.linalg import splu
        lu = splu(Kc.tocsc())

        def solve(rhs):
            return lu.solve(rhs)
    else:
        # Big mesh: conjugate gradients with algebraic multigrid as the
        # preconditioner. The rigid body modes tell it what "cheap"
        # (low-energy) motion looks like.
        ml = pyamg.smoothed_aggregation_solver(Kc, B=B[I], max_coarse=500, smooth="energy",
                                               strength=("symmetric", {"theta": 0.0}))
        M = ml.aspreconditioner()

        def solve(rhs):
            x, info = cg(Kc, rhs, M=M, rtol=1e-8, maxiter=3000)
            if info != 0:
                raise RuntimeError(f"{part.name}: solver didn't converge ({info})")
            return x

    # Element centroids and their distance to the supports (for the "away" peak).
    cent = mesh.p[:, mesh.t].mean(axis=1)
    sup_pts = mesh.p[:, np.unique(mesh.facets[:, fixed_f])]
    from scipy.spatial import cKDTree
    dist, _ = cKDTree(sup_pts.T).query(cent.T)
    away = dist > away_mm

    out = []
    for label, forces in load_cases.items():
        f = load_vector(forces)
        u = np.zeros(n)
        u[I] = solve(f[I])
        vm = _von_mises(basis, u, lam, mu)               # per element, max over its points
        i_all = int(np.argmax(vm))
        i_away = int(np.argmax(np.where(away, vm, -1)))
        disp = np.linalg.norm(u.reshape(-1, 3), axis=1).max()
        out.append(FEAResult(label, mesh_size, mesh.t.shape[1], n, float(vm[i_all]),
                             float(vm[i_away]), cent[:, i_away], float(disp)))
    return out


def _von_mises(basis, u, lam, mu) -> np.ndarray:
    """Von Mises stress (MPa), the largest at any integration point of each element."""
    from skfem.helpers import eye, sym_grad, trace
    du = basis.interpolate(u)
    eps = sym_grad(du)                                    # 3 x 3 x elements x points
    sig = 2 * mu * eps + lam * eye(trace(eps), 3)
    s = sig
    vm = np.sqrt(0.5 * ((s[0, 0] - s[1, 1]) ** 2 + (s[1, 1] - s[2, 2]) ** 2 + (s[2, 2] - s[0, 0]) ** 2)
                 + 3 * (s[0, 1] ** 2 + s[1, 2] ** 2 + s[0, 2] ** 2))
    return vm.max(axis=1)


def convergence(part: Part, label: str, forces: dict[str, np.ndarray], material: dict,
                sizes: tuple[float, ...]) -> list[FEAResult]:
    """The same load case on coarser-to-finer meshes."""
    return [solve_part(part, {label: forces}, material, s)[0] for s in sizes]
