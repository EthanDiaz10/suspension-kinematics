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
        # Turn it into place with a true rotation (an affine transform would
        # turn its flat faces into spline surfaces), then move it.
        from scipy.spatial.transform import Rotation
        rv = Rotation.from_matrix(np.column_stack([s.length_dir, s.width_dir, s.height_dir])).as_rotvec()
        angle = float(np.linalg.norm(rv))
        if angle > 1e-12:
            occ.rotate([(3, tag)], 0, 0, 0, *(rv / angle), angle)
        occ.translate([(3, tag)], *s.center)
        return tag
    raise TypeError(type(s))


def _edges_of(vol: int) -> list[int]:
    import gmsh
    out = set()
    for _, srf in gmsh.model.getBoundary([(3, vol)], combined=False, oriented=False):
        for _, c in gmsh.model.getBoundary([(2, srf)], combined=False, oriented=False):
            out.add(abs(c))
    return sorted(out)


def _junction_edges(vol: int, cyl: Cylinder) -> list[int]:
    """Edges where something meets the side of a cylinder (not its round end edges).

    These are the inside corners where an arm joins a housing or boss.
    """
    import gmsh
    a = cyl.axis
    out = []
    for c in _edges_of(vol):
        lo, hi = gmsh.model.getParametrizationBounds(1, c)
        ts = np.linspace(lo[0], hi[0], 7)
        pts = np.array([gmsh.model.getValue(1, c, [t]) for t in ts])
        rel = pts - cyl.p1
        ax = rel @ a
        rad = np.linalg.norm(rel - np.outer(ax, a), axis=1)
        on_side = np.all(np.abs(rad - cyl.radius) < 1e-3) and np.all(ax > -1e-3) and np.all(ax < cyl.length + 1e-3)
        if on_side and np.ptp(ax) > 1e-3:    # a round end edge has constant ax
            out.append(c)
    return out


def _concave_plane_edges(vol: int) -> list[int]:
    """Inside (re-entrant) corners where two flat faces meet, e.g. arm meets arm.

    For each edge shared by two flat faces: step from the edge into face 1
    (towards its centre). At a convex (outside) edge that step goes against
    face 2's outward normal; at a concave (inside) edge it goes with it.
    """
    import gmsh
    out = []
    for c in _edges_of(vol):
        faces = [abs(f) for f in gmsh.model.getAdjacencies(1, c)[0]]
        faces = [f for f in faces if gmsh.model.getType(2, f) == "Plane"]
        if len(faces) != 2:
            continue
        lo, hi = gmsh.model.getParametrizationBounds(1, c)
        m = np.array(gmsh.model.getValue(1, c, [(lo[0] + hi[0]) / 2]))
        normals, centres = [], []
        for f in faces:
            ctr = np.array(gmsh.model.occ.getCenterOfMass(2, f))
            uv = gmsh.model.getParametrization(2, f, list(ctr))
            normals.append(np.array(gmsh.model.getNormal(f, uv)))
            centres.append(ctr)
        t1 = centres[0] - m
        if np.linalg.norm(t1) < 1e-9:
            continue
        if (t1 / np.linalg.norm(t1)) @ normals[1] > 0.05:
            out.append(c)
    return out


def _fillet_junctions(vol: int, part: Part, radius: float) -> tuple[int, list[str]]:
    """Round the inside corners where arms meet the cylinders (housing, bosses).

    Sharp inside corners have (in theory) infinite stress, so the peak never
    converges as the mesh gets finer. Real parts are filleted, so we fillet
    them here too. Tries each cylinder in turn and shrinks the radius if the
    CAD kernel can't fit it; anything it can't round is listed in the notes.
    """
    import gmsh
    occ = gmsh.model.occ
    notes = []
    for cyl in [s for s in part.solids if isinstance(s, Cylinder)]:
        edges = _junction_edges(vol, cyl)
        if not edges:
            continue
        for r in (radius, 0.75 * radius, 0.5 * radius):
            try:
                out = occ.fillet([vol], edges, [r], removeVolume=False)
                occ.remove([(3, vol)], recursive=True)   # keep only the filleted copy
                occ.synchronize()
                vol = out[0][1]
                if r < radius:
                    notes.append(f"{cyl.name}: fillet reduced to {r:g} mm")
                break
            except Exception:
                occ.synchronize()
        else:
            notes.append(f"{cyl.name}: couldn't fillet (left sharp)")
    # Then inside corners between flat faces (arm meeting arm, arm meeting its
    # root block), all at once. One at a time is tempting, but a fillet the
    # kernel half-manages can leave a broken solid, so if the set fails they
    # stay sharp (and the notes say so).
    edges = _concave_plane_edges(vol)
    if edges:
        try:
            out = occ.fillet([vol], edges, [radius], removeVolume=False)
            occ.remove([(3, vol)], recursive=True)
            occ.synchronize()
            vol = out[0][1]
        except Exception:
            occ.synchronize()
            notes.append(f"{len(edges)} flat inside corner(s) couldn't be filleted (left sharp)")
    return vol, notes


def build_occ(part: Part, fillet_mm: float = 0.0):
    """Make the part in gmsh's current model; returns its volume tags.

    ``fillet_mm`` > 0 rounds the arm-to-cylinder corners before the holes are cut.
    """
    import gmsh
    occ = gmsh.model.occ
    tags = [(3, _add_solid(occ, s)) for s in part.solids]
    if len(tags) > 1:
        fused, _ = occ.fuse(tags[:1], tags[1:])
    else:
        fused = tags
    occ.synchronize()
    part.notes = [n for n in part.notes if not n.startswith("FEA fillet:")]
    if fillet_mm > 0 and len(fused) == 1:
        vol, notes = _fillet_junctions(fused[0][1], part, fillet_mm)
        fused = [(3, vol)]
        part.notes += [f"FEA fillet: {n}" for n in notes]
    cuts = [(3, _add_solid(occ, c)) for c in part.cuts]
    if cuts:
        fused, _ = occ.cut(fused, cuts)
    occ.synchronize()
    if len(fused) != 1 and not part.flat_supports:   # welded tabs may be separate pieces
        raise RuntimeError(f"{part.name}: the solids don't join into one piece ({len(fused)} pieces)")
    return [t for _, t in fused]


def write_step(part: Part, path: str | Path, fillet_mm: float = 0.0) -> float:
    """Write the part as STEP (mm). Returns its volume in mm^3."""
    _require()
    import gmsh
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add(part.name)
        tags = build_occ(part, fillet_mm)
        vol = sum(gmsh.model.occ.getMass(3, t) for t in tags)
        with _quiet():
            gmsh.write(str(path))
        return float(vol)
    finally:
        gmsh.finalize()


def mesh_part(part: Part, size: float, fillet_mm: float = 0.0):
    """Tet mesh of the part: (node coordinates N x 3, tets M x 4 corner node indices)."""
    import gmsh
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add(part.name)
        build_occ(part, fillet_mm)
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
    # For plotting (only with keep_field=True): mesh corner points (3 x N),
    # boundary triangles (3 x F, point indices) and the stress of the element behind each.
    field: tuple | None = None


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
               mesh_size: float, away_mm: float = 4.0, fillet_mm: float = 0.0,
               keep_field: bool = False) -> list[FEAResult]:
    """Linear static FEA of one part for several load cases (one mesh, one stiffness matrix)."""
    _require()
    import pyamg
    from scipy.sparse.linalg import cg
    from skfem import (Basis, ElementTetP2, ElementVector, FacetBasis, Functional, LinearForm,
                       MeshTet, asm, condense)
    from skfem.models.elasticity import lame_parameters, linear_elasticity

    pts, tets = mesh_part(part, mesh_size, fillet_mm)
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
        field = None
        if keep_field:
            bf = mesh.boundary_facets()
            field = (mesh.p, mesh.facets[:, bf], vm[mesh.f2t[0, bf]])
        out.append(FEAResult(label, mesh_size, mesh.t.shape[1], n, float(vm[i_all]),
                             float(vm[i_away]), cent[:, i_away], float(disp), field))
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
                sizes: tuple[float, ...], fillet_mm: float = 0.0) -> list[FEAResult]:
    """The same load case on coarser-to-finer meshes."""
    return [solve_part(part, {label: forces}, material, s, fillet_mm=fillet_mm)[0] for s in sizes]


def plot_stress(panels: list[tuple[str, FEAResult, float]], path: str | Path,
                required_sf: float = 2.0, views: list[tuple[float, float]] | None = None) -> None:
    """Von Mises stress on the surface of each part, side by side.

    ``panels`` = (title, result kept with ``keep_field=True``, yield stress).
    One colour scale (light = low, dark red = high) from 0 to the largest
    allowable stress (yield / required SF); each title gives the part's own
    safety factor. ``views`` = (elevation, azimuth) per panel, in degrees.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    limit = max(sy for _, _, sy in panels) / required_sf
    cmap = plt.get_cmap("Reds")
    fig = plt.figure(figsize=(4.2 * len(panels), 4.6))
    for i, (title, r, sy) in enumerate(panels):
        p, tri, vm = r.field
        ax = fig.add_subplot(1, len(panels), i + 1, projection="3d")
        verts = p.T[tri.T]
        colors = cmap(np.clip(vm / limit, 0, 1) * 0.85 + 0.1)
        # Simple shading so the shape reads in 3D: darker where a face turns away.
        n = np.cross(verts[:, 1] - verts[:, 0], verts[:, 2] - verts[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        light = np.array([0.4, -0.5, 0.75])
        shade = 0.7 + 0.3 * np.abs(n @ (light / np.linalg.norm(light)))
        colors[:, :3] *= shade[:, None]
        ax.add_collection3d(Poly3DCollection(verts, facecolors=colors, edgecolors="none"))
        lo, hi = p.min(axis=1), p.max(axis=1)
        mid, span = (lo + hi) / 2, (hi - lo).max() / 2
        ax.set_xlim(mid[0] - span, mid[0] + span)
        ax.set_ylim(mid[1] - span, mid[1] + span)
        ax.set_zlim(mid[2] - span, mid[2] + span)
        ax.set_box_aspect((1, 1, 1))
        elev, azim = views[i] if views else (18, -60)
        ax.view_init(elev=elev, azim=azim)
        ax.set_axis_off()
        ax.set_title(f"{title}\n{r.case}: peak {r.max_vm_away_mpa:.0f} MPa "
                     f"(SF {sy / r.max_vm_away_mpa if r.max_vm_away_mpa else 0:.1f})",
                     fontsize=10, color="#333333")
    sm = plt.cm.ScalarMappable(cmap=matplotlib.colors.LinearSegmentedColormap.from_list(
        "r", cmap(np.linspace(0.1, 0.95, 64))), norm=matplotlib.colors.Normalize(0, limit))
    cb = fig.colorbar(sm, ax=fig.axes, shrink=0.6, pad=0.02)
    cb.set_label("von Mises stress, MPa", color="#333333")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
