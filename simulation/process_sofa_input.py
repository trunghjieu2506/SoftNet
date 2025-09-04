"""
Convert a dense 3D SDF tensor (from SDFusion VQ-VAE) into SOFA-ready meshes:
- finger.vtu    : tetrahedral solid (outer minus cavities)
- outer.stl     : outer surface
- cavity_*.stl  : each cavity wall (normals into the cavity)
- outer_from_tet.stl, cavity_from_tet_*.stl : re-extracted from tet mesh (recommended for SOFA)

Assumptions:
- SDF tensor dec has shape [B, 1, R, R, R], here R=64 per your config.
- Sign convention: negative = solid, positive = air, 0 = surface (set invert_sign=True if opposite)
- You choose physical spacing (m/voxel) so everything is in SI (m, kg, s, Pa).

Usage (example at bottom):
    from sdf_to_sofa import export_sdf_volume_to_sofa
    export_sdf_volume_to_sofa(dec, voxel_size=(0.002,0.002,0.002), origin=(0,0,0), out_dir="out")

Author: adapted for your SDFusion setup.
"""

from __future__ import annotations
import os
import numpy as np
from typing import Tuple, Optional, List

# optional torch import (if your SDF is a torch tensor)
try:
    import torch
except ImportError:
    torch = None

from skimage.measure import marching_cubes, label
import trimesh
import meshio
from pathlib import Path
from scipy.interpolate import RegularGridInterpolator

# pygalmesh uses CGAL; make sure wheels are available for your Python
import pygalmesh

# --------------------------- utilities ---------------------------

def _to_numpy_3d(vol) -> np.ndarray:
    """Accept torch.Tensor [B,1,D,H,W] or numpy [D,H,W]; return float32 [D,H,W]."""
    if torch is not None and isinstance(vol, torch.Tensor):
        if vol.ndim == 5:
            vol = vol[0, 0]  # take the first item/channel
        vol = vol.detach().cpu().numpy()
    vol = np.asarray(vol)
    if vol.ndim == 5:
        vol = vol[0, 0]
    assert vol.ndim == 3, f"Expected 3D array, got shape {vol.shape}"
    return vol.astype(np.float32, copy=False)


def _flood_fill_exterior_air(air: np.ndarray) -> np.ndarray:
    """air: bool 3D; return bool mask of air voxels connected to the grid boundary (exterior)."""
    from collections import deque
    exterior = np.zeros_like(air, dtype=bool)
    # seeds: any air touching boundary
    exterior[0, :, :]  |= air[0, :, :]
    exterior[-1, :, :] |= air[-1, :, :]
    exterior[:, 0, :]  |= air[:, 0, :]
    exterior[:, -1, :] |= air[:, -1, :]
    exterior[:, :, 0]  |= air[:, :, 0]
    exterior[:, :, -1] |= air[:, :, -1]

    q = deque([tuple(idx) for idx in np.argwhere(exterior)])
    visited = exterior.copy()
    shape = air.shape
    while q:
        i, j, k = q.popleft()
        for di, dj, dk in ((1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)):
            ni, nj, nk = i+di, j+dj, k+dk
            if 0 <= ni < shape[0] and 0 <= nj < shape[1] and 0 <= nk < shape[2]:
                if not visited[ni, nj, nk] and air[ni, nj, nk]:
                    visited[ni, nj, nk] = True
                    q.append((ni, nj, nk))
    return visited


def _crop_to_bbox(mask: np.ndarray, pad: int = 2):
    idx = np.argwhere(mask)
    if idx.size == 0:
        return (slice(0,0), slice(0,0), slice(0,0)), (0,0,0)
    lo = np.maximum(idx.min(axis=0) - pad, 0)
    hi = np.minimum(idx.max(axis=0) + pad + 1, mask.shape)
    sl = tuple(slice(lo[d], hi[d]) for d in range(3))
    return sl, tuple(int(lo[d]) for d in range(3))

def _marching_cubes_boolean(mask, spacing, origin):
    # Crop
    sl, lo = _crop_to_bbox(mask)
    sub = mask[sl].astype(np.float32, copy=False)

    # Try GPU (CuMCubes) if available; fallback to skimage
    try:
        import cumcubes, torch
        t = torch.from_numpy(sub).to('cuda' if torch.cuda.is_available() else 'cpu')
        V, F = cumcubes.marching_cubes(t, 0.5)  # 0.5 iso for boolean
        V = V.cpu().numpy() * np.asarray(spacing, float)
    except Exception:
        from skimage.measure import marching_cubes
        V, F, _, _ = marching_cubes(sub, level=0.5, spacing=spacing)

    # shift back to world coords
    offset = origin + np.multiply(lo, spacing)
    V = V + np.asarray(offset, dtype=float)
    return V, F


def _save_trimesh_as_stl(verts, faces, path, flip=False):
    m = trimesh.Trimesh(verts, faces, process=False)
    if flip: m.invert()
    m.export(path)


# --------------------------- main pipeline ---------------------------

from scipy import ndimage as ndi

def _exterior_air_mask(air: np.ndarray) -> np.ndarray:
    # Label all AIR components (26-conn is robust for cavities)
    lbl, n = ndi.label(air, structure=np.ones((3,3,3), dtype=bool))
    if n == 0:
        return np.zeros_like(air, dtype=bool)
    # Any label touching the volume boundary is exterior
    boundary_labels = np.unique(np.concatenate([
        lbl[0, :, :].ravel(), lbl[-1, :, :].ravel(),
        lbl[:, 0, :].ravel(), lbl[:, -1, :].ravel(),
        lbl[:, :, 0].ravel(), lbl[:, :, -1].ravel(),
    ]))
    boundary_labels = boundary_labels[boundary_labels != 0]
    return np.isin(lbl, boundary_labels)

# Replace your adj_to() with a compiled dilation:
_STRUCT_6 = ndi.generate_binary_structure(3, 1)  # 6-neighbors
def _adjacent_solid_to(mask_air_or_cavity: np.ndarray, solid: np.ndarray) -> np.ndarray:
    return solid & ndi.binary_dilation(mask_air_or_cavity, structure=_STRUCT_6)

from scipy import ndimage as ndi

def _label_interior_air(air: np.ndarray, min_voxels: int = 50) -> np.ndarray:
    """Return labeled interior-air (cavities), removing tiny blobs."""
    # seal 1-voxel leaks so cavities don't connect to exterior
    air_closed = ndi.binary_closing(air, structure=np.ones((3,3,3), bool), iterations=1)

    # exterior after closing
    ext = _exterior_air_mask(air_closed)
    interior = air_closed & ~ext

    lbl, n = ndi.label(interior, structure=np.ones((3,3,3), bool))
    if n == 0:
        return lbl

    # remove tiny components
    sizes = np.bincount(lbl.ravel())
    kill = np.where(sizes < min_voxels)[0]  # label 0 is background, fine to include
    if len(kill) > 0:
        mask = ~np.isin(lbl, kill)
        lbl = lbl * mask  # set removed labels to 0
    return lbl

def _label_cavities_from_sdf(sdf: np.ndarray,
                             close_iters: int = 1,
                             min_voxels: int = 100) -> np.ndarray:
    """
    Return a labeled array of interior-air components (cavities).
    We first close the AIR to seal hairline leaks, then flood-fill exterior,
    then label interior components and remove tiny ones.
    """
    solid = sdf < 0.0
    air   = ~solid

    # 1) seal small leaks so cavities don't connect to exterior
    if close_iters > 0:
        air = ndi.binary_closing(air, structure=np.ones((3,3,3), bool),
                                 iterations=close_iters)

    # 2) exterior flood fill on the CLOSED air
    # label all air with 26-connectivity
    lbl_all, n_all = ndi.label(air, structure=np.ones((3,3,3), bool))
    if n_all == 0:
        return np.zeros_like(air, dtype=np.int32)

    # labels touching the grid boundary are exterior
    boundary_labels = np.unique(np.concatenate([
        lbl_all[0, :, :].ravel(), lbl_all[-1, :, :].ravel(),
        lbl_all[:, 0, :].ravel(), lbl_all[:, -1, :].ravel(),
        lbl_all[:, :, 0].ravel(), lbl_all[:, :, -1].ravel(),
    ]))
    boundary_labels = boundary_labels[boundary_labels != 0]
    exterior = np.isin(lbl_all, boundary_labels)

    # 3) interior = air & ~exterior
    interior = air & ~exterior

    # 4) label interior cavities and filter by size
    lbl_int, n_int = ndi.label(interior, structure=np.ones((3,3,3), bool))
    if n_int == 0:
        return lbl_int  # all zeros

    sizes = np.bincount(lbl_int.ravel())
    small = np.where(sizes < min_voxels)[0]  # includes label 0, fine
    if len(small) > 0:
        keep_mask = ~np.isin(lbl_int, small)
        lbl_int = lbl_int * keep_mask

    return lbl_int.astype(np.int32)

def _postprocess_surface(V, F, smooth_iters=5, target_reduction=0.6):
    m = trimesh.Trimesh(V, F, process=False)
    # Taubin smoothing (volume-preserving-ish)
    try:
        from trimesh.smoothing import filter_taubin
        filter_taubin(m, lamb=0.5, nu=-0.53, iterations=smooth_iters)
    except Exception:
        pass
    # Quadratic decimation (reduce faces by ~60%)
    try:
        target_faces = max(1000, int(m.faces.shape[0] * (1.0 - target_reduction)))
        m = m.simplify_quadratic_decimation(target_faces)
    except Exception:
        pass
    m.remove_degenerate_faces()
    m.remove_unreferenced_vertices()
    return np.asarray(m.vertices), np.asarray(m.faces, dtype=np.int64)

import numpy as np

def block_from_sdf_band(grid, dx, dy, dz, band=None, vote=2):
    """
    Build the 'block' (solid + internal air) using only the zero-band logic:
      1) surf = |SDF| <= band            (band defaults to ~0.5 * min voxel step)
      2) fill between first/last surf along X, Y, Z
      3) block = voxels picked by at least `vote` axes (default 2-of-3)
    Inputs:
      grid: (Z,Y,X) SDF samples (same layout as sdf_to_points)
      xs,ys,zs: 1D arrays of world coords for X,Y,Z
    Returns:
      block_zyx: boolean mask (Z,Y,X)
    """
    nz, ny, nx = grid.shape


    if band is None:
        band = 0.5 * min(dx, dy, dz)

    surf = np.abs(grid) <= band
    votes = np.zeros_like(grid, dtype=np.uint8)

    # ---- X-axis fill (work per (z,y) line) ----
    has_x   = surf.any(axis=2)                                   # (Z,Y)
    first_x = np.where(has_x, surf.argmax(axis=2), 0)            # (Z,Y)
    last_x  = np.where(has_x, nx - 1 - surf[:, :, ::-1].argmax(axis=2), 0)
    i = np.arange(nx)[None, None, :]                             # (1,1,X)
    fill_x = has_x[:, :, None] & (i >= first_x[:, :, None]) & (i <= last_x[:, :, None])
    votes += fill_x.astype(np.uint8)

    # ---- Y-axis fill (work per (z,x) line) ----
    has_y   = surf.any(axis=1)                                   # (Z,X)
    first_y = np.where(has_y, surf.argmax(axis=1), 0)            # (Z,X)
    last_y  = np.where(has_y, ny - 1 - surf[:, ::-1, :].argmax(axis=1), 0)
    j = np.arange(ny)[None, :, None]                             # (1,Y,1)
    fill_y = has_y[:, None, :] & (j >= first_y[:, None, :]) & (j <= last_y[:, None, :])
    votes += fill_y.astype(np.uint8)

    # ---- Z-axis fill (work per (y,x) line) ----
    has_z   = surf.any(axis=0)                                   # (Y,X)
    first_z = np.where(has_z, surf.argmax(axis=0), 0)            # (Y,X)
    last_z  = np.where(has_z, nz - 1 - surf[::-1, :, :].argmax(axis=0), 0)
    k = np.arange(nz)[:, None, None]                             # (Z,1,1)
    fill_z = has_z[None, :, :] & (k >= first_z[None, :, :]) & (k <= last_z[None, :, :])
    votes += fill_z.astype(np.uint8)

    # majority vote
    block_zyx = votes >= vote
    return block_zyx


def extract_surfaces_from_sdf(
    sdf: np.ndarray,
    cal_band,
    voxel_size: Tuple[float, float, float],
    origin: Tuple[float, float, float],
    invert_sign: bool = False,
    clean_outer: bool = True
):
    """
    From a dense SDF, produce:
      - outer surface: outer.stl
      - cavity surfaces: [ (verts, faces), ... ]
    Returns: (outer_verts, outer_faces, list_of_cavity_meshes)
    """
    # if invert_sign:
    #     sdf = -sdf
    
    # xs, ys, zs = cal_band
    # dx = xs[1] - xs[0] if xs.size > 1 else 1.0
    # dy = ys[1] - ys[0] if ys.size > 1 else 1.0
    # dz = zs[1] - zs[0] if zs.size > 1 else 1.0
    # band = 0.5 * min(dx, dy, dz)  # thin shell ≈ surface
    # solid/air masks from sign
    exterior_air = sdf <= 0
    interior_air = exterior_air & (sdf >= 0)   # air-side of the surface only

    # air = ~solid
    # grid: (Z,Y,X) SDF; xs,ys,zs: coord arrays
    # block = block_from_sdf_band(sdf, dx, dy, dz, band=None, vote=2)
    # interior_air = block & ~solid
    # exterior_air = air & ~interior_air

    # find exterior air (touches boundary) and interior air (cavities)
    # exterior_air = _exterior_air_mask(air)
    # interior_air = air & ~exterior_air

    # label individual cavities
    # cavity_labels = _label_interior_air(interior_air, min_voxels=100)  # tune threshold
    cavity_labels = _label_cavities_from_sdf(sdf, close_iters=1, min_voxels=100)
    num_cav = int(cavity_labels.max())

    # outer_iface = _adjacent_solid_to(exterior_air, solid)
    # inner_iface = _adjacent_solid_to(interior_air, solid)

    # marching cubes
    outer_V, outer_F = _marching_cubes_boolean(exterior_air, voxel_size, origin)
    outer_V, outer_F = _postprocess_surface(outer_V, outer_F, smooth_iters=4, target_reduction=0.5)
    # inner_V, inner_F = _marching_cubes_boolean(interior_air, voxel_size, origin)
    # inner_V, inner_F = _postprocess_surface(outer_V, outer_F, smooth_iters=4, target_reduction=0.5)

    if clean_outer:
        mout = trimesh.Trimesh(outer_V, outer_F, process=False)  # <- no heavy global processing
        # only the ops you need:
        try:
            mout.remove_degenerate_faces()    # fast
        except Exception:
            pass
        mout.remove_unreferenced_vertices()
        # (hole filling can be very slow; consider skipping unless you really need it)
        outer_V = np.asarray(mout.vertices)
        outer_F = np.asarray(mout.faces, dtype=np.int64)

    # cavity_meshes: List[Tuple[np.ndarray, np.ndarray]] = []
    # for k in range(1, num_cav + 1):
    #     cav = (cavity_labels == k)
    #     cav_iface = _adjacent_solid_to(cav, solid)
    #     if not cav_iface.any():
    #         continue
    #     V, F = _marching_cubes_boolean(cav_iface, voxel_size, origin)
    #     V, F = _postprocess_surface(V, F, smooth_iters=4, target_reduction=0.5)
    #     cavity_meshes.append((V, F))
    return outer_V, outer_F

# def extract_surfaces_from_sdf(
#     sdf: np.ndarray,
#     voxel_size: Tuple[float, float, float],
#     origin: Tuple[float, float, float],
#     invert_sign: bool = False,
#     clean_outer: bool = True
# ):
#     if invert_sign:
#         sdf = -sdf

#     solid = sdf < 0.0
#     air   = ~solid

#     # compute EXTERIOR on closed air to avoid leaks
#     air_closed = ndi.binary_closing(air, structure=np.ones((3,3,3), bool), iterations=1)
#     # exterior after closing
#     lbl_all, _ = ndi.label(air_closed, structure=np.ones((3,3,3), bool))
#     boundary_labels = np.unique(np.concatenate([
#         lbl_all[0, :, :].ravel(), lbl_all[-1, :, :].ravel(),
#         lbl_all[:, 0, :].ravel(), lbl_all[:, -1, :].ravel(),
#         lbl_all[:, :, 0].ravel(), lbl_all[:, :, -1].ravel(),
#     ]))
#     boundary_labels = boundary_labels[boundary_labels != 0]
#     exterior_air = np.isin(lbl_all, boundary_labels)

#     # labeled cavities (robust)
#     cavity_labels = _label_cavities_from_sdf(sdf, close_iters=1, min_voxels=100)
#     num_cav = int(cavity_labels.max())

#     # outer interface: solid adjacent to exterior air
#     outer_iface = _adjacent_solid_to(exterior_air, solid)
#     outer_V, outer_F = _marching_cubes_boolean(outer_iface, voxel_size, origin)

#     if clean_outer:
#         mout = trimesh.Trimesh(outer_V, outer_F, process=False)
#         try: mout.remove_degenerate_faces()
#         except: pass
#         mout.remove_unreferenced_vertices()
#         outer_V = np.asarray(mout.vertices)
#         outer_F = np.asarray(mout.faces, dtype=np.int64)

#     cavity_meshes: List[Tuple[np.ndarray, np.ndarray]] = []
#     for k in range(1, num_cav + 1):
#         cav_mask = (cavity_labels == k)
#         cav_iface = _adjacent_solid_to(cav_mask, solid)   # <-- FIXED: pass solid
#         if not cav_iface.any():
#             continue
#         V, F = _marching_cubes_boolean(cav_iface, voxel_size, origin)
#         cavity_meshes.append((V, F))  # (flip normals later on export if you want)

#     return outer_V, outer_F, cavity_meshes

def tet_mesh_from_sdf(
    sdf: np.ndarray,
    voxel_size: Tuple[float, float, float],
    origin: Tuple[float, float, float],
    out_vtu_path: str,
    max_cell_circumradius: float = 0.012,
    max_facet_distance: float = 0.004
):
    D, H, W = sdf.shape
    xs = origin[0] + np.arange(D) * voxel_size[0]
    ys = origin[1] + np.arange(H) * voxel_size[1]
    zs = origin[2] + np.arange(W) * voxel_size[2]

    interp = RegularGridInterpolator((xs, ys, zs), sdf, bounds_error=False, fill_value=1.0)

    # ---- pygalmesh domain wrapper (new API uses DomainBase) ----
    if hasattr(pygalmesh, "ImplicitDomain"):
        # very old pygalmesh
        class Domain(pygalmesh.ImplicitDomain):
            def __init__(self):
                super().__init__()
                self.bmin = np.array([xs.min(), ys.min(), zs.min()], dtype=np.float64)
                self.bmax = np.array([xs.max(), ys.max(), zs.max()], dtype=np.float64)
            def eval(self, x):
                return float(interp(np.asarray(x).reshape(1, 3))[0])
    else:
        # current pygalmesh API
        class Domain(pygalmesh.DomainBase):
            def __init__(self):
                super().__init__()
                self.bmin = np.array([xs.min(), ys.min(), zs.min()], dtype=np.float64)
                self.bmax = np.array([xs.max(), ys.max(), zs.max()], dtype=np.float64)
                self._center = 0.5 * (self.bmin + self.bmax)
                # radius^2 big enough for the whole box
                self._rad2 = float(np.max((self.bmax - self._center) ** 2))
            def eval(self, x):
                # CGAL expects negative inside
                return float(interp(np.asarray(x).reshape(1, 3))[0])
            # Some versions only need squared radius; others also call center():
            def get_bounding_sphere_center(self):
                return self._center
            def get_bounding_sphere_squared_radius(self):
                return self._rad2

    dom = Domain()

    mesh = pygalmesh.generate_mesh(
        dom,
        # surface fidelity
        min_facet_angle=25.0,
        max_facet_distance=max_facet_distance,
        # volume sizing / quality
        max_cell_circumradius=max_cell_circumradius,
        max_circumradius_edge_ratio=2.0,
        # optional toggles (safe defaults)
        lloyd=False, odt=False, perturb=False, exude=False,
        verbose=False,
    )

    if "tetra" not in mesh.cells_dict:
        raise RuntimeError("No tetra cells produced. Try relaxing meshing parameters.")
    meshio.write(out_vtu_path, mesh)

def reextract_surfaces_from_tet_fast(
    vtu_path: str,
    sdf: np.ndarray,
    voxel_size: Tuple[float, float, float],
    origin: Tuple[float, float, float],
    out_dir: str
):
    """
    Fast & robust re-extraction of outer/cavity surfaces from a tetra mesh:
    - Build all faces from tets
    - Boundary = faces that occur exactly once
    - Split boundary into connected components
    - Classify components via SDF sign at face centroids (air-side => cavity)
    """
    import numpy as np
    import meshio, trimesh

    m = meshio.read(vtu_path)
    pts = m.points
    if "tetra" in m.cells_dict:
        tets = m.cells_dict["tetra"]
    else:
        # gmsh sometimes uses "tetra10" etc.; reduce to linear tets if needed
        for k, arr in m.cells_dict.items():
            if k.startswith("tetra"):
                tets = arr[:, :4]
                break
        else:
            raise RuntimeError("No tetra cells in VTU.")

    # ---- 1) collect all faces (each tet contributes 4 faces) ----
    a, b, c, d = tets[:,0], tets[:,1], tets[:,2], tets[:,3]
    faces = np.vstack([
        np.stack([a, b, c], axis=1),
        np.stack([a, b, d], axis=1),
        np.stack([a, c, d], axis=1),
        np.stack([b, c, d], axis=1),
    ])  # (4*N, 3)

    # sort vertex indices within each face for consistent identification
    faces_sorted = np.sort(faces, axis=1)
    faces_sorted = np.ascontiguousarray(faces_sorted)  # <-- CRITICAL

    # ---- 2) boundary faces = those appearing exactly once ----
    uniq, counts = np.unique(faces_sorted, axis=0, return_counts=True)
    boundary = uniq[counts == 1]  # (M, 3) int

    # ---- 3) split boundary into connected components ----
    # Make a raw mesh for connectivity; disable processing to keep indices intact
    boundary_mesh = trimesh.Trimesh(pts, boundary, process=False)
    components = boundary_mesh.split(only_watertight=False)

    # ---- 4) classify each component via SDF sign at face centroids ----
    outer_faces_all = []
    cavity_meshes = []
    vox = np.asarray(voxel_size, dtype=np.float64)
    ori = np.asarray(origin, dtype=np.float64)

    for comp in components:
        if comp.faces.shape[0] == 0:
            continue
        # face centroids in world coords
        centroids = comp.triangles_center  # (F, 3)

        # map to voxel indices
        ijk = np.floor((centroids - ori) / vox).astype(int)
        ijk[:, 0] = np.clip(ijk[:,0], 0, sdf.shape[0]-1)
        ijk[:, 1] = np.clip(ijk[:,1], 0, sdf.shape[1]-1)
        ijk[:, 2] = np.clip(ijk[:,2], 0, sdf.shape[2]-1)

        # heuristic: if most centroids are on AIR side (sdf>0), it's a cavity wall
        air_ratio = np.mean(sdf[ijk[:,0], ijk[:,1], ijk[:,2]] > 0.0)

        if air_ratio > 0.5:
            # cavity: export each as its own STL (normals inward for SOFA pressure)
            cm = trimesh.Trimesh(comp.vertices, comp.faces, process=True)
            cm.invert()
            cavity_meshes.append(cm)
        else:
            # outer shell: collect faces (we’ll merge and export once)
            outer_faces_all.append(comp.faces)

    # Merge and export outer_from_tet
    if outer_faces_all:
        outer_faces_merge = np.vstack(outer_faces_all)
        out_mesh = trimesh.Trimesh(pts, outer_faces_merge, process=True)
        out_mesh.export(os.path.join(out_dir, "outer_from_tet.stl"))
    else:
        # Fallback: export everything as one outer if classification failed
        fallback = trimesh.Trimesh(pts, boundary, process=True)
        fallback.export(os.path.join(out_dir, "outer_from_tet.stl"))
    
    # Export cavities
    for i, cm in enumerate(cavity_meshes, start=1):
        cm.export(os.path.join(out_dir, f"cavity_from_tet_{i}.stl"))
        print(f"Exported")


def _downsample_sdf(sdf: np.ndarray, factor: int = 1) -> np.ndarray:
    """Downsample by an integer factor using trilinear filtering."""
    if factor <= 1:
        return sdf
    import numpy as np
    from skimage.transform import resize
    D, H, W = sdf.shape
    new_shape = (max(1, D // factor), max(1, H // factor), max(1, W // factor))
    return resize(sdf, new_shape, order=1, anti_aliasing=True, mode="reflect").astype(np.float32)

def export_sdf_volume_to_sofa(
    dec_tensor,
    cal_band,
    voxel_size: Tuple[float, float, float],
    origin: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    out_dir: str = "sofa_assets",
    invert_sign: bool = False,
    clean_outer: bool = True,
    max_cell_circumradius: float = 0.004,
    max_facet_distance: float = 0.001,
) -> None:
    """
    High-level one-call export:
      1) Extract outer & cavity surfaces from SDF and save STLs
      2) Tetrahedralize solid with pygalmesh and save VTU
      3) Re-extract surfaces from tet and save *_from_tet.stl
    """
    os.makedirs(out_dir, exist_ok=True)

    sdf = _to_numpy_3d(dec_tensor)

    # scale your voxel_size if you downsample (so geometry is preserved)
    voxel_ds_factor = 2          # try 2 for ~8× fewer voxels -> huge size/time cut
    if voxel_ds_factor > 1:
        sdf = _downsample_sdf(sdf, voxel_ds_factor)
        voxel_size = tuple(v * voxel_ds_factor for v in voxel_size)

    # 1) surfaces from SDF
    # 1) surfaces (fast MC + crop)
    # outer_V, outer_F= extract_surfaces_from_sdf(
    #     sdf, cal_band, voxel_size, origin, invert_sign=invert_sign, clean_outer=clean_outer
    # )

    # trimesh.Trimesh(outer_V, outer_F, process=False).export(os.path.join(out_dir, "outer.stl"))
    # trimesh.Trimesh(inner_V, inner_F, process=False).export(os.path.join(out_dir, "cavity.stl"))
    
    # 2) tetrahedralize (array-domain fast path)
    vtu_path = os.path.join(out_dir, "finger.vtu")
    tet_mesh_from_sdf(sdf, voxel_size, origin, vtu_path,
                                 max_cell_circumradius=max_cell_circumradius,
                                 max_facet_distance=max_facet_distance)

    # 3) re-extract (vectorized)
    reextract_surfaces_from_tet_fast(vtu_path, sdf, voxel_size, origin, out_dir)

    # print(f"[OK] Exported to: {os.path.abspath(out_dir)}")
    # print("  - finger.vtu")
    # print("  - outer.stl, cavity_*.stl")
    # print("  - outer_from_tet.stl, cavity_from_tet_*.stl  (use these in SOFA for best coupling)")
    

