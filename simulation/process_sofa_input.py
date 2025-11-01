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
from typing import Tuple, Optional, List,Union

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
from scipy.ndimage import map_coordinates

from skimage.measure import marching_cubes
# import pyvista as pv


# pygalmesh uses CGAL; make sure wheels are available for your Python
import pygalmesh

# --------------------------- utilities ---------------------------

def _to_numpy_3d(vol) -> np.ndarray:
    """Accept torch.Tensor [B,1,D,H,W] or numpy [D,H,W]; return float32 [D,H,W]."""
    if torch is not None and isinstance(vol, torch.Tensor):
        if vol.ndim == 5:
            vol = vol[0, 0]  # shape becomes [D,H,W]
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

def verify_sdf_cavities(sdf: np.ndarray, voxel_size: Tuple[float, float, float], out_dir: Optional[str] = None):
    """
    Verify whether the SDF input has interior cavities (holes).
    
    Args:
        sdf: 3D SDF array [D,H,W], negative=solid, positive=air
        voxel_size: Physical spacing per voxel
        out_dir: Optional directory to save visualization slices
    
    Returns:
        dict with cavity statistics
    """
    import numpy as np
    from scipy import ndimage as ndi
    
    print("\n" + "="*60)
    print("SDF CAVITY VERIFICATION")
    print("="*60)
    
    # Basic SDF statistics
    print(f"\n📊 SDF Statistics:")
    print(f"  Shape: {sdf.shape}")
    print(f"  Value range: [{sdf.min():.4f}, {sdf.max():.4f}]")
    print(f"  Voxel size: {voxel_size}")
    
    solid = sdf < 0.0
    air = ~solid
    
    solid_count = np.sum(solid)
    air_count = np.sum(air)
    total = sdf.size
    
    print(f"  Solid voxels (SDF < 0): {solid_count:,} ({100*solid_count/total:.1f}%)")
    print(f"  Air voxels (SDF > 0): {air_count:,} ({100*air_count/total:.1f}%)")
    
    # Label all air regions with 26-connectivity
    print(f"\n🔍 Analyzing Air Regions (26-connectivity):")
    air_labels, num_air_regions = ndi.label(air, structure=np.ones((3,3,3), bool))
    print(f"  Total air regions found: {num_air_regions}")
    
    if num_air_regions == 0:
        print("  ⚠️  NO AIR REGIONS - SDF is completely solid!")
        return {"has_cavities": False, "num_cavities": 0, "error": "No air regions"}
    
    # CORRECT METHOD: Flood-fill from boundary to find ALL exterior air
    # (not just air regions touching boundary - handles objects smaller than grid)
    print(f"\n🌊 Flood-Filling Exterior Air from Boundary:")
    exterior_air = _flood_fill_exterior_air(air)
    num_exterior_voxels = np.sum(exterior_air)
    print(f"  Exterior air voxels: {num_exterior_voxels:,} ({100*num_exterior_voxels/air_count:.1f}% of all air)")
    
    # Interior air = all air that flood-fill didn't reach
    interior_air = air & ~exterior_air
    num_interior_voxels = np.sum(interior_air)
    print(f"  Interior air voxels: {num_interior_voxels:,} ({100*num_interior_voxels/air_count:.1f}% of all air)")
    
    if num_interior_voxels == 0:
        print(f"  ✓ All air is reachable from boundary - no enclosed cavities")
        return {
            "has_cavities": False,
            "num_cavities": 0,
            "num_air_regions": num_air_regions,
            "num_exterior_regions": num_air_regions,
            "cavity_details": [],
            "solid_voxels": int(solid_count),
            "air_voxels": int(air_count),
            "sdf_range": [float(sdf.min()), float(sdf.max())],
            "after_closing_cavities": 0
        }
    
    # Label the interior air to find individual cavities
    interior_labels_array, num_cavities = ndi.label(interior_air, structure=np.ones((3,3,3), bool))
    print(f"  Interior cavities found: {num_cavities}")
    
    # Get set of cavity label IDs
    interior_labels = set(range(1, num_cavities + 1))
    
    # Analyze each cavity
    cavity_info = []
    if interior_labels:
        print(f"\n🕳️  Cavity Details:")
        for i, label_id in enumerate(sorted(interior_labels), 1):
            cavity_mask = interior_labels_array == label_id
            cavity_voxels = np.sum(cavity_mask)
            
            # Compute cavity bounding box
            coords = np.argwhere(cavity_mask)
            bbox_min = coords.min(axis=0)
            bbox_max = coords.max(axis=0)
            bbox_size = bbox_max - bbox_min + 1
            
            # Physical dimensions
            phys_size = bbox_size * np.array(voxel_size)
            cavity_volume = cavity_voxels * np.prod(voxel_size)
            
            print(f"  Cavity {i} (label {label_id}):")
            print(f"    Voxels: {cavity_voxels:,}")
            print(f"    Volume: {cavity_volume:.6f} cubic units")
            print(f"    Bounding box (voxels): {bbox_size}")
            print(f"    Physical size: [{phys_size[0]:.4f}, {phys_size[1]:.4f}, {phys_size[2]:.4f}]")
            print(f"    Center (voxel): {(bbox_min + bbox_max) / 2}")
            
            cavity_info.append({
                "label": int(label_id),
                "voxels": int(cavity_voxels),
                "volume": float(cavity_volume),
                "bbox_voxels": bbox_size.tolist(),
                "bbox_physical": phys_size.tolist(),
                "center": ((bbox_min + bbox_max) / 2).tolist()
            })
    
    # Check for potential leaks (cavities connected to exterior via thin channels)
    print(f"\n🔬 Leak Detection (closing test):")
    air_closed = ndi.binary_closing(air, structure=np.ones((3,3,3), bool), iterations=1)
    closed_labels, num_closed = ndi.label(air_closed, structure=np.ones((3,3,3), bool))
    
    closed_boundary = np.unique(np.concatenate([
        closed_labels[0, :, :].ravel(), closed_labels[-1, :, :].ravel(),
        closed_labels[:, 0, :].ravel(), closed_labels[:, -1, :].ravel(),
        closed_labels[:, :, 0].ravel(), closed_labels[:, :, -1].ravel(),
    ]))
    closed_boundary = set(closed_boundary) - {0}
    closed_interior = set(range(1, num_closed + 1)) - closed_boundary
    
    print(f"  After binary closing (sealing 1-voxel gaps):")
    print(f"    Interior cavities: {len(closed_interior)}")
    if len(closed_interior) > len(interior_labels):
        print(f"    ✅ Closing revealed {len(closed_interior) - len(interior_labels)} additional sealed cavities")
        print(f"    → Original cavities likely have small leaks to exterior")
    elif len(closed_interior) < len(interior_labels):
        print(f"    ⚠️  Closing reduced cavities by {len(interior_labels) - len(closed_interior)}")
        print(f"    → Some cavities may be artifacts or very thin")
    else:
        print(f"    ✅ Cavity count unchanged - no detectable leaks")
    
    # Visualize middle slices if output directory provided
    if out_dir and interior_labels:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        
        os.makedirs(out_dir, exist_ok=True)
        
        mid_z, mid_y, mid_x = np.array(sdf.shape) // 2
        
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        
        # SDF slices
        axes[0, 0].imshow(sdf[mid_z, :, :], cmap='RdBu', origin='lower')
        axes[0, 0].set_title(f'SDF Z={mid_z}')
        axes[0, 0].contour(sdf[mid_z, :, :], levels=[0], colors='black', linewidths=2)
        
        axes[0, 1].imshow(sdf[:, mid_y, :], cmap='RdBu', origin='lower')
        axes[0, 1].set_title(f'SDF Y={mid_y}')
        axes[0, 1].contour(sdf[:, mid_y, :], levels=[0], colors='black', linewidths=2)
        
        axes[0, 2].imshow(sdf[:, :, mid_x], cmap='RdBu', origin='lower')
        axes[0, 2].set_title(f'SDF X={mid_x}')
        axes[0, 2].contour(sdf[:, :, mid_x], levels=[0], colors='black', linewidths=2)
        
        # Cavity visualization using interior_labels_array
        axes[1, 0].imshow(interior_labels_array[mid_z, :, :], cmap='tab20', origin='lower')
        axes[1, 0].set_title(f'Cavities Z={mid_z}')
        
        axes[1, 1].imshow(interior_labels_array[:, mid_y, :], cmap='tab20', origin='lower')
        axes[1, 1].set_title(f'Cavities Y={mid_y}')
        
        axes[1, 2].imshow(interior_labels_array[:, :, mid_x], cmap='tab20', origin='lower')
        axes[1, 2].set_title(f'Cavities X={mid_x}')
        
        plt.tight_layout()
        viz_path = os.path.join(out_dir, "sdf_cavity_verification.png")
        plt.savefig(viz_path, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"\n💾 Saved visualization: {viz_path}")
    
    # Summary
    print(f"\n" + "="*60)
    print("SUMMARY:")
    has_cavities = len(interior_labels) > 0
    if has_cavities:
        print(f"✅ SDF HAS {len(interior_labels)} INTERIOR CAVIT{'Y' if len(interior_labels)==1 else 'IES'}")
        total_cavity_voxels = sum(info['voxels'] for info in cavity_info)
        print(f"   Total cavity volume: {sum(info['volume'] for info in cavity_info):.6f} cubic units")
        print(f"   Total cavity voxels: {total_cavity_voxels:,} ({100*total_cavity_voxels/air_count:.1f}% of all air)")
    else:
        print(f"❌ SDF HAS NO INTERIOR CAVITIES")
        print(f"   All air regions connect to the boundary (exterior air only)")
    print("="*60 + "\n")
    
    return {
        "has_cavities": has_cavities,
        "num_cavities": len(interior_labels),
        "num_air_regions": num_air_regions,
        "num_exterior_voxels": int(num_exterior_voxels),
        "num_interior_voxels": int(num_interior_voxels),
        "cavity_details": cavity_info,
        "solid_voxels": int(solid_count),
        "air_voxels": int(air_count),
        "sdf_range": [float(sdf.min()), float(sdf.max())],
        "after_closing_cavities": len(closed_interior)
    }


def tet_mesh_from_sdf(
    sdf: np.ndarray,
    voxel_size: Tuple[float, float, float],
    origin: Tuple[float, float, float],
    out_vtu_path: str,
    max_cell_circumradius: float = 0.012,
    max_facet_distance: float = 0.004
):
    import numpy as np
    import pygalmesh

    # D, H, W = sdf.shape
    # xs = origin[0] + np.arange(D) * voxel_size[0]
    # ys = origin[1] + np.arange(H) * voxel_size[1]
    # zs = origin[2] + np.arange(W) * voxel_size[2]

    # # pygalmesh expects continuous implicit function while our input sdf is discrete grid. Thus, using Interpolator.
    # interp = RegularGridInterpolator((xs, ys, zs), sdf, bounds_error=False, fill_value=1.0)

    # class Domain(pygalmesh.DomainBase):
    #     def __init__(self):
    #         super().__init__()
    #         self.bmin = np.array([xs.min(), ys.min(), zs.min()], dtype=np.float64)
    #         self.bmax = np.array([xs.max(), ys.max(), zs.max()], dtype=np.float64)
    #         self._center = 0.5 * (self.bmin + self.bmax)
    #         self._rad2 = float(np.max((self.bmax - self._center) ** 2))
    #         # half = self.bmax - self._center        # (hx, hy, hz)
    #         # self._rad2 = float(np.dot(half, half)) # hx^2 + hy^2 + hz^2
    #     def eval(self, x):
    #         return float(interp(np.asarray(x).reshape(1, 3))[0])
    #     # Some versions only need squared radius; others also call center():
    #     def get_bounding_sphere_center(self):
    #         return self._center
    #     def get_bounding_sphere_squared_radius(self):
    #         return self._rad2
    # dom = Domain()

    class Domain(pygalmesh.DomainBase):
        """
        SDF grid domain with fast trilinear eval using scipy.ndimage.map_coordinates.
        sdf: np.ndarray [D,H,W], negative=inside
        voxel_size: (sx, sy, sz) in world units
        origin: (ox, oy, oz) world coords of sdf[0,0,0]
        """
        def __init__(self, sdf: np.ndarray, voxel_size, origin):
            super().__init__()
            assert sdf.ndim == 3
            self.sdf = np.asarray(sdf, dtype=np.float32)
            self.sx, self.sy, self.sz = map(float, voxel_size)
            self.ox, self.oy, self.oz = map(float, origin)

            # world AABB in case you want a quick “outside → +1” check (optional)
            D, H, W = self.sdf.shape
            self.bmin = np.array([self.ox, self.oy, self.oz], dtype=float)
            self.bmax = self.bmin + np.array([(D-1)*self.sx, (H-1)*self.sy, (W-1)*self.sz])

            # bounding sphere (center + radius^2) that encloses the whole box
            self._center = 0.5*(self.bmin + self.bmax)
            half = self.bmax - self._center
            self._rad2 = float(np.dot(half, half))   # encloses the whole box

        def _world_to_index(self, x):
                # map world → fractional index in [0..D-1], [0..H-1], [0..W-1]
                # IMPORTANT: we assume sdf[i,j,k] corresponds to world (x,y,z) with:
                # x = ox + i*sx, y = oy + j*sy, z = oz + k*sz
                i = (x[0] - self.ox)/self.sx
                j = (x[1] - self.oy)/self.sy
                k = (x[2] - self.oz)/self.sz
                return i, j, k

        def eval(self, x):
            x = np.asarray(x, dtype=float)
            # OPTIONAL quick outside test (free speed): if well outside AABB, return +1.0
            if (x < self.bmin).any() or (x > self.bmax).any():
                return 1.0

            i, j, k = self._world_to_index(x)
            # Use constant mode with cval=+1 to match your old fill_value behavior.
            val = map_coordinates(
                self.sdf,
                coordinates=np.array([[i], [j], [k]]),
                order=1,        # trilinear
                mode="constant",
                cval=1.0,
                prefilter=False # faster; fine for order=1
            )[0]
            return float(val)

        def get_bounding_sphere_center(self):
            return self._center

        def get_bounding_sphere_squared_radius(self):
            return self._rad2 

    dom = Domain(sdf, voxel_size, origin)

    mesh = pygalmesh.generate_mesh(
        dom,
        # surface fidelity
        min_facet_angle=25.0,
        max_facet_distance=max_facet_distance,
        # volume sizing / quality
        max_cell_circumradius=max_cell_circumradius,
        max_circumradius_edge_ratio=2.0,
        ## mesh-gen optimisation options, set True if needed, but slower 
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
    - Split boundary into watertight connected components
    - Pick the largest volume component as the outer shell
    - For each cavity component:
        * quickly fix triangle winding (consistent orientation)
        * flip normals inward (needed for pressure BCs)
        * export as STL
    Notes on speed:
      - We avoid global 'process=True' on the big boundary mesh.
      - We only run small repairs on each per-component mesh just before export.
    """
    import os
    import numpy as np
    import meshio, trimesh

    os.makedirs(out_dir, exist_ok=True)

    m = meshio.read(vtu_path)
    pts = m.points
    if "tetra" in m.cells_dict:
        tets = m.cells_dict["tetra"]
    else:
        # Some generators write tetra10 etc.; if you need that, adapt here
        raise RuntimeError("No tetra cells in VTU.")

    # ---- 1) collect all faces (each tet contributes 4 faces) ----
    a, b, c, d = tets[:, 0], tets[:, 1], tets[:, 2], tets[:, 3]  # (N,) int

    faces = np.vstack([
        np.stack([a, b, c], axis=1),
        np.stack([a, b, d], axis=1),
        np.stack([a, c, d], axis=1),
        np.stack([b, c, d], axis=1),
    ])  # (4*N, 3)

    # sort vertices within each face so shared faces match exactly
    faces_sorted = np.ascontiguousarray(np.sort(faces, axis=1))

    # ---- 2) boundary/surface faces = those appearing exactly once ----
    uniq, counts = np.unique(faces_sorted, axis=0, return_counts=True)
    boundary = uniq[counts == 1]  # (M,3) int indices into pts

    # ---- 3) split boundary into connected components (no global processing)
    boundary_mesh = trimesh.Trimesh(pts, boundary, process=False)
    components = boundary_mesh.split(only_watertight=True)  # pneumatic parts need closed shells

    if len(components) == 0:
        return False
    # ---- 4) choose outer shell by volume (fallback to bbox volume)
    comp_metrics = []
    for comp in components:
        if comp.faces.size == 0:
            comp_metrics.append(0.0)
            continue
        # Using signed volume requires consistent winding; for speed, use bbox as a fallback metric
        if getattr(comp, "is_volume", False):
            # comp.is_volume is True when watertight + consistent winding
            metric = abs(comp.volume)
        else:
            # fast fallback if winding not yet consistent
            ext = comp.bounds[1] - comp.bounds[0]
            metric = float(np.prod(ext))
        comp_metrics.append(metric)

    outer_idx = int(np.argmax(comp_metrics))

    # ---- 5) export cavity components with fixed winding + inward normals
    from trimesh import repair

    cavity_count = 0
    for i, comp in enumerate(components):
        if comp.faces.size == 0 or i == outer_idx:
            continue

        # --- Minimal, fast cleanup (local to this component)
        # don't rezero or recenter: we want to keep world coordinates intact
        comp.remove_duplicate_faces()
        comp.remove_degenerate_faces()
        comp.remove_unreferenced_vertices()
        comp.merge_vertices()  # weld near-duplicates (fast)

        # --- Make triangle winding globally consistent
        # (This fixes the "winding_consistent=False" issue and gives meaningful signed volume.)
        repair.fix_winding(comp)

        # --- For cavity pressure: normals should point INWARD.
        # Convention: for a closed shell, positive volume => outward normals.
        # Flip if needed so we end up with inward normals.
        if comp.volume < 0:
            comp.invert()

        # Export
        cavity_count += 1
        comp.export(os.path.join(out_dir, f"cavity_from_tet_{cavity_count}.stl"))
    return True
    # print(f"[reextract] cavities exported: {cavity_count}")


def _downsample_sdf(sdf: np.ndarray, factor: int = 1) -> np.ndarray:
    """Downsample by an integer factor using trilinear filtering."""
    if factor <= 1:
        return sdf
    import numpy as np
    from skimage.transform import resize
    D, H, W = sdf.shape
    new_shape = (max(1, D // factor), max(1, H // factor), max(1, W // factor))
    return resize(sdf, new_shape, order=1, anti_aliasing=True, mode="reflect").astype(np.float32)

# def sdf_to_tets(
#     sdf: Union[np.ndarray, "torch.Tensor"],
#     level: float = 0.0,
#     voxel_size: Tuple[float, float, float] = (1.0, 1.0, 1.0),
#     origin: Tuple[float, float, float] = (0.0, 0.0, 0.0),
#     *,
#     # TetGen quality/behavior knobs (tune for speed/quality)
#     order: int = 1,
#     minratio: float = 1.5,        # quality target (bigger => stricter; slower)
#     mindihedral: float = 20.0,    # minimum dihedral angle
#     steinerleft: int = 100000,    # extra points TetGen may insert; -1 = unlimited
#     verbose: bool = False,
# ):
#     """
#     Convert an SDF volume (D,H,W) into a tetrahedral mesh using marching cubes + TetGen.

#     Returns:
#         grid: pyvista.UnstructuredGrid (tet mesh). Use grid.points and grid.cells to access arrays.
#     """
#     # ---- 1) Normalize input to a 3D NumPy array ----
#     if torch is not None and isinstance(sdf, torch.Tensor):
#         sdf_np = sdf.detach().cpu().numpy()
#     else:
#         sdf_np = np.asarray(sdf)
#     assert sdf_np.ndim == 3, f"Expected [D,H,W], got {sdf_np.shape}"

#     # ---- 2) Extract surface with marching cubes at 'level' ----
#     # spacing maps voxel indices to world units (e.g., mm). marching_cubes returns vertices in world coords if spacing is given.
#     verts, faces, _, _ = marching_cubes(sdf_np, level=level, spacing=voxel_size)
#     # apply origin offset (so the mesh sits in your world frame)
#     verts = verts + np.array(origin, dtype=float)

#     # ---- 3) Build a PyVista surface (faces must be encoded as [3, i, j, k] per triangle) ----
#     faces_encoded = np.hstack([np.full((faces.shape[0], 1), 3, dtype=np.int64), faces]).ravel()
#     surf = pv.PolyData(verts, faces_encoded)
#     # (optional) clean/triangulate just in case
#     surf = surf.triangulate().clean()

#     # ---- 4) Tetrahedralize with TetGen ----
#     tet = tetgen.TetGen(surf)
#     tet.tetrahedralize(
#         order=order,
#         minratio=minratio,
#         mindihedral=mindihedral,
#         steinerleft=steinerleft,
#         verbose=verbose,
#     )
#     grid = tet.grid  # pyvista.UnstructuredGrid containing tets
#     return grid

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
    verify_cavities: bool = True, 
) -> None:
    """
    High-level one-call export:
      1) Verify SDF has cavities (optional)
      2) Extract outer & cavity surfaces from SDF and save STLs
      3) Tetrahedralize solid with pygalmesh and save VTU
      4) Re-extract surfaces from tet and save *_from_tet.stl
    """
    os.makedirs(out_dir, exist_ok=True)

    sdf = _to_numpy_3d(dec_tensor) #convert input sdf to numpy array & ensure shape is [D,H,W]

    # downsample the sdf to speed up meshing (optional)
    voxel_ds_factor = 1          
    if voxel_ds_factor > 1:
        sdf = _downsample_sdf(sdf, voxel_ds_factor)
        voxel_size = tuple(v * voxel_ds_factor for v in voxel_size)
    
    # VERIFY CAVITIES BEFORE MESHING
    if verify_cavities:
        cavity_stats = verify_sdf_cavities(sdf, voxel_size, out_dir=out_dir)
        if not cavity_stats["has_cavities"]:
            return False

    # # 1) surfaces from SDF
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
    # mesh = sdf_to_tets(sdf=sdf, voxel_size=voxel_size, origin=origin)
    # pv.save_meshio(vtu_path, mesh)


    # 3) re-extract (vectorized)
    is_watertight = reextract_surfaces_from_tet_fast(vtu_path, sdf, voxel_size, origin, out_dir)
    if not is_watertight:
        return False
    return True
    # print(f"[OK] Exported to: {os.path.abspath(out_dir)}")
    # print("  - finger.vtu")
    # print("  - outer.stl, cavity_*.stl")
    # print("  - outer_from_tet.stl, cavity_from_tet_*.stl  (use these in SOFA for best coupling)")
    

