from pathlib import Path
import numpy as np
import trimesh

# ------------------------------------------------------------
# Helper: load OBJ and return either a Trimesh or PointCloud
# ------------------------------------------------------------
def load_any_obj(path, color=(200, 200, 200, 255), point_size=5.0):
    """
    1) Try to read as a normal triangular mesh.
    2) If it has zero faces, read vertices and return PointCloud instead.
    """
    mesh = trimesh.load(path, force='mesh', skip_materials=True, process=False)

    if isinstance(mesh, trimesh.Trimesh) and len(mesh.faces):
        mesh.visual.vertex_colors = color
        return mesh

    # -------- No faces: build a PointCloud ----------
    # (trimesh already parsed vertices for us)
    verts = np.asarray(mesh.vertices) if hasattr(mesh, "vertices") else np.zeros((0, 3))
    if verts.size == 0:                                  # very old trimesh versions
        verts = np.loadtxt(path, comments='#', usecols=(1, 2, 3))

    cloud = trimesh.points.PointCloud(verts, color=np.array(color))
    cloud.metadata["point_size"] = float(point_size)     # pyglet respects this
    return cloud


# ------------------- Load your two files --------------------
mesh_a = load_any_obj("/Users/trunghjieu/Desktop/SSLSoftneet/simulation/pc_norm.obj", color=(220, 50, 50, 255))   # vertex-only OBJ

# move the second model so they don’t overlap
# mesh_b.apply_translation([2.0, 0.0, 0.0])

# --------------------- Display them -------------------------
scene = trimesh.Scene([mesh_a])
scene.show()        # interactive window – rotate, zoom, take screenshots