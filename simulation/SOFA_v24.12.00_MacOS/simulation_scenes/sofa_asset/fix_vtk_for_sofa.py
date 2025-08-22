# fix_vtk_for_sofa.py
import argparse, meshio, numpy as np, os, sys

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", required=True, help="input .vtu or .vtk")
    ap.add_argument("--out", dest="out", required=False, help="output legacy ASCII .vtk")
    args = ap.parse_args()

    m = meshio.read(args.inp)

    if "tetra" not in m.cells_dict or len(m.cells_dict["tetra"]) == 0:
        print("[ERROR] input has no tetra cells. Found:", list(m.cells_dict.keys()))
        sys.exit(1)

    # keep only tets for the volume file; (surfaces stay as STL in SOFA)
    pts = m.points.astype(np.float32, copy=False)
    tets = m.cells_dict["tetra"].astype(np.int32, copy=False)

    # minimal mesh with just tets
    mm = meshio.Mesh(
        points=pts,
        cells=[("tetra", tets)],
        point_data={},   # drop extra data to avoid parser confusion
        cell_data={}
    )

    out = args.out or os.path.splitext(args.inp)[0] + "_ascii.vtk"
    meshio.write(out, mm, file_format="vtk-ascii")
    print("[OK] wrote legacy ASCII VTK:", out)

    # sanity print
    chk = meshio.read(out)
    print(f"[CHECK] points={len(chk.points)}, tets={len(chk.cells_dict.get('tetra', []))}")

if __name__ == "__main__":
    main()