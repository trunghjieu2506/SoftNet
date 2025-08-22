import numpy as np
import open3d as o3d

pcd = o3d.io.read_point_cloud("/Users/trunghjieu/Desktop/SSLSoftneet/simulation/obj2sdf_utils/isosurf_no_band.ply")  # reads xyz (+ optional rgb)
print(pcd)  # quick stats

# Simple viewer
o3d.visualization.draw_geometries([pcd])

# If you want bigger points + white background:
vis = o3d.visualization.Visualizer()
vis.create_window(width=1280, height=720)
vis.add_geometry(pcd)
opt = vis.get_render_option()
opt.point_size = 3.0
opt.background_color = np.array([1, 1, 1])
vis.run()
