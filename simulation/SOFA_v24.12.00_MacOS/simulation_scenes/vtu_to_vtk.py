import meshio
m = meshio.read("/Users/trunghjieu/Downloads/SOFA_v24.12.00_MacOS/simulation_scenes/sofa_asset/finger.vtu")
meshio.write("/Users/trunghjieu/Downloads/SOFA_v24.12.00_MacOS/simulation_scenes/sofa_asset/finger.vtk", m, file_format="vtk")
