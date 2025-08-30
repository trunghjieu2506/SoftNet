from simulation.process_sofa_input import export_sdf_volume_to_sofa
from simulation.plot_scene import cal_resultant_force
from simulation.legacy_vtk_converter import write_legacy_vtk_tetra

def run_simulation(sdf):
    n_cell = sdf.shape[-1]
    voxel_size = 1 / n_cell
    vox = (voxel_size for i in range(3))
    origin = (-0.5, -0.5, -0.5)

    export_sdf_volume_to_sofa(
    sdf,
    (0,0,0),
    voxel_size=vox,
    origin=origin,
    out_dir="workspace/SSLSoftneet/simulation",
    max_cell_circumradius=0.1,
    max_facet_distance=0.02
    )
    import subprocess

    import sys
    cmd = [
    sys.executable,     
    "simulation/legacy_vtk_converter.py", 
    "--in", "/simulation/out_dir/finger.vtu",
    "--out", "/simulation/out_dir/finger_legacy_ascii.vtk"
    ]
    subprocess.run(cmd, check=True)

    # command as a list of strings
    cmd = [
        "/workspace/SSLSoftneet/simulation/SOFA_v25.06.00_Linux/bin/runSofa-25.06.00",
        "-l", "SofaPython3",
        "-g", "batch",
        "-n", "1000",
        "-c", "10",
        "/workspace/SSLSoftneet/simulation/idealscene.py",
    ]
    # run the command
    subprocess.run(cmd, check=True)

    force = cal_resultant_force("simulation/fingerMonitorA_x.txt", "simulation/fingerMonitorA_f.txt")
    return force


    
