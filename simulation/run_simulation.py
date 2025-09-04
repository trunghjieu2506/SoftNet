from simulation.process_sofa_input import export_sdf_volume_to_sofa
from simulation.plot_scene import cal_resultant_force
from simulation.legacy_vtk_converter import write_legacy_vtk_tetra
import time


def run_simulation(sdf):
    n_cell = sdf.shape[-1]
    voxel_size = 1 / n_cell
    vox = (voxel_size for i in range(3))
    origin = (-0.5, -0.5, -0.5)

    start = time.time()
    export_sdf_volume_to_sofa(
    sdf,
    (0,0,0),
    voxel_size=vox,
    origin=origin,
    out_dir="simulation/out_dir",
    max_cell_circumradius=0.1,
    max_facet_distance=0.02
    )
    end = time.time()
    print(f"Time taken for tetrahyrdization {end-start} seconds")

    import subprocess

    import sys
    cmd = [
    sys.executable,     
    "simulation/legacy_vtk_converter.py", 
    "--in", "simulation/out_dir/finger.vtu",
    "--out", "simulation/out_dir/finger_legacy_ascii.vtk"
    ]
    subprocess.run(cmd, check=True)
    

    start = time.time()
    # command as a list of strings
    with open("/dev/null", "w") as devnull:
        cmd = [
            "/workspace/SSLSoftneet/simulation/SOFA_v25.06.00_Linux/bin/runSofa-25.06.00",
            "-l", "SofaPython3",
            "-g", "batch",
            "-n", "1000",
            "/workspace/SSLSoftneet/simulation/idealscene.py",
        ]
    
        # Run the command and redirect stdout and stderr to /dev/null
        subprocess.run(cmd, check=True, stdout=devnull, stderr=devnull)
    end = time.time()
    print(f"Time taken for simulation {end-start} seconds")
    force = cal_resultant_force("/workspace/SSLSoftneet/fingerMonitorA_x.txt", "/workspace/SSLSoftneet/fingerMonitorA_f.txt")
    print("force: ", force)
    return force


    
