# sofa_live_runner.py
import os, sys, time 

sys.path.append("/workspace/SSLSoftneet/simulation/SOFA_v25.06.00_Linux/plugins/SofaPython3/lib/python3/site-packages")
sys.path.append("/SSLSoftneet")
sys.path.append("SSLSoftneet/simulation")
os.environ["OMP_NUM_THREADS"] = "12"

import time
from pathlib import Path

import numpy as np
import meshio
import math

# --- SOFA (SofaPython3) ---
import Sofa

# your pipeline bits
from simulation.process_sofa_input import export_sdf_volume_to_sofa
from simulation.plot_scene import cal_resultant_force
from simulation.legacy_vtk_converter import convert_vtu_to_legacy_vtk

# ------------------------------------------------------------
# Helpers
# ------------------------------------------------------------
def _ensure_dir(p):
    Path(p).mkdir(parents=True, exist_ok=True)

def choose_base_mid_tip_from_vtk(vtk_path: str,
                                base_roi_box=None  # [xmin, ymin, zmin, xmax, ymax, zmax] or None
                                ) -> tuple[int, int, int]:
    """
    Read a VTK/VTU mesh and choose base/mid/tip indices automatically.
    Strategy:
        - PCA on all points to get main axis.
        - If base_roi_box is provided, pick base from points inside the box.
        - Tip = max projection on the axis.
        - Mid = point whose projection is closest to midway between base & tip.
    """
    # 1) Load points from VTK/VTU via meshio (works with ASCII legacy .vtk and XML .vtu)
    mesh = meshio.read(vtk_path)
    P = np.asarray(mesh.points, dtype=float)  # shape (N, 3)
    if P.ndim != 2 or P.shape[1] != 3 or P.shape[0] < 3:
        raise ValueError(f"Unexpected mesh points shape {P.shape} in {vtk_path}")

    # 2) PCA (principal axis)
    C = P.mean(axis=0)
    X = P - C
    # SVD for principal direction (first right-singular vector)
    _, _, Vt = np.linalg.svd(X, full_matrices=False)
    axis = Vt[0]
    axis /= (np.linalg.norm(axis) + 1e-12)

    # 3) Project all nodes onto the principal axis
    proj = X @ axis  # shape (N,)

    # 4) Base selection (prefer a provided ROI if any)
    def inside_box(points, box):
        xmin, ymin, zmin, xmax, ymax, zmax = box
        return np.where(
            (points[:, 0] >= xmin) & (points[:, 0] <= xmax) &
            (points[:, 1] >= ymin) & (points[:, 1] <= ymax) &
            (points[:, 2] >= zmin) & (points[:, 2] <= zmax)
        )[0]

    if base_roi_box is not None:
        cand = inside_box(P, base_roi_box)
        if cand.size > 0:
            base_idx = int(cand[np.argmin(proj[cand])])
        else:
            # Fallback if ROI was empty (bad box or different units)
            base_idx = int(np.argmin(proj))
    else:
        base_idx = int(np.argmin(proj))

    # 5) Tip = global maximum projection
    tip_idx = int(np.argmax(proj))

    # 6) Mid = closest to halfway projection
    mid_target = 0.5 * (proj[base_idx] + proj[tip_idx])
    mid_idx = int(np.argmin(np.abs(proj - mid_target)))

    # 7) Ensure distinct indices (rare but possible on coarse meshes)
    if len({base_idx, mid_idx, tip_idx}) < 3:
        mask = np.ones(P.shape[0], dtype=bool)
        mask[[base_idx, tip_idx]] = False
        # Re-pick mid from remaining nodes
        mid_idx = int(np.where(mask)[0][np.argmin(np.abs(proj[mask] - mid_target))])

    return base_idx, mid_idx, tip_idx

# ------------------------------------------------------------
# Scene builder (self-contained, no external idealscene.py)
#   - If you prefer using your existing idealscene.py, you can import it
#     and call createScene() instead; this inline version avoids file deps.
# ------------------------------------------------------------
def simulation_settup(dt=1e-3):
    rootNode = Sofa.Core.Node("root")
    rootNode.dt = dt
    rootNode.addObject('VisualStyle', displayFlags='showForceFields showBehaviorModels')
    rootNode.addObject('RequiredPlugin', pluginName='SoftRobots SofaPython3')

    rootNode.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop')  # Needed to use components [FreeMotionAnimationLoop]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction')  # Needed to use components [LinearSolverConstraintCorrection]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Solver')  # Needed to use components [GenericConstraintSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Engine.Select')  # Needed to use components [BoxROI]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh')  # Needed to use components [MeshSTLLoader,MeshVTKLoader]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct')  # Needed to use components [SparseLDLSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear')  # Needed to use components [BarycentricMapping]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mass')  # Needed to use components [UniformMass]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward')  # Needed to use components [EulerImplicitSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic')  # Needed to use components [TetrahedronFEMForceField]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.Spring')  # Needed to use components [RestShapeSpringsForceField]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.StateContainer')  # Needed to use components [MechanicalObject]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant')  # Needed to use components [MeshTopology]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Visual')  # Needed to use components [VisualStyle]  
    rootNode.addObject('RequiredPlugin', name='Sofa.GUI.Component')  # Needed to use components [AttachBodyButtonSetting] 
    rootNode.addObject('RequiredPlugin', name='SofaValidation')  # Needed to use components [AttachBodyButtonSetting] 

    rootNode.gravity.value = [-9810, 0, 0]
    rootNode.addObject('AttachBodyButtonSetting', stiffness=10)
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-7, maxIterations=1000)
    return rootNode



def build_scene(
    rootNode,
    vol_vtk: str,
    cavity_stl: str,
    monitor_node ="0 60 120",
    dt: float = 1e-3,
    young: float = 500,
    nu: float = 0.3,
    pressure_init: float = 1,
):
    finger = rootNode.addChild('Finger')
    finger.addObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
    finger.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixd')
    finger.addObject('MeshVTKLoader', name='loader', filename=vol_vtk)
    finger.addObject('MeshTopology', src='@loader', name='container')
    finger.addObject('MechanicalObject', name='tetras', template='Vec3', showObject=True, showObjectScale=1)
    finger.addObject('Monitor', name="fingerMonitor",
                    template="Vec3d", listening=True, indices="0 60 120",
                    ExportPositions=True, positionFile="fingerMonitorA_x.txt")
    finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                     youngModulus=500)
    finger.addObject('UniformMass', totalMass=0.04)
    boxROISubTopo = finger.addObject('BoxROI', name='boxROISubTopo', box=[-100, 22.5, -8, -19, 28, 8], strict=False)
    boxROI = finger.addObject('BoxROI', name='boxROI', box=[-10, 0, -20, 0, 30, 20], drawBoxes=True)
    finger.addObject('RestShapeSpringsForceField', points=boxROI.indices.linkpath, stiffness=1e12, angularStiffness=1e12)
    finger.addObject('GenericConstraintCorrection')

    modelSubTopo = finger.addChild('SubTopology')
    modelSubTopo.addObject('MeshTopology', position='@loader.position', tetrahedra=boxROISubTopo.tetrahedraInROI.linkpath,
                           name='container')
    modelSubTopo.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                           youngModulus=1500)

    cavity = finger.addChild('Cavity')
    cavity.addObject('MeshSTLLoader', name='cavityLoader', filename=cavity_stl)
    cavity.addObject('MeshTopology', src='@cavityLoader', name='cavityMesh')
    cavity.addObject('MechanicalObject', name='cavity')
    spc = cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=1,
                     triangles='@cavityMesh.triangles', valueType='pressure')
    cavity.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)
    return finger, spc


class SofaLiveRunner:
    """
    Keep SOFA alive across calls:
      - One Sofa runtime (plugins loaded once)
      - Rebuild/replace only the Finger subtree when meshes change
    """
    def __init__(self, out_dir="simulation/out_dir", dt=1e-3):
        self.out_dir = out_dir
        _ensure_dir(out_dir)
        self.finger = None
        self.spc = None
        self.dt = dt
        self.root = simulation_settup(dt=dt)


    def _build_or_replace_scene(self, vol_vtk, cavity_stl, monitor_nodes):
        old_assets = self.root.getChild('Finger') 
        if old_assets is None:
            self.finger, self.spc = build_scene(
                root=self.root,
                vol_vtk=vol_vtk,
                cavity_stl=cavity_stl,
                monitor_nodes=monitor_nodes,
                dt=self.dt,
            )
            Sofa.Simulation.initRoot(self.root)
        else:
            self.root.removeChild(old_assets)
            self.finger, self.spc = build_scene(
                root=self.root,
                vol_vtk=vol_vtk,
                cavity_stl=cavity_stl,
                monitor_nodes=monitor_nodes,
                dt=self.dt,
            )
            Sofa.Simulation.initRoot(self.root)

    def max_bend_angle(self, pos_file):
        A = np.loadtxt(pos_file)           # angleRefs_x.txt written by Monitor
        # Columns: time, (x,y,z for node0), (x,y,z for node1), (x,y,z for node2)
        t  = A[:,0]
        p0 = A[:,1:4] 
        p1 = A[:,4:7]
        p2 = A[:,7:10]

        v1 = p1 - p0      # base->mid
        v2 = p2 - p0      # base->tip
        n1 = np.linalg.norm(v1, axis=1)
        n2 = np.linalg.norm(v2, axis=1)
        cos = np.clip(np.sum(v1*v2, axis=1) / (n1*n2), -1.0, 1.0)
        ang = np.arccos(cos)               # radians
        k   = int(np.argmax(ang))
        return {
            "max_angle_rad": float(ang[k]),
            "max_angle_deg": float(math.degrees(ang[k])),
            "time_s_at_max": float(t[k])
    }

    def run_sdf(self, sdf: np.ndarray,
                voxel_size=(0.002, 0.002, 0.002),
                origin=(0.0, 0.0, 0.0),
                n_steps=200,
                pressure=None):
        """
        - Export meshes for this SDF
        - Convert VTU->VTK legacy ASCII (SOFA-friendly)
        - Replace (or build) the Finger subtree with new mesh paths
        - Advance simulation n_steps, return resultant force (via your helper)
        """
        # 1) Export SOFA assets for this sdf
        has_cavity = export_sdf_volume_to_sofa(
            sdf,
            (0,0,0),
            voxel_size=tuple(voxel_size),       
            origin=tuple(origin),
            out_dir=self.out_dir,
            max_cell_circumradius=0.1,
            max_facet_distance=0.02
        )
        if not has_cavity:
            return 0
        # 2) Convert VTU -> VTK legacy ASCII (in-process)
        vtu = os.path.join(self.out_dir, "finger.vtu")
        vtk = os.path.join(self.out_dir, "finger_legacy_ascii.vtk")
        convert_vtu_to_legacy_vtk(vtu, vtk)
        # 3) Point scene to new meshes (keep runtime alive)
        cavity = os.path.join(self.out_dir, "cavity_from_tet_1.stl")
        base, mid, tip = choose_base_mid_tip_from_vtk(vtk)
        monitor_nodes = f"{base} {mid} {tip}"
        self._build_or_replace_scene(vtk, cavity, monitor_nodes)

        # Optional: set pressure for this episode
        if pressure is not None:
            self.spc.value = [float(pressure)]

        # 4) Advance steps
        t0 = time.time()
        Sofa.Simulation.animateNSteps(root_node = self.root, n_steps = n_steps, dt = self.root.dt.value)
        t1 = time.time()
        print(f"Simulated {n_steps} steps in {t1-t0:.3f}s")
        print(f"Tetrahydrization Time {t3-t2}s")

        # 5) Read monitor files (or read from component directly to avoid disk)
        pos_file = os.path.join(os.getcwd(), "fingerMonitorA_x.txt")
        angle = self.max_bend_angle(pos_file)
        # frc_file = os.path.join(os.getcwd(), "fingerMonitorA_f.txt")
        # force = cal_resultant_force(pos_file, frc_file)
        return angle

# ------------------------------------------------------------
# Convenience function matching your previous API
# ------------------------------------------------------------
def run_simulation_keepalive(runner: SofaLiveRunner, sdf: np.ndarray, n_steps=1000):
    n_cell = sdf.shape[-1]
    voxel_size = (1.0/n_cell, 1.0/n_cell, 1.0/n_cell)  # BUGFIX: tuple, not a generator
    origin = (-0.5, -0.5, -0.5)
    return runner.run_sdf(
        sdf=sdf,
        voxel_size=voxel_size,
        origin=origin,
        n_steps=n_steps
    )


# def simulation_settup(dt=1e-3):
#     root = Sofa.Core.Node("root")
#     root.dt = dt
#     # root.addObject('RequiredPlugin', name='MultiThreading') # Needed to use components [ParallelBVHNarrowPhase,ParallelBruteForceBroadPhase,ParallelCGLinearSolver,ParallelTetrahedronFEMForceField]  
#     root.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction') # Needed to use components [GenericConstraintCorrection]  
#     root.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Solver') # Needed to use components [GenericConstraintSolver]  
#     root.addObject('RequiredPlugin', name='Sofa.Component.Engine.Select') # Needed to use components [BoxROI]  
#     root.addObject('RequiredPlugin', name='Sofa.Component.Setting') # Needed to use components [BackgroundSetting]  
#     root.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.Spring') # Needed to use components [RestShapeSpringsForceField]  
#     root.addObject('RequiredPlugin', name='Sofa.Component.StateContainer') # Needed to use components [MechanicalObject]  
#     root.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant') # Needed to use components [MeshTopology]  
#     root.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D') # Needed to use components [OglModel,OglSceneFrame]  
    
#     root.addObject('RequiredPlugin', pluginName='SofaPython3 SoftRobots')
#     root.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop')
#     root.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh')
#     root.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Dynamic')   # TetrahedronSetTopologyContainer
#     root.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic')
#     root.addObject('RequiredPlugin', name='Sofa.Component.Mass')
#     root.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct')
#     root.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward')
#     root.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear')
#     root.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Algorithm')
#     root.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Intersection')
#     root.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry')
#     root.addObject('RequiredPlugin', name='Sofa.Component.Collision.Response.Contact')
#     root.addObject('RequiredPlugin', name='Sofa.Component.Visual')
#     root.addObject('RequiredPlugin', name='SofaValidation')  
#     root.addObject('RequiredPlugin', name='Sofa.GUI.Component') # Needed to use components [AttachBodyButtonSetting]  
#     root.addObject("RequiredPlugin", name='Sofa.Component.LinearSolver.Preconditioner')
#     root.addObject('AttachBodyButtonSetting', stiffness=10)
#     root.addObject('GenericConstraintSolver', tolerance=1e-7, maxIterations=1000)
#     root.gravity = [0.0, -9.81, 0.0]
#     root.addObject('FreeMotionAnimationLoop')
#     root.addObject('FreeMotionAnimationLoop')
#     root.addObject('GenericConstraintSolver', tolerance=1e-7, maxIterations=1000)
#     root.gravity = [0.0, -9.81, 0.0]
#     # root.addObject('CollisionPipeline')
#     # root.addObject('ParallelBruteForceBroadPhase')
#     # root.addObject('ParallelBVHNarrowPhase')
#     # rootNode.addObject('BruteForceBroadPhase')
#     # rootNode.addObject('BVHNarrowPhase')
#     root.addObject('LocalMinDistance', name='Proximity', alarmDistance=1.0, contactDistance=0.5)
#     # root.addObject('CollisionResponse', response='FrictionContactConstraint', responseParams='mu=0.6')

#     root.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1.0])
#     root.addObject('OglSceneFrame', style='Arrows', alignment='TopRight')

#     return root

# def build_scene(
#     root,
#     vol_vtk: str,
#     outer_stl: str,
#     cavity_stl: str,
#     dt: float = 1e-3,
#     young: float = 500,
#     nu: float = 0.3,
#     pressure_init: float = 1,
# ):
#     # ---- Finger subtree
#     finger = root.addChild('Finger')
#     finger.addObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
#     finger.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixd')

#     finger.addObject('MeshVTKLoader', name='volLoader', filename=vol_vtk)
#     finger.addObject('TetrahedronSetTopologyContainer', name='topo', src='@volLoader')
#     finger.addObject('MechanicalObject', name='dofs', template='Vec3d')
#     # A small monitor to peek performance or signals; you can remove file exports to avoid disk IO
#     finger.addObject('Monitor', name="fingerMonitor",
#                      template="Vec3d", listening=True, indices="0 60 120",
#                      ExportPositions=True, positionFile="fingerMonitorA_x.txt")

#     finger.addObject('GenericConstraintCorrection')
#     # finger.addObject('ParallelTetrahedronFEMForceField', template='Vec3d', method='large', youngModulus=young, poissonRatio=nu)
#     finger.addObject('TetrahedronFEMForceField', template='Vec3d', method='large', youngModulus=young, poissonRatio=nu)
#     finger.addObject('UniformMass', totalMass=0.04)

#     ## stiff layer
#     boxROISubTopo = finger.addObject('BoxROI', name='boxROISubTopo', box=[-100, 22.5, -8, -19, 28, 8], strict=False)
#     boxROI = finger.addObject('BoxROI', name='boxROI', box=[-10, 0, -20, 0, 30, 20], drawBoxes=True)
#     finger.addObject('RestShapeSpringsForceField', points=boxROI.indices.linkpath, stiffness=1e12, angularStiffness=1e12)
#     finger.addObject('GenericConstraintCorrection')

#     modelSubTopo = finger.addChild('SubTopology')
#     modelSubTopo.addObject('MeshTopology', position='@loader.position', tetrahedra=boxROISubTopo.tetrahedraInROI.linkpath,
#                            name='container')
#     modelSubTopo.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
#                            youngModulus=1500)

#     # # Collision
#     # coll = finger.addChild('Collision')
#     # coll.addObject('MeshSTLLoader', name='loader', filename=outer_stl)
#     # coll.addObject('MeshTopology', src='@loader', name='topo')
#     # coll.addObject('MechanicalObject', name='mo')
#     # coll.addObject('TriangleCollisionModel', selfCollision=False)
#     # coll.addObject('LineCollisionModel')
#     # coll.addObject('PointCollisionModel')
#     # coll.addObject('BarycentricMapping')

#     # # Visual (is this necessary when running headless workload?)
#     # visu = finger.addChild('Visu')
#     # visu.addObject('MeshSTLLoader', name='loader', filename=outer_stl)
#     # visu.addObject('OglModel', src='@loader', color=[0.8, 0.8, 0.5, 0.6])
#     # visu.addObject('BarycentricMapping')

#     # Cavity
#     cav = finger.addChild('Cavity1')
#     cav.addObject('MeshSTLLoader', name='loader', filename=cavity_stl)
#     cav.addObject('MeshTopology', src='@loader', name='topo')
#     cav.addObject('MechanicalObject', name='mo')
#     spc = cav.addObject('SurfacePressureConstraint', name='spc', template='Vec3',
#                         value=pressure_init, triangles='@topo.triangles', valueType='pressure')
#     cav.addObject('BarycentricMapping', mapForces=False, mapMasses=False)


#     return finger, spc