import Sofa
from PneumaticController import PneumaticController

def createScene(rootNode):
    # --- Plugins / core ---
    rootNode.addObject('RequiredPlugin', name='MultiThreading') # Needed to use components [ParallelBVHNarrowPhase,ParallelBruteForceBroadPhase,ParallelCGLinearSolver,ParallelTetrahedronFEMForceField]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction') # Needed to use components [GenericConstraintCorrection]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Solver') # Needed to use components [GenericConstraintSolver]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Engine.Select') # Needed to use components [BoxROI]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Setting') # Needed to use components [BackgroundSetting]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.Spring') # Needed to use components [RestShapeSpringsForceField]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.StateContainer') # Needed to use components [MechanicalObject]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant') # Needed to use components [MeshTopology]  
    rootNode.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D') # Needed to use components [OglModel,OglSceneFrame]  
    
    rootNode.addObject('RequiredPlugin', pluginName='SofaPython3 SoftRobots')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Dynamic')   # TetrahedronSetTopologyContainer
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mass')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Algorithm')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Intersection')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Response.Contact')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Visual')
    rootNode.addObject('RequiredPlugin', name='SofaValidation')  
    rootNode.addObject('RequiredPlugin', name='Sofa.GUI.Component') # Needed to use components [AttachBodyButtonSetting]  


    rootNode.addObject('AttachBodyButtonSetting', stiffness=10)
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-7, maxIterations=1000)
    rootNode.gravity = [0.0, -9.81, 0.0]
    rootNode.addObject('FreeMotionAnimationLoop')

    rootNode.addObject('CollisionPipeline')
    # rootNode.addObject('ParallelBruteForceBroadPhase')
    # rootNode.addObject('ParallelBVHNarrowPhase')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('LocalMinDistance', name='Proximity', alarmDistance=1.0, contactDistance=0.5)
    rootNode.addObject('CollisionResponse', response='FrictionContactConstraint', responseParams='mu=0.6')

    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1.0])
    rootNode.addObject('OglSceneFrame', style='Arrows', alignment='TopRight')

    # --- Finger (volume) ---
    finger = rootNode.addChild('Finger')
    finger.addObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
    finger.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixd')
    # finger.addObject('ParallelCGLinearSolver',
    #              iterations=200,           # cap work per step
    #              tolerance=1e-8,           # can relax to 1e-7 ~ 1e-6 if stable
    #              threshold=1e-12,
    #              warmStart=True)

    # Pick one preconditioner you have:
    # finger.addObject('JacobiPreconditioner')      # simplest & cheap
    
    finger.addObject('MeshVTKLoader',
                     name='volLoader',
                     filename='/workspace/SSLSoftneet/simulation/out_dir/finger_legacy_ascii.vtk')

    finger.addObject('TetrahedronSetTopologyContainer', name='topo', src='@volLoader')
    finger.addObject('MechanicalObject', name='dofs', template='Vec3d')

    # Physics
    finger.addObject('UniformMass', totalMass=0.4)
    finger.addObject('GenericConstraintCorrection')
    # Use the parallel component when available (as your log suggested)
    # finger.addObject('ParallelTetrahedronFEMForceField', template='Vec3d',
    #                 method='large', youngModulus=5e5, poissonRatio=0.45)
    finger.addObject('TetrahedronFEMForceField', template='Vec3f',
                    method='large', youngModulus=5e5, poissonRatio=0.45)
    
    finger.addObject('Monitor', name="fingerMonitorA", 
                template="Vec3d",
                listening=True, 
                indices="1",  #  Track vertex index 1 (Change this to the point you want to track)
                showPositions=True, PositionsColor="1 1 0 1",
                showVelocities=True, VelocitiesColor="0 1 0 1",
                showForces=True, ForcesColor="1 0 0 1",
                ExportPositions=True,  # Save positions to "fingerMonitorA_x.txt"
                ExportVelocities=True, # Save velocities to "fingerMonitorA_v.txt"
                ExportForces=True)     # Save forces to "fingerMonitorA_f.txt"
    
    # finger.addObject('Monitor', name="fingerMonitorB", 
    #              template="Vec3d",
    #              listening=True, 
    #              indices="54",  #  Track 2nd vertex index (Change this to the point you want to track)
    #              showPositions=True, PositionsColor="1 1 0 1",
    #              showVelocities=True, VelocitiesColor="0 1 0 1",
    #              showForces=True, ForcesColor="1 0 0 1",
    #              ExportPositions=True,  # Save positions to "fingerMonitorB_x.txt"
    #              ExportVelocities=True, # Save velocities to "fingerMonitorB_v.txt"
    #              ExportForces=True)     # Save forces to "fingerMonitorB_f.txt"


    # (Optional) base fixation by ROI in world units—update the box to match your mesh scale/origin
    box = finger.addObject('BoxROI', name='boxROI', box=[-0.05, -0.01, -0.02, 0.0, 0.01, 0.02], drawBoxes=True)
    finger.addObject('RestShapeSpringsForceField',
                     points=box.indices.linkpath,
                     stiffness=1e12, angularStiffness=1e12)

    # --- Collision surface mapped to volume ---
    coll = finger.addChild('Collision')
    coll.addObject('MeshSTLLoader', name='loader', filename='/workspace/SSLSoftneet/simulation/out_dir/outer_from_tet.stl')
    coll.addObject('MeshTopology', src='@loader', name='topo')
    coll.addObject('MechanicalObject', name='mo')
    coll.addObject('TriangleCollisionModel', selfCollision=False)
    coll.addObject('LineCollisionModel')
    coll.addObject('PointCollisionModel')
    coll.addObject('BarycentricMapping')  # maps Finger/dofs -> Collision/mo

    # --- Visualization surface mapped to volume ---
    visu = finger.addChild('Visu')
    visu.addObject('MeshSTLLoader', name='loader', filename='/workspace/SSLSoftneet/simulation/out_dir/outer_from_tet.stl')
    visu.addObject('OglModel', src='@loader', color=[0.8, 0.8, 0.5, 0.6])
    visu.addObject('BarycentricMapping')

    # --- Cavity surface (normals must point INTO the cavity) ---
    cav1 = finger.addChild('Cavity1')
    cav1.addObject('MeshSTLLoader', name='loader', filename='/workspace/SSLSoftneet/simulation/out_dir/cavity_from_tet_1.stl')
    cav1.addObject('MeshTopology', src='@loader', name='topo')
    cav1.addObject('MechanicalObject', name='mo')

    cav1.addObject('SurfacePressureConstraint', name='spc', template='Vec3d',
                   value=0.02,  # start from 0 Pa, controller will change
                   triangles='@topo.triangles',
                   valueType='pressure')
    cav1.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

    # --- Controller ---
    # If your PneumaticController searches for 'spc' under children, leave as is.
    # If it expects a different path or list, adapt accordingly.
    rootNode.addObject(PneumaticController(node=rootNode))

    return rootNode