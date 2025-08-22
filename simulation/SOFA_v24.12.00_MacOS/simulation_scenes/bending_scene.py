import Sofa
import numpy as np
import csv
from PneumaticController import PneumaticController

def createScene(rootNode):
    # Basic simulation setup

    rootNode.addObject('RequiredPlugin', name='Sofa.GUI.Component') # Needed to use components [AttachBodyButtonSetting]
    rootNode.addObject('RequiredPlugin', pluginName='SoftRobots SofaPython3')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop')  # Needed to use components [FreeMotionAnimationLoop]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh')  # Needed to use components [MeshVTKLoader] NEED TO CHANGE
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.StateContainer')  # Needed to use components [MechanicalObject]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant')  # Needed to use components [MeshTopology]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Dynamic')  # Needed to use components [TetrahedronSetTopologyContainer]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mass')  # Needed to use components [UniformMass]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic')  # Needed to use components [TetrahedronFEMForceField]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.Spring')  # Needed to use components [RestShapeSpringsForceField]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Visual')  # Needed to use components [VisualStyle]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Engine.Select')  # Needed to use components [BoxROI] 
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct')  # Needed to use components [SparseLDLSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Iterative')  # Needed to use components [CGLinearSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear')  # Needed to use components [BarycentricMapping]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear')  # Needed to use components [RigidMapping]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward')  # Needed to use components [EulerImplicitSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction')  # Needed to use components [GenericConstraintCorrection,UncoupledConstraintCorrection]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Solver')  # Needed to use components [GenericConstraintSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Setting')  # Needed to use components [BackgroundSetting] 

    rootNode.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D') # Needed to use components [OglModel]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Algorithm')  # Needed to use components [BVHNarrowPhase,BruteForceBroadPhase,CollisionPipeline]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Intersection')  # Needed to use components [LocalMinDistance]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry')  # Needed to use components [LineCollisionModel,PointCollisionModel,TriangleCollisionModel]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Response.Contact')  # Needed to use components [CollisionResponse]
    rootNode.addObject('RequiredPlugin', name='SofaValidation')

    rootNode.addObject('AttachBodyButtonSetting', stiffness=10)
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-7, maxIterations=1000)
    rootNode.gravity.value = [4, -9.81, 0] # change gravity value in this case its in the y axis
    rootNode.addObject('CollisionPipeline')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('CollisionResponse', response='FrictionContactConstraint', responseParams='mu=0.6')
    rootNode.addObject('LocalMinDistance', name='Proximity', alarmDistance=1, contactDistance=0.5)
    rootNode.addObject("FreeMotionAnimationLoop")
   
    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1.])
    rootNode.addObject('OglSceneFrame', style='Arrows', alignment='TopRight')
    # rootNode.addObject('VisualStyle', displayFlags='showForceFields')

    # Finger loading
    finger = rootNode.addChild('Finger')
    finger.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
    finger.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixd')
    finger.addObject('MeshOBJLoader', name='loader', filename='../../data/Softnet/Bending/a16/model.obj', rotation=[90, 0, 270], translation=[120,9,10]) # load the object in
    finger.addObject('TriangleSetTopologyContainer', src='@loader')
    finger.addObject('TriangleSetTopologyModifier')
    finger.addObject('MechanicalObject', name='tetras', template='Vec3', showObject=False, showObjectScale=1) # store the DOF of the object
    finger.addObject('TriangleFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=0.45, youngModulus=5)  # Object material
    finger.addObject('UniformMass', totalMass=0.4) # assumes a uniform distribution of the vertexMass inside the body
    finger.addObject('GenericConstraintCorrection')
    finger.addObject('Monitor', name="fingerMonitorA", 
                 template="Vec3d",
                 listening=True, 
                 indices="9",  # Change this to the point you want to track
                 showPositions=True, PositionsColor="1 1 0 1",
                 showVelocities=True, VelocitiesColor="0 1 0 1",
                 showForces=True, ForcesColor="1 0 0 1",
                 ExportPositions=True,  # Save positions to "fingerMonitor_x.txt"
                 ExportVelocities=True, # Save velocities to "fingerMonitor_v.txt"
                 ExportForces=True)     # Save forces to "fingerMonitor_f.txt"
    finger.addObject('Monitor', name="fingerMonitorB", 
                 template="Vec3d",
                 listening=True, 
                 indices="92",  #  Track 2nd vertex index (Change this to the point you want to track)
                 showPositions=True, PositionsColor="1 1 0 1",
                 showVelocities=True, VelocitiesColor="0 1 0 1",
                 showForces=True, ForcesColor="1 0 0 1",
                 ExportPositions=True,  # Save positions to "fingerMonitorB_x.txt"
                 ExportVelocities=True, # Save velocities to "fingerMonitorB_v.txt"
                 ExportForces=True)     # Save forces to "fingerMonitorB_f.txt"


    boxROI = finger.addObject('BoxROI', name='boxROI', box=[102, 5, 0, 106, 10, 30], drawBoxes=True)
    finger.addObject('RestShapeSpringsForceField', points=boxROI.indices.linkpath, stiffness=1e12, angularStiffness=1e12)

    # boxROI2 = finger.addObject('BoxROI', name='boxROI', box=[150, -95, 8, 151, 0, 23], drawBoxes=True)
    # finger.addObject('RestShapeSpringsForceField', points=boxROI2.indices.linkpath, stiffness=1e12, angularStiffness=1e12)

    # stiff layer
    # boxROISubTopo = finger.addObject('BoxROI', name='boxROISubTopo', box=[100, -90, 8, 150, -37, 23],
    #                                  drawBoxes=True, strict=False)

    # modelSubTopo = finger.addChild('SubTopology')
    # modelSubTopo.addObject('MeshTopology', position='@loader.position', triangles=boxROISubTopo.trianglesInROI.linkpath,
    #                        name='container')
    # modelSubTopo.addObject('TriangleFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
    #                        youngModulus=1e12)
    
    # collision
    finger.addObject('TriangleCollisionModel', selfCollision=False)

    # visual 
    modelVisu = finger.addChild('Visu')
    modelVisu.addObject('MeshOBJLoader', name='loader', filename='../../data/Softnet/Bending/a16/model.obj', rotation=[90, 0, 270], translation=[120,9,10])
    modelVisu.addObject('OglModel', src='@loader', color=[0.8, 0.8, 0.5])
    modelVisu.addObject('BarycentricMapping')

    # add air pressure to cavity 

    # cavity = finger.addChild('Cavity')
    # cavity.addObject('MeshOBJLoader', name='cavityLoader', filename='../../data/Softnet/Bending/a16/model.obj',
    #                 rotation=[90, 0, 270], translation=[160,3,10], scale=0.5)  # Scale down slightly
    # cavity.addObject('TriangleSetTopologyContainer', src='@cavityLoader')  # Use same topology
    # cavity.addObject('TriangleSetTopologyModifier')
    # cavity.addObject('MechanicalObject', name='cavityMO')

    # using the cavity STL file

    # cavity = finger.addChild('cavity')
    # cavity.addObject('MeshSTLLoader', name='loader', filename='pneunetCavityCut.stl', rotation=[180, 0, 270], translation=[170,-75,15], scale=0.7)
    # cavity.addObject('Mesh', src='@loader', name='cavityMesh')
    # cavity.addObject('MechanicalObject', name='cavity')
    # cavity.addObject('SurfacePressureConstraint', name="SurfacePressureConstraint", template='Vec3d', value=0, triangles='@topo.triangles', valueType="pressure")
    # cavity.addObject('BarycentricMapping', name='mapping')

    # # Apply pressure to the cavity
    # cavity.addObject('SurfacePressureConstraint', name='CavitySurfacePressureConstraint', template='Vec3',
    #                 value=1000, triangles='@cavityLoader.triangles', valueType='pressure')

    # # Map the cavity to the main finger structure
    # cavity.addObject('BarycentricMapping', name='cavityMapping', mapForces=False, mapMasses=False)

    # # cavity visual 
    # cavityVisu = cavity.addChild('cavityVisu')
    # cavityVisu.addObject('MeshSTLLoader', name='loader', filename='pneunetCavityCut.stl', rotation=[180, 0, 270], translation=[170,-75,15], scale=0.7)
    # cavityVisu.addObject('OglModel', src='@loader', color=[0, 0, 0.5])
    # cavityVisu.addObject('BarycentricMapping')

    # # # Cavity simulation (box)
    cavity = finger.addChild('Cavity')
    # Define an imaginary cavity using BoxROI
    cavity.addObject('MechanicalObject', name='cavityMechanicalObject', position="@../tetras.position")
    cavityBox = cavity.addObject('BoxROI', name='cavityBox', box=[105, -85, 8, 115, -5, 23], drawBoxes=True, strict=True)
    # cavity.addObject('Oglmodel', src='@cavityMechanicalObject', color=[0, 1, 1])
    

    # Add MechanicalObject and SurfacePressureConstraint to simulate air pressure
    cavity.addObject('SurfacePressureConstraint', name='CavitySurfacePressureConstraint', template='Vec3',
                     value=50000,  # Pressure value the higher the slower the outer part moves #power 5 is max
                     triangles='@cavityBox.trianglesInROI',
                     valueType='pressure')
    # Map the forces from the cavity to the finger
    cavity.addObject('BarycentricMapping', name='cavityMapping', mapForces=True, mapMasses=False)

    rootNode.addObject(PneumaticController(node=rootNode))

    return rootNode
