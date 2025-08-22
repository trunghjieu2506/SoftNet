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
    rootNode.gravity.value = [0, -9.81, 0] # change gravity value in this case its in the y axis
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
    finger.addObject('MeshVTKLoader', name='loader', filename='/Users/trunghjieu/Downloads/simulation_scenes/pneunetCutCoarse.vtk', rotation=[180, 270, 270]) # load the object in
    finger.addObject('MeshTopology', src='@loader', name='container')
    finger.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale=4e-5)
    finger.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=0.45, youngModulus=5)  # Object material
    finger.addObject('UniformMass', totalMass=0.4) # assumes a uniform distribution of the vertexMass inside the body
    finger.addObject('GenericConstraintCorrection')
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
    
    finger.addObject('Monitor', name="fingerMonitorB", 
                 template="Vec3d",
                 listening=True, 
                 indices="54",  #  Track 2nd vertex index (Change this to the point you want to track)
                 showPositions=True, PositionsColor="1 1 0 1",
                 showVelocities=True, VelocitiesColor="0 1 0 1",
                 showForces=True, ForcesColor="1 0 0 1",
                 ExportPositions=True,  # Save positions to "fingerMonitorB_x.txt"
                 ExportVelocities=True, # Save velocities to "fingerMonitorB_v.txt"
                 ExportForces=True)     # Save forces to "fingerMonitorB_f.txt"

    boxROI = finger.addObject('BoxROI', name='boxROI', box=[-25, -10, -5, 1, 10, 10], drawBoxes=True)
    finger.addObject('RestShapeSpringsForceField', points=boxROI.indices.linkpath, stiffness=1e12, angularStiffness=1e12)

    # Collision
    collisionFinger = finger.addChild('Collision')
    collisionFinger.addObject('MeshSTLLoader', name='loader', filename='/Users/trunghjieu/Downloads/simulation_scenes/pneunetCut.stl', rotation=[180, 270, 270])
    collisionFinger.addObject('MeshTopology', src='@loader', name='topo')
    collisionFinger.addObject('MechanicalObject')
    collisionFinger.addObject('TriangleCollisionModel', selfCollision=False)
    collisionFinger.addObject('LineCollisionModel')
    collisionFinger.addObject('PointCollisionModel')
    collisionFinger.addObject('BarycentricMapping')

    # Visualization
    modelVisu = finger.addChild('Visu')
    modelVisu.addObject('MeshSTLLoader', name='loader', filename='/Users/trunghjieu/Downloads/simulation_scenes/pneunetCut.stl', rotation=[180, 270, 270])
    modelVisu.addObject('OglModel', src='@loader', color=[0.8, 0.8, 0.5, 0.6])
    modelVisu.addObject('BarycentricMapping')

    cavity = finger.addChild('Cavity')
    cavity.addObject('MeshSTLLoader', name='loader', filename='/Users/trunghjieu/Downloads/simulation_scenes/pneunetCavityCut.stl', rotation=[180, 270, 270])
    cavity.addObject('MeshTopology', src='@loader', name='topo')
    cavity.addObject('MechanicalObject', name='cavity')
    cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3',
                         value=0.02,
                         triangles='@topo.triangles', valueType='pressure')
    cavity.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)

    # # cavity visual 
    # cavityVisu = cavity.addChild('cavityVisu')
    # cavityVisu.addObject('MeshSTLLoader', name='loader', filename='pneunetCavityCut.stl', rotation=[180, 0, 270], translation=[170,-75,15], scale=0.7)
    # cavityVisu.addObject('OglModel', src='@loader', color=[0, 0, 0.5])
    # cavityVisu.addObject('BarycentricMapping')


    rootNode.addObject(PneumaticController(node=rootNode))

    return rootNode
