import Sofa
import numpy as np
import csv

def createScene(rootNode):
    # Basic simulation setup

    rootNode.addObject('RequiredPlugin', name='Sofa.GUI.Component') # Needed to use components [AttachBodyButtonSetting]
    rootNode.addObject('RequiredPlugin', pluginName='SoftRobots SofaPython3')
    rootNode.addObject("DefaultAnimationLoop")

    rootNode.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh')  # Needed to use components [MeshVTKLoader] NEED TO CHANGE
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.StateContainer')  # Needed to use components [MechanicalObject]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant')  # Needed to use components [MeshTopology]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mass')  # Needed to use components [UniformMass]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic')  # Needed to use components [TetrahedronFEMForceField]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.Spring')  # Needed to use components [RestShapeSpringsForceField]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Visual')  # Needed to use components [VisualStyle]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Engine.Select')  # Needed to use components [BoxROI] 
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct')  # Needed to use components [SparseLDLSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward')  # Needed to use components [EulerImplicitSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction')  # Needed to use components [GenericConstraintCorrection,UncoupledConstraintCorrection]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Solver')  # Needed to use components [GenericConstraintSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Setting')  # Needed to use components [BackgroundSetting] 

    rootNode.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D') # Needed to use components [OglModel]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Algorithm')  # Needed to use components [BVHNarrowPhase,BruteForceBroadPhase,CollisionPipeline]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Intersection')  # Needed to use components [LocalMinDistance]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry')  # Needed to use components [LineCollisionModel,PointCollisionModel,TriangleCollisionModel]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Response.Contact')  # Needed to use components [CollisionResponse]

    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1.])
    rootNode.addObject('OglSceneFrame', style='Arrows', alignment='TopRight')
    # First step loading
    finger = rootNode.addChild('Finger')
    finger.addObject('MeshOBJLoader', name='loader', filename='../../data/Softnet/Bending/a1/model.obj') # load the object in
    finger.addObject('MeshTopology', src='@loader', name='container')
    finger.addObject('MechanicalObject', name='tetras', template='Vec3', showObject=True, showObjectScale=1) # store the DOF of the object

    # Second step law of material and mass
    #rootNode.addObject('VisualStyle', displayFlags='showVisualModels showForceFields showWireframe')
    rootNode.addObject('VisualStyle', displayFlags='showAll')        
    rootNode.gravity.value = [0, -9.81, 0] # change gravity value in this case its in the y axis

    finger.addObject('TriangleFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=0.3, youngModulus=500)  # Object material
    finger.addObject('UniformMass', totalMass=0.04) # assumes a uniform distribution of the vertexMass inside the body

    # Third step stiff layer
    # To define the constitutive law of the stiff layer, we will create a new node and define a new ForceField with stiffer parameters
    # only on the points which constitute the layer. To easily define the indices of the points which will be selected, we use the boxROI component 
    # which allows to define a box that will contain all the points of the layer (refered by the node "modelSubTopo"). 
    # The box component contains successively the extreme coordinates along x, y and z
    
    boxROISubTopo = finger.addObject('BoxROI', name='boxROISubTopo', box=[0, 20, -3, 160, 100, 20],
                                     drawBoxes=True, strict=False)

    modelSubTopo = finger.addChild('SubTopology')
    modelSubTopo.addObject('MeshTopology', position='@loader.position', tetrahedra=boxROISubTopo.tetrahedraInROI.linkpath,
                           name='container')
    modelSubTopo.addObject('TriangleFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                           youngModulus=1500)
    
    # Fourth step boundary conditions fixes some postions of the object
    boxROI = finger.addObject('BoxROI', name='boxROI', box=[160, 0, -3, 170, 10, 20], drawBoxes=True)
    finger.addObject('RestShapeSpringsForceField', points=boxROI.indices.linkpath, stiffness=1e12, angularStiffness=1e12)
    finger.addObject('OglModel', name='Visual', src='@loader', color=[0.0, 1, 1])

    # all minimal objects of the scene are added by this point

    # Fifth step time integration and matrix solver
    # in order to simulate, we need a time integration scheme to be solved at each time step in the simulation
    rootNode.addObject('AttachBodyButtonSetting', stiffness=10)

    finger.addObject('EulerImplicitSolver')
    finger.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixd')

    # at this point there will just be deformation due to gravity

    # Sixth step cavity 

    # cavity = finger.addChild('Cavity')
    # cavity.addObject('MeshOBJLoader', name='cavityLoader', filename='../../data/Softnet/Bending/a1/model.obj')  # Use the same .obj file
    # cavity.addObject('UniformScaling', name='scaleCavity', scaling=0.9)
    # cavity.addObject('MeshTopology', src='@loader', name='cavitycontainer')
    # cavity.addObject('MechanicalObject', name='tetraz')
    # cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3',
    #                      value=0.0001,
    #                      triangles='@topo.triangles', valueType='pressure')
    # cavity.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)


    # # Add a data logging controller
    # rootNode.addObject(DataLogger(name="logger", pressure_field="@pressure", monitored_indices=[10, 20, 30]))

    # pressureRegion = finger.addObject('BoxROI', name='pressureRegion', box=[165, 2, 7, 175, 5, 10], drawBoxes=True, strict=False)
    # finger.addObject('TetrahedronPressureForceField', template='Vec3d', name='pressureField',
    #                 indices=pressureRegion.indices.linkpath, pressure=0.1)  # Initial pressure value
    
    # # Sixth step: Add bounding box below the object to act as a floor

    # floorBox = finger.addObject('BoxROI', name='floorBox', box=[0, -0.2, 0, 175, 0, 30], drawBoxes=True)
    # finger.addObject('RestShapeSpringsForceField', points=floorbox.indices.linkpath, stiffness=1e12, angularStiffness=1e12)

    # Plane
    ##########################################
    plane = rootNode.addChild('Plane')
    plane.addObject('MeshOBJLoader', name='plane', filename='floorFlat.obj',
                    rotation=[0, 0, 0], scale=5, translation=[70, 0, 0])
    plane.addObject('MeshTopology', src='@plane')
    plane.addObject('MechanicalObject', src='@plane')
    plane.addObject('TriangleCollisionModel')
    plane.addObject('LineCollisionModel')
    plane.addObject('PointCollisionModel')
    plane.addObject('OglModel', name='Visual', src='@plane', color=[1, 0, 1, 1])

    finger.addObject('TriangleCollisionModel')
    finger.addObject('LineCollisionModel')
    finger.addObject('PointCollisionModel')
    finger.addObject('BarycentricMapping')

    rootNode.addObject('CollisionPipeline')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('CollisionResponse', response='FrictionContactConstraint', responseParams='mu=0.6')
    rootNode.addObject('LocalMinDistance', name='Proximity', alarmDistance=5, contactDistance=1)
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-7, maxIterations=1000)
    finger.addObject('GenericConstraintCorrection')


        # Plane

    # plane = rootNode.addChild('Plane')
    # plane.addObject('EulerImplicitSolver')
    # plane.addObject('CGLinearSolver', threshold=1e-5, tolerance=1e-5, iterations=50)
    # plane.addObject('MeshOBJLoader', name='plane', filename='floorFlat.obj',
    #                 triangulate=True, rotation=[0, 0, 0], scale=5, translation=[70, -2, 0])
    # plane.addObject('MeshTopology', src='@plane')
    # plane.addObject('MechanicalObject', src='@plane')
    # # plane.addObject('UniformMass', totalMass=1)
    # plane.addObject('TriangleCollisionModel', simulated="0", moving="0")
    # plane.addObject('LineCollisionModel', simulated="0", moving="0")
    # plane.addObject('PointCollisionModel', simulated="0", moving="0")
    # #plane.addObject('UncoupledConstraintCorrection')
    # plane.addObject('OglModel', name='planeVisual', src='@plane',texturename="textures/floor.bmp")

    # plane.addObject('BarycentricMapping')

    # # Cube
    # ##########################################
    # cube = rootNode.addChild('Cube')
    # cube.addObject('EulerImplicitSolver')
    # cube.addObject('CGLinearSolver', threshold=1e-5, tolerance=1e-5, iterations=50)
    # cube.addObject('MechanicalObject', template='Rigid3', position=[-100, 70, 0, 0, 0, 0, 1])
    # cube.addObject('UniformMass', totalMass=10)
    # cube.addObject('UncoupledConstraintCorrection')

    # # collision
    # cubeCollis = cube.addChild('Collision')
    # cubeCollis.addObject('MeshOBJLoader', name='loader', filename='smCube27.obj', translation=[100,-30,5], scale=6)
    # cubeCollis.addObject('MeshTopology', src='@loader')
    # cubeCollis.addObject('MechanicalObject')
    # cubeCollis.addObject('TriangleCollisionModel')
    # cubeCollis.addObject('LineCollisionModel')
    # cubeCollis.addObject('PointCollisionModel')
    # cubeCollis.addObject('RigidMapping')

    # # visualization
    # cubeVisu = cube.addChild('Visu')
    # cubeVisu.addObject('MeshOBJLoader', name='loader', filename='smCube27.obj')
    # cubeVisu.addObject('OglModel', name='Visual', src='@loader', color=[0.0, 0.1, 0.5], translation=[100,-30,5], scale=6.2)
    # cubeVisu.addObject('RigidMapping')

    return rootNode
