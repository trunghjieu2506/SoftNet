import Sofa

def createScene(root):
    root.addObject('RequiredPlugin', pluginName='SofaPython3 SoftRobots')
    root.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh')
    root.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Dynamic')
    root.addObject('RequiredPlugin', name='Sofa.Component.Mass')
    root.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic')
    root.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct')
    root.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward')

    root.addObject('FreeMotionAnimationLoop')
    root.gravity = [0, -9.81, 0]

    finger = root.addChild('Finger')
    finger.addObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
    finger.addObject('SparseLDLSolver')

    finger.addObject('MeshVTKLoader', name='vol', filename='/Users/trunghjieu/Downloads/SOFA_v24.12.00_MacOS/simulation_scenes/sofa_asset/finger.vtk')
    finger.addObject('TetrahedronSetTopologyContainer', name='topo', src='@vol')
    finger.addObject('MechanicalObject', name='dofs', template='Vec3d')
    finger.addObject('UniformMass', totalMass=0.4)
    finger.addObject('TetrahedronFEMForceField', name='FEM', template='Vec3d',
                     method='large', poissonRatio=0.45, youngModulus=5e5)

    # Visual quick check (SOFA will draw the tet edges)
    finger.addObject('VisualStyle', displayFlags='showBehaviorModels showWireframe')
    return root
