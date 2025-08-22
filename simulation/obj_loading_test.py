import Sofa


def main():
    # Call the SOFA function to create the root node
    root = Sofa.Core.Node("root")

    # Call the createScene function, as runSofa does
    createScene(root)

    # Once defined, initialization of the scene graph
    Sofa.Simulation.initRoot(root)

    # Run as many simulation steps (here 10 steps are computed)
    for iteration in range(10):
        Sofa.Simulation.animate(root, root.dt.value)
        print("Computing iteration "+str(iteration+1))

    print("Computation is done.")

def createScene(root):
    root.addObject('RequiredPlugin', name='SofaPython3')
    for p in ['SofaAssimp','Sofa.Component.IO.Mesh','SofaGeneralLoader','SofaLoader',
              'Sofa.GL.Component.Rendering3D','SofaOpenglVisual']:
        try: root.addObject('RequiredPlugin', name=p)
        except: pass

    root.addObject('DefaultAnimationLoop')
    root.gravity = [0,-9.81,0]
    n = root.addChild('objtest')

    # pick ONE of these; Assimp as fallback:
    try:
        n.addObject('MeshOBJLoader', name='loader', filename='/Users/trunghjieu/Desktop/SoftNet/Soft-Fusion-FYP 2/data/Softnet/norm_mesh_dir_v1/a28/pc_norm.obj', triangulate=True)
    except:
        n.addObject('AssimpLoader',  name='loader', filename='/ABSOLUTE/PATH/TO/pc_norm.obj', triangulate=True)

    n.addObject('MeshTopology', src='@loader')
    n.addObject('MechanicalObject', name='surface', template='Vec3d')
    n.addObject('UniformMass', totalMass=1.0)
    n.addObject('TriangleSpringForceField', stiffness=50, damping=1)
    v = n.addChild('Visu')
    v.addObject('OglModel', src='@loader')
    v.addObject('IdentityMapping', input='@../surface', output='@visual')

# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()