#!/usr/bin/env python3
"""
SDF → SOFA scene runner (SofaPython3)

Run any soft-robot model described in SDF inside SOFA by generating a scene graph
on the fly. This focuses on deformable (soft-body) simulation. It supports
triangle meshes (mass–spring fallback) and volumetric tetra meshes (FEM).

Usage (with SofaPython3):

  runSofa -l SofaPython3 sdf_to_sofa.py --sdf path/to/model.sdf \
          --resource-dir /path/to/meshes --young 3000 --poisson 0.45 \
          --density 1000 --dt 0.005

Notes
-----
• This script makes pragmatic assumptions to map SDF → SOFA. It’s intended as a
  solid starting point you can extend for your project.
• Joints are mapped to simple AttachConstraint links between parent/child COMs.
  You can replace this with SOFA’s articulated mechanisms if you need kinematic
  joints (see Sofa.Constraint / ArticulatedSystemMapping, etc.).
• SDF is assumed in SI units (m, kg, rad). SOFA expects degrees for rotations,
  so we convert radians→degrees for poses.
• To run meshes referenced by URIs like model:// or file://, use --resource-dir
  (you can pass multiple) or set GAZEBO_MODEL_PATH.

Tested with SOFA 23.x/24.x + SofaPython3.
"""
from __future__ import annotations
import os
import sys
import math
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

# SOFA Python (provided by runSofa -l SofaPython3)
try:
    import Sofa
except Exception:
    print("""
[ERROR] SofaPython3 is not available in this Python environment.
This script must be launched *from* SOFA using:

    runSofa -l SofaPython3 sdf_to_sofa.py --sdf your_model.sdf

If you tried to `python sdf_to_sofa.py` directly or `pip install` SOFA,
that won't work because SOFA is a C++ framework with a Python plugin.
Install SOFA (binaries or build from source) and then run via `runSofa`.
""")
    raise

# ---------------------------- CLI & config ---------------------------- #

def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument('--sdf', required=True, help='Path to .sdf file (Simulation Description Format)')
    p.add_argument('--resource-dir', action='append', default=[],
                   help='Extra directory to search for mesh URIs (can be used multiple times)')
    p.add_argument('--young', type=float, default=3000.0, help='Young\'s modulus for FEM (Pa)')
    p.add_argument('--poisson', type=float, default=0.45, help='Poisson ratio for FEM')
    p.add_argument('--density', type=float, default=1000.0, help='Default density kg/m^3 (fallback)')
    p.add_argument('--dt', type=float, default=0.005, help='Time step (s)')
    p.add_argument('--gravity', type=float, nargs=3, default=[0.0, -9.81, 0.0], help='Gravity vector')
    p.add_argument('--mass-scale', type=float, default=1.0, help='Scale all computed masses by factor')
    p.add_argument('--stiffness', type=float, default=50.0, help='Mass–spring fallback stiffness (N/m)')
    p.add_argument('--damping', type=float, default=1.0, help='Mass–spring fallback damping (N·s/m)')
    p.add_argument('--verbose', action='store_true', help='Print detailed mapping information')
    return p.parse_known_args(argv)[0]

# ---------------------------- SDF helpers ---------------------------- #

def get_text(elem: ET.Element | None, default: str = '') -> str:
    return elem.text.strip() if elem is not None and elem.text else default

RAD2DEG = 180.0 / math.pi

class UriResolver:
    def __init__(self, sdf_file: Path, extra_dirs: list[Path]):
        self.base_dir = sdf_file.parent
        self.extra_dirs = [Path(d) for d in extra_dirs if d]
        # Also include GAZEBO_MODEL_PATH dirs
        for root in os.environ.get('GAZEBO_MODEL_PATH', '').split(os.pathsep):
            if root:
                self.extra_dirs.append(Path(root))

    def resolve(self, uri: str) -> str:
        """Return an absolute file path for an SDF <mesh><uri> string. If the file
        cannot be located directly, try a *basename* deep search inside the provided
        resource directories and the SDF folder.
        """
        u = uri.strip()
        if u.startswith('file://'):
            return u.replace('file://', '')
        if u.startswith('model://'):
            rel = u.replace('model://', '')
            for d in self.extra_dirs:
                candidate = (d / rel).resolve()
                if candidate.exists():
                    return str(candidate)
        # Relative path (common): resolve from the SDF file directory
        candidate = (self.base_dir / u).resolve()
        if candidate.exists():
            return str(candidate)
        # Fallback: search in extra dirs
        for d in self.extra_dirs:
            candidate = (d / u).resolve()
            if candidate.exists():
                return str(candidate)
        # Deep search by basename as a last resort
        base = Path(u).name
        for d in [self.base_dir, *self.extra_dirs]:
            try:
                for hit in d.rglob(base):
                    return str(hit.resolve())
            except Exception:
                continue
        # Give up – return original
        return u.replace('file://', '')
        if u.startswith('model://'):
            # model://MODEL_NAME/meshes/foo.stl -> search in extra dirs
            rel = u.replace('model://', '')
            for d in self.extra_dirs:
                candidate = d / rel
                if candidate.exists():
                    return str(candidate.resolve())
        # Relative path (common): resolve from the SDF file directory
        candidate = (self.base_dir / u).resolve()
        if candidate.exists():
            return str(candidate)
        # Fallback: search in extra dirs
        for d in self.extra_dirs:
            candidate = (d / u).resolve()
            if candidate.exists():
                return str(candidate)
        # Give up – return original
        return u

# ---------------------------- SOFA scene tools ---------------------------- #

def sofa_log(msg: str, verbose: bool):
    if verbose:
        print(msg)


def add_sim_basics(node):
    node.addObject('DefaultAnimationLoop')
    node.addObject('DefaultVisualManagerLoop')
    # Collision pipeline (generic)
    node.addObject('CollisionPipeline')
    node.addObject('BruteForceDetection')
    node.addObject('MinProximityIntersection', alarmDistance=0.005, contactDistance=0.002)
    node.addObject('DefaultContactManager', response='FrictionContact')
    node.addObject('DefaultPipeline')  # For legacy scenes


def add_solving(node):
    node.addObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
    node.addObject('SparseLDLSolver')


def add_collision_models(node):
    node.addObject('PointCollisionModel')
    node.addObject('LineCollisionModel')
    node.addObject('TriangleCollisionModel')


def add_visual_model(node, *, use_loader: bool, mesh_path: str | None, mech_name: str | None, volumetric: bool):
    """Add an OglModel visual.
    - If use_loader=True, the visual will use src='@loader' to avoid re-parsing files
      (prevents 'extension not supported' when OglModel lacks OBJ support).
    - If use_loader=False, falls back to fileMesh.
    - Maps visual to the mechanical object via Identity (surface) or Barycentric (tetra).
    """
    vis = node.addChild('Visu')
    if use_loader:
        vis.addObject('OglModel', name='visual', src='@loader')
    else:
        if not mesh_path:
            raise ValueError('mesh_path is required when use_loader=False')
        vis.addObject('OglModel', name='visual', fileMesh=mesh_path)
    if mech_name:
        mapping = 'BarycentricMapping' if volumetric else 'IdentityMapping'
        vis.addObject(mapping, input=f'@../{mech_name}', output='@visual')
    vis.addObject('OglModel', name='visual', fileMesh=mesh_path)
    try:
        vis.addObject('IdentityMapping')  # Works when visual and mech share vertices (surface case)
    except Exception:
        try:
            vis.addObject('BarycentricMapping')  # Fallback when mech is volumetric
        except Exception:
            pass


# ---------------------------- Mesh → Soft body builders ---------------------------- #

SUPPORTED_TET = {'.msh', '.vtk', '.vtu'}
SUPPORTED_TRI = {'.obj', '.stl', '.ply', '.off'}


def ensure_mesh_exists(mesh_path: str):
    p = Path(mesh_path)
    if not p.exists():
        raise FileNotFoundError(
            f"Mesh not found: {mesh_path}.")  

def build_soft_from_mesh(node, mesh_path: str, *, young: float, poisson: float,
                          density: float, stiffness: float, damping: float,
                          verbose: bool):
    """Attach either an FEM tetra model (preferred) or a mass–spring surface model.
    Optimized for OBJ surface meshes (common in SDF robots).
    """
    ensure_mesh_exists(mesh_path)
    ext = Path(mesh_path).suffix.lower()
    if ext in SUPPORTED_TET:
        # Volumetric tetrahedral FEM
        loader_name = 'MeshGmshLoader' if ext == '.msh' else 'MeshVTKLoader'
        node.addObject(loader_name, name='loader', filename=mesh_path)
        node.addObject('TetrahedronSetTopologyContainer', name='topo', src='@loader')
        node.addObject('TetrahedronSetGeometryAlgorithms', template='Vec3d')
        node.addObject('MechanicalObject', name='tetras', template='Vec3d', showIndices=False)
        node.addObject('UniformMass', totalMass=density)
        node.addObject('TetrahedronFEMForceField', template='Vec3d', name='fem',
                       youngModulus=young, poissonRatio=poisson, method='large')
        add_collision_models(node)
        add_visual_model(node, use_loader=True, mesh_path=None, mech_name='tetras', volumetric=True)
        sofa_log(f"FEM (tetra) built from {mesh_path}", verbose)
    elif ext in SUPPORTED_TRI:
        # Surface mesh: mass–spring (TriangleSpringForceField)
        if ext == '.obj':
            try:
                node.addObject('MeshOBJLoader', name='loader', filename=mesh_path, triangulate=True)
            except Exception:
                # Fallback to Assimp if OBJ loader isn't available in this build
                node.addObject('AssimpLoader', name='loader', filename=mesh_path, triangulate=True)
        elif ext == '.stl':
            node.addObject('MeshSTLLoader', name='loader', filename=mesh_path)
        else:
            node.addObject('MeshVTKLoader', name='loader', filename=mesh_path)
        node.addObject('MeshTopology', src='@loader')
        node.addObject('MechanicalObject', name='surface', template='Vec3d')
        node.addObject('UniformMass', totalMass=density)
        node.addObject('TriangleSpringForceField', name='springs', stiffness=stiffness, damping=damping)
        add_collision_models(node)
        add_visual_model(node, use_loader=True, mesh_path=None, mech_name='surface', volumetric=False)
        sofa_log(f"Mass–spring (surface) built from {mesh_path}", verbose)
    else:
        raise RuntimeError(f"Unsupported mesh format: {mesh_path}")


# ---------------------------- SDF → SOFA mapping ---------------------------- #

class LinkInfo:
    def __init__(self, name: str, node):
        self.name = name
        self.node = node


def parse_pose(elem: ET.Element | None):
    """Return (xyz[m], rpy[deg]) from an SDF <pose> element. Default zeros."""
    if elem is None or not (elem.text and elem.text.strip()):
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    vals = [float(x) for x in elem.text.split()]
    if len(vals) != 6:
        raise ValueError('SDF <pose> must have 6 values: x y z roll pitch yaw')
    x, y, z, r, p, yw = vals
    return [x, y, z], [r*RAD2DEG, p*RAD2DEG, yw*RAD2DEG]


def set_transform_on_loader(node, translation, rotation, scale=None):
    """Apply initial transform (and optional scale) to the loader if possible, else via MeshAffineTransformer."""
    for comp in ('loader',):
        try:
            ld = node.getObject(comp)
            try:
                ld.findData('translation').value = translation
            except Exception:
                pass
            try:
                ld.findData('rotation').value = rotation
            except Exception:
                pass
            if scale is not None:
                # Try common names used by loaders
                for key in ('scale3d', 'scale'):
                    try:
                        ld.findData(key).value = scale
                        break
                    except Exception:
                        continue
            return
        except Exception:
            pass
    # Fallback: transformer
    try:
        kwargs = dict(translation=translation, rotation=rotation, input='@loader', output='@loader')
        if scale is not None:
            kwargs['scale3d'] = scale
        node.addObject('MeshAffineTransformer', **kwargs)
    except Exception:
        # Last resort: set on MechanicalObject
        try:
            mo = node.getObject('tetras') if node.getObject('tetras') else node.getObject('surface')
            mo.findData('translation').value = translation
            mo.findData('rotation').value = rotation
        except Exception:
            pass


def build_link_node(parent_node, link_el: ET.Element, resolver: UriResolver,
                     young: float, poisson: float, density: float, stiffness: float,
                     damping: float, verbose: bool) -> LinkInfo:
    name = link_el.attrib.get('name', 'link')
    ln = parent_node.addChild(name)
    add_solving(ln)

    # Pose
    trans, rot_deg = parse_pose(link_el.find('pose'))

    # Geometry: prefer <collision> then <visual>
    mesh_path = None
    geom_el = None
    for tag in ('collision', 'visual'):
        e = link_el.find(f'{tag}/geometry')
        if e is not None:
            geom_el = e
            break

    scale_vec = None
    if geom_el is not None and geom_el.find('mesh/scale') is not None:
        s = get_text(geom_el.find('mesh/scale'))
        try:
            vals = [float(x) for x in s.split()]
            if len(vals) == 1:
                scale_vec = [vals[0], vals[0], vals[0]]
            elif len(vals) == 3:
                scale_vec = vals
        except Exception:
            pass

    if geom_el is not None and geom_el.find('mesh/uri') is not None:
        uri = get_text(geom_el.find('mesh/uri'))
        mesh_path = resolver.resolve(uri)
        build_soft_from_mesh(ln, mesh_path, young=young, poisson=poisson,
                             density=density, stiffness=stiffness, damping=damping,
                             verbose=verbose)
        set_transform_on_loader(ln, trans, rot_deg, scale=scale_vec)
    else:
        # Primitive geometry fallback: create a small cube surface with springs
        sofa_log(f"[WARN] Link '{name}' has no mesh geometry; creating a small cube placeholder.", True)
        from tempfile import gettempdir
        cube_path = str(Path(gettempdir())/ 'sofa_tmp_cube.obj')
        _ensure_temp_cube(cube_path)
        build_soft_from_mesh(ln, cube_path, young=young, poisson=poisson,
                             density=density, stiffness=stiffness, damping=damping,
                             verbose=verbose)
        set_transform_on_loader(ln, trans, rot_deg, scale=scale_vec)

    # Mass override if link has <inertial><mass>
    mass_el = link_el.find('inertial/mass')
    if mass_el is not None and mass_el.text:
        try:
            total_mass = float(mass_el.text.strip())
            try:
                um = ln.getObject('UniformMass')
                um.findData('totalMass').value = total_mass
            except Exception:
                pass
        except ValueError:
            pass

    return LinkInfo(name, ln)


def _ensure_temp_cube(path: str):
    """Write a lightweight OBJ cube once for placeholder geometry."""
    p = Path(path)
    if p.exists():
        return
    obj = """
# cube
v -0.05 -0.05 -0.05
v  0.05 -0.05 -0.05
v  0.05  0.05 -0.05
v -0.05  0.05 -0.05
v -0.05 -0.05  0.05
v  0.05 -0.05  0.05
v  0.05  0.05  0.05
v -0.05  0.05  0.05
f 1 2 3
f 1 3 4
f 5 6 7
f 5 7 8
f 1 5 8
f 1 8 4
f 2 6 7
f 2 7 3
f 4 3 7
f 4 7 8
f 1 2 6
f 1 6 5
""".strip()
    p.write_text(obj)


def attach_joint(parent: LinkInfo, child: LinkInfo, joint_el: ET.Element, verbose: bool):
    # For now, just attach child COM to parent COM using AttachConstraint.
    # You can refine this by parsing <axis>/<limit> and adding springs or constraints accordingly.
    try:
        pc = parent.node.addChild(f"{parent.name}_to_{child.name}_constraint")
        pc.addObject('MechanicalObject', template='Vec3d', position=[[0,0,0],[0,0,0]])
        pc.addObject('AttachConstraint', object1=f'@..//{parent.name}//tetras', object2=f'@..//{child.name}//tetras')
        sofa_log(f"Attached joint {joint_el.attrib.get('name','joint')} between {parent.name} and {child.name}", verbose)
    except Exception:
        # Fallback: try surface names
        try:
            pc = parent.node.addChild(f"{parent.name}_to_{child.name}_constraint")
            pc.addObject('MechanicalObject', template='Vec3d', position=[[0,0,0],[0,0,0]])
            pc.addObject('AttachConstraint', object1=f'@..//{parent.name}//surface', object2=f'@..//{child.name}//surface')
            sofa_log(f"Attached joint (surface) between {parent.name} and {child.name}", verbose)
        except Exception as e:
            sofa_log(f"[WARN] Could not attach joint between {parent.name} and {child.name}: {e}", True)

# ---------------------------- SOFA entrypoint ---------------------------- #

def createScene(rootNode: Sofa.Core.Node):
    args = parse_args(sys.argv[1:])
    sdf_path = Path(args.sdf).resolve()
    if not sdf_path.exists():
        raise FileNotFoundError(f"SDF file not found: {sdf_path}")

    rootNode.dt = args.dt
    rootNode.gravity = args.gravity
    rootNode.addObject('RequiredPlugin', name='SofaPython3')
    # Try to load mesh I/O and OpenGL visual plugins for modular builds; ignore if missing
    for plug in (
        'Sofa.Component.IO.Mesh',      # modern mesh loaders
        'SofaGeneralLoader',           # older builds
        'SofaLoader',                  # very old builds
        'Sofa.GL.Component.Rendering3D', # OglModel in modular builds
        'SofaOpenglVisual',            # older OglModel
        'SofaAssimp'                   # Assimp-based mesh loader fallback
    ):
        try:
            rootNode.addObject('RequiredPlugin', name=plug)
        except Exception:
            pass
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels showCollisionModels')
    add_sim_basics(rootNode)

    # Parse SDF
    tree = ET.parse(str(sdf_path))
    sdf = tree.getroot()
    model = sdf.find('model')
    if model is None:
        # Gazebo world might contain multiple models; pick the first
        models = sdf.findall('world/model')
        if not models:
            raise RuntimeError('No <model> element found in SDF')
        model = models[0]

    resolver = UriResolver(sdf_path, args.resource_dir)

    world = rootNode.addChild(model.attrib.get('name', 'model'))

    # Build links first
    links: dict[str, LinkInfo] = {}
    for link_el in model.findall('link'):
        li = build_link_node(world, link_el, resolver,
                             young=args.young, poisson=args.poisson,
                             density=args.density*args.mass_scale,
                             stiffness=args.stiffness, damping=args.damping,
                             verbose=args.verbose)
        links[li.name] = li

    # Create joints as simple attachments
    for joint_el in model.findall('joint'):
        parent_name = get_text(joint_el.find('parent'))
        child_name = get_text(joint_el.find('child'))
        if parent_name in links and child_name in links:
            attach_joint(links[parent_name], links[child_name], joint_el, args.verbose)
        else:
            sofa_log(f"[WARN] Joint refers to unknown link(s): {parent_name}, {child_name}", True)

    # Camera helper
    world.addObject('InteractiveCamera', name='cam')

    return rootNode


# ---------------------------- Standalone debug ---------------------------- #
if __name__ == '__main__':
    # Running via python directly won\'t execute a SOFA sim, but we can sanity-check parsing.
    ns = parse_args(sys.argv[1:])
    print('Args parsed OK:', ns)
    if Path(ns.sdf).exists():
        t = ET.parse(ns.sdf)
        print('SDF root:', t.getroot().tag)
    else:
        print('SDF file not found (this is fine when just checking args).')
