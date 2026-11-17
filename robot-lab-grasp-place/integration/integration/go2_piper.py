"""Assemble the pinned Go2-Piper model, MoE-CTS policy and HSSD bottle scene."""

import copy
import hashlib
import xml.etree.ElementTree as ET

def install_robot(root, robot_xml):
    """Replace the mocap body with the training robot and retain the scene and jaw interface."""
    robot = ET.parse(robot_xml).getroot()
    world = root.find("worldbody")
    old_base = world.find("body[@name='base']")
    old_motors = root.find("actuator")
    base = robot.find("worldbody/body[@name='base_link']")
    base.set("name", "base")
    base.set("pos", "0 0 .35")
    base.attrib.pop("euler")
    base.find("freejoint").set("name", "go2_base_free")
    base.append(copy.deepcopy(old_base.find("camera[@name='body_rgbd']")))
    mount = base.find("body[@name='Piper']")
    mount.set("name", "arm_base")
    wrist = mount.find(".//body[@name='link6']")
    wrist.append(copy.deepcopy(old_base.find(".//camera[@name='wrist_rgbd']")))
    for i in (7, 8):
        name = f"link{i}"
        jaw = wrist.find(f"body[@name='{name}']")
        # Both MJCFs have the same finger body transforms. Reuse the vendor slides and servos.
        joint = copy.deepcopy(old_base.find(f".//body[@name='{name}']/joint"))
        joint.set("armature", "0")
        joint.set("frictionloss", "0")
        jaw.append(joint)
        jaw.find("geom").set("name", f"{name}_collision")
        motor = copy.deepcopy(old_motors.find(f"position[@joint='joint{i}']"))
        robot.find("actuator").append(motor)
    for geom in base.iter("geom"):
        geom.set("group", "3")
    world.remove(old_base)
    world.append(base)
    # The vendor and Go2 mesh names overlap. The old robot's mesh assets are no longer used.
    assets = root.find("asset")
    for mesh in list(assets.findall("mesh")):
        assets.remove(mesh)
    for item in robot.find("asset"):
        if item.get("file"):
            item.set("file", str((robot_xml.parent / item.get("file")).resolve()))
        assets.append(item)
    root.remove(old_motors)
    root.append(robot.find("actuator"))
    root.append(robot.find("default"))
    root.find("option").attrib.update(robot.find("option").attrib)
    root.find("option").set("timestep", ".005")
    root.find("option").set("impratio", "10")
    root.find("worldbody/geom[@name='floor']").set("friction", "1.5 .5 .5")
    root.set("model", "go2_piper_integration")


def add_tabletop_objects(world, bottle, objects):
    import math
    names = {body.get('name') for body in world.iter('body')}
    for item in objects:
        name = item['body_name']
        if not name.startswith('tabletop_') or name in names:
            raise ValueError('Tabletop body name must be unique and start with tabletop_')
        position, rgba = item['position'], item['rgba']
        if (len(position) != 3 or len(rgba) != 4 or
                not all(math.isfinite(value) for value in position+rgba) or
                not all(0 <= value <= 1 for value in rgba)):
            raise ValueError('Invalid tabletop position or color')
        if item['shape'] == 'bottle':
            body = copy.deepcopy(bottle)
            body.set('name', name)
            body.find('freejoint').set('name', name+'_free')
            for geom in body.findall('geom'):
                original = geom.get('name')
                geom.set('name', name+'_'+original)
                if original in ('bottle_body', 'bottle_shoulder', 'bottle_neck'):
                    geom.set('rgba', ' '.join(map(str, rgba)))
        elif item['shape'] in ('box', 'cylinder'):
            size = item['size']
            if (len(size) != (3 if item['shape'] == 'box' else 2) or
                    not all(math.isfinite(value) and value > 0 for value in size) or
                    not math.isfinite(item['mass_kg']) or item['mass_kg'] <= 0):
                raise ValueError('Invalid tabletop object dimensions or mass')
            body = ET.Element('body', name=name)
            ET.SubElement(body, 'freejoint', name=name+'_free')
            ET.SubElement(body, 'geom', name=name+'_collision', type=item['shape'],
                size=' '.join(map(str, size)), mass=str(item['mass_kg']),
                rgba=' '.join(map(str, rgba)), group='2', condim='4', priority='1',
                solref='.02 1', friction='1 .01 .003')
        else:
            raise ValueError('Unsupported tabletop shape')
        body.set('pos', ' '.join(map(str, position)))
        world.append(body)
        names.add(name)


def build_preview_assets(workspace, output=None):
    """Use the pinned training model and selected controller, retaining HSSD collisions."""
    import json
    import shutil
    import subprocess
    import tempfile
    from pathlib import Path
    from integration.paths import ROOT
    from integration.world import build_scene
    from integration.scene_assets import export_robot_visual
    ws = Path(workspace)
    target, source = Path(output) if output else ws/'assets/hssd-go2-piper', ws/'assets/hssd-go2'
    if target.resolve().parent != (ws/'assets').resolve():
        raise ValueError('Asset output must be directly inside the external workspace assets directory')
    if target.exists():
        raise RuntimeError('Preserve the existing Go2/Piper bundle before rebuilding: '+str(target))
    spec = json.loads((ROOT.parent/'profiles/hssd-go2-piper.json').read_text())
    training = ws/'sources/LeggedManip_Lab'
    revision = subprocess.check_output(['git','-C',str(training),'rev-parse','HEAD']).decode().strip()
    if revision != spec['model_commit']:
        raise ValueError('Go2/Piper training model revision mismatch')
    piper = ws/'sources/A2_loco_manipulation/third_party/piper_ros/src/piper_description/mujoco_model/piper_description.xml'
    robot_xml = training/'mujoco/robots/go2_piper/go2piper.xml'
    root = ET.fromstring(build_scene(piper))
    bottle = copy.deepcopy(root.find("worldbody/body[@name='bottle']"))
    # Replace the complete old robot, including its base drive.
    world = root.find('worldbody')
    for child in list(world):
        if child.tag == 'body' and child.get('name') not in ('base',):
            world.remove(child)
    equality = root.find('equality')
    if equality is not None:
        root.remove(equality)
    install_robot(root, robot_xml)
    for child in list(world):
        if child.tag == 'geom':
            world.remove(child)
    env = ET.parse(source/'scene.xml').getroot()
    static = env.findall('worldbody/geom')
    world.extend(copy.deepcopy(static))
    placement = spec.get('placed_bottle')
    if placement:
        bottle.set('pos',' '.join(map(str,placement['position'])))
        for geom in bottle.findall('geom'):
            geom.set('group','2')
            if geom.get('contype','1') != '0':
                geom.attrib.update(condim='4',priority='1',solref='.02 1',friction='1 .01 .003')
        # Append after the robot: its free joint must keep qpos[0:7].
        world.append(bottle)
    add_tabletop_objects(world, bottle, spec.get('tabletop_objects', []))
    meshes = {g.get('mesh') for g in static if g.get('mesh')}
    materials = {g.get('material') for g in static if g.get('material')}
    textures = {m.get('texture') for m in env.findall('asset/material') if m.get('name') in materials}
    assets = root.find('asset')
    imported = [copy.deepcopy(x) for x in env.find('asset') if
                (x.tag == 'mesh' and x.get('name') in meshes) or
                (x.tag == 'material' and x.get('name') in materials) or
                (x.tag == 'texture' and x.get('name') in textures)]
    root.find('compiler').set('meshdir','.')
    size = root.find('size')
    if size is not None:
        root.remove(size)
    with tempfile.TemporaryDirectory(prefix='.hssd-go2-piper-',dir=str(target.parent)) as temp:
        out = Path(temp)/'bundle'
        shutil.copytree(source,out,ignore=shutil.ignore_patterns('robot','bundle.json'))
        (out/'robot/model').mkdir(parents=True)
        inputs = {}
        for mesh in assets.findall('mesh'):
            file = Path(mesh.get('file'))
            mesh.set('name',mesh.get('name',file.stem))
            rel = 'robot/model/'+file.name
            shutil.copy2(file,out/rel)
            inputs[str(file)] = hashlib.sha256(file.read_bytes()).hexdigest()
            mesh.set('file',rel)
        assets.extend(imported)
        ET.ElementTree(root).write(out/'scene.xml')
        policy = (source/'robot/policy.pt').read_bytes()
        shutil.copy2(source/'robot/policy_config.json',out/'robot/policy_config.json')
        (out/'robot/arm_hold.json').write_text(json.dumps(spec['arm_hold'],indent=2))
        if hashlib.sha256(policy).hexdigest() != spec['policy_sha256']:
            raise ValueError('Locomotion policy checksum mismatch')
        (out/'robot/policy.pt').write_bytes(policy)
        shutil.copy2(training/'LICENSE',out/'licenses/LeggedManip-LICENSE')
        for file in (robot_xml,):
            inputs[str(file)] = hashlib.sha256(file.read_bytes()).hexdigest()
        provenance = {'model_commit':revision,
                      'policy_sha256':spec['policy_sha256'],'controller':spec['motion_mode'],'inputs':inputs}
        (out/'robot/sources.json').write_text(json.dumps(provenance,indent=2))
        from integration.go2 import Go2PiperMoeWorld
        preview = Go2PiperMoeWorld(out)
        if placement:
            manifest = json.loads((out/'manifest.json').read_text())
            manifest['placed_objects'] = [dict(placement,body_name='bottle',
                body_id=preview.model.body('bottle').id,source='integration/integration/world.py')]
            manifest['placed_objects'].extend(dict(item,
                body_id=preview.model.body(item['body_name']).id,
                support_table=item.get('support_table', placement['support_table']),
                source='profiles/hssd-go2-piper.json') for item in spec.get('tabletop_objects', []))
            manifest['manipulation_surfaces'] = spec.get('grasp_surfaces', [])
            (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
        export_robot_visual(preview.model,preview.data,out/'robot/go2_piper.glb')
        lock = {'schema_version':1,'scene':spec['scene'],'robot':'go2-piper',
                'locomotion_ready':True,'motion_ready':True,'motion_mode':spec['motion_mode'],
                'policy_sha256':spec['policy_sha256'],'files':{}}
        for file in out.rglob('*'):
            if file.is_file():
                lock['files'][str(file.relative_to(out))] = hashlib.sha256(file.read_bytes()).hexdigest()
        (out/'bundle.json').write_text(json.dumps(lock,indent=2))
        shutil.move(str(out),target)
    return str(target)
