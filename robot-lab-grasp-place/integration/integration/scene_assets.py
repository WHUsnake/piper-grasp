"""Optional authored scene and official robot assets; no control/perception truth injection."""

import copy, json
import xml.etree.ElementTree as ET
from pathlib import Path
import argparse, hashlib, json, urllib.request, xml.etree.ElementTree as ET


def export_robot_visual(model, data, output):
    """Export flat body nodes animated by the authoritative MuJoCo poses."""
    import mujoco
    import numpy as np
    import trimesh
    scene = trimesh.Scene()
    for bid in range(1, model.nbody):
        scene.graph.update(frame_to='body_'+str(bid), matrix=np.eye(4))
    for gid in range(model.ngeom):
        bid = int(model.geom_bodyid[gid])
        if not bid:
            continue
        kind = int(model.geom_type[gid])
        if kind == mujoco.mjtGeom.mjGEOM_MESH:
            mid = int(model.geom_dataid[gid])
            v = model.mesh_vert[model.mesh_vertadr[mid]:model.mesh_vertadr[mid]+model.mesh_vertnum[mid]]
            f = model.mesh_face[model.mesh_faceadr[mid]:model.mesh_faceadr[mid]+model.mesh_facenum[mid]]
        elif model.geom_group[gid] == 2:
            # Explicit primitive visuals, such as the existing grasp bottle.
            # Robot collision-only primitives remain hidden.
            size = model.geom_size[gid]
            if kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
                primitive = trimesh.creation.cylinder(radius=size[0],height=2*size[1],sections=48)
            elif kind == mujoco.mjtGeom.mjGEOM_BOX:
                primitive = trimesh.creation.box(extents=2*size)
            elif kind in (mujoco.mjtGeom.mjGEOM_SPHERE,mujoco.mjtGeom.mjGEOM_ELLIPSOID):
                primitive = trimesh.creation.icosphere(subdivisions=3)
                primitive.vertices *= size[0] if kind == mujoco.mjtGeom.mjGEOM_SPHERE else size
            else:
                continue
            v, f = primitive.vertices, primitive.faces
        else:
            continue
        points = v@data.geom_xmat[gid].reshape(3,3).T+data.geom_xpos[gid]
        points = (points-data.xpos[bid])@data.xmat[bid].reshape(3,3)
        mesh = trimesh.Trimesh(points, f, process=False)
        material = model.geom_matid[gid]
        color = model.mat_rgba[material] if material >= 0 else model.geom_rgba[gid]
        mesh.visual.face_colors = (color*255).astype(np.uint8)
        scene.add_geometry(mesh, node_name='geom_'+str(gid), parent_node_name='body_'+str(bid))
    scene.export(output)


def install(root, scene_manifest=None, robot_visual=None):
    world = root.find("worldbody")
    base = world.find("body[@name='base']")
    if robot_visual:
        robot = ET.parse(robot_visual).getroot()
        root.find("asset").extend(copy.deepcopy(list(robot.find("asset"))))
        item = copy.deepcopy(robot.find("worldbody/body"))
        for geom in item.iter("geom"):
            geom.set("group", "3")
        base.append(item)
        envelope = base.find("geom[@name='a2_envelope']")
        envelope.set("rgba", "0 0 0 0")
    if not scene_manifest:
        return None
    path = Path(scene_manifest)
    scene = json.loads(path.read_text())
    if (
        scene.get("schema_version") != 1
        or scene.get("coordinate_system") != "RH_Z_UP_METRES"
    ):
        raise ValueError("Unsupported scene coordinates")
    for name in ("floor", "table"):
        world.remove(world.find("geom[@name='%s']" % name))
    environment = ET.parse(path.parent / scene["mujoco_environment"]).getroot()
    for asset in environment.findall("asset/*"):
        item = copy.deepcopy(asset)
        if item.get("file"):
            item.set("file", str((path.parent / item.get("file")).resolve()))
        root.find("asset").append(item)
    if scene.get("lighting"):
        for light in list(world.findall("light")):
            world.remove(light)
        visual = root.find("visual")
        if visual is None:
            visual = ET.SubElement(root, "visual")
        headlight = visual.find("headlight")
        if headlight is None:
            headlight = ET.SubElement(visual, "headlight")
        for key, value in scene["lighting"]["headlight"].items():
            headlight.set(key, value)
    world.extend(copy.deepcopy(list(environment.find("worldbody"))))
    dynamics = path.parent / "movable-objects.json"
    if dynamics.exists():
        spec = json.loads(dynamics.read_text())
        if spec["scene_sha256"] != hashlib.sha256(path.read_bytes()).hexdigest():
            raise ValueError("Movable object resources belong to another scene")
        for obj in spec["objects"]:
            node = ET.SubElement(
                world, "body", name=obj["body"], pos=" ".join(map(str, obj["center"]))
            )
            ET.SubElement(node, "freejoint", name=obj["body"] + "_free")
            ET.SubElement(
                node,
                "inertial",
                pos="0 0 0",
                mass=str(obj["mass"]),
                diaginertia=" ".join(map(str, obj["inertia"])),
            )
            for name in obj["geoms"]:
                geom = world.find("geom[@name='%s']" % name)
                if geom is None:
                    raise ValueError("Movable source geometry missing: " + name)
                world.remove(geom)
                geom.set("pos", " ".join(str(-x) for x in obj["center"]))
                node.append(geom)
    world.find("body[@name='bottle']").set(
        "pos", " ".join(map(str, scene["bottle_position_m"]))
    )
    return scene


def prepare_movable_objects(scene_path, dataset):
    """Group small source-dynamic resources into independent physical bodies.

    Preserve source mass and convex parts. Inertia uses the documented bounding
    box proxy when the original resource supplies no inertia tensor.
    """
    import numpy as np
    from integration.replicacad import mesh_parts, pose

    scene_path = Path(scene_path)
    dataset = Path(dataset)
    scene = json.loads(scene_path.read_text())
    source = (
        dataset / "configs/scenes" / (scene["source_scene"] + ".scene_instance.json")
    )
    data = json.loads(source.read_text())
    conversion = np.array(scene["registration"])
    removed = {
        o["instance_id"]
        for o in scene.get("scene_modifications", {}).get("removed_instances", [])
    }
    stage_path = (
        dataset
        / "configs/stages"
        / (data["stage_instance"]["template_name"] + ".stage_config.json")
    )
    stage = json.loads(stage_path.read_text())
    serial = len(
        list(
            mesh_parts(
                (stage_path.parent / stage["render_asset"]).resolve(),
                conversion @ pose(data["stage_instance"]),
            )
        )
    )
    collision = 0
    objects = []
    for oid, instance in enumerate(data["object_instances"], 10):
        if oid in removed:
            continue
        config = (
            dataset / "configs" / (instance["template_name"] + ".object_config.json")
        )
        desc = json.loads(config.read_text())
        transform = conversion @ pose(instance)
        parts = list(
            mesh_parts((config.parent / desc["render_asset"]).resolve(), transform)
        )
        convex = list(
            mesh_parts(
                (
                    config.parent / desc.get("collision_asset", desc["render_asset"])
                ).resolve(),
                transform,
            )
        )
        lo = np.min([m.bounds[0] for m in parts], axis=0)
        hi = np.max([m.bounds[1] for m in parts], axis=0)
        size = hi - lo
        mass = desc.get("mass")
        if (
            instance.get("motion_type") == "DYNAMIC"
            and isinstance(mass, (int, float))
            and mass > 0
            and np.max(size) <= 0.65
            and np.prod(size) <= 0.03
            and len(convex) > 0
        ):
            centre = (lo + hi) / 2
            inertia = (
                mass
                / 12
                * np.array(
                    [
                        size[1] ** 2 + size[2] ** 2,
                        size[0] ** 2 + size[2] ** 2,
                        size[0] ** 2 + size[1] ** 2,
                    ]
                )
            )
            objects.append(
                dict(
                    body="scene_object_" + str(oid),
                    source_instance=oid,
                    mass=mass,
                    center=centre.tolist(),
                    size=size.tolist(),
                    inertia=np.maximum(inertia, 1e-7).tolist(),
                    geoms=[f"visual_{i}" for i in range(serial, serial + len(parts))]
                    + [
                        f"collision_{i}"
                        for i in range(collision, collision + len(convex))
                    ],
                    collision_bounds=[
                        dict(
                            position=m.bounds.mean(0).tolist(),
                            size=np.maximum(m.extents, 0.005).tolist(),
                        )
                        for m in convex
                    ],
                    inertia_source="bounding_box_with_source_mass",
                )
            )
        serial += len(parts)
        collision += len(convex)
    result = dict(
        schema_version=1,
        scene_sha256=hashlib.sha256(scene_path.read_bytes()).hexdigest(),
        objects=objects,
    )
    destination = scene_path.parent / "movable-objects.json"
    temporary = destination.with_suffix(".tmp")
    temporary.write_text(json.dumps(result, indent=2))
    temporary.replace(destination)
    return result


REVISION = "4005a1b0c3f464f515f380be4145cc617fbca725"
SOURCE = (
    "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/" + REVISION + "/"
)


def export_a2(output):
    import numpy as np
    import trimesh
    from scipy.spatial.transform import Rotation

    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    hashes = {}

    def fetch(remote, local):
        p = output / local
        p.parent.mkdir(parents=True, exist_ok=True)
        data = urllib.request.urlopen(SOURCE + remote, timeout=60).read()
        p.write_bytes(data)
        hashes[remote] = hashlib.sha256(data).hexdigest()
        return p

    fetch("LICENSE", "LICENSE")
    xml = fetch("robots/a2_description/a2.xml", "source/a2.xml")
    fetch("robots/a2_description/urdf/a2.urdf", "source/a2.urdf")
    root = ET.parse(xml).getroot()
    meshes = {}
    for m in root.findall("asset/mesh"):
        file = m.get("file")
        meshes[m.get("name")] = fetch(
            "robots/a2_description/meshes/" + file, "source/meshes/" + file
        )
    body = root.find("worldbody/body")
    records = []
    parts = []

    def walk(node, parent):
        local = np.eye(4)
        local[:3, 3] = np.fromstring(node.get("pos", "0 0 0"), sep=" ")
        q = np.fromstring(node.get("quat", "1 0 0 0"), sep=" ")
        local[:3, :3] = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
        for j in node.findall("joint"):
            if j.get("type") == "free":
                continue
            angle = (
                0.837
                if "thigh" in j.get("name", "")
                else -1.674 if "calf" in j.get("name", "") else 0.0
            )
            axis = np.fromstring(j.get("axis", "0 0 1"), sep=" ")
            local[:3, :3] = (
                local[:3, :3] @ Rotation.from_rotvec(axis * angle).as_matrix()
            )
        if node is body:
            local = np.eye(4)
        matrix = parent @ local
        for g in node.findall("geom"):
            if g.get("type") != "mesh":
                continue
            mesh = trimesh.load(meshes[g.get("mesh")], force="mesh", process=False)
            gp = np.fromstring(g.get("pos", "0 0 0"), sep=" ")
            gq = np.fromstring(g.get("quat", "1 0 0 0"), sep=" ")
            transform = np.eye(4)
            transform[:3, 3] = gp
            transform[:3, :3] = Rotation.from_quat(gq[[1, 2, 3, 0]]).as_matrix()
            mesh.apply_transform(matrix @ transform)
            rgba = np.fromstring(g.get("rgba", ".7 .7 .7 1"), sep=" ")
            parts.append((mesh, rgba))
        for child in node.findall("body"):
            walk(child, matrix)

    walk(body, np.eye(4))
    all_vertices = np.concatenate([m.vertices for m, c in parts])
    bounds = [all_vertices.min(0).tolist(), all_vertices.max(0).tolist()]
    if not (
        0.6 < np.ptp(all_vertices[:, 0]) < 1.5
        and 0.3 < np.ptp(all_vertices[:, 1]) < 1.0
    ):
        raise ValueError("A2 dimensions out of expected metre range")
    visual = ET.Element("mujoco")
    assets = ET.SubElement(visual, "asset")
    world = ET.SubElement(visual, "worldbody")
    base = ET.SubElement(world, "body", name="a2_official_fixed")
    for i, (mesh, color) in enumerate(parts):
        file = f"part-{i}.obj"
        mesh.export(output / file)
        ET.SubElement(
            assets, "mesh", name=f"a2_visual_{i}", file=str((output / file).resolve())
        )
        ET.SubElement(
            base,
            "geom",
            type="mesh",
            mesh=f"a2_visual_{i}",
            contype="0",
            conaffinity="0",
            density="0",
            rgba=" ".join(map(str, color)),
        )
        triangles = mesh.triangles.copy()
        normals = np.repeat(mesh.face_normals[:, None, :], 3, axis=1)
        data = np.concatenate(
            [
                triangles * [100, -100, 100],
                normals * [1, -1, 1],
                np.broadcast_to(color, (*triangles.shape[:2], 4)),
            ],
            axis=2,
        )
        binfile = f"part-{i}.bin"
        data.astype("<f4").tofile(output / binfile)
        records.append(
            {"mesh": binfile, "position": [0, 0, 0], "rotation": [0, 0, 0, 1]}
        )
    ET.ElementTree(visual).write(output / "visual.xml")
    manifest = {
        "model": "Unitree A2",
        "source": "https://github.com/unitreerobotics/unitree_ros",
        "revision": REVISION,
        "license": "BSD-3-Clause",
        "sha256": hashes,
        "bounds_m": bounds,
        "standing_base_height_cm": 40.0,
        "standing_height_cm": 100 * (bounds[1][2] - bounds[0][2]),
        "scope": "Official appearance; fixed standing pose; kinematic base; provisional arm mount",
        "links": [
            {
                "name": "a2_fixed",
                "parent": "",
                "position": [0, 0, 0],
                "rotation": [0, 0, 0, 1],
                "axis": [0, 0, 1],
                "angle": 0,
                "limits": [0, 0],
                "visuals": records,
            }
        ],
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def main():
    p = argparse.ArgumentParser(description="Build pinned official A2 visual assets")
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    print(json.dumps(export_a2(a.output), indent=2))


if __name__ == "__main__":
    main()
