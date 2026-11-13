"""Render an initial DaBai DC1 wrist-view preview from a saved simulation."""
import argparse,json,os,xml.etree.ElementTree as ET
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import imageio.v2 as imageio
import mujoco,numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent
profile=json.loads((ROOT/'site/dist/dc1_profile.json').read_text())
p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path);p.add_argument('--video',type=Path);a=p.parse_args()
root=ET.parse(a.run/'scene.xml').getroot();link=root.find(".//body[@name='p_link6']")
if link is None:raise ValueError('p_link6 body missing')
q=Rotation.from_quat(profile['quaternion_xyzw'])*Rotation.from_euler('x',180,degrees=True)
xyzw=q.as_quat();wxyz=xyzw[[3,0,1,2]]
ET.SubElement(link,'camera',name='dc1_preview',pos=' '.join(map(str,profile['translation_m'])),quat=' '.join(map(str,wxyz)),fovy=str(profile['fovy_deg']))
m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m)
qpos=np.load(a.run/'initial_qpos.npy');d.qpos[:len(qpos)]=qpos;mujoco.mj_forward(m,d)
r=mujoco.Renderer(m,width=profile['color_width'],height=profile['color_height']);r.update_scene(d,camera='dc1_preview')
out=a.output or a.run/'web/assets/dc1_initial.png';out.parent.mkdir(parents=True,exist_ok=True);imageio.imwrite(out,r.render().copy());print(out)
if a.video:
    frames=json.loads((a.run/'frames.json').read_text())
    a.video.parent.mkdir(parents=True,exist_ok=True)
    with imageio.get_writer(a.video,fps=25,codec='libx264',quality=7) as writer:
        for frame in frames:
            d.qpos[:]=frame['qpos'];mujoco.mj_forward(m,d);r.update_scene(d,camera='dc1_preview')
            writer.append_data(r.render().copy())
    print(a.video)
r.close()
