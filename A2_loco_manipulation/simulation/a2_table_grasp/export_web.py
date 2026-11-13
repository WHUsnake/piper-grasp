"""Export MuJoCo mesh geometry and actual body transforms for browser playback."""
import argparse,json,shutil,subprocess,sys
from pathlib import Path
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('output',type=Path,nargs='?',default=ROOT/'output_tcp95');p.add_argument('--assets',type=Path,default=ROOT/'site/dist/assets');p.add_argument('--skip-geometry',action='store_true');args=p.parse_args()
out=args.output;assets=args.assets;assets.mkdir(parents=True,exist_ok=True)
m=mujoco.MjModel.from_xml_path(str(out/'scene.xml'));d=mujoco.MjData(m)
frames=json.loads((out/'frames.json').read_text())
initial=np.load(out/'initial_qpos.npy') if (out/'initial_qpos.npy').is_file() else np.array(frames[0]['qpos']);initial[:6]=0;initial[6:8]=[.035,-.035]
d.qpos[:]=initial;mujoco.mj_forward(m,d)
chunks=[];offset=0;meshes=[]
for i in range(m.nmesh):
    va,vn=m.mesh_vertadr[i],m.mesh_vertnum[i];fa,fn=m.mesh_faceadr[i],m.mesh_facenum[i]
    vertices=np.asarray(m.mesh_vert[va:va+vn],dtype='<f4');faces=np.asarray(m.mesh_face[fa:fa+fn],dtype='<u4')
    record=dict(position_offset=offset,position_count=vertices.size,index_offset=offset+vertices.nbytes,index_count=faces.size)
    chunks += [vertices.tobytes(),faces.tobytes()];offset+=vertices.nbytes+faces.nbytes;meshes.append(record)
if not args.skip_geometry:(assets/'geometry.bin').write_bytes(b''.join(chunks))
geoms=[]
for i in range(m.ngeom):
    typ=int(m.geom_type[i]);group=int(m.geom_group[i])
    if group==3:continue
    mat=m.geom_matid[i];rgba=m.mat_rgba[mat] if mat>=0 else m.geom_rgba[i]
    geoms.append(dict(name=m.geom(i).name or f'geom_{i}',body=int(m.geom_bodyid[i]),type=typ,
                      mesh=int(m.geom_dataid[i]) if typ==int(mujoco.mjtGeom.mjGEOM_MESH) else -1,
                      size=m.geom_size[i].tolist(),position=m.geom_pos[i].tolist(),quaternion=m.geom_quat[i][[1,2,3,0]].tolist(),rgba=rgba.tolist()))
scene=dict(meshes=meshes,geoms=geoms,bodies=[m.body(i).name for i in range(m.nbody)],
           initial_positions=d.xpos.round(6).tolist(),initial_quaternions=d.xquat[:,[1,2,3,0]].round(6).tolist())
(assets/'scene.json').write_text(json.dumps(scene,separators=(',',':')))
for frame in frames:
    d.qpos[:]=frame.pop('qpos');mujoco.mj_forward(m,d)
    frame['positions']=d.xpos.round(6).tolist();frame['quaternions']=d.xquat[:,[1,2,3,0]].round(6).tolist()
(assets/'motion.json').write_text(json.dumps(frames,separators=(',',':')))
for name in ['overview.mp4','closeup.mp4','initial.png','result.png','camera_rgb.png','mask.png','metrics.json','trace.json','pointcloud.json','proposals.json','timeline.json','detected_objects.json']:
    if (out/name).is_file():shutil.copyfile(out/name,assets/name)
subprocess.run([sys.executable,str(ROOT/'render_dc1_preview.py'),str(out),'--output',str(assets/'dc1_initial.png'),'--video',str(assets/'dc1_view.mp4')],check=True,stdout=subprocess.DEVNULL)
if not (assets/'result.png').is_file():shutil.copyfile(out/'initial.png',assets/'result.png')
print(json.dumps(dict(geoms=len(geoms),meshes=len(meshes),geometry_bytes=offset,frames=len(frames),duration=frames[-1]['t'])))
