"""A2 + PiPER fixed-standing MuJoCo table grasp. No robot communications."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse, copy, hashlib, json, sys, time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
import mujoco
import imageio.v2 as imageio
ROOT=Path(__file__).resolve().parent
SHE=ROOT.parents[2]
A2=SHE/'A2_Piper_WBC/vendor/unitree_ros/robots/a2_description'
PIPER=SHE/'PiPER抓取完整项目/PiPER抓取项目/vendor/agilex_piper'
sys.path.insert(0,str(SHE/'A2_loco_manipulation/src/manipulation/grasp_generation'))
from grasp_generation.worker_client import Worker
from grasp_generation.graspnet_backend import adapt
from grasp_refinement import refine_candidates,FINGER_TIP_FROM_LINK6_M


def vec(v):return ' '.join(map(str,v))
def quat(r):
    q=r.as_quat();return vec(q[[3,0,1,2]])


def build(out, bottle_x=.72, table_z=.76, tcp_m=.095, bottle_y=0.):
    root=ET.parse(A2/'a2.xml').getroot();root.set('model','A2_PiPER_table_grasp')
    root.find('compiler').set('meshdir',str(A2/'meshes'))
    root.find('option').set('timestep','.002');root.find('option').set('integrator','implicitfast')
    root.find('option').set('cone','elliptic')
    for mesh in root.findall('asset/mesh'):mesh.set('file',str(A2/'meshes'/mesh.get('file')))
    wb=root.find('worldbody');base=wb.find("body[@name='base_link']");base.set('pos','0 0 .45')
    # Freeze the official joint chain in a standing configuration. This is NOT a balance controller.
    base.remove(base.find("joint[@type='free']"))
    for body in base.iter('body'):
        joint=body.find('joint')
        if joint is None:continue
        angle=0 if 'hip' in joint.get('name') else .8 if 'thigh' in joint.get('name') else -1.6
        wxyz=np.fromstring(body.get('quat','1 0 0 0'),sep=' ')
        r=Rotation.from_quat(wxyz[[1,2,3,0]])*Rotation.from_rotvec(np.fromstring(joint.get('axis'),sep=' ')*angle)
        body.set('quat',quat(r));body.remove(joint)
    for tag in ['actuator','sensor']:
        if root.find(tag) is not None:root.remove(root.find(tag))
    # Prefix PiPER resources so A2's base_link/collision classes remain distinct.
    arm=ET.parse(PIPER/'piper.xml').getroot()
    for mesh in arm.findall('asset/mesh'):
        mesh.set('name',mesh.get('name',Path(mesh.get('file')).stem))
        mesh.set('file',str(PIPER/'assets'/mesh.get('file')))
    names=set()
    for element in arm.iter():
        if element.get('name'): names.add(element.get('name'))
        if element.tag=='default' and element.get('class'):names.add(element.get('class'))
    for element in arm.iter():
        for attr in ['name','class','childclass','mesh','material','joint','joint1','joint2','body1','body2']:
            if element.get(attr) in names:element.set(attr,'p_'+element.get(attr))
    for child in arm.find('default'):root.find('default').append(copy.deepcopy(child))
    for child in arm.find('asset'):root.find('asset').append(copy.deepcopy(child))
    mount=arm.find('worldbody/body');mount.set('pos','.0899 0 .1416')
    base.append(copy.deepcopy(mount))
    # Use the previously recorded estimated mount, not a new calibration claim.
    for tag in ['contact','equality','actuator']:
        if arm.find(tag) is not None:
            target=root.find(tag)
            if target is None:target=ET.SubElement(root,tag)
            for child in arm.find(tag):target.append(copy.deepcopy(child))
    for i in range(1,7):
        a=root.find(f"actuator/position[@name='p_joint{i}']")
        a.set('kp',str([300,450,400,180,180,100][i-1]));a.set('kv','25')
    a=root.find("actuator/position[@name='p_gripper']");a.set('kp','350');a.set('kv','8')
    a.set('forcerange','-15 15')
    tip=root.find(".//body[@name='p_link6']")
    ET.SubElement(tip,'site',name='tcp',pos=vec([0,0,tcp_m]),size='.006',rgba='0 .95 .75 1')
    for g in root.findall(".//body[@name='p_link7']/geom")+root.findall(".//body[@name='p_link8']/geom"):
        if g.get('class')=='p_collision':g.set('friction','1.2 .01 .001');g.set('condim','4')
    ET.SubElement(root,'visual').append(ET.Element('global',offwidth='1280',offheight='720'))
    ET.SubElement(root.find('visual'),'quality',shadowsize='2048')
    ET.SubElement(root.find('visual'),'headlight',ambient='.35 .35 .35',diffuse='.65 .65 .65',specular='.15 .15 .15')
    ET.SubElement(root.find('visual'),'rgba',haze='.12 .16 .21 1')
    ET.SubElement(wb,'light',pos='0 -2 3',dir='0 .3 -1',diffuse='.9 .9 .9',castshadow='true')
    ET.SubElement(wb,'light',pos='2 1 2',diffuse='.45 .5 .6')
    ET.SubElement(wb,'geom',name='floor',type='plane',size='4 4 .05',rgba='.12 .16 .21 1',friction='1 .01 .001')
    ET.SubElement(wb,'geom',name='table_top',type='box',pos=vec([bottle_x+.20,0,table_z-.025]),size='.28 .40 .025',rgba='.60 .43 .27 1',friction='1 .01 .001')
    for xi,x in enumerate([bottle_x-.01,bottle_x+.41]):
        for yi,y in enumerate([-.32,.32]):ET.SubElement(wb,'geom',name=f'table_leg_{xi}_{yi}',type='box',pos=vec([x,y,(table_z-.05)/2]),size=vec([.022,.022,(table_z-.05)/2]),rgba='.18 .23 .28 1')
    bottle=ET.SubElement(wb,'body',name='bottle',pos=vec([bottle_x,bottle_y,table_z+.105]))
    ET.SubElement(bottle,'freejoint',name='bottle_free')
    ET.SubElement(bottle,'geom',name='bottle_body',type='cylinder',size='.026 .09',pos='0 0 -.015',mass='.18',rgba='.12 .63 .73 1',friction='1 .01 .001',condim='4')
    ET.SubElement(bottle,'geom',name='bottle_shoulder',type='ellipsoid',size='.026 .026 .022',pos='0 0 .073',mass='.01',rgba='.15 .68 .77 1')
    ET.SubElement(bottle,'geom',name='bottle_neck',type='cylinder',size='.014 .017',pos='0 0 .095',mass='.005',rgba='.18 .72 .80 1')
    ET.SubElement(bottle,'geom',name='bottle_cap',type='cylinder',size='.016 .008',pos='0 0 .117',mass='.005',rgba='.96 .77 .23 1')
    ET.SubElement(bottle,'geom',name='bottle_label',type='cylinder',size='.0262 .03',pos='0 0 -.005',contype='0',conaffinity='0',mass='0',rgba='.83 .94 .94 1')
    # Virtual RGB-D camera on a fixed experimental tripod, looking toward the front surface.
    camera_pos=np.array([min(.35,bottle_x-.34),-.04,table_z+.11]);look=np.array([bottle_x,bottle_y,table_z+.10])
    z=(camera_pos-look);z/=np.linalg.norm(z);x=np.cross([0,0,1],z);x/=np.linalg.norm(x);y=np.cross(z,x)
    ET.SubElement(wb,'camera',name='depth_camera',pos=vec(camera_pos),xyaxes=vec(np.r_[x,y]),fovy='48')
    xml=ET.tostring(root,encoding='unicode');model=mujoco.MjModel.from_xml_string(xml)
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    # Set standing height from the official feet positions, so all four feet rest on the ground.
    feet=[i for i in range(model.ngeom) if model.geom_type[i]==int(mujoco.mjtGeom.mjGEOM_SPHERE) and model.geom_size[i,0]>.03]
    lowest=min(data.geom_xpos[i,2]-model.geom_size[i,0] for i in feet)
    base.set('pos',vec([0,0,.45-lowest]));xml=ET.tostring(root,encoding='unicode')
    (out/'scene.xml').write_text(xml)
    return mujoco.MjModel.from_xml_string(xml)


def capture(m,d,out):
    r=mujoco.Renderer(m,height=480,width=640)
    r.update_scene(d,camera='depth_camera');rgb=r.render().copy()
    r.enable_depth_rendering();r.update_scene(d,camera='depth_camera');depth=r.render().copy()
    r.disable_depth_rendering();r.enable_segmentation_rendering();r.update_scene(d,camera='depth_camera');seg=r.render().copy();r.close()
    ids=[m.geom(n).id for n in ['bottle_body','bottle_shoulder','bottle_neck','bottle_cap','bottle_label']]
    mask=np.isin(seg[:,:,0],ids)&(seg[:,:,1]==int(mujoco.mjtObj.mjOBJ_GEOM))
    v,u=np.nonzero(mask&(depth>.05)&(depth<2))
    z=depth[v,u];f=240/np.tan(np.deg2rad(48)/2)
    camera=np.column_stack([(u-319.5)*z/f,(v-239.5)*z/f,z])
    cid=m.camera('depth_camera').id;rot=d.cam_xmat[cid].reshape(3,3)@np.diag([1,-1,-1]);trans=d.cam_xpos[cid].copy()
    world=camera@rot.T+trans
    np.savez_compressed(out/'observation.npz',points_camera_m=camera,points_world_m=world,depth_m=depth,mask=mask,rotation=rot,translation=trans)
    imageio.imwrite(out/'camera_rgb.png',rgb);imageio.imwrite(out/'mask.png',mask.astype('uint8')*255)
    return camera,world,rot,trans


class IK:
    def __init__(self,m):
        self.m=m;self.d=mujoco.MjData(m);self.jids=[m.joint(f'p_joint{i}').id for i in range(1,7)]
        self.qids=m.jnt_qposadr[self.jids];self.dofs=m.jnt_dofadr[self.jids];self.sid=m.site('tcp').id
        self.bounds=m.jnt_range[self.jids].T
    def fk(self,q):
        self.d.qpos[self.qids]=q;mujoco.mj_forward(self.m,self.d)
        return self.d.site('tcp').xpos.copy(),self.d.site('tcp').xmat.reshape(3,3).copy()
    def solve(self,p,r,seed=None):
        def residual(q):
            pos,rot=self.fk(q)
            return np.r_[pos-p,.16*Rotation.from_matrix(r@rot.T).as_rotvec()]
        seeds=[seed] if seed is not None else []
        seeds += [np.array([0,1.4,-.5,0,-1,0]),np.array([0,.8,-1.3,0,.4,0]),np.array([0,2,-1.5,0,-1.1,0])]
        best=None
        for q in seeds:
            fit=least_squares(residual,np.clip(q,self.bounds[0]+1e-6,self.bounds[1]-1e-6),bounds=self.bounds,max_nfev=130,ftol=1e-9,xtol=1e-9,gtol=1e-9)
            pos,rot=self.fk(fit.x);err=np.linalg.norm(pos-p);ang=Rotation.from_matrix(r@rot.T).magnitude()
            if best is None or err+.1*ang<best[1]+.1*best[2]:best=(fit.x,err,ang)
            if err<.002 and ang<.04:return best
        return best


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=ROOT/'output');p.add_argument('--bottle-x',type=float,default=.72);p.add_argument('--bottle-y',type=float,default=0.);p.add_argument('--table-z',type=float,default=.76);p.add_argument('--tcp-m',type=float,default=.095);p.add_argument('--scene-only',action='store_true');a=p.parse_args()
    if not (.50 <= a.bottle_x <= 1.10 and .50 <= a.table_z <= 1.05 and .07 <= a.tcp_m <= .13 and -.25 <= a.bottle_y <= .25):
        p.error('Parameters outside experiment bounds')
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
    parameters=dict(bottle_x=a.bottle_x,bottle_y=a.bottle_y,table_z=a.table_z,tcp_m=a.tcp_m)
    def progress(stage):
        temporary=out/'progress.tmp';temporary.write_text(json.dumps(dict(stage=stage)));temporary.replace(out/'progress.json')
    progress('building')
    m=build(out,a.bottle_x,a.table_z,a.tcp_m,a.bottle_y);d=mujoco.MjData(m);ik=IK(m)
    d.qpos[ik.qids]=0;d.qpos[m.jnt_qposadr[m.joint('p_joint7').id]]=.035;d.qpos[m.jnt_qposadr[m.joint('p_joint8').id]]=-.035
    mujoco.mj_forward(m,d)
    renderer=mujoco.Renderer(m,height=720,width=1280);cam=mujoco.MjvCamera();cam.lookat[:]=[.3,0,.5];cam.distance=2.45;cam.azimuth=135;cam.elevation=-20
    renderer.update_scene(d,cam);imageio.imwrite(out/'initial.png',renderer.render())
    points,world,rot,trans=capture(m,d,out)
    print(json.dumps({'points':len(points),'base':d.body('base_link').xpos.tolist(),'zero_tcp':d.site('tcp').xpos.tolist()}),flush=True)
    np.save(out/'initial_qpos.npy',d.qpos.copy())
    (out/'pointcloud.json').write_text(json.dumps(world[::max(1,len(world)//2200)].round(5).tolist()))
    (out/'parameters.json').write_text(json.dumps(parameters,indent=2))
    if a.scene_only:renderer.close();return
    progress('inference')
    cfg_path=SHE/'A2_loco_manipulation/src/manipulation/grasp_generation/config/graspnet.json';cfg=json.loads(cfg_path.read_text())
    worker=Worker(cfg['worker_python'],SHE/'A2_loco_manipulation/src/manipulation/grasp_generation/grasp_generation/worker.py',str(cfg_path))
    try:reply=worker.predict(points.astype('float32'))
    finally:worker.close()
    adapter=dict(cfg['adapter'],max_candidates=100,horizontal_tolerance_deg=25,closing_tolerance_deg=25,
                 tcp_from_tip_m=.135-a.tcp_m)
    progress('planning')
    cs,counts=adapt(reply['rows'],rot,adapter)
    for c in cs:
        c['position']=(rot@c['position']+trans).tolist()
        c['rotation_world']=(rot@Rotation.from_quat(c['quaternion']).as_matrix()).tolist()
    network_candidates=copy.deepcopy(cs)
    cs,bottle_fit=refine_candidates(cs,world,a.tcp_m)
    selected=None;attempts=[]
    for c in cs:
        r=np.array(c['rotation_world']);pos=np.array(c['position']);axis=r[:,2]
        if axis[0]<.7:continue
        qg,eg,ag=ik.solve(pos,r);qp,ep,ap=ik.solve(pos-.10*axis,r,qg);ql,el,al=ik.solve(pos+[0,0,.13],r,qg)
        attempts.append(dict(raw_index=c['raw_index'],position_error=eg,angle_error=ag,pre_error=ep,lift_error=el))
        if max(eg,ep,el)<.004 and max(ag,ap,al)<.06:
            selected=c;break
    (out/'proposals.json').write_text(json.dumps(dict(raw_count=len(reply['rows']),filter_counts=counts,candidates=cs,network_candidates=network_candidates,bottle_fit=bottle_fit,attempts=attempts,selected=selected),indent=2))
    if selected is None:
        report=dict(success=False,status='no_reachable_candidate',failure_reason='没有候选同时通过水平进近、抓取和抬升位姿的 IK 检查。可尝试调整距离或桌高。',
                    parameters=parameters,tcp_m=a.tcp_m,grasp_strategy='body_middle_pad_center_v2',bottle_fit=bottle_fit,point_count=len(points),graspnet_inference_sec=reply['inference_sec'],
                    raw_candidates=len(reply['rows']),horizontal_candidates=len(cs),selected=None,
                    min_hold_lift_m=0,bilateral_hold_fraction=0,execute_robot=False,video_available=False)
        (out/'metrics.json').write_text(json.dumps(report,indent=2))
        (out/'frames.json').write_text(json.dumps([dict(t=0,phase='zero',qpos=d.qpos.tolist(),lift_m=0,contacts=0)]))
        (out/'trace.json').write_text(json.dumps([dict(t=0,phase='zero',bottle_lift_m=0,contacts=0,q=[0]*6,gripper_width_m=.07)]))
        renderer.close();progress('done');print(json.dumps(report),flush=True);return
    print('SELECTED',json.dumps(selected),flush=True)
    # Cartesian approach/lift waypoints keep the TCP orientation of the network proposal.
    r=np.array(selected['rotation_world']);pos=np.array(selected['position']);axis=r[:,2]
    pre=pos-.1*axis
    pre_q=ik.solve(pre,r)[0];grasp_q=ik.solve(pos,r,pre_q)[0]
    stages=[('zero',1.5,np.zeros(6),.035),('pregrasp',3.,pre_q,.035)]
    seed=pre_q
    for t in np.linspace(0,1,21)[1:]:
        seed=ik.solve(pre+(pos-pre)*t,r,seed)[0];stages.append(('approach',.09,seed,.035))
    stages.append(('close',1.5,grasp_q,0.))
    for t in np.linspace(0,1,27)[1:]:
        seed=ik.solve(pos+[0,0,.13*t],r,seed)[0];stages.append(('lift',.085,seed,0.))
    stages.append(('hold',2.,seed,0.))
    trace=[];frames=[];step=0;previous=np.zeros(6);opening=.035
    finger_ids={m.body('p_link7').id,m.body('p_link8').id};bottle_ids={m.geom(n).id for n in ['bottle_body','bottle_shoulder','bottle_neck','bottle_cap']}
    bottle0=d.body('bottle').xpos.copy();hold=[];hold_insets=[];close_insets=[];close_heights=[]
    writers=[imageio.get_writer(out/'overview.mp4',fps=25),imageio.get_writer(out/'closeup.mp4',fps=25)]
    cams=[cam,mujoco.MjvCamera()];cams[1].lookat[:]=[a.bottle_x-.04,a.bottle_y,a.table_z+.13];cams[1].distance=.82;cams[1].azimuth=115;cams[1].elevation=-18
    try:
        last_phase=None
        for phase,duration,target,grip in stages:
            if phase!=last_phase:progress(phase);last_phase=phase
            n=round(duration/m.opt.timestep)
            for j in range(n):
                t=(j+1)/n;s=t*t*(3-2*t)
                d.ctrl[:6]=previous+(target-previous)*s;d.ctrl[6]=opening+(grip-opening)*s
                mujoco.mj_step(m,d);step+=1
                contacts=set();force=0.;insets=[];heights=[]
                for ci,contact in enumerate(d.contact):
                    g1,g2=int(contact.geom1),int(contact.geom2)
                    other=g2 if g1 in bottle_ids else g1 if g2 in bottle_ids else -1
                    if other>=0 and m.geom_bodyid[other] in finger_ids:
                        contacts.add(int(m.geom_bodyid[other]))
                        link=d.body('p_link6');local=link.xmat.reshape(3,3).T@(contact.pos-link.xpos)
                        bottle=d.body('bottle');b_local=bottle.xmat.reshape(3,3).T@(contact.pos-bottle.xpos)
                        insets.append(float(FINGER_TIP_FROM_LINK6_M-local[2]));heights.append(float(b_local[2]+.105))
                        f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);force+=abs(f[0])
                lift=float(d.body('bottle').xpos[2]-bottle0[2])
                if phase=='hold':hold.append((lift,len(contacts)==2));hold_insets.extend(insets)
                if phase=='close':close_insets.extend(insets);close_heights.extend(heights)
                if step%20==0:
                    row=dict(t=round(float(d.time),3),phase=phase,bottle_lift_m=lift,contacts=len(contacts),contact_force_n=force,q=d.qpos[ik.qids].tolist(),gripper_width_m=float(d.qpos[m.jnt_qposadr[m.joint('p_joint7').id]]*2),
                             contact_inset_m=float(np.median(insets)) if insets else None,
                             contact_height_from_bottom_m=float(np.median(heights)) if heights else None)
                    trace.append(row)
                    frames.append(dict(t=row['t'],phase=phase,qpos=d.qpos.tolist(),lift_m=lift,contacts=len(contacts)))
                    for ci,camera in enumerate(cams):
                        renderer.update_scene(d,camera);frame=renderer.render().copy();writers[ci].append_data(frame)
                        if ci==0 and phase=='hold':imageio.imwrite(out/'result.png',frame)
            previous=target.copy();opening=grip
            if phase in ['zero','pregrasp','close','hold']:print(phase,round(lift,4),len(contacts),flush=True)
    finally:
        for w in writers:w.close()
        renderer.close()
    pad_supported=bool(hold_insets and np.median(hold_insets)>=.008)
    success=bool(hold and min(x[0] for x in hold)>.08 and np.mean([x[1] for x in hold])>.9 and pad_supported)
    report=dict(success=success,status='success' if success else 'grasp_failed',parameters=parameters,tcp_m=a.tcp_m,grasp_strategy='body_middle_pad_center_v2',bottle_fit=bottle_fit,video_available=True,physics='MuJoCo '+mujoco.__version__,fixed_base=True,legs_frozen=True,initial_arm_q=[0]*6,
                bottle_mass_kg=.2,bottle_radius_m=.026,table_height_m=a.table_z,bottle_initial_world=bottle0.tolist(),mount_xyz=[.0899,0,.1416],mount_calibrated=False,
                oracle_segmentation=True,depth_rendered=True,point_count=len(points),graspnet_inference_sec=reply['inference_sec'],raw_candidates=len(reply['rows']),horizontal_candidates=len(cs),
                min_hold_lift_m=min(x[0] for x in hold),bilateral_hold_fraction=float(np.mean([x[1] for x in hold])),
                hold_contact_inset_median_m=float(np.median(hold_insets)) if hold_insets else None,
                close_contact_inset_median_m=float(np.median(close_insets)) if close_insets else None,
                close_contact_height_from_bottom_m=float(np.median(close_heights)) if close_heights else None,
                pad_support_verified=pad_supported,physical_attachment=False,object_pose_written_during_rollout=False,execute_robot=False,selected=selected,phases=[dict(name=n,duration=d) for n,d,q,g in stages])
    (out/'metrics.json').write_text(json.dumps(report,indent=2));(out/'trace.json').write_text(json.dumps(trace));(out/'frames.json').write_text(json.dumps(frames))
    (out/'pointcloud.json').write_text(json.dumps(world[::max(1,len(world)//2200)].round(5).tolist()))
    progress('done')
    print(json.dumps({k:v for k,v in report.items() if k not in ['selected','phases']}),flush=True)

if __name__=='__main__':main()
