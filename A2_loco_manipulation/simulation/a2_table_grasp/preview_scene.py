"""Render a fresh DC1 view of the requested multi-object MuJoCo scene."""
import argparse
import json
from pathlib import Path
import imageio.v2 as imageio
import mujoco
import numpy as np
from simulate import build


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--bottle-x',type=float,required=True)
    p.add_argument('--bottle-y',type=float,required=True)
    p.add_argument('--table-z',type=float,required=True)
    p.add_argument('--tcp-m',type=float,required=True)
    p.add_argument('--target-kind',required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    model=build(a.output,a.bottle_x,a.table_z,a.tcp_m,a.bottle_y,a.target_kind,True)
    data=mujoco.MjData(model);data.qpos[6:8]=[.035,-.035];mujoco.mj_forward(model,data)
    renderer=mujoco.Renderer(model,height=480,width=640)
    renderer.update_scene(data,camera='dc1_rgbd')
    imageio.imwrite(a.output/'camera_rgb.png',renderer.render())
    renderer.enable_segmentation_rendering()
    renderer.update_scene(data,camera='dc1_rgbd')
    segmentation=renderer.render()
    objects=[]
    object_ids=np.zeros(segmentation.shape[:2],dtype=np.uint8)
    for object_id,kind in enumerate(('bottle','apple','small_carton','can','cup'),start=1):
        body_name=('bottle' if kind=='bottle' else 'round_target') if a.target_kind==kind else f'demo_{kind}'
        body_id=model.body(body_name).id
        geom_ids=[i for i in range(model.ngeom) if model.geom_bodyid[i]==body_id]
        mask=(segmentation[:,:,1]==mujoco.mjtObj.mjOBJ_GEOM)
        mask &= np.isin(segmentation[:,:,0],geom_ids)
        yy,xx=np.where(mask)
        object_ids[mask]=object_id
        if len(xx):objects.append({'id':object_id,'kind':kind,'bbox_xyxy':[int(xx.min()),int(yy.min()),int(xx.max()+1),int(yy.max()+1)],'centroid_uv':[int(np.median(xx)),int(np.median(yy))]})
    imageio.imwrite(a.output/'selection_ids.png',object_ids)
    (a.output/'objects.json').write_text(json.dumps(objects))
    renderer.close()


if __name__=='__main__':main()
