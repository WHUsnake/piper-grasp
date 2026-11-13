"""Local interactive MuJoCo experiment service. No ROS or hardware interfaces."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from datetime import datetime, timezone
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit, unquote
from uuid import uuid4

ROOT=Path(__file__).resolve().parent
STATIC=ROOT/'site/dist'
PYTHON=ROOT.parents[2]/'supermimic训练机械臂抓取/.venv/bin/python'
BOUNDS={'bottle_y':(-.25,.25),'bottle_x':(.50,1.10),'table_z':(.50,1.05),'tcp_m':(.07,.13)}
TARGET_KINDS=('bottle','apple','orange','egg','can','cup','round_proxy','small_carton','tissue_box','cup_inverted')
DEFAULT={'bottle_y':0.,'bottle_x':.72,'table_z':.76,'tcp_m':.095,'target_kind':'bottle','place_after_grasp':False,'multi_objects':False,'selected_pixel_u':-1,'selected_pixel_v':-1}


def validate_parameters(value):
    if not isinstance(value,dict) or set(value)-set(BOUNDS)-{'target_kind','place_after_grasp','multi_objects','selected_pixel_u','selected_pixel_v'} or not {'bottle_x','table_z','tcp_m'}<=set(value):
        raise ValueError('必须提供目标前后距离、桌高和 TCP；左右位置可选')
    value=dict(DEFAULT,**value)
    if value['target_kind'] not in TARGET_KINDS:raise ValueError('未知目标类型')
    if not isinstance(value['place_after_grasp'],bool):raise ValueError('放置选项必须是布尔值')
    if not isinstance(value['multi_objects'],bool):raise ValueError('多物体选项必须是布尔值')
    if value['multi_objects'] and value['target_kind'] not in ('bottle','apple','small_carton','can','cup'):
        raise ValueError('多物体场景目前可选水瓶、苹果、小纸盒、易拉罐或水杯')
    result={}
    for key,(lo,hi) in BOUNDS.items():
        v=value[key]
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not lo<=v<=hi:
            raise ValueError(f'{key} 必须在 {lo}–{hi} 米范围内')
        result[key]=float(v)
    result['target_kind']=value['target_kind']
    result['place_after_grasp']=value['place_after_grasp']
    result['multi_objects']=value['multi_objects']
    u,v=value['selected_pixel_u'],value['selected_pixel_v']
    if type(u) is not int or type(v) is not int or ((u,v)!=(-1,-1) and not (0<=u<640 and 0<=v<480)):
        raise ValueError('DC1 点选位置必须在 640×480 图像内')
    result['selected_pixel_u'],result['selected_pixel_v']=u,v
    if result['multi_objects'] and u<0:raise ValueError('请先在 DC1 预览图中点选要抓的物体')
    return result


def failure_message(detail):
    reason=str(detail.get('failure',''))
    if detail.get('execution_started'):
        return '执行验证中止：'+reason.split(' {',1)[0]+'；未判定为抓取成功。'
    if 'fresh' in reason.lower() or 'stale' in reason.lower():
        return '执行前未取得新鲜观测，本次未执行抓取。详细原因见本次日志。'
    if 'budget exhausted' in reason:
        return '规划搜索预算已用尽，尚未找到完整可执行抓取，本次未运动。'
    try:records=json.loads(detail.get('planner','{}')).get('candidates',[])
    except (ValueError,TypeError):records=[]
    if records and all(x.get('stage')=='grasp_endpoint' for x in records):
        return f'已搜索{len(records)}个候选，未找到满足当前关节和避碰约束的夹持位姿，本次未运动。该结果不等于证明所有姿态都不可达。'
    stages={'pregrasp':'预抓取','approach':'接近','lift':'抬升','grasp_endpoint':'夹持位姿'}
    counts={}
    for record in records:
        name=stages.get(record.get('stage'),'候选检查');counts[name]=counts.get(name,0)+1
    if counts:
        return '未找到完整可执行抓取：'+ '、'.join(f'{k}阶段拒绝{v}个候选' for k,v in counts.items())+'。本次未运动；详细原因见本次日志。'
    return '规划/观测验证未通过，本次未执行抓取：'+reason[:180]

class Experiments:
    def __init__(self):
        self.lock=threading.RLock();self.job=None;self.process=None;self.cancelled=threading.Event()
        self.current={'asset_base':'./assets/','parameters':DEFAULT}
        pointer=ROOT/'current_run.json'
        if pointer.is_file():
            item=json.loads(pointer.read_text())
            if (ROOT/'runs'/item.get('id','')/'web/assets/metrics.json').is_file():
                item['parameters']=dict(DEFAULT,**item['parameters']);self.current=item

    def snapshot(self):
        with self.lock:
            job=dict(self.job) if self.job else None
            if job and job['status']=='running':
                path=ROOT/'runs'/job['id']/'progress.json'
                try:
                    if job['stage']=='exporting':raise OSError('导出阶段以服务端状态为准')
                    stage=json.loads(path.read_text())['stage']
                    if stage!=job['stage']:
                        self.job['stage']=stage
                        self.job['stage_started_at']=datetime.fromtimestamp(path.stat().st_mtime,timezone.utc).isoformat()
                    job['stage']=stage
                    job['stage_started_at']=self.job['stage_started_at']
                except (OSError,ValueError):pass
                job['elapsed_sec']=round(time.monotonic()-job.pop('_started',time.monotonic()),1)
                job['stage_elapsed_sec']=round(max(0,time.time()-datetime.fromisoformat(job['stage_started_at']).timestamp()),1)
            if job:
                if job.get('finished_at'):
                    job['elapsed_sec']=round(datetime.fromisoformat(job['finished_at']).timestamp()-datetime.fromisoformat(job['started_at']).timestamp(),1)
                job.pop('_started',None)
            return {'current':dict(self.current),'job':job,'defaults':DEFAULT,'bounds':BOUNDS,'backend':'ros2_moveit'}

    def start(self,parameters):
        parameters=validate_parameters(parameters)
        with self.lock:
            if self.job and self.job['status']=='running':raise RuntimeError('已有实验正在运行，请等待或停止当前计算')
            ident=uuid4().hex;folder=ROOT/'runs'/ident;folder.mkdir(parents=True)
            (folder/'request.json').write_text(json.dumps(parameters,indent=2))
            self.cancelled.clear()
            started=datetime.now(timezone.utc).isoformat()
            self.job=dict(id=ident,status='running',stage='building',parameters=parameters,_started=time.monotonic(),started_at=started,stage_started_at=started,log_url=f'/runs/{ident}/run.log')
            threading.Thread(target=self.run,args=(ident,parameters,folder),daemon=True).start()
            return self.snapshot()

    def command(self,args,folder,log):
        with self.lock:
            if self.cancelled.is_set():raise InterruptedError('实验已停止')
            env=os.environ.copy();env.pop('PYTHONPATH',None);env['MUJOCO_GL']='egl';env['PYTHONUNBUFFERED']='1'
            self.process=subprocess.Popen([str(x) for x in args],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            process=self.process
        try:
            code=process.wait(timeout=240)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
            raise TimeoutError('本次实验计算超过240秒，已停止。请查看日志确认规划或执行耗时')
        finally:
            with self.lock:self.process=None
        if self.cancelled.is_set():raise InterruptedError('实验已停止')
        if code:
            failure=folder/'pipeline_failure.json'
            if failure.is_file():
                detail=json.loads(failure.read_text())
                raise RuntimeError(failure_message(detail))
            raise RuntimeError('仿真进程未完成，请查看本次实验日志')

    def run(self,ident,parameters,folder):
        try:
            with (folder/'run.log').open('w') as log:
                self.command([ROOT.parent/'ros_pipeline/run.sh','--output',folder,'--target-kind',parameters['target_kind'],'--bottle-x',parameters['bottle_x'],
                              '--bottle-y',parameters['bottle_y'],'--table-z',parameters['table_z'],'--tcp-m',parameters['tcp_m'],
                              '--selected-pixel-u',parameters['selected_pixel_u'],'--selected-pixel-v',parameters['selected_pixel_v'],
                              *(['--multi-objects'] if parameters['multi_objects'] else []),
                              *(['--place-after-grasp'] if parameters['place_after_grasp'] else [])],folder,log)
                with self.lock:self.job.update(stage='exporting',stage_started_at=datetime.now(timezone.utc).isoformat())
                self.command([PYTHON,ROOT/'export_web.py',folder,'--assets',folder/'web/assets','--skip-geometry'],folder,log)
            report=json.loads((folder/'metrics.json').read_text())
            with self.lock:
                if self.cancelled.is_set():raise InterruptedError('实验已停止')
                self.current=dict(id=ident,asset_base=f'/runs/{ident}/web/assets/',parameters=parameters)
                temp=ROOT/'current_run.tmp';temp.write_text(json.dumps(self.current));temp.replace(ROOT/'current_run.json')
                self.job.update(status='done',stage='done',success=report['success'],asset_base=self.current['asset_base'],
                                finished_at=datetime.now(timezone.utc).isoformat(),
                                message='物理抓取验证通过' if report['success'] else report.get('failure_reason','抓取未通过抬升/接触保持判据'))
        except Exception as exc:
            with self.lock:self.job.update(status='cancelled' if self.cancelled.is_set() else 'failed',message=str(exc),finished_at=datetime.now(timezone.utc).isoformat())

    def cancel(self,ident):
        with self.lock:
            if not self.job or self.job['id']!=ident or self.job['status']!='running':raise ValueError('没有对应的运行中实验')
            self.cancelled.set()
            if self.process and self.process.poll() is None:os.killpg(self.process.pid,signal.SIGTERM)
            return self.snapshot()


class Handler(SimpleHTTPRequestHandler):
    def __init__(self,*args,**kwargs):super().__init__(*args,directory=STATIC,**kwargs)

    def json_response(self,data,code=200):
        raw=json.dumps(data,ensure_ascii=False,allow_nan=False).encode()
        self.send_response(code);self.send_header('Content-Type','application/json; charset=utf-8');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)

    def end_headers(self):
        self.send_header('Cache-Control','no-cache')
        super().end_headers()

    def do_GET(self):
        if urlsplit(self.path).path=='/api/state':self.json_response(self.server.experiments.snapshot());return
        super().do_GET()

    def do_POST(self):
        # Same-origin local UI only; never accept arbitrary command or output paths.
        origin=self.headers.get('Origin')
        if origin and urlsplit(origin).netloc!=self.headers.get('Host'):
            self.json_response({'error':'请从本机实验网页提交'},403);return
        try:
            length=int(self.headers.get('Content-Length',0))
            if not 0<length<=2048:raise ValueError('请求大小无效')
            body=json.loads(self.rfile.read(length))
            path=urlsplit(self.path).path
            if path=='/api/preview':
                proposed=dict(body,selected_pixel_u=0,selected_pixel_v=0)
                parameters=validate_parameters(proposed)
                if not parameters['multi_objects']:raise ValueError('预览仅用于多物体场景')
                ident='preview_'+uuid4().hex;folder=ROOT/'runs'/ident;folder.mkdir(parents=True)
                command=[str(PYTHON),str(ROOT/'preview_scene.py'),'--output',str(folder),
                         '--bottle-x',str(parameters['bottle_x']),'--bottle-y',str(parameters['bottle_y']),
                         '--table-z',str(parameters['table_z']),'--tcp-m',str(parameters['tcp_m']),
                         '--target-kind',parameters['target_kind']]
                subprocess.run(command,cwd=ROOT,check=True,timeout=45,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
                objects=json.loads((folder/'objects.json').read_text())
                self.json_response({'image_url':f'/runs/{ident}/camera_rgb.png','selection_url':f'/runs/{ident}/selection_ids.png','parameters':parameters,'objects':objects})
            elif path=='/api/run':self.json_response(self.server.experiments.start(body),202)
            elif path=='/api/cancel':self.json_response(self.server.experiments.cancel(body.get('id')))
            else:self.json_response({'error':'未知接口'},404)
        except ValueError as exc:self.json_response({'error':str(exc)},400)
        except RuntimeError as exc:self.json_response({'error':str(exc)},409)
        except (subprocess.CalledProcessError,subprocess.TimeoutExpired):self.json_response({'error':'DC1 场景预览生成失败，请检查物体是否在视野和桌面范围内'},500)

    def translate_path(self,path):
        path=unquote(urlsplit(path).path)
        if path.startswith('/runs/'):
            target=(ROOT/path.lstrip('/')).resolve()
            if target.is_relative_to((ROOT/'runs').resolve()):return str(target)
            return str(STATIC/'not-found')
        return super().translate_path(path)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--port',type=int,default=8766);args=parser.parse_args()
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler);server.experiments=Experiments()
    print(f'A2 × PiPER 交互实验台：http://127.0.0.1:{args.port}/',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        state=server.experiments.snapshot()
        if state['job'] and state['job']['status']=='running':server.experiments.cancel(state['job']['id'])
        server.server_close()

if __name__=='__main__':main()
