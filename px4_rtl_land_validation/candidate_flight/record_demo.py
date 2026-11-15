"""Record actual Gazebo/QGC windows during an authorized local SITL flight.

Only the two application windows are captured. Virtual sticks are visible, but
flight commands are scripted. No generated flight imagery or synthetic telemetry.
"""
import pathlib,sys,subprocess,datetime,json,signal,time,shutil,hashlib,argparse,os
os.environ["GST_XINITTHREADS"]="1"
ROOT=pathlib.Path(__file__).resolve().parent;OUT=ROOT/'demo';OUT.mkdir(exist_ok=True)
from test_flight import CandidateFlight,mavutil

def windows():
 result={}
 for line in subprocess.check_output(['wmctrl','-lG'],text=True).splitlines():
  f=line.split(None,7)
  if len(f)!=8:continue
  if f[7]=='Gazebo Sim':result['gazebo']={'xid':f[0],'geometry':list(map(int,f[2:6]))}
  if f[7]=='QGroundControl':result['qgc']={'xid':f[0],'geometry':list(map(int,f[2:6]))}
 assert set(result)=={'gazebo','qgc'},'Both actual application windows must be open'
 return result

class Recorder:
 def __init__(self,name):self.name=name;self.windows=windows();self.process=None;self.output=OUT/f'{name}.webm'
 def pipeline(self,filename,snapshot=False):
  cmd=['gst-launch-1.0','-e','-q','compositor','name=mix','background=black','latency=100000000','sink_0::xpos=0','sink_0::ypos=56','sink_1::xpos=960','sink_1::ypos=56','!','video/x-raw,width=1920,height=960,framerate=20/1','!','videoconvert','!','textoverlay','auto-resize=false','text="自研 A 无人机 · Gazebo + PX4 + QGC 联合演示"','font-desc="Noto Sans CJK SC 22"','valignment=top','ypad=10','!','textoverlay','auto-resize=false','text="固定构形 · 近似参数 · 非实机验证\n脚本执行自动起降；QGC 显示实时状态与虚拟摇杆"','font-desc="Noto Sans CJK SC 16"','valignment=bottom','ypad=9','!','videoconvert']
  if snapshot:cmd+=['!','pngenc','snapshot=true']
  else:cmd+=['!','video/x-raw,format=I420','!','vp8enc','deadline=1','cpu-used=8','threads=4','target-bitrate=6000000','keyframe-max-dist=40','!','webmmux']
  cmd+=['!','filesink','location='+str(filename)]
  for i,key in enumerate(['gazebo','qgc']):
   cmd+=['ximagesrc','xid='+self.windows[key]['xid'],'use-damage=false','show-pointer=false','do-timestamp=true']
   if snapshot:cmd+=['num-buffers=4']
   cmd+=['!','video/x-raw,framerate=20/1','!','videoconvert','!','videoscale','!','video/x-raw,width=960,height=804,pixel-aspect-ratio=1/1','!','queue','!','mix.sink_'+str(i)]
  return cmd
 def capture(self,tag):
  f=OUT/f'{self.name}_{tag}.png';subprocess.run(self.pipeline(f,True),check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,timeout=15);return str(f)
 def start(self):
  self.started=datetime.datetime.now(datetime.timezone.utc).isoformat();self.log=(OUT/f'{self.name}_recording.log').open('w');cmd=self.pipeline(self.output)
  self.process=subprocess.Popen(cmd,stdout=self.log,stderr=subprocess.STDOUT);time.sleep(.4)
  assert self.process.poll() is None,'Recorder did not stay running'
 def stop(self):
  if not self.process:return
  if self.process.poll() is None:self.process.send_signal(signal.SIGINT)
  self.process.wait(timeout=20);self.log.close();assert self.process.returncode==0,'Recording pipeline failed'
  self.ended=datetime.datetime.now(datetime.timezone.utc).isoformat();self.process=None
  (OUT/f'{self.name}_recording.json').write_text(json.dumps({'run':self.name,'windows':self.windows,'started_utc':self.started,'ended_utc':self.ended,'source':'actual X11 application windows, composited side by side','flight_commands':'scripted takeoff/hover/Land; visible virtual sticks not manipulated','webm':self.output.name,'no_audio':True},ensure_ascii=False,indent=2))
 def mp4(self):
  target=OUT/f'{self.name}.mp4'
  code="""import cv2,sys,json,hashlib,pathlib
source,target,metadata_path,run=sys.argv[1:];cap=cv2.VideoCapture(source);fps=cap.get(cv2.CAP_PROP_FPS);size=(int(cap.get(3)),int(cap.get(4)))
assert cap.isOpened() and fps>0;writer=cv2.VideoWriter(target,cv2.VideoWriter_fourcc(*'avc1'),fps,size);assert writer.isOpened();count=0
while True:
 ok,frame=cap.read()
 if not ok:break
 writer.write(frame);count+=1
cap.release();writer.release();verify=cv2.VideoCapture(target);assert verify.isOpened();assert abs(verify.get(cv2.CAP_PROP_FRAME_COUNT)-count)<=1;verify.release()
metadata={'run':run,'mp4':pathlib.Path(target).name,'frames':count,'fps':fps,'duration_s':count/fps,'width':size[0],'height':size[1],'sha256':hashlib.sha256(pathlib.Path(target).read_bytes()).hexdigest(),'conversion':'Recorded WebM to H264, no frame edits or speed changes'}
pathlib.Path(metadata_path).write_text(json.dumps(metadata,ensure_ascii=False,indent=2));print(json.dumps(metadata,ensure_ascii=False))"""
  subprocess.run(['/usr/bin/python3','-c',code,str(self.output),str(target),str(OUT/f'{self.name}_video_metadata.json'),self.name],check=True)

class DemoFlight(CandidateFlight):
 def __init__(self,name,recorder):self.recorder=recorder;super().__init__(name)
 def event(self,what,**kw):
  super().event(what,**kw)
  if what=='ready_ground':self.recorder.start();self.pump(3);self.recorder.capture('ready')
  elif what=='hover_confirmed':self.pump(2);self.recorder.capture('hover')
  elif what=='auto_disarmed':self.pump(3);self.recorder.capture('landed')
 def snapshot(self,tag):
  if tag=='after':self.recorder.stop()
  super().snapshot(tag)

if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--preview',action='store_true');args=a.parse_args()
 if args.preview:print(Recorder('layout_preview').capture('ready'));raise SystemExit
 top=subprocess.check_output(['docker','top','px4-rtl-land-harmonic','-eo','pid,args'],text=True);assert '/work/candidate_flight/startup.sh' in top
 name='demo_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S');rec=Recorder(name)
 for source,suffix in [(ROOT/'flight_config.json','model_config.json'),(ROOT/'models/candidate_rigid/model.sdf','model.sdf'),(ROOT/'candidate_world.sdf','world.sdf')]:shutil.copyfile(source,ROOT/'logs'/f'{name}_{suffix}')
 f=DemoFlight(name,rec)
 try:f.run_candidate()
 except Exception as e:
  f.monitor_flight=False;f.event('test_failed',error=str(e))
  if f.state.get('armed'):
   try:f.command(mavutil.mavlink.MAV_CMD_NAV_LAND,[0,0,0,float('nan'),float('nan'),float('nan'),float('nan')]);f.wait('recovery_disarmed',lambda:not f.state.get('armed'),timeout=150)
   except Exception as e2:f.event('recovery_failed',error=str(e2))
  raise
 finally:
  rec.stop();paths=list((ROOT/'runtime/log').rglob('*.ulg'))
  if paths:shutil.copyfile(max(paths,key=lambda p:p.stat().st_mtime),ROOT/'logs'/f'{name}.ulg')
  result={'name':name,'events':f.events,'status':'passed' if f.events[-1]['event']=='test_complete' else 'failed'};(ROOT/'last_run.json').write_text(json.dumps(result,indent=2));(OUT/'last_demo.json').write_text(json.dumps(result,indent=2))
 rec.mp4()
