from __future__ import annotations
import copy,json,math,time,threading,concurrent.futures
from datetime import datetime,timezone
from pathlib import Path
import numpy as np
import viser
from viser.transforms import SO3
from .core import *
from .model import Robot,Renderer,wxyz,rotation
from .bus import MockBus,DynamixelBus
ROOT=Path(__file__).resolve().parents[1]
BUILD='COM7 feedback 3'

class App:
 def __init__(self,args):
  self.args=args;self.manual=bool(getattr(args,'manual',False));self.manual_preview_anchor=None;self.cfg_path=ROOT/'config'/('demo.json' if args.mock else 'hardware.json')
  self.c=json.loads(self.cfg_path.read_text());variant=args.gripper or self.c['gripper_variant']
  if variant!=self.c['gripper_variant']:self.c['tool']=nominal_tool(variant)
  self.c['gripper_variant']=variant
  validate_config(self.c)
  self.dir=ROOT/'data'/('demo' if args.mock else 'hardware')/args.session;self.dir.mkdir(parents=True,exist_ok=True)
  self.posepath=self.dir/'poses.json';self.seqpath=self.dir/'sequence.json'
  self.poses=json.loads(self.posepath.read_text()) if self.posepath.exists() else {}
  if args.mock and not self.manual and any(p.get('calibration')!=fingerprint(self.c) for p in self.poses.values()):raise Fault('Existing mock poses use an older mapping or jaw variant. Start a new --session name for this configuration.')
  self.sequence=json.loads(self.seqpath.read_text()) if self.seqpath.exists() else json.loads((ROOT/'config/sequence.template.json').read_text())
  if args.mock and not self.manual and not self.poses:self.demo_poses()
  logpath=ROOT/'logs'/f'{datetime.now().strftime("%Y%m%d_%H%M%S")}_{"demo" if args.mock else "hardware"}.jsonl';logpath.parent.mkdir(exist_ok=True)
  self.logfile=logpath.open('a',encoding='utf-8');self.loglock=threading.Lock()
  self.bus=MockBus(self.c) if args.mock else DynamixelBus(self.c,args.port)
  self.ctrl=Controller(self.bus,self.c,log=self.log);self.state=self.ctrl.read();self.message='Connected read-only. No motion until armed.'
  if self.manual:
   self.c=manual_session_config(self.c,[x['raw'] for x in self.state]);self.bus.c=self.c;self.ctrl.config=self.c
   self.message='Manual joint mode. Click Arm manual joint motion, then change one slider by 0.5 degrees and Execute preview.'
  self.q=[j['model_reference_rad'] for j in self.c['joints']]
  if self.manual or all(j['verified'] for j in self.c['joints']):self.q=raw_to_q(self.c,[x['raw'] for x in self.state])
  self.guard=False;self.pending=False;self.tasklock=threading.Lock();self.pool=concurrent.futures.ThreadPoolExecutor(max_workers=1)
  self.server=viser.ViserServer(host='127.0.0.1',port=args.web_port,label='Plexus OMX-F control')
  self.server.scene.set_up_direction('+z');self.server.scene.add_grid('/table',width=1,height=1,cell_size=.05)
  self.robot=Robot(ROOT,variant)
  self.actual=Renderer(self.server,self.robot,'/actual',(40,135,155),1)
  self.preview=Renderer(self.server,self.robot,'/preview',(244,165,65),.35)
  self.actual.visible(self.manual or all(j['verified'] for j in self.c['joints']))
  self.vertical_plan=None;self.execute_count=0
  self.gui();self.draw_preview()
  @self.server.on_client_connect
  def camera(event):
   event.client.camera.position=(.65,-.6,.45);event.client.camera.look_at=(.12,0,.16)
  actual_port=self.server.get_port()
  print(f'[{BUILD}] Open http://127.0.0.1:{actual_port} | {"SIMULATION ONLY" if args.mock else "REAL HARDWARE"} | serial {args.port or self.c["serial"]["port"]}',flush=True)
 def log(self,event,data):
  with self.loglock:
   self.logfile.write(json.dumps({'utc':datetime.now(timezone.utc).isoformat(),'event':event,'data':data})+'\n');self.logfile.flush()
 def save(self):atomic_json(self.posepath,self.poses);atomic_json(self.seqpath,self.sequence)
 def copy_feedback_preview(self):
  raw=[x['raw'] for x in self.ctrl.read()];self.update_model(raw_to_q(self.c,raw));self.manual_preview_anchor=raw
 def demo_poses(self):
  self.poses=mock_pose_set(self.c);self.save()
 def set_message(self,message,level='info'):
  self.message=message
  if hasattr(self,'action_status'):self.action_status.content=f'**{level.upper()}:** {message}'
  print(f'[{level.upper()}] {message}',flush=True)
 def job(self,fn,label='Operation'):
  with self.tasklock:
   if self.pending:self.set_message('Another operation is active; wait for its result.','busy');return
   self.pending=True
  self.set_message(label+': started.')
  def execute():
   try:
    result=fn();self.set_message(result if isinstance(result,str) else label+': completed.','ok')
   except Exception as exc:self.set_message(str(exc),'error');self.log('UI_ERROR',{'operation':label,'message':str(exc)})
   finally:self.pending=False
  self.pool.submit(execute)
 def update_model(self,q):
  self.guard=True
  try:
   self.q=list(q)
   for s,v in zip(self.sliders,q_to_motor_degrees(self.c,q)):s.value=float(np.clip(v,s.min,s.max))
   self.q=motor_degrees_to_q(self.c,[s.value for s in self.sliders])
   self.draw_preview()
  finally:self.guard=False
 def draw_preview(self):
  self.preview.update(self.q,self.c['tool']);poses,before=self.robot.fk(self.q)
  index=NAMES.index(self.selected.value);j=next(j for j in self.robot.joints if j['name']==self.selected.value)
  B=before[j['name']];R=B[:3,:3]@rotation(j['axis'],self.q[index]);self.gizmo.position=B[:3,3];self.gizmo.wxyz=wxyz(R);self.gizmo.active_axes=tuple(bool(abs(x)>.5) for x in j['axis'])
  T=self.robot.tcp(self.q,self.c['tool']);self.xyz.content='Nominal preview TCP, mm: '+', '.join(f'{x*1000:.1f}' for x in T[:3,3])+'. Tool offset needs physical calibration.'
  selected=self.c['joints'][index];self.direction_info.content=f"**ID {selected['id']}:** {selected.get('direction_note','Check the physical direction.')}"
 def gui(self):
  g=self.server.gui
  g.add_markdown(f'**Plexus | {BUILD} | {"SIMULATION" if self.args.mock else "REAL HARDWARE"} | {self.args.port or self.c["serial"]["port"]}**\n\nTeal = feedback model. Orange = preview. Dragging/sliders never send motor commands. No collision or grip-success detection.')
  self.banner=g.add_markdown('Starting...');self.telemetry=g.add_markdown('')
  stop=g.add_button('STOP / hold',color='red')
  @stop.on_click
  def stop_now(_):self.ctrl.request_stop();self.message='Stop requested. Re-arm before continuing.';self.pool.submit(self.ctrl.hold)
  with g.add_folder('1. Joint preview and manual motion'):
   if self.manual:g.add_markdown('**Manual encoder mode:** arm first, then change a slider. Arming copies the current feedback into the preview. Change one joint by at most 2° per execute, within a temporary ±5° startup window (intersected with measured limits). This window is not a measured mechanical limit. Automatic pick-and-place and IK remain in calibrated mode.')
   g.add_markdown('Sliders show absolute motor degrees: 180° = raw 2048. This centered reference is closed without gripping. Opening/gripping targets must be taught separately.')
   self.selected=g.add_dropdown('Joint to drag',options=tuple(NAMES),initial_value=NAMES[0])
   self.direction_info=g.add_markdown('')
   self.sliders=[]
   for index,j in enumerate(self.c['joints']):
    limits=[-180.,540.]
    if j['limits'] is not None and j['raw_reference'] is not None:
     limits=[raw*360/j['counts_per_rev'] for raw in j['limits']]
    slider=g.add_slider(f"ID {j['id']} motor degrees",min=limits[0],max=limits[1],step=.1,initial_value=float(np.clip(q_to_motor_degrees(self.c,self.q)[index],*limits)))
    self.sliders.append(slider)
    @slider.on_update
    def slider_changed(_,i=index):
     if self.guard:return
     self.q=motor_degrees_to_q(self.c,[s.value for s in self.sliders]);self.draw_preview()
   self.xyz=g.add_markdown('')
   self.gizmo=self.server.scene.add_transform_controls('/joint_drag',scale=.08,disable_axes=True,disable_sliders=True)
   @self.selected.on_update
   def selected_changed(_):self.draw_preview()
   @self.gizmo.on_update
   def dragged(_):
    if self.guard:return
    i=NAMES.index(self.selected.value);j=next(x for x in self.robot.joints if x['name']==self.selected.value)
    _,before=self.robot.fk(self.q);R=before[j['name']][:3,:3].T@SO3(self.gizmo.wxyz).as_matrix();axis=int(np.argmax(np.abs(j['axis'])))
    a=[math.atan2(R[2,1],R[1,1]),math.atan2(R[0,2],R[0,0]),math.atan2(R[1,0],R[0,0])][axis]
    old=self.q[i];a=old+math.atan2(math.sin(a-old),math.cos(a-old));q=self.q.copy();q[i]=a;self.update_model(q)
   sync=g.add_button('Copy feedback into preview')
   @sync.on_click
   def sync_preview(_):self.job(self.copy_feedback_preview)
   reference=g.add_button('Preview centered 180° reference (no motor movement)')
   @reference.on_click
   def reference_preview(_):self.update_model(motor_degrees_to_q(self.c,self.c['reference_pose']['motor_degrees']))
   self.supported=g.add_checkbox('Arm is supported; work area and power disconnect are ready',initial_value=False)
   arm=g.add_button('Arm manual joint motion' if self.manual else 'Arm full motion')
   @arm.on_click
   def arm_full(_):
    if not self.supported.value:self.set_message('Check the support/work-area checkbox, then click Arm.','error');return
    def perform():
     self.ctrl.arm(manual_only=self.manual)
     if self.manual:self.copy_feedback_preview()
    self.job(perform,label='Arm manual joint motion' if self.manual else 'Arm full motion')
   move=g.add_button('Execute preview')
   self.action_status=g.add_markdown('**Last action:** no Execute preview request yet. Arm first, then change a slider.')
   @move.on_click
   def execute_preview(_):
    self.execute_count+=1;request_number=self.execute_count
    degrees=[float(s.value) for s in self.sliders];anchor=copy.deepcopy(self.manual_preview_anchor)
    self.log('EXECUTE_CLICK',{'request':request_number,'motor_degrees':degrees,'armed':self.ctrl.armed})
    def perform():
     # Read current widget values, not a potentially stale slider-render callback.
     target=motor_degrees_to_raw(self.c,degrees)
     if self.manual and anchor is not None and target==anchor:raise Fault('Preview is unchanged. Arming/copying feedback resets it. Now change one slider by 1 degree, then Execute preview.')
     before=[x['raw'] for x in self.ctrl.read()]
     self.log('EXECUTE_REQUEST',{'request':request_number,'before_raw':before,'target_raw':target})
     print(f'[REQUEST {request_number}] actual={before} requested={target}',flush=True)
     self.ctrl.move(target,preview_anchor=anchor)
     after=[x['raw'] for x in self.ctrl.read()]
     changed=[i for i,(a,b) in enumerate(zip(target,anchor if self.manual and anchor is not None else before)) if a!=b]
     summary='; '.join(f"ID {self.c['joints'][i]['id']}: before {before[i]}, requested {target[i]}, encoder now {after[i]}" for i in changed)
     if self.manual:self.copy_feedback_preview()
     return f'Execute #{request_number} completed. {summary or "No encoder change was requested."}'
    self.job(perform,label=f'Execute preview #{request_number}')
   self.delta=g.add_slider('Raw jog ticks',min=1,max=8,step=1,initial_value=4)
   for sign,label in [(-1,'Jog selected motor -'),(1,'Jog selected motor +')]:
    button=g.add_button(label)
    @button.on_click
    def jog(_,direction=sign):
     i=NAMES.index(self.selected.value);delta=direction*int(self.delta.value)
     def perform():
      self.ctrl.jog(i,delta)
      if self.manual:self.copy_feedback_preview()
     self.job(perform)
  if self.manual:
   g.add_markdown('Manual mode uses the configured motor IDs and actual encoder degrees, without saving temporary windows as calibration. The arm model is nominal; its XYZ is not verified. For pick-and-place, finish physical calibration and restart without --manual.')
   return
  with g.add_folder('2. Calibration / software limits',expand_by_default=False):
   g.add_markdown('180°/2048 is the supplied reference. Nominal URDF reference angles below are 0°; confirm the photo pose against the model and adjust these offsets if necessary. Enter measured limits, save unverified, check signs with tiny raw jogs, then save verified. URDF limits are NOT mechanical limits.')
   self.low=[];self.high=[];self.sign=[];self.reference_model=[]
   for j in self.c['joints']:
    self.low.append(g.add_text(f"ID {j['id']} raw minimum",initial_value='' if j['limits'] is None else str(j['limits'][0])))
    self.high.append(g.add_text(f"ID {j['id']} raw maximum",initial_value='' if j['limits'] is None else str(j['limits'][1])))
    self.sign.append(g.add_dropdown(f"ID {j['id']} raw-to-model sign",options=('+1','-1'),initial_value=f"{j['sign']:+d}"))
    self.reference_model.append(g.add_number(f"ID {j['id']} model reference angle (deg)",initial_value=math.degrees(j['model_reference_rad'])))
   model_reference=g.add_button('Preview model reference geometry')
   @model_reference.on_click
   def model_reference_preview(_):self.update_model([math.radians(f.value) for f in self.reference_model])
   self.verified=g.add_checkbox('I verified every motor mapping, direction, reference pose and raw limit',initial_value=False)
   save=g.add_button('Capture reference and save calibration')
   @save.on_click
   def save_calibration(_):
    def perform():
     if self.ctrl.armed:raise Fault('Stop/disarm before changing calibration.')
     candidate=copy.deepcopy(self.c);raw=[x['raw'] for x in self.ctrl.read()]
     for i,j in enumerate(candidate['joints']):j.update(raw_reference=raw[i],model_reference_rad=math.radians(self.reference_model[i].value),sign=int(self.sign[i].value),limits=[int(self.low[i].value),int(self.high[i].value)],verified=self.verified.value)
     validate_config(candidate);validate_raw(candidate,raw)
     self.c.clear();self.c.update(candidate);atomic_json(self.cfg_path,self.c);self.sequence['approved_digest']=None;self.save();self.update_model(raw_to_q(self.c,raw))
    self.job(perform)
   jogarm=g.add_button('Arm raw-jog calibration only')
   @jogarm.on_click
   def arm_jog(_):
    if not self.supported.value:self.message='Confirm support/work area first.';return
    self.job(lambda:self.ctrl.arm(calibration_only=True))
  with g.add_folder('3. Teach poses / pick-and-place'):
   self.pose_name=g.add_text('Pose name',initial_value='source_approach')
   teach=g.add_button('Record current feedback as pose')
   @teach.on_click
   def teach_pose(_):
    def perform():
     name=self.pose_name.value.strip()
     if not name or len(name)>80 or not all(ch.isalnum() or ch in '_-' for ch in name):raise Fault('Use a simple pose name with letters/numbers/underscores.')
     raw=[x['raw'] for x in self.ctrl.read()];validate_raw(self.c,raw)
     if not self.args.mock:validate_config(self.c,True)
     self.poses[name]={'raw':raw,'source':'mock' if self.args.mock else 'hardware','calibration':fingerprint(self.c),'utc':datetime.now(timezone.utc).isoformat()}
     self.sequence['approved_digest']=None;self.save();self.refresh_poses()
    self.job(perform)
   self.pose_choice=g.add_dropdown('Saved pose',options=tuple(sorted(self.poses)) or ('(none)',))
   preview=g.add_button('Preview saved pose')
   @preview.on_click
   def preview_pose(_):
    try:self.update_model(raw_to_q(self.c,self.poses[self.pose_choice.value]['raw']))
    except Exception as exc:self.message=str(exc)
   goto=g.add_button('Move to saved pose')
   @goto.on_click
   def goto_pose(_):
    p=self.poses.get(self.pose_choice.value)
    if p is None:return
    if not self.args.mock and (p.get('source')!='hardware' or p.get('calibration')!=fingerprint(self.c)):self.message='Pose is simulated or calibration changed. Teach again.';return
    self.job(lambda:self.ctrl.move(p['raw']))
   self.route=g.add_markdown('');self.refresh_poses()
   reload=g.add_button('Reload edited sequence.json')
   @reload.on_click
   def reload_sequence(_):
    def perform():
     if self.ctrl.armed:raise Fault('Stop/disarm before changing the sequence.')
     self.sequence=json.loads(self.seqpath.read_text());self.sequence['approved_digest']=None;self.save();self.refresh_poses()
    self.job(perform)
   self.review=g.add_checkbox('I physically checked the complete path and gripper actions at low speed',initial_value=False)
   approve=g.add_button('Approve this exact taught sequence')
   @approve.on_click
   def approve_sequence(_):
    def perform():
     if not self.review.value:raise Fault('Complete the path review first.')
     validate_sequence(self.c,self.poses,self.sequence,False)
     if not self.args.mock:
      validate_config(self.c,True)
      for step in self.sequence['steps']:
       p=self.poses[step['pose']]
       if p.get('source')!='hardware' or p.get('calibration')!=fingerprint(self.c):raise Fault('Every hardware pose must be retaught with the current calibration.')
     self.sequence['approved_digest']=sequence_digest(self.c,self.poses,self.sequence);self.save()
    self.job(perform)
   run=g.add_button('Run pick-and-place sequence',color='green')
   @run.on_click
   def run_sequence(_):self.job(lambda:self.ctrl.run(copy.deepcopy(self.poses),copy.deepcopy(self.sequence)))
  with g.add_folder('4. Cartesian vertical IK lift'):
   g.add_markdown('Fix the ring center X/Y and preserve its normal while changing Z. Preview plans the complete lift first. This is model-based; encoder feedback cannot measure backlash or the real post position.')
   self.tool_fields=[g.add_text(f'Ring-center TCP {axis}, mm',initial_value=str(value*1000)) for axis,value in zip('XYZ',self.c['tool']['offset_m'])]
   self.tool_check=g.add_checkbox('I measured the ring-center tool offset and physically verified the ring is level',initial_value=self.c['tool']['verified'])
   tool_save=g.add_button('Save measured tool offset')
   @tool_save.on_click
   def save_tool(_):
    def perform():
     if self.ctrl.armed:raise Fault('Stop/disarm before changing the tool calibration.')
     values=[float(x.value)/1000 for x in self.tool_fields]
     if not all(math.isfinite(v) and abs(v)<.3 for v in values):raise Fault('Invalid tool offset.')
     self.c['tool']={'offset_m':values,'verified':self.tool_check.value};atomic_json(self.cfg_path,self.c);self.sequence['approved_digest']=None;self.save();self.draw_preview()
    self.job(perform)
   self.travel=g.add_slider('Vertical travel mm (+up / -down)',min=-150.,max=150.,step=.5,initial_value=5.)
   self.ikinfo=g.add_markdown('No vertical plan yet. Start with a 1-2 mm lift test on the actual arm.')
   plan_button=g.add_button('Preview vertical IK from feedback')
   @plan_button.on_click
   def preview_vertical(_):
    def perform():
     from .ik import plan_vertical
     if not self.args.mock:
      validate_config(self.c,True)
      if not self.c['tool']['verified']:raise Fault('Save a physically measured/verified tool offset first.')
     self.ctrl.stop_event.clear()
     def heartbeat():
      self.ctrl.check_stop();state=self.ctrl.read();self.ctrl.health(state,self.ctrl.armed)
     raw=[x['raw'] for x in self.ctrl.read()];self.vertical_plan=plan_vertical(self.robot,self.c,raw,float(self.travel.value),heartbeat)
     atomic_json(self.dir/'last_vertical_plan.json',self.vertical_plan)
     points=np.array([self.vertical_plan['start_xyz_m']]+[f['nominal_xyz_m'] for f in self.vertical_plan['frames']],dtype=np.float32)
     self.server.scene.add_line_segments('/vertical_path',points=np.stack([points[:-1],points[1:]],axis=1),colors=np.tile(np.array([255,170,40],dtype=np.uint8),(len(points)-1,2,1)),line_width=3.)
     self.update_model(raw_to_q(self.c,self.vertical_plan['frames'][-1]['raw']))
     self.ikinfo.content=f"Planned {len(self.vertical_plan['frames'])} points over {self.vertical_plan['duration_s']:.2f}s. Maximum nominal XY error: {self.vertical_plan['max_nominal_xy_error_mm']:.3f} mm. No physical accuracy claim."
    self.job(perform)
   execute_button=g.add_button('Execute the planned vertical IK lift')
   @execute_button.on_click
   def run_vertical(_):
    from .ik import execute_vertical
    if self.vertical_plan is None:self.message='Preview a lift first.';return
    plan=copy.deepcopy(self.vertical_plan);self.job(lambda:execute_vertical(self.ctrl,self.robot,plan))
 def refresh_poses(self):
  if hasattr(self,'pose_choice'):self.pose_choice.options=tuple(sorted(self.poses)) or ('(none)',)
  if hasattr(self,'route'):
   self.route.content='**Route:**\n\n'+'\n'.join(f"{i+1}. {s['kind']}: `{s['pose']}`" for i,s in enumerate(self.sequence['steps']))+'\n\nMissing: '+', '.join(sorted({s['pose'] for s in self.sequence['steps'] if s['pose'] not in self.poses}))
 def loop(self):
  try:
   while True:
    if not self.pending:
     try:self.state=self.ctrl.read()
     except Exception as exc:self.ctrl.request_stop();self.message=str(exc)
    elif self.ctrl.last is not None:self.state=self.ctrl.last
    good=self.manual or all(j['verified'] for j in self.c['joints'])
    self.actual.visible(good)
    if good:self.actual.update(raw_to_q(self.c,[s['raw'] for s in self.state]),self.c['tool'])
    mode='MANUAL JOINTS' if self.manual else 'SIMULATION' if self.args.mock else 'HARDWARE'
    self.banner.content=f"**{mode} | {'ARMED RAW JOG ONLY' if self.ctrl.armed and self.ctrl.calibration_only else 'ARMED' if self.ctrl.armed else 'DISARMED'}**\n\n{self.message}\n\n"+('Encoder-based manual control. Model XYZ is unverified.' if self.manual else 'Model mapping verified.' if good else 'Unverified model mapping: do not interpret preview XYZ as real position.')
    self.telemetry.content='| ID | Raw | Motor ° | V | °C | HW error | Torque |\n|---|---:|---:|---:|---:|---:|---|\n'+'\n'.join(f"|{s['id']}|{s['raw']}|{deg:.2f}|{s['voltage']:.1f}|{s['temperature']}|{s['error']}|{s['torque']}|" for s,deg in zip(self.state,raw_to_motor_degrees(self.c,[s['raw'] for s in self.state])))
    time.sleep(.2)
  except KeyboardInterrupt:pass
  finally:
   active=self.ctrl.armed or self.ctrl.busy;self.ctrl.request_stop();self.pool.shutdown(wait=True,cancel_futures=True)
   if active:self.ctrl.hold()
   self.bus.close();self.logfile.close();self.server.stop()
