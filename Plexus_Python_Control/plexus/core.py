"""Pure, transport-independent calibration, planning and sequence rules."""
from __future__ import annotations
import copy,hashlib,json,math,time,threading
from pathlib import Path
TAU=2*math.pi
NAMES=['joint1','joint2','joint3','joint4','joint5','gripper_joint_1']
MANUAL_STEP_DEG=2.0
MANUAL_WINDOW_DEG=5.0
class Fault(RuntimeError):pass
class Cancelled(Fault):pass

def atomic_json(path,data):
 path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
 temp=path.with_suffix(path.suffix+'.tmp');temp.write_text(json.dumps(data,indent=2),encoding='utf-8');temp.replace(path)

def fingerprint(config):
 return hashlib.sha256(json.dumps({'joints':config['joints'],'motion':config['motion'],'tool':config['tool'],'gripper':config.get('gripper_variant'),'grip_settings':config.get('gripper'),'ik':config.get('ik')},sort_keys=True).encode()).hexdigest()

def nominal_tool(variant):
 if variant not in ['stock','v','slot']:raise Fault('Unknown jaw variant.')
 return {'offset_m':[.09193,-.0016,0. if variant=='stock' else -.018],'verified':False}

def manual_session_config(config,raw):
 """Temporary encoder window for uncalibrated joint jogging, never saved as calibration."""
 validate_config(config)
 if len(raw)!=6 or any(type(r)!=int for r in raw):raise Fault('Manual setup needs six integer encoder readings.')
 result=copy.deepcopy(config)
 for index,(j,r) in enumerate(zip(result['joints'],raw)):
  window=max(1,int(MANUAL_WINDOW_DEG*j['counts_per_rev']/360))
  lo=max(-1048575,r-window);hi=min(1048575,r+window)
  if j['mode']==3:lo=max(0,lo);hi=min(4095,hi)
  if j['limits'] is not None:lo=max(lo,j['limits'][0]);hi=min(hi,j['limits'][1])
  if index==5:
   neutral=round(config.get('gripper',{}).get('neutral_motor_degrees',180)*j['counts_per_rev']/360)
   lo=max(lo,min(r,neutral))  # Do not close past the reported closed position.
  if not lo<=r<=hi or lo>=hi:raise Fault(f"ID {j['id']}: current position is outside the usable manual window or measured limits.")
  j['limits']=[lo,hi];j['verified']=False
 result['manual_session']=True
 return result

def valid_number(x):return isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x)

def validate_config(c,hardware=False):
 js=c['joints']
 if len(js)!=6:raise Fault('Expected six joint records.')
 ids=[j['id'] for j in js]
 if any(type(i)!=int or not 0<=i<=252 for i in ids) or len(set(ids))!=6:raise Fault('Motor IDs must be six unique integers in 0..252, in joint order.')
 if [j['name'] for j in js]!=NAMES:raise Fault('Joint names must match the configured motor order.')
 tool=c['tool']
 if len(tool['offset_m'])!=3 or not all(valid_number(v) and abs(v)<.3 for v in tool['offset_m']):raise Fault('Invalid ring-center tool offset.')
 if type(tool.get('verified'))!=bool:raise Fault('Tool verification must be true or false.')
 for j in js:
  if j['name'] not in NAMES or j['sign'] not in [-1,1]:raise Fault('Invalid joint mapping/sign.')
  if j.get('mode') not in [3,4]:raise Fault('Only position mode 3 or extended-position mode 4 is supported.')
  if hardware:
   if not j.get('verified') or j.get('limits') is None or j.get('raw_reference') is None:raise Fault(f"Calibrate/verify ID {j['id']} first.")
  if j.get('limits') is not None:
   lo,hi=j['limits']
   if type(lo)!=int or type(hi)!=int or not -1048575<=lo<hi<=1048575:raise Fault('Invalid raw software limits.')
   if j['mode']==3 and not 0<=lo<hi<=4095:raise Fault('Mode 3 limits must stay within 0..4095.')
  if j.get('raw_reference') is not None and type(j['raw_reference'])!=int:raise Fault('Reference counts must be integers.')
  if not valid_number(j['model_reference_rad']):raise Fault('Invalid reference angle.')
  if not valid_number(j['counts_per_rev']) or j['counts_per_rev']<=0:raise Fault('Invalid encoder scale.')
 m=c['motion']
 for key in ['ticks_per_second','ticks_per_second2','period_s','max_command_step','following_error_ticks','arrival_ticks','settle_timeout_s']:
  if not valid_number(m[key]) or m[key]<=0:raise Fault(f'Invalid motion setting: {key}')
 if m['period_s']>0.2:raise Fault('Control period must be <=0.2 seconds.')
 for key in ['profile_velocity','profile_acceleration']:
  if type(m[key])!=int or not 1<=m[key]<=32767:raise Fault('Profiles must be nonzero integer values.')
 if not 1<=m['watchdog_units']<=127:raise Fault('Invalid watchdog interval.')

 if 'ik' in c:
  for key,value in c['ik'].items():
   if not valid_number(value) or value<=0:raise Fault(f'Invalid IK setting: {key}')
  if c['ik']['max_goal_xy_error_mm']>c['ik']['max_feedback_xy_drift_mm']:raise Fault('Goal XY bound exceeds feedback bound.')

def raw_to_q(c,raw):
 if len(raw)!=6:raise Fault('Expected six encoder positions.')
 return [j['model_reference_rad']+j['sign']*(r-(j['raw_reference'] or 0))*TAU/j['counts_per_rev'] for j,r in zip(c['joints'],raw)]

def q_to_raw(c,q):
 if len(q)!=6 or not all(valid_number(v) for v in q):raise Fault('Expected six finite joint angles.')
 return [int(round((j['raw_reference'] or 0)+j['sign']*(a-j['model_reference_rad'])*j['counts_per_rev']/TAU)) for j,a in zip(c['joints'],q)]

def raw_to_motor_degrees(c,raw):
 """Absolute encoder degrees, without modulo wrapping in extended mode."""
 if len(raw)!=6 or not all(valid_number(v) for v in raw):raise Fault('Expected six finite encoder positions.')
 return [r*360/j['counts_per_rev'] for j,r in zip(c['joints'],raw)]

def motor_degrees_to_raw(c,degrees):
 if len(degrees)!=6 or not all(valid_number(v) for v in degrees):raise Fault('Expected six finite motor angles.')
 return [int(round(d*j['counts_per_rev']/360)) for j,d in zip(c['joints'],degrees)]

def motor_degrees_to_q(c,degrees):
 """Continuous UI conversion. Only actual motor commands are encoder-rounded."""
 if len(degrees)!=6 or not all(valid_number(v) for v in degrees):raise Fault('Expected six finite motor angles.')
 return [j['model_reference_rad']+j['sign']*(math.radians(d)-(j['raw_reference'] or 0)*TAU/j['counts_per_rev']) for j,d in zip(c['joints'],degrees)]

def q_to_motor_degrees(c,q):
 if len(q)!=6 or not all(valid_number(v) for v in q):raise Fault('Expected six finite joint angles.')
 return [(j['raw_reference'] or 0)*360/j['counts_per_rev']+j['sign']*math.degrees(a-j['model_reference_rad']) for j,a in zip(c['joints'],q)]

def mock_pose_set(c):
 """Invented mock-only poses; 180 degrees is neutral, not a qualified grip."""
 base=[j['raw_reference'] or 0 for j in c['joints']]
 offsets={'source_approach':[0,0,0,0,0,0],'source_grasp':[0,50,-30,-20,0,0],
  'source_lift_01':[0,20,-20,0,0,0],'source_lift_02':[0,-10,-10,20,0,0],'source_clear':[0,-40,0,40,0,0],
  'transit_clear':[80,-40,0,40,0,0],'target_clear':[160,-40,0,40,0,0],'target_lower_01':[160,0,-15,15,0,0],
  'target_seated':[160,50,-30,-20,0,0],'target_retracted':[160,-20,-10,30,0,0],'return_clear':[80,-40,0,40,0,0],
  'gripper_open':[0,0,0,0,0,150],'gripper_closed':[0,0,0,0,0,-8]}
 return {name:{'raw':[a+b for a,b in zip(base,off)],'source':'mock','calibration':fingerprint(c)} for name,off in offsets.items()}

def validate_raw(c,raw):
 if len(raw)!=6:raise Fault('Expected all six motors.')
 for j,r in zip(c['joints'],raw):
  if type(r)!=int or j.get('limits') is None:raise Fault('Missing limits or noninteger goal.')
  if not j['limits'][0]<=r<=j['limits'][1]:raise Fault(f"ID {j['id']}: {r} outside {j['limits']}.")

def trajectory(c,start,end,min_duration=0):
 validate_raw(c,start);validate_raw(c,end)
 if not valid_number(min_duration) or min_duration<0:raise Fault('Invalid duration.')
 m=c['motion'];delta=max(abs(a-b) for a,b in zip(start,end));dt=m['period_s']
 duration=max(dt,min_duration,1.875*delta/m['ticks_per_second'],math.sqrt(5.774*delta/m['ticks_per_second2']))
 count=max(1,math.ceil(duration/dt));dt=duration/count
 frames=[];previous=list(start)
 for k in range(1,count+1):
  u=k/count;s=10*u**3-15*u**4+6*u**5
  raw=[int(round(a+(b-a)*s)) for a,b in zip(start,end)]
  validate_raw(c,raw)
  if max(abs(a-b) for a,b in zip(previous,raw))>m['max_command_step']:raise Fault('Planned increment exceeds configured bound; reduce speed/control period.')
  frames.append((dt,raw));previous=raw
 return frames

def sequence_digest(c,poses,sequence):
 return hashlib.sha256(json.dumps([fingerprint(c),poses,sequence['steps']],sort_keys=True).encode()).hexdigest()

def validate_sequence(c,poses,sequence,hardware=False):
 if not sequence.get('steps'):raise Fault('No sequence steps.')
 if not any(s.get('kind')=='arm' for s in sequence['steps']):raise Fault('Sequence needs an arm entry pose.')
 for step in sequence['steps']:
  if step['kind'] not in ['arm','grip','vertical']:raise Fault('Sequence kind must be arm, grip or vertical.')
  if step['kind']=='vertical' and (not valid_number(step.get('travel_mm')) or not .1<=abs(step['travel_mm'])<=150):raise Fault('Invalid vertical travel.')
  name=step['pose']
  if name not in poses:raise Fault(f'Missing taught pose: {name}')
  p=poses[name];validate_raw(c,p['raw'])
  for key in ['minimum_seconds','dwell_seconds']:
   if not valid_number(step.get(key,0)) or not 0<=step.get(key,0)<=120:raise Fault('Invalid segment timing.')
  if hardware and (p.get('source')!='hardware' or p.get('calibration')!=fingerprint(c)):raise Fault(f'{name}: simulated or stale-calibration pose; teach again on hardware.')
 if hardware and sequence.get('approved_digest')!=sequence_digest(c,poses,sequence):raise Fault('Preview and approve this exact sequence after teaching/calibration changes.')

def merged_goal(kind,current,pose):
 return pose[:5]+[current[5]] if kind=='arm' else current[:5]+[pose[5]]

class Controller:
 def __init__(self,bus,config,on_state=None,log=None):
  self.bus=bus;self.config=config;self.on_state=on_state or (lambda s:None);self.log=log or (lambda *a:None)
  self.armed=False;self.calibration_only=False;self.manual_only=False;self.busy=False;self.stop_event=threading.Event();self.lock=threading.Lock();self.last=None
 def read(self):
  self.last=self.bus.snapshot();self.on_state(self.last);return self.last
 def health(self,s,require_torque=True):
  if [x['id'] for x in s]!=[j['id'] for j in self.config['joints']]:raise Fault('Feedback motor IDs/order do not match the configured joints.')
  validate_raw(self.config,[x['raw'] for x in s])
  for j,x in zip(self.config['joints'],s):
   if x['error']:raise Fault(f"ID {j['id']}: hardware error {x['error']}.")
   if x['watchdog']<0:raise Fault(f"ID {j['id']}: watchdog expired; inspect and recover explicitly.")
   if require_torque and not x['torque']:raise Fault(f"ID {j['id']}: torque is off.")
 def arm(self,calibration_only=False,manual_only=False):
  if self.busy:raise Fault('A motion is active.')
  self.armed=False
  if calibration_only and manual_only:raise Fault('Choose one arming mode.')
  if manual_only and not self.config.get('manual_session'):raise Fault('Start the app with --manual to use temporary encoder limits.')
  validate_config(self.config,hardware=not self.bus.is_mock and not calibration_only and not manual_only)
  s=self.read();self.health(s,False)
  self.bus.prepare_hold(self.config)  # RAM only, no mode/ID/EEPROM changes.
  s=self.read();self.health(s);self.armed=True;self.calibration_only=calibration_only;self.manual_only=manual_only;self.stop_event.clear();self.log('ARM',{'raw':[x['raw'] for x in s],'manual_only':manual_only})
 def request_stop(self):
  self.stop_event.set();self.armed=False
 def hold(self):
  # Fresh positions only; never send stale coordinates if the bus cannot be read.
  try:
   s=self.read()
   if any(x['error'] or x['watchdog']<0 or not x['torque'] for x in s):return
   raw=[x['raw'] for x in s];validate_raw(self.config,raw);self.bus.goals(raw);self.log('HOLD',{'raw':raw})
  except Exception as exc:self.log('HOLD_FAILED',{'error':str(exc)})
 def check_stop(self):
  if self.stop_event.is_set():raise Cancelled('Stop requested; sequence aborted. Re-arm before moving.')
 def _move(self,target,min_duration=0):
  self.check_stop()
  s=self.read();self.health(s);start=[x['raw'] for x in s]
  frames=trajectory(self.config,start,target,min_duration)
  previous=start
  for delay,goal in frames:
   self.check_stop();s=self.read();self.health(s);present=[x['raw'] for x in s]
   if max(abs(a-b) for a,b in zip(previous,present))>self.config['motion']['following_error_ticks']:raise Fault('Following error exceeded; inspect obstruction or calibration.')
   # Limit a new command relative to both previous command and actual feedback.
   if max(abs(a-b) for a,b in zip(goal,present))>self.config['motion']['following_error_ticks']:raise Fault('Goal too far ahead of feedback.')
   self.bus.goals(goal);self.log('GOAL',{'raw':goal});previous=goal
   if self.stop_event.wait(delay):self.check_stop()
  deadline=time.monotonic()+self.config['motion']['settle_timeout_s'];stable=0
  while time.monotonic()<deadline:
   self.check_stop();s=self.read();self.health(s)
   if max(abs(x['raw']-g) for x,g in zip(s,target))<=self.config['motion']['arrival_ticks']:
    stable+=1
    if stable>=3:return
   else:stable=0
   if self.stop_event.wait(self.config['motion']['period_s']):self.check_stop()
  raise Fault('Arrival timeout: target not reached. Grip force is not verified by position alone.')
 def _job(self,fn):
  with self.lock:
   if self.busy:raise Fault('Already moving.')
   if not self.armed:raise Fault('Arm the controller first.')
   self.busy=True
  try:fn()
  except Exception as exc:
   self.armed=False;self.hold();self.log('FAULT',{'message':str(exc)});raise
  finally:self.busy=False
 def jog(self,index,delta):
  if type(index)!=int or not 0<=index<6 or type(delta)!=int or not 0<abs(delta)<=8:raise Fault('Raw jog must be 1..8 ticks on one motor.')
  def perform():
   target=[x['raw'] for x in self.read()];target[index]+=delta;self._move(target)
  self._job(perform)
 def move(self,target,min_duration=0,preview_anchor=None):
  if self.calibration_only:raise Fault('Raw-jog mode: finish sign calibration and re-arm for model/pose movement.')
  def perform():
   goal=list(target)
   if self.manual_only:
    if preview_anchor is None:raise Fault('Click Copy feedback into preview, then change one slider by at most 2 degrees.')
    validate_raw(self.config,goal);validate_raw(self.config,preview_anchor)
    changed=[i for i,(a,b) in enumerate(zip(goal,preview_anchor)) if a!=b]
    if len(changed)!=1:raise Fault('Manual mode needs exactly one changed joint. Copy feedback, then change one slider.')
    state=self.read();self.health(state);present=[x['raw'] for x in state];i=changed[0]
    maximum=max(1,int(MANUAL_STEP_DEG*self.config['joints'][i]['counts_per_rev']/360))
    if abs(goal[i]-preview_anchor[i])>maximum or abs(goal[i]-present[i])>maximum:raise Fault('Manual Execute preview is limited to 2 degrees on one joint per command. Copy feedback and make a smaller change.')
    selected_goal=goal[i];goal=present.copy();goal[i]=selected_goal
   self._move(goal,min_duration)
  self._job(perform)
 def run(self,poses,sequence):
  if self.manual_only or self.config.get('manual_session'):raise Fault('Manual mode is for individual joints. Use calibrated hardware mode for pick-and-place.')
  if self.calibration_only:raise Fault('Complete calibration and re-arm before sequence playback.')
  validate_sequence(self.config,poses,sequence,not self.bus.is_mock)
  def execute():
   s=self.read();self.health(s);start=[x['raw'] for x in s]
   first=next((poses[x['pose']]['raw'] for x in sequence['steps'] if x['kind']=='arm'),None)
   if first is None:raise Fault('Sequence needs an arm entry pose.')
   if max(abs(a-b) for a,b in zip(start[:5],first[:5]))>self.config['motion']['arrival_ticks']:raise Fault('Move to the first arm pose before automatic playback; entry path is not assumed clear.')
   # Compile every Cartesian lift before sending the first movement command.
   actions=[];predicted=start.copy();robot=None
   if any(step['kind']=='vertical' for step in sequence['steps']):
    from .model import Robot
    from .ik import plan_vertical,execute_vertical
    robot=Robot(Path(__file__).resolve().parents[1],self.config['gripper_variant'])
    if not self.bus.is_mock and not self.config['tool'].get('verified'):raise Fault('Verify the measured tool offset before hardware IK.')
   for step in sequence['steps']:
    if step['kind']=='vertical':
     anchor=poses[step['pose']]['raw']
     if max(abs(a-b) for a,b in zip(predicted[:5],anchor[:5]))>self.config['motion']['arrival_ticks']:raise Fault('Vertical step anchor does not match the preceding arm pose.')
     plan=plan_vertical(robot,self.config,predicted,step['travel_mm'],lambda:(self.check_stop(),self.health(self.read())))
     actions.append((step,plan));predicted=plan['frames'][-1]['raw']
    else:
     goal=merged_goal(step['kind'],predicted,poses[step['pose']]['raw']);trajectory(self.config,predicted,goal,step.get('minimum_seconds',0));actions.append((step,goal));predicted=goal
   for step,target in actions:
    self.check_stop();self.log('STEP_START',step)
    if step['kind']=='vertical':execute_vertical(self,robot,target,nested=True)
    else:self._move(target,step.get('minimum_seconds',0))
    until=time.monotonic()+step.get('dwell_seconds',0)
    while time.monotonic()<until:
     self.check_stop();self.health(self.read())
     if self.stop_event.wait(min(self.config['motion']['period_s'],max(0,until-time.monotonic()))):self.check_stop()
    self.log('STEP_DONE',step)
   self.log('SEQUENCE_COMPLETE',{'note':'Position sequence complete; verify ring placement physically.'})
  self._job(execute)
