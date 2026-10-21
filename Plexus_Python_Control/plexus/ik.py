"""Task-space vertical IK: XYZ plus tool-normal direction, with gripper fixed.
Five arm axes cannot generally satisfy six independent pose constraints.
All planning completes before any motor command is sent.
"""
import itertools,math
import numpy as np
from scipy.optimize import least_squares
from .core import Fault,raw_to_q,q_to_raw,validate_raw,fingerprint

def angle(a,b):return math.degrees(math.acos(float(np.clip(np.dot(a,b),-1,1))))

def plan_vertical(robot,c,start_raw,travel_mm,heartbeat=None):
 validate_raw(c,start_raw)
 if not isinstance(travel_mm,(int,float)) or not math.isfinite(travel_mm) or not .1<=abs(travel_mm)<=150:raise Fault('Vertical travel must be 0.1..150 mm in magnitude.')
 settings=c['ik'];q0=np.array(raw_to_q(c,start_raw));T0=robot.tcp(q0,c['tool']);p0=T0[:3,3].copy();normal=T0[:3,2].copy()
 if angle(normal,np.array([0.,0.,1.]))>settings['initial_level_tolerance_deg']:raise Fault('Ring/tool plane is not level in the calibrated model. Align it before engaging the post.')
 bounds=[]
 for j in c['joints'][:5]:
  v=[j['model_reference_rad']+j['sign']*(r-j['raw_reference'])*2*math.pi/j['counts_per_rev'] for r in j['limits']]
  bounds.append((max(-2*math.pi,min(v)),min(2*math.pi,max(v))))
 lo=np.array([x[0] for x in bounds]);hi=np.array([x[1] for x in bounds])
 distance=abs(travel_mm);duration=max(.2,1.875*distance/settings['vertical_mm_per_second'],math.sqrt((10/math.sqrt(3))*distance/settings['vertical_mm_per_second2']))
 dt=c['motion']['period_s'];count=math.ceil(duration/dt);dt=duration/count
 previous_q=q0.copy();previous_raw=list(start_raw);frames=[];worst_xy=0.;worst_z=0.;worst_angle=0.
 candidates=list(itertools.product([-1,0,1],repeat=5))
 for i in range(1,count+1):
  if heartbeat is not None:heartbeat()
  u=i/count;s=10*u**3-15*u**4+6*u**5;target=p0+np.array([0.,0.,travel_mm/1000*s])
  def residual(arm):
   q=np.r_[arm,q0[5]];T=robot.tcp(q,c['tool'])
   return np.r_[(T[:3,3]-target)/.0001,np.cross(T[:3,2],normal)/.001]
  result=least_squares(residual,np.clip(previous_q[:5],lo+1e-10,hi-1e-10),bounds=(lo,hi),xtol=1e-11,ftol=1e-11,gtol=1e-11,max_nfev=100)
  exact=np.r_[result.x,q0[5]];T=robot.tcp(exact,c['tool'])
  singular=np.linalg.svd(result.jac,compute_uv=False)
  if not result.success or np.linalg.norm(T[:3,3]-target)>.00005 or angle(T[:3,2],normal)>.05:raise Fault(f'Lift IK unreachable at sample {i}/{count}; no motion sent.')
  if singular[-1]/max(singular[0],1e-12)<1e-5:raise Fault('Lift approaches a singularity; move to a different grasp configuration.')
  if max(abs(exact[:5]-previous_q[:5]))>math.radians(3):raise Fault('Discontinuous IK branch; no motion sent.')
  nearest=q_to_raw(c,exact);best=None
  # Search nearby encoder-count combinations because the motors accept integers.
  for offset in candidates:
   raw=[nearest[k]+offset[k] for k in range(5)]+[start_raw[5]]
   if any(not j['limits'][0]<=r<=j['limits'][1] for j,r in zip(c['joints'],raw)):continue
   if max(abs(a-b) for a,b in zip(raw,previous_raw))>c['motion']['max_command_step']:continue
   q=raw_to_q(c,raw);T=robot.tcp(q,c['tool']);xy=np.linalg.norm(T[:2,3]-p0[:2])*1000;zerr=abs(T[2,3]-target[2])*1000;tilt=angle(T[:3,2],normal)
   if xy>settings['max_goal_xy_error_mm'] or zerr>settings['max_goal_z_error_mm'] or tilt>settings['orientation_tolerance_deg']:continue
   score=(xy/.1)**2+(zerr/.25)**2+(tilt/.15)**2+.015*sum((a-b)**2 for a,b in zip(raw,previous_raw))
   if best is None or score<best[0]:best=(score,raw,T,xy,zerr,tilt)
  if best is None:raise Fault(f'No encoder-rounded IK goal meets the lateral/level tolerance at sample {i}; no motion sent. Reconfigure posture or calibrate; do not simply loosen tolerances.')
  _,raw,T,xy,zerr,tilt=best
  # Check the nominal joint interpolation between successive rounded goals.
  qa=np.array(raw_to_q(c,previous_raw));qb=np.array(raw_to_q(c,raw))
  for fraction in [.25,.5,.75]:
   Ti=robot.tcp(qa+(qb-qa)*fraction,c['tool'])
   if np.linalg.norm(Ti[:2,3]-p0[:2])*1000>settings['max_feedback_xy_drift_mm']:raise Fault('Interpolated IK segment exceeds the lateral model-error bound.')
  frames.append({'dt_s':dt,'raw':raw,'target_xyz_m':target.tolist(),'nominal_xyz_m':T[:3,3].tolist()});previous_raw=raw;previous_q=exact
  worst_xy=max(worst_xy,xy);worst_z=max(worst_z,zerr);worst_angle=max(worst_angle,tilt)
 return {'calibration':fingerprint(c),'start_raw':list(start_raw),'start_xyz_m':p0.tolist(),'normal':normal.tolist(),'travel_mm':travel_mm,'duration_s':duration,'frames':frames,'max_nominal_xy_error_mm':worst_xy,'max_nominal_z_error_mm':worst_z,'max_nominal_orientation_error_deg':worst_angle}

def execute_vertical(controller,robot,plan,nested=False):
 c=controller.config
 if controller.manual_only or c.get('manual_session'):raise Fault('Vertical IK requires calibrated hardware mode; manual joint mode does not run IK.')
 if controller.calibration_only:raise Fault('Finish calibration and arm full motion first.')
 if plan['calibration']!=fingerprint(c):raise Fault('Calibration/settings changed. Replan the lift.')
 if not controller.bus.is_mock and not c['tool'].get('verified'):raise Fault('Measure and verify the actual ring-center tool offset before hardware IK.')
 def perform():
  state=controller.read();controller.health(state);raw=[x['raw'] for x in state]
  if max(abs(a-b) for a,b in zip(raw,plan['start_raw']))>c['motion']['arrival_ticks']:raise Fault('Arm moved since the lift was planned. Plan from current feedback again.')
  initial=robot.tcp(raw_to_q(c,raw),c['tool'])
  if np.linalg.norm(initial[:2,3]-np.array(plan['start_xyz_m'])[:2])*1000>.1:raise Fault('Start shifted laterally since planning; replan.')
  previous=raw
  for frame in plan['frames']:
   controller.check_stop();state=controller.read();controller.health(state);present=[x['raw'] for x in state]
   T=robot.tcp(raw_to_q(c,present),c['tool'])
   drift=np.linalg.norm(T[:2,3]-np.array(plan['start_xyz_m'])[:2])*1000
   tilt=angle(T[:3,2],np.array(plan['normal']))
   if drift>c['ik']['max_feedback_xy_drift_mm'] or tilt>c['ik']['feedback_orientation_tolerance_deg']:raise Fault('Feedback-predicted lateral drift/tilt exceeded the IK bound. Inspect alignment; sequence aborted.')
   if max(abs(a-b) for a,b in zip(previous,present))>c['motion']['following_error_ticks']:raise Fault('Following error during vertical lift.')
   if max(abs(a-b) for a,b in zip(frame['raw'],present))>c['motion']['following_error_ticks']:raise Fault('Vertical goal too far ahead of feedback.')
   controller.bus.goals(frame['raw']);controller.log('IK_FRAME',frame);previous=frame['raw']
   if controller.stop_event.wait(frame['dt_s']):controller.check_stop()
  # Settle at the final prechecked command, with the same lateral feedback guard.
  deadline=__import__('time').monotonic()+c['motion']['settle_timeout_s'];stable=0
  while __import__('time').monotonic()<deadline:
   controller.check_stop();state=controller.read();controller.health(state);present=[x['raw'] for x in state];T=robot.tcp(raw_to_q(c,present),c['tool'])
   if np.linalg.norm(T[:2,3]-np.array(plan['start_xyz_m'])[:2])*1000>c['ik']['max_feedback_xy_drift_mm']:raise Fault('Lateral model drift while settling.')
   if angle(T[:3,2],np.array(plan['normal']))>c['ik']['feedback_orientation_tolerance_deg']:raise Fault('Tool tilt while settling.')
   if max(abs(a-b) for a,b in zip(previous,present))<=c['motion']['arrival_ticks']:
    stable+=1
    if stable>=3:return
   else:stable=0
   if controller.stop_event.wait(c['motion']['period_s']):controller.check_stop()
  raise Fault('Vertical lift arrival timeout.')
 if nested:perform()
 else:controller._job(perform)
