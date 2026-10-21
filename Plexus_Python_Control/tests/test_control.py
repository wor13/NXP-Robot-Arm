import copy,json,math,time,threading
from pathlib import Path
import pytest
from plexus.core import *
from plexus.bus import MockBus,signed,le32,DynamixelBus
from plexus.model import Robot
ROOT=Path(__file__).resolve().parents[1]
@pytest.fixture
def cfg():return json.loads((ROOT/'config/demo.json').read_text())

def fast(c):
 c=copy.deepcopy(c);c['motion'].update(period_s=.001,ticks_per_second=10000,ticks_per_second2=100000,max_command_step=16,settle_timeout_s=.03);return c

def test_signed_and_wire_bytes():
 assert signed(0xffffff68)==-152
 assert le32(-152)==[104,255,255,255]
 assert signed(0xffff,16)==-1

def test_calibration_round_trip_including_negative_reference(cfg):
 cfg['joints'][3]['raw_reference']=-152;cfg['joints'][3]['sign']=-1
 raw=[2050,1900,2400,-170,2100,2200]
 assert q_to_raw(cfg,raw_to_q(cfg,raw))==raw

def test_missing_calibration_blocks_hardware_arm_before_writes(cfg):
 class HW(MockBus):is_mock=False
 cfg['joints'][0]['verified']=False;b=HW(cfg);controller=Controller(b,cfg)
 with pytest.raises(Fault):controller.arm()
 assert not b.writes

def test_bad_limit_and_nonfinite_goals_rejected(cfg):
 with pytest.raises(Fault):trajectory(cfg,[2048]*6,[4000]*6)
 with pytest.raises(Fault):q_to_raw(cfg,[float('nan')]*6)
 with pytest.raises(Fault):validate_raw(cfg,[2048.,2048,2048,2048,2048,2048])

def test_quintic_limits_endpoints_and_monotonicity(cfg):
 start=[2048]*6;end=[2500,1600,2100,2048,1800,2048]
 frames=trajectory(cfg,start,end);last=start
 assert frames[-1][1]==end
 for dt,x in frames:
  assert max(abs(a-b) for a,b in zip(last,x))<=cfg['motion']['max_command_step']
  assert (x[0]>=last[0]) and (x[1]<=last[1]);last=x
 total=sum(dt for dt,_ in frames)
 assert total>=1.875*452/cfg['motion']['ticks_per_second']

def test_group_motion_moves_all_axes_and_settles(cfg):
 c=fast(cfg);b=MockBus(c);ctrl=Controller(b,c);ctrl.arm();goal=[2056,2044,2052,2048,2049,2048];ctrl.move(goal)
 assert b.raw==goal and b.writes and all(len(x)==6 for x in b.writes)

def test_stall_aborts_sequence_without_opening_gripper(cfg):
 class Stalled(MockBus):
  def goals(self,raw):self.writes.append(list(raw))
 c=fast(cfg);c['motion']['settle_timeout_s']=.004;b=Stalled(c);ctrl=Controller(b,c);ctrl.arm()
 with pytest.raises(Fault,match='Arrival timeout'):ctrl.move([2056,2048,2048,2048,2048,2048])
 assert not ctrl.armed and b.writes[-1]==[2048]*6

def test_hardware_error_blocks_command(cfg):
 class Faulty(MockBus):
  def snapshot(self):
   out=super().snapshot();out[2]['error']=4;return out
 b=Faulty(cfg);ctrl=Controller(b,cfg)
 with pytest.raises(Fault):ctrl.arm()
 assert not b.writes

def test_fault_during_motion_aborts_and_no_stale_hold(cfg):
 class Lost(MockBus):
  def snapshot(self):
   if self.writes:raise Fault('Communication lost')
   return super().snapshot()
 c=fast(cfg);b=Lost(c);ctrl=Controller(b,c);ctrl.arm()
 with pytest.raises(Fault):ctrl.move([2056,2048,2048,2048,2048,2048])
 assert len(b.writes)==1 and not ctrl.armed

def test_stop_aborts_without_torque_off(cfg):
 c=fast(cfg);b=MockBus(c);ctrl=Controller(b,c);ctrl.arm();ctrl.request_stop()
 with pytest.raises(Fault):ctrl.move([2052]*6)
 assert b.enabled and not ctrl.armed

def test_raw_jog_only_cannot_run_full_motion(cfg):
 c=fast(cfg);b=MockBus(c);ctrl=Controller(b,c);ctrl.arm(calibration_only=True);ctrl.jog(3,-4)
 assert b.raw[3]==2044
 with pytest.raises(Fault,match='Raw-jog'):ctrl.move([2048]*6)
 with pytest.raises(Fault):ctrl.jog(3,9)

def test_phase_masks_prevent_gripper_changes_during_arm_moves():
 now=[1,2,3,4,5,100];pose=[10,20,30,40,50,200]
 assert merged_goal('arm',now,pose)==[10,20,30,40,50,100]
 assert merged_goal('grip',now,pose)==[1,2,3,4,5,200]

def test_sequence_provenance_and_approval_invalidation(cfg):
 seq={'steps':[{'kind':'arm','pose':'a'}],'approved_digest':None}
 poses={'a':{'raw':[2048]*6,'source':'mock','calibration':fingerprint(cfg)}}
 with pytest.raises(Fault,match='simulated'):validate_sequence(cfg,poses,seq,True)
 poses['a']['source']='hardware';seq['approved_digest']=sequence_digest(cfg,poses,seq);validate_sequence(cfg,poses,seq,True)
 poses['a']['raw'][0]+=1
 with pytest.raises(Fault,match='approve'):validate_sequence(cfg,poses,seq,True)

def test_first_arm_entry_must_already_match(cfg):
 c=fast(cfg);b=MockBus(c);ctrl=Controller(b,c);ctrl.arm()
 poses={'entry':{'raw':[2200]*6,'source':'mock'}};seq={'steps':[{'kind':'arm','pose':'entry'}]}
 with pytest.raises(Fault,match='first arm pose'):ctrl.run(poses,seq)
 assert all(x==[2048]*6 for x in b.writes)

def test_complete_sequence_and_grip_isolation(cfg):
 c=fast(cfg);b=MockBus(c);ctrl=Controller(b,c);ctrl.arm()
 poses={'entry':{'raw':[2048]*6},'closed':{'raw':[2999]*5+[2036]},'lift':{'raw':[2052]*5+[1001]},'open':{'raw':[1999]*5+[2056]}}
 seq={'steps':[{'kind':'arm','pose':'entry'},{'kind':'grip','pose':'closed'},{'kind':'arm','pose':'lift'},{'kind':'grip','pose':'open'}]}
 ctrl.run(poses,seq);assert b.raw==[2052]*5+[2056]
 assert any(x==[2048]*5+[2036] for x in b.writes)

def test_model_fk_and_all_variants_have_meshes():
 import numpy as np
 for v in ['stock','v','slot']:
  r=Robot(ROOT,v);poses,_=r.fk([0]*6)
  assert set(['link0','link1','link2','link3','link4','link5','link6','link7']).issubset(poses)
  assert np.allclose(poses['link5'][:3,3],[.22095,0,.21065])
  assert len(r.visuals)==8 and all(x[2].exists() for x in r.visuals)

def test_model_joint_axes_and_mimic():
 import numpy as np
 r=Robot(ROOT,'v');p,_=r.fk([0,0,0,0,0,.2]);a=p['link6'][:3,:3];b=p['link7'][:3,:3]
 assert np.allclose(a,b.T)

def test_model_fk_against_independent_urdf_reference():
 # Optional developer cross-check; yourdfpy is deliberately NOT a runtime dependency.
 yp=pytest.importorskip('yourdfpy');import numpy as np
 from yourdfpy.urdf import filename_handler_magic
 path=ROOT/'models/open_manipulator_description/urdf/omx_f/omx_f.urdf'
 ref=yp.URDF.load(path,load_meshes=False,load_collision_meshes=False)
 q=[.1,.3,-.8,.4,.2,.25];ref.update_cfg(dict(zip(NAMES,q)));ours,_=Robot(ROOT,'stock').fk(q)
 for name in ['link1','link2','link3','link4','link5','link6','link7']:assert np.allclose(ours[name],ref.get_transform(name),atol=1e-10)
