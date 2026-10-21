import json,copy
from pathlib import Path
import numpy as np
import pytest
from plexus.core import *
from plexus.ik import plan_vertical,execute_vertical
from plexus.model import Robot
from plexus.bus import MockBus
ROOT=Path(__file__).resolve().parents[1]
@pytest.fixture
def c():return json.loads((ROOT/'config/demo.json').read_text())
@pytest.fixture
def robot():return Robot(ROOT,'v')

def test_vertical_plan_holds_xy_normal_and_gripper(c,robot):
 p=plan_vertical(robot,c,[2048]*6,2)
 assert p['max_nominal_xy_error_mm']<=.2
 assert p['max_nominal_orientation_error_deg']<=.35
 assert p['frames'][-1]['target_xyz_m'][2]==pytest.approx(p['start_xyz_m'][2]+.002)
 assert all(f['raw'][5]==2048 for f in p['frames'])
 assert all(np.allclose(f['target_xyz_m'][:2],p['start_xyz_m'][:2]) for f in p['frames'])

def test_encoder_rounding_and_joint_interpolation(c,robot):
 p=plan_vertical(robot,c,[2048]*6,1)
 previous=p['start_raw']
 for f in p['frames']:
  assert max(abs(a-b) for a,b in zip(previous,f['raw']))<=c['motion']['max_command_step']
  qa=np.array(raw_to_q(c,previous));qb=np.array(raw_to_q(c,f['raw']))
  for t in np.linspace(0,1,7):
   T=robot.tcp(qa+(qb-qa)*t,c['tool']);assert np.linalg.norm(T[:2,3]-np.array(p['start_xyz_m'])[:2])*1000<=.3
  previous=f['raw']

def test_tilted_start_rejected(c,robot):
 c['joints'][4]['model_reference_rad']=.2
 with pytest.raises(Fault,match='not level'):plan_vertical(robot,c,[2048]*6,2)

def test_unreachable_lift_is_rejected_before_any_command(c,robot):
 b=MockBus(c)
 with pytest.raises(Fault):plan_vertical(robot,c,b.raw,150)
 assert not b.writes

def test_changed_calibration_invalidates_preplanned_lift(c,robot):
 p=plan_vertical(robot,c,[2048]*6,1);b=MockBus(c);controller=Controller(b,c);controller.arm();c['tool']['offset_m'][0]+=.001
 with pytest.raises(Fault,match='changed'):execute_vertical(controller,robot,p)
 assert not b.writes

def test_feedback_lateral_drift_aborts(c,robot):
 p=plan_vertical(robot,c,[2048]*6,1)
 class Drifts(MockBus):
  def goals(self,raw):super().goals(raw);self.raw[0]+=10
 b=Drifts(c);controller=Controller(b,c);controller.arm()
 with pytest.raises(Fault,match='lateral'):execute_vertical(controller,robot,p)
 assert not controller.armed

def test_hardware_requires_measured_tool_offset(c,robot):
 p=plan_vertical(robot,c,[2048]*6,1)
 class Hardware(MockBus):is_mock=False
 controller=Controller(Hardware(c),c);controller.arm()
 with pytest.raises(Fault,match='tool offset'):execute_vertical(controller,robot,p)

def test_fk_local_tool_offset_changes_with_rotation(c,robot):
 q=raw_to_q(c,[2048]*6);A=robot.tcp(q,c['tool']);q[4]+=.2;B=robot.tcp(q,c['tool'])
 assert not np.allclose(A[:3,3],B[:3,3])
