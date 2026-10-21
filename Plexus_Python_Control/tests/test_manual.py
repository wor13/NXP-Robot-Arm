"""Manual encoder execution must not require or fabricate IK calibration."""
import copy,json
from pathlib import Path
import pytest
from plexus.core import *
from plexus.bus import MockBus
from plexus.ik import execute_vertical
ROOT=Path(__file__).resolve().parents[1]
@pytest.fixture
def base():return json.loads((ROOT/'config/hardware.json').read_text())

def session(base,start=None):
 c=manual_session_config(base,start or [2048]*6)
 c['motion'].update(period_s=.001,ticks_per_second=10000,ticks_per_second2=100000,settle_timeout_s=.05)
 class Hardware(MockBus):is_mock=False
 b=Hardware(c);b.raw=list(start or [2048]*6);ctrl=Controller(b,c);ctrl.arm(manual_only=True)
 return c,b,ctrl

def test_com7_is_default(base):assert base['serial']['port']=='COM7'

def test_manual_window_is_temporary_and_never_marks_calibration(base):
 before=copy.deepcopy(base);c=manual_session_config(base,[2048]*6)
 assert base==before and c['manual_session']
 assert all(not j['verified'] for j in c['joints'])
 assert all(j['limits'] for j in c['joints'])
 assert c['joints'][5]['limits'][0]==2048
 with pytest.raises(Fault,match='Calibrate'):validate_config(c,True)

def test_existing_measured_limits_are_never_widened(base):
 base['joints'][0]['limits']=[2040,2055]
 c=manual_session_config(base,[2048]*6)
 assert c['joints'][0]['limits']==[2040,2055]
 with pytest.raises(Fault):manual_session_config(base,[2100]+[2048]*5)

@pytest.mark.parametrize('axis',[0,1,2,3,4,5])
def test_manual_moves_a_single_joint_without_verified_calibration(base,axis):
 c,b,ctrl=session(base);anchor=b.raw.copy();goal=anchor.copy();goal[axis]+=6
 ctrl.move(goal,preview_anchor=anchor)
 assert b.raw==goal and ctrl.armed and ctrl.manual_only
 assert all(all(raw[i]==2048 for i in range(6) if i!=axis) for raw in b.writes)

def test_manual_holds_other_axes_from_fresh_feedback(base):
 c,b,ctrl=session(base);anchor=b.raw.copy();goal=anchor.copy();goal[0]+=6
 b.raw[1]+=3  # Feedback changed since the preview copy.
 ctrl.move(goal,preview_anchor=anchor)
 assert b.raw==[2054,2051,2048,2048,2048,2048]

def test_manual_rejects_multiple_changed_sliders(base):
 c,b,ctrl=session(base);anchor=b.raw.copy();goal=anchor.copy();goal[:2]=[2050,2050]
 with pytest.raises(Fault,match='exactly one'):ctrl.move(goal,preview_anchor=anchor)
 assert all(raw==anchor for raw in b.writes)

def test_manual_rejects_large_or_stale_preview(base):
 c,b,ctrl=session(base);anchor=b.raw.copy()
 with pytest.raises(Fault,match='2 degrees'):ctrl.move([2080]+anchor[1:],preview_anchor=anchor)
 assert all(raw==anchor for raw in b.writes)
 c,b,ctrl=session(base);anchor=b.raw.copy();b.raw[0]-=25
 with pytest.raises(Fault,match='2 degrees'):ctrl.move([2054]+anchor[1:],preview_anchor=anchor)
 assert all(raw==[2023]+anchor[1:] for raw in b.writes)

def test_manual_requires_preview_anchor_and_rejects_past_neutral(base):
 c,b,ctrl=session(base)
 with pytest.raises(Fault,match='Copy feedback'):ctrl.move([2054]+[2048]*5)
 c,b,ctrl=session(base);anchor=b.raw.copy()
 with pytest.raises(Fault,match='outside'):ctrl.move([2048]*5+[2040],preview_anchor=anchor)


def test_manual_will_not_run_ik_or_pick_place(base):
 c,b,ctrl=session(base)
 with pytest.raises(Fault,match='Manual mode'):ctrl.run({}, {'steps':[]})
 with pytest.raises(Fault,match='calibrated hardware mode'):execute_vertical(ctrl,None,{})
 assert not b.writes

def test_manual_requires_explicit_session_and_retains_full_calibration_gate(base):
 b=MockBus(base);ctrl=Controller(b,base)
 with pytest.raises(Fault,match='--manual'):ctrl.arm(manual_only=True)
 c,b,ctrl=session(base);ctrl.request_stop()
 with pytest.raises(Fault,match='Calibrate'):ctrl.arm()
 assert not ctrl.armed
