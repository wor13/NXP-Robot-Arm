"""Regression checks for Thomas's 180-degree reference and renamed motor IDs."""
import copy,json,math,types
from pathlib import Path
import numpy as np
import pytest
from plexus.core import *
from plexus.bus import MockBus,DynamixelBus
from plexus.model import Robot
ROOT=Path(__file__).resolve().parents[1]
@pytest.fixture
def cfg():return json.loads((ROOT/'config/demo.json').read_text())

def test_reported_ids_and_neutral_are_in_both_configs():
 for name in ['demo','hardware']:
  c=json.loads((ROOT/f'config/{name}.json').read_text());validate_config(c)
  assert [j['id'] for j in c['joints']]==[11,12,13,1,2,3]
  assert all(j['raw_reference']==2048 and j['model_reference_rad']==0 for j in c['joints'])
  assert c['gripper']['neutral_state']=='closed_without_gripping'
  assert c['gripper']['grip_motor_degrees'] is None
  assert np.allclose(raw_to_q(c,[2048]*6),[0]*6)
  assert motor_degrees_to_raw(c,[180]*6)==[2048]*6
  assert np.allclose(q_to_motor_degrees(c,[0]*6),[180]*6)
 c=json.loads((ROOT/'config/hardware.json').read_text())
 assert all(not j['verified'] and j['limits'] is None for j in c['joints'])
 with pytest.raises(Fault,match='Calibrate'):validate_config(c,True)

def test_absolute_motor_degrees_round_trip_and_do_not_wrap(cfg):
 degrees=[180,65.5,276.3,-13.4,540,181.5]
 raw=motor_degrees_to_raw(cfg,degrees)
 assert raw[3]<0 and raw[4]>4095
 assert np.allclose(raw_to_motor_degrees(cfg,raw),degrees,atol=360/4096/2)
 assert np.allclose(q_to_motor_degrees(cfg,motor_degrees_to_q(cfg,degrees)),degrees)
 cfg['joints'][4]['sign']=-1
 assert np.allclose(q_to_motor_degrees(cfg,motor_degrees_to_q(cfg,degrees)),degrees)
 with pytest.raises(Fault):motor_degrees_to_raw(cfg,[180]*5+[float('nan')])

@pytest.mark.parametrize('axis',[1,2,3])
def test_increasing_pitch_motor_degrees_moves_tcp_down(cfg,axis):
 r=Robot(ROOT,'stock');a=[180]*6;b=a.copy();b[axis]+=1
 initial=r.tcp(motor_degrees_to_q(cfg,a),cfg['tool'])
 moved=r.tcp(motor_degrees_to_q(cfg,b),cfg['tool'])
 assert moved[2,3]<initial[2,3]

def test_base_left_and_roll_clockwise_from_behind(cfg):
 r=Robot(ROOT,'stock');a=motor_degrees_to_q(cfg,[180]*6);p,_=r.fk(a)
 b=motor_degrees_to_q(cfg,[181,180,180,180,180,180]);m,_=r.fk(b)
 assert m['link5'][1,3]>p['link5'][1,3]  # +Y is left looking along +X.
 roll=motor_degrees_to_q(cfg,[180]*4+[181,180]);t=r.tcp(roll,cfg['tool'])
 assert t[1,2]<0 and t[2,2]>0  # Top (+Z) rotates toward screen right (-Y).

def test_gripper_opening_increases_jaw_separation(cfg):
 r=Robot(ROOT,'stock');neutral=motor_degrees_to_q(cfg,[180]*6);opened=motor_degrees_to_q(cfg,[180]*5+[185])
 A,_=r.fk(neutral);B,_=r.fk(opened)
 # Matching points toward the jaw tips in each driven/idler local frame.
 def width(poses):
  left=poses['link6']@np.array([.065,0,0,1]);right=poses['link7']@np.array([.065,0,0,1])
  return left[1]-right[1]
 assert width(B)>width(A)
 poses=mock_pose_set(cfg)
 assert poses['gripper_open']['raw'][5]>2048
 assert poses['gripper_closed']['raw'][5]<2048
 assert poses['gripper_closed']['source']=='mock'

@pytest.mark.parametrize('bad_ids',[[11,12,13,1,2,2],[11,12,13,1,2,254],[11,12,13,1,2,-1],[11,12,13,1,2,True]])
def test_duplicate_reserved_or_invalid_ids_rejected(cfg,bad_ids):
 for j,i in zip(cfg['joints'],bad_ids):j['id']=i
 with pytest.raises(Fault,match='Motor IDs'):validate_config(cfg)

def test_feedback_order_is_checked(cfg):
 b=MockBus(cfg);ctrl=Controller(b,cfg);s=b.snapshot();s[3],s[4]=s[4],s[3]
 with pytest.raises(Fault,match='IDs/order'):ctrl.health(s)
 assert [s['voltage'] for s in b.snapshot()]==[12,12,12,5,5,5]

def test_sdk_groups_use_configured_ids_without_renumbering_hardware(cfg,monkeypatch):
 import dynamixel_sdk as sdk
 class Port:
  def __init__(self,*args):pass
  def openPort(self):return True
  def setBaudRate(self,*args):return True
  def closePort(self):pass
 class Writer:
  def __init__(self,*args):self.sent=[];self.pending=[]
  def clearParam(self):self.pending=[]
  def addParam(self,i,data):self.pending.append((i,data));return True
  def txPacket(self):self.sent.append(self.pending.copy());return 0
 class Reader:
  def __init__(self,*args):self.ids=[]
  def addParam(self,i):self.ids.append(i);return True
 monkeypatch.setattr(sdk,'PortHandler',Port);monkeypatch.setattr(sdk,'PacketHandler',lambda *a:object())
 monkeypatch.setattr(sdk,'GroupSyncWrite',Writer);monkeypatch.setattr(sdk,'GroupSyncRead',Reader)
 b=DynamixelBus(cfg,'COM_TEST')
 assert b.writer.sent==[]
 assert all(r.ids==[11,12,13,1,2,3] for r in b.readers.values())
 b.goals([2048]*6)
 assert [i for i,data in b.writer.sent[0]]==[11,12,13,1,2,3]

def test_changing_jaw_variant_clears_tool_verification():
 for variant,z in [('stock',0),('v',-.018),('slot',-.018)]:
  tool=nominal_tool(variant)
  assert tool['offset_m'][2]==z and tool['verified'] is False
