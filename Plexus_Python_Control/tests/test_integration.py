from pathlib import Path
import copy,json,shutil,subprocess,threading
import numpy as np
import pytest
from plexus.core import *
from plexus.bus import MockBus,DynamixelBus
from plexus.model import Robot
from plexus.ik import plan_vertical,execute_vertical
from plexus.__main__ import export_header
ROOT=Path(__file__).resolve().parents[1]
@pytest.fixture
def c():return json.loads((ROOT/'config/demo.json').read_text())

def test_vertical_execution_reaches_preplanned_goal(c):
 r=Robot(ROOT,'v');p=plan_vertical(r,c,[2048]*6,.5)
 b=MockBus(c);ctrl=Controller(b,c);ctrl.arm();execute_vertical(ctrl,r,p)
 assert b.raw==p['frames'][-1]['raw'] and ctrl.armed
 assert len(b.writes)==len(p['frames'])

def test_sequence_preplans_unreachable_lift_before_grip(c):
 b=MockBus(c);ctrl=Controller(b,c);ctrl.arm()
 poses={'entry':{'raw':[2048]*6},'closed':{'raw':[2048]*5+[2036]}}
 seq={'steps':[{'kind':'arm','pose':'entry'},{'kind':'grip','pose':'closed'},{'kind':'vertical','pose':'entry','travel_mm':150}]}
 with pytest.raises(Fault):ctrl.run(poses,seq)
 assert all(x==[2048]*6 for x in b.writes)  # Fresh-position fault hold only.

@pytest.mark.parametrize('change',[{'model':999},{'mode':3},{'drive':4}])
def test_sdk_arming_preflight_rejects_mismatch_before_register_writes(c,change):
 b=DynamixelBus.__new__(DynamixelBus);b.lock=threading.RLock();calls=[]
 info=MockBus(c).info();info[0].update(change)
 b.info=lambda:info;b.snapshot=lambda:MockBus(c).snapshot()
 b.goals=lambda raw:calls.append(('goal',raw));b.write_reg=lambda *args:calls.append(args)
 with pytest.raises(Fault):b.prepare_hold(c)
 assert calls==[]

def test_ported_c_and_cpp_compile_and_match_python(c,tmp_path):
 cc=shutil.which('cc');cxx=shutil.which('g++')
 if not cc or not cxx:pytest.skip('C/C++ toolchain unavailable')
 c['tool']['verified']=True
 poses={name:{'raw':raw,'source':'hardware','calibration':fingerprint(c)} for name,raw in {'entry':[2048]*6,'closed':[2048]*5+[2040]}.items()}
 seq={'steps':[{'kind':'arm','pose':'entry'},{'kind':'grip','pose':'closed'},{'kind':'vertical','pose':'entry','travel_mm':.5}]}
 seq['approved_digest']=sequence_digest(c,poses,seq)
 header=tmp_path/'generated.h';export_header(c,poses,seq,header)
 shutil.copy(ROOT/'porting/motion_core.h',tmp_path)
 source=tmp_path/'check.c';source.write_text('''#include "generated.h"
#include <stdio.h>
int main(void) {
 if(plexus_ids[0]!=11 || plexus_ids[3]!=1 || plexus_ids[4]!=2 || plexus_ids[5]!=3) return 3;
 if(plexus_frame_count<3 || !plexus_in_limits(&plexus_entry,plexus_min,plexus_max)) return 1;
 for(unsigned k=0;k<plexus_frame_count;k++) if(!plexus_in_limits(&plexus_frames[k].pose,plexus_min,plexus_max) || !plexus_frames[k].dt_us) return 2;
 PlexusPose a={{2048,2048,2048,2048,2048,2048}},b={{2500,1600,2100,2048,1800,2048}};
 for(unsigned t=0;t<=1000;t+=100) {PlexusPose p=plexus_interpolate(a,b,t,1000);printf("%d %d\\n",p.raw[0],p.raw[1]);}
 return 0;
}''')
 for compiler,standard in [(cc,'c11'),(cxx,'c++17')]:
  exe=tmp_path/('check_'+standard);subprocess.run([compiler,'-x','c++' if compiler==cxx else 'c','-std='+standard,str(source),'-lm','-o',str(exe)],check=True,capture_output=True)
  output=subprocess.run([str(exe)],check=True,capture_output=True,text=True).stdout
  expected=[]
  for k in range(11):
   u=k/10;s=10*u**3-15*u**4+6*u**5;expected.append([round(2048+452*s),round(2048-448*s)])
  assert [list(map(int,row.split())) for row in output.splitlines()]==expected
