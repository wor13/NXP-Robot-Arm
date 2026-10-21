import argparse,json,sys
from pathlib import Path
from .core import *
ROOT=Path(__file__).resolve().parents[1]

def export_header(config,poses,sequence,path):
 validate_config(config,True);validate_sequence(config,poses,sequence,True)
 from .model import Robot
 from .ik import plan_vertical
 current=next(poses[s['pose']]['raw'].copy() for s in sequence['steps'] if s['kind']=='arm')
 entry=current.copy();frames=[]
 for step in sequence['steps']:
  if step['kind']=='vertical':
   if not config['tool']['verified']:raise Fault('Verify the measured tool offset first.')
   anchor=poses[step['pose']]['raw']
   if max(abs(a-b) for a,b in zip(current[:5],anchor[:5]))>config['motion']['arrival_ticks']:raise Fault('Vertical anchor mismatch.')
   plan=plan_vertical(Robot(ROOT,config['gripper_variant']),config,current,step['travel_mm'])
   rows=[(f['dt_s'],f['raw']) for f in plan['frames']]
  else:
   goal=merged_goal(step['kind'],current,poses[step['pose']]['raw']);rows=trajectory(config,current,goal,step.get('minimum_seconds',0))
  frames.extend(rows);current=rows[-1][1].copy()
  remaining=step.get('dwell_seconds',0)
  while remaining>1e-9:
   delay=min(config['motion']['period_s'],remaining);frames.append((delay,current.copy()));remaining-=delay
 lines=['// Qualified hardware-taught route; not an autonomous firmware implementation.','#pragma once','#include "motion_core.h"','static const uint8_t plexus_ids[6] = {'+','.join(str(j['id']) for j in config['joints'])+'};','static const PlexusPose plexus_entry = {{'+','.join(map(str,entry))+'}};','static const PlexusTimedPose plexus_frames[] = {']
 for delay,raw in frames:lines.append('  {{{'+','.join(map(str,raw))+'}},'+str(round(delay*1000000))+'},')
 lines+=['};','static const int32_t plexus_min[6] = {'+','.join(str(j['limits'][0]) for j in config['joints'])+'};','static const int32_t plexus_max[6] = {'+','.join(str(j['limits'][1]) for j in config['joints'])+'};','static const unsigned plexus_frame_count = sizeof(plexus_frames)/sizeof(plexus_frames[0]);']
 Path(path).parent.mkdir(parents=True,exist_ok=True);Path(path).write_text('\n'.join(lines)+'\n')

def main():
 p=argparse.ArgumentParser(description='Plexus OMX-F control. Launch defaults to no hardware connection.')
 sub=p.add_subparsers(dest='command',required=True)
 a=sub.add_parser('app');a.add_argument('--mock',action='store_true');a.add_argument('--hardware',action='store_true');a.add_argument('--manual',action='store_true',help='Encoder-based one-joint control without full IK calibration');a.add_argument('--port',default=None);a.add_argument('--web-port',type=int,default=8080);a.add_argument('--gripper',choices=['stock','v','slot'],default=None);a.add_argument('--session',default='centered_180')
 sub.add_parser('ports')
 d=sub.add_parser('diagnose');d.add_argument('--port',default=None)
 j=sub.add_parser('jog',help='Direct foreground one-joint encoder test; no browser needed')
 j.add_argument('--port',default=None);j.add_argument('--id',type=int,required=True);j.add_argument('--delta-deg',type=float,required=True);j.add_argument('--mock',action='store_true')
 e=sub.add_parser('export');e.add_argument('--session',default='centered_180');e.add_argument('--output',default=str(ROOT/'porting/generated_poses.h'))
 args=p.parse_args()
 if hasattr(args,'session') and (not args.session or any(not(ch.isalnum() or ch in '_-') for ch in args.session)):p.error('Session names use letters, digits, underscores or hyphens.')
 try:
  if args.command=='ports':
   from serial.tools import list_ports
   for port in list_ports.comports():print(port.device,port.description)
  elif args.command=='jog':
   from .bus import MockBus,DynamixelBus
   if not valid_number(args.delta_deg) or not .1<=abs(args.delta_deg)<=1.9:raise Fault('Use a relative test change of 0.1..1.9 degrees.')
   base=json.loads((ROOT/'config/hardware.json').read_text());ids=[j['id'] for j in base['joints']]
   if args.id not in ids:raise Fault('Select a configured motor ID: '+str(ids))
   bus=MockBus(base) if args.mock else DynamixelBus(base,args.port);controller=None
   try:
    state=bus.snapshot();start=[x['raw'] for x in state];c=manual_session_config(base,start);bus.c=c
    controller=Controller(bus,c,log=lambda event,data:print(f'[{event}] {json.dumps(data)}',flush=True) if event in ['FAULT','HOLD_FAILED'] else None)
    i=ids.index(args.id);goal=start.copy();goal[i]+=round(args.delta_deg*c['joints'][i]['counts_per_rev']/360)
    print(f'{"SIMULATION" if args.mock else "REAL HARDWARE"} on {args.port or c["serial"]["port"]}; ID {args.id}: before={start[i]}, requested={goal[i]} ({args.delta_deg:+.2f} degrees)',flush=True)
    controller.arm(manual_only=True);controller.move(goal,preview_anchor=start)
    after=controller.read();present=after[i]['raw']
    print(f'RESULT ID {args.id}: before={start[i]}, requested={goal[i]}, present={present}, encoder_change={present-start[i]}, torque={after[i]["torque"]}, hardware_error={after[i]["error"]}',flush=True)
   except KeyboardInterrupt:
    if controller is not None:controller.request_stop();controller.hold()
    raise Fault('Jog interrupted; attempted fresh-position hold.')
   finally:
    if controller is not None:controller.request_stop()
    bus.close()
  elif args.command=='diagnose':
   from .bus import DynamixelBus
   c=json.loads((ROOT/'config/hardware.json').read_text());bus=DynamixelBus(c,args.port)
   try:
    complete=True
    for j in c['joints']:
     try:
      i=j['id'];model,result,error=bus.packet.ping(bus.port,i);bus.ok(result,error)
      row={'id':i,'model':model,'expected_model':j['model'],'mode':bus.read_reg(i,11,1),'drive':bus.read_reg(i,10,1),'torque':bus.read_reg(i,64,1),'hardware_error':bus.read_reg(i,70,1),'raw':__import__('plexus.bus',fromlist=['signed']).signed(bus.read_reg(i,132,4))}
      row['motor_degrees']=row['raw']*360/j['counts_per_rev'];row['goal_raw']=__import__('plexus.bus',fromlist=['signed']).signed(bus.read_reg(i,116,4));print(json.dumps(row))
     except Exception as exc:complete=False;print(f"ID {j['id']}: {exc}")
    if complete:print(json.dumps(bus.snapshot(),indent=2))
    else:raise Fault('Some motors did not respond. Check firmware, selected follower COM port, power path, baud and IDs. No movement was requested.')
   finally:bus.close()
  elif args.command=='export':
   c=json.loads((ROOT/'config/hardware.json').read_text());directory=ROOT/'data/hardware'/args.session
   export_header(c,json.loads((directory/'poses.json').read_text()),json.loads((directory/'sequence.json').read_text()),args.output);print(args.output)
  else:
   if args.mock and args.hardware:p.error('Choose either --mock or --hardware.')
   if not args.hardware:args.mock=True
   from .app import App
   App(args).loop()
 except (Fault,ValueError,KeyError,FileNotFoundError) as exc:print('STOPPED:',exc,file=sys.stderr);return 1
 return 0
if __name__=='__main__':raise SystemExit(main())
