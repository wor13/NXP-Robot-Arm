"""Official Protocol 2.0 transport and a separate simulation transport."""
import threading,time
from .core import Fault,validate_raw

def signed(x,bits=32):return x-(1<<bits) if x&(1<<(bits-1)) else x

def le32(x):return list((int(x)&0xffffffff).to_bytes(4,'little'))

class MockBus:
 is_mock=True
 def __init__(self,c):
  self.c=c;self.raw=[j['raw_reference'] or 0 for j in c['joints']];self.enabled=True;self.writes=[]
 def snapshot(self):return [dict(id=j['id'],model=j['model'],raw=r,torque=self.enabled,error=0,watchdog=0,voltage=12 if j['model']==1060 else 5,temperature=25,effort_raw=0,velocity_raw=0) for j,r in zip(self.c['joints'],self.raw)]
 def info(self):return [dict(id=j['id'],model=j['model'],mode=j['mode'],drive=0,profile_velocity=10,profile_acceleration=5) for j in self.c['joints']]
 def prepare_hold(self,c):validate_raw(c,self.raw);self.enabled=True
 def goals(self,raw):self.writes.append(list(raw));self.raw=list(raw)
 def close(self):pass

class DynamixelBus:
 is_mock=False
 def __init__(self,c,port=None):
  from dynamixel_sdk import PortHandler,PacketHandler,GroupSyncWrite,GroupSyncRead,COMM_SUCCESS
  self.sdk_ok=COMM_SUCCESS;self.c=c;self.ids=[j['id'] for j in c['joints']];self.lock=threading.RLock();self.watchdog_owned=set()
  self.port=PortHandler(port or c['serial']['port']);self.packet=PacketHandler(2.0)
  if not self.port.openPort():raise Fault('Cannot open COM port. Close Wizard, Serial Monitor and previous controllers.')
  if not self.port.setBaudRate(c['serial']['baud']):self.port.closePort();raise Fault('Cannot set baud rate.')
  self.writer=GroupSyncWrite(self.port,self.packet,116,4)
  self.readers={}
  for address,length in [(120,27),(64,7),(98,1)]:
   reader=GroupSyncRead(self.port,self.packet,address,length)
   for i in self.ids:
    if not reader.addParam(i):raise Fault('Cannot build read group.')
   self.readers[address]=reader
  # Constructor opens the serial connection only. No register writes.
 def ok(self,result,error=0):
  if result!=self.sdk_ok:raise Fault(self.packet.getTxRxResult(result))
  if error:raise Fault(self.packet.getRxPacketError(error))
 def read_reg(self,i,address,length):
  with self.lock:
   value,result,error=getattr(self.packet,f'read{length}ByteTxRx')(self.port,i,address);self.ok(result,error);return value
 def write_reg(self,i,address,length,value):
  with self.lock:
   result,error=getattr(self.packet,f'write{length}ByteTxRx')(self.port,i,address,value);self.ok(result,error)
 def info(self):
  with self.lock:
   rows=[]
   for j in self.c['joints']:
    i=j['id'];model,result,error=self.packet.ping(self.port,i);self.ok(result,error)
    rows.append(dict(id=i,model=model,mode=self.read_reg(i,11,1),drive=self.read_reg(i,10,1),profile_acceleration=self.read_reg(i,108,4),profile_velocity=self.read_reg(i,112,4)))
   return rows
 def snapshot(self):
  with self.lock:
   for r in self.readers.values():self.ok(r.txRxPacket())
   out=[]
   for j in self.c['joints']:
    i=j['id']
    def get(group,address,length):
     r=self.readers[group]
     if not r.isAvailable(i,address,length):raise Fault(f'Incomplete feedback from ID {i}.')
     return r.getData(i,address,length)
    out.append(dict(id=i,model=j['model'],raw=signed(get(120,132,4)),torque=bool(get(64,64,1)),error=get(64,70,1),watchdog=signed(get(98,98,1),8),voltage=get(120,144,2)/10,temperature=get(120,146,1),effort_raw=signed(get(120,126,2),16),velocity_raw=signed(get(120,128,4))))
   return out
 def prepare_hold(self,c):
  with self.lock:
   infos=self.info();state=self.snapshot()
   for j,i,s in zip(c['joints'],infos,state):
    if i['model']!=j['model']:raise Fault(f"ID {j['id']}: model {i['model']} does not match configured {j['model']}.")
    if i['mode']!=j['mode'] or i['mode'] not in [3,4]:raise Fault('Operating mode differs from calibrated config. Configure intentionally in Wizard, then recalibrate.')
    if i['drive']&4:raise Fault('Time-based profile detected. This version requires velocity-based profiles; configure and recalibrate separately.')
    if s['error'] or s['watchdog']<0:raise Fault('Motor error/watchdog fault. Inspect and recover before arming.')
    if not s['torque'] and i['mode']==3:raise Fault('Mode 3 torque-on can reset encoder coordinates. Establish supported holding torque in Wizard, re-read and recalibrate first, or use mode 4 deliberately.')
   before=[x['raw'] for x in state];validate_raw(c,before)
   # Preload every goal before enabling any torque. Do not alter modes or EEPROM.
   self.goals(before)
   for j,s in zip(c['joints'],state):
    i=j['id'];self.write_reg(i,108,4,c['motion']['profile_acceleration']);self.write_reg(i,112,4,c['motion']['profile_velocity'])
    self.write_reg(i,98,1,c['motion']['watchdog_units'])
    self.watchdog_owned.add(i)
    if not s['torque']:self.write_reg(i,64,1,1)
   after=self.snapshot()
   if max(abs(a['raw']-p) for a,p in zip(after,before))>c['motion']['arrival_ticks']:raise Fault('Encoder coordinates changed on torque enable. Stop and recalibrate; do not replay old poses.')
 def goals(self,raw):
  with self.lock:
   validate_raw(self.c,raw);self.writer.clearParam()
   try:
    for i,r in zip(self.ids,raw):
     if not self.writer.addParam(i,le32(r)):raise Fault('Cannot add synchronized goal.')
    self.ok(self.writer.txPacket())
   finally:self.writer.clearParam()
 def close(self):
  # End a healthy owned session with a fresh hold before retiring its watchdog.
  # Read-only sessions write nothing. Never silently clear an existing fault.
  with self.lock:
   try:
    owned=getattr(self,'watchdog_owned',set())
    if owned:
     state=self.snapshot()
     if any(x['error'] or x['watchdog']<0 or not x['torque'] for x in state):
      print('[CLOSE WARNING] Motor/watchdog fault or torque-off remains latched; inspect before recovery.',flush=True)
     else:
      self.goals([x['raw'] for x in state])
      for i in sorted(owned):self.write_reg(i,98,1,0)
   except Exception as exc:print(f'[CLOSE WARNING] Could not complete hold/watchdog cleanup: {exc}',flush=True)
   finally:self.port.closePort()
