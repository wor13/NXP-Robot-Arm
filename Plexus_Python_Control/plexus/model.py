"""Small URDF FK/mesh loader. No ROS, physics engine, or URDF runtime dependency."""
import math,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from .core import NAMES,Fault

def rotation(axis,a):
 axis=np.asarray(axis,dtype=float);axis/=np.linalg.norm(axis);x,y,z=axis;c=math.cos(a);s=math.sin(a);C=1-c
 return np.array([[c+x*x*C,x*y*C-z*s,x*z*C+y*s],[y*x*C+z*s,c+y*y*C,y*z*C-x*s],[z*x*C-y*s,z*y*C+x*s,c+z*z*C]])

def origin(e):
 T=np.eye(4)
 if e is not None:
  T[:3,3]=np.fromstring(e.get('xyz','0 0 0'),sep=' ')
  roll,pitch,yaw=np.fromstring(e.get('rpy','0 0 0'),sep=' ')
  T[:3,:3]=rotation([0,0,1],yaw)@rotation([0,1,0],pitch)@rotation([1,0,0],roll)
 return T

def wxyz(R):
 # Stable eigenvector formulation of matrix-to-quaternion conversion.
 from viser.transforms import SO3
 return SO3.from_matrix(R).wxyz

class Robot:
 def __init__(self,root,variant='v'):
  self.root=Path(root);path=self.root/'models/open_manipulator_description/urdf/omx_f/omx_f.urdf'
  self.xml=ET.parse(path).getroot();self.joints=[];self.visuals=[]
  for e in self.xml.findall('joint'):
   self.joints.append(dict(name=e.get('name'),type=e.get('type'),parent=e.find('parent').get('link'),child=e.find('child').get('link'),T=origin(e.find('origin')),axis=np.fromstring(e.find('axis').get('xyz'),sep=' ') if e.find('axis') is not None else np.array([0.,0.,1.]),mimic=e.find('mimic')))
  for link in self.xml.findall('link'):
   for k,v in enumerate(link.findall('visual')):
    mesh=v.find('geometry/mesh')
    if mesh is None:continue
    filename=mesh.get('filename').replace('package://','')
    file=self.root/'models'/filename
    if variant!='stock' and link.get('name') in ['link6','link7']:
     file=self.root/f'models/grippers/{variant}_{link.get("name")}.stl';scale=[1,1,1]
    else:scale=np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
    if not file.exists():raise Fault(f'Missing bundled mesh: {file}')
    self.visuals.append((link.get('name'),k,file,scale,origin(v.find('origin'))))
 def fk(self,q):
  angles=dict(zip(NAMES,q));poses={'world':np.eye(4)};before={};pending=list(self.joints)
  while pending:
   progressed=False
   for j in pending[:]:
    if j['parent'] not in poses:continue
    B=poses[j['parent']]@j['T'];before[j['name']]=B
    a=angles.get(j['name'],0)
    if j['mimic'] is not None:
     m=j['mimic'];a=angles[m.get('joint')]*float(m.get('multiplier','1'))+float(m.get('offset','0'))
    M=np.eye(4)
    if j['type']=='revolute':M[:3,:3]=rotation(j['axis'],a)
    elif j['type']!='fixed':raise Fault('Unsupported joint type in bundled URDF.')
    poses[j['child']]=B@M;pending.remove(j);progressed=True
   if not progressed:raise Fault('URDF contains a disconnected/cyclic joint tree.')
  return poses,before
 def tcp(self,q,tool):
  poses,_=self.fk(q);T=poses['link5'].copy();T[:3,3]=(T@np.array([*tool['offset_m'],1]))[:3];return T

class Renderer:
 def __init__(self,server,robot,prefix,color,opacity):
  import trimesh
  self.robot=robot;self.frames={}
  for link,k,file,scale,T in robot.visuals:
   if link not in self.frames:self.frames[link]=server.scene.add_frame(f'{prefix}/{link}',show_axes=False)
   mesh=trimesh.load(file,force='mesh',process=False);mesh.apply_scale(scale);mesh.apply_transform(T)
   server.scene.add_mesh_simple(f'{prefix}/{link}/mesh{k}',vertices=np.asarray(mesh.vertices,dtype=np.float32),faces=np.asarray(mesh.faces,dtype=np.uint32),color=color,opacity=opacity)
  self.tcp=server.scene.add_frame(f'{prefix}/nominal_tcp',axes_length=.035,axes_radius=.0015)
 def visible(self,value):
  for handle in self.frames.values():handle.visible=value
  self.tcp.visible=value
 def update(self,q,tool):
  poses,_=self.robot.fk(q)
  for link,handle in self.frames.items():handle.position=poses[link][:3,3];handle.wxyz=wxyz(poses[link][:3,:3])
  T=self.robot.tcp(q,tool);self.tcp.position=T[:3,3];self.tcp.wxyz=wxyz(T[:3,:3])
