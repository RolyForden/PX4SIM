"""REFERENCE geometry bench, NOT final aircraft. No PX4, motors, gravity or flight."""
import json,math,pathlib,xml.etree.ElementTree as E
import numpy as np
from kinematics import NAMES,SIGNS,HINGES,BASE

ROOT=pathlib.Path(__file__).resolve().parent
CONTAINER_ROOT='/work/morphing_prototype'
def add(parent,tag,text=None,**attrs):
    e=E.SubElement(parent,tag,attrs)
    if text is not None:e.text=str(text)
    return e
def vec(a):return ' '.join(f'{float(v):.12g}' for v in a)
def flu(a):return np.asarray(a)*[1,-1,-1]
def link(model,name,position,mass):
    l=add(model,'link',name=name);add(l,'pose',vec([*position,0,0,0]))
    inertial=add(l,'inertial');add(inertial,'mass',mass);tensor=add(inertial,'inertia')
    for key,value in [('ixx',.001),('iyy',.001),('izz',.001),('ixy',0),('ixz',0),('iyz',0)]:add(tensor,key,value)
    return l
def visual(l,name,mesh,offset,color):
    v=add(l,'visual',name=name);add(v,'pose',vec([*offset,0,0,0]));g=add(v,'geometry');m=add(g,'mesh');add(m,'uri',CONTAINER_ROOT+'/'+mesh)
    mat=add(v,'material');add(mat,'ambient',color);add(mat,'diffuse',color)
def joint(model,name,parent,child,axis=None):
    j=add(model,'joint',name=name,type='revolute' if axis is not None else 'fixed');add(j,'parent',parent);add(j,'child',child)
    if axis is not None:
        a=add(j,'axis');add(a,'xyz',vec(axis));limit=add(a,'limit');add(limit,'lower',-.6);add(limit,'upper',.6);add(limit,'effort',10);add(limit,'velocity',.8)
        plugin=add(model,'plugin',filename='gz-sim-joint-position-controller-system',name='gz::sim::systems::JointPositionController')
        add(plugin,'joint_name',name);add(plugin,'use_velocity_commands','true');add(plugin,'cmd_max',.35);add(plugin,'initial_position',0)
        add(plugin,'topic','/morph_bench/'+name+'/cmd_pos')

cad=json.loads((ROOT/'cad_inspection.json').read_text())
sdf=E.Element('sdf',version='1.9');world=add(sdf,'world',name='morph_bench');add(world,'gravity','0 0 0')
physics=add(world,'physics',name='bench',type='ignored');add(physics,'max_step_size',.002);add(physics,'real_time_factor',1)
for filename,name in [('gz-sim-physics-system','Physics'),('gz-sim-user-commands-system','UserCommands'),('gz-sim-scene-broadcaster-system','SceneBroadcaster')]:
    add(world,'plugin',filename=filename,name='gz::sim::systems::'+name)
light=add(world,'light',name='sun',type='directional');add(light,'pose','0 0 5 0 0 0');add(light,'direction','-0.5 0.2 -1');add(light,'diffuse','0.9 0.9 0.9 1')
scene=add(world,'scene');add(scene,'ambient','0.6 0.6 0.6 1');add(scene,'background','0.85 0.88 0.92 1');add(scene,'grid','true')
floor=add(world,'model',name='display_floor');add(floor,'static','true');fl=add(floor,'link',name='surface');add(fl,'pose','0 0 -0.10 0 0 0');v=add(fl,'visual',name='floor');g=add(v,'geometry');plane=add(g,'plane');add(plane,'normal','0 0 1');add(plane,'size','3 3');mat=add(v,'material');add(mat,'ambient','0.7 0.73 0.76 1');add(mat,'diffuse','0.7 0.73 0.76 1')
model=add(world,'model',name='REFERENCE_ONLY_NOT_FINAL');add(model,'self_collide','false')
base=link(model,'base_link',[0,0,0],1.0);visual(base,'central_cad','meshes/body_central.stl',[0,0,0],'0.20 0.35 0.48 1')
joint(model,'bench_fixed','world','base_link')
for name,(sx,sy),hinge,origin in zip(NAMES,SIGNS,HINGES,BASE):
    h=flu(hinge);y=link(model,'yaw_frame_'+name,h,.01);arm=link(model,'arm_'+name,h,.1)
    joint(model,'yaw_'+name,'base_link','yaw_frame_'+name,[0,0,1])
    radial=(origin-hinge).copy();radial[2]=0;radial/=np.linalg.norm(radial);axis=flu(np.cross(radial,[0,0,-1.]))
    joint(model,'pitch_'+name,'yaw_frame_'+name,'arm_'+name,axis)
    visual(arm,'cad_arm_'+name,'meshes/arm_'+name+'.stl',-h,'0.35 0.57 0.70 1')
    for part in cad['solids']:
        if part['classification']=='simplified_body':continue
        centroid=part['cad_uniform_volume_centroid_mm'];frd=np.array([-centroid[2],centroid[0],-centroid[1]])/1000
        if int(np.argmin(np.linalg.norm(BASE-frd,axis=1)))==NAMES.index(name):
            visual(arm,'part_'+str(part['part_index']),part['mesh'],-h,'0.90 0.44 0.14 1' if part['classification']=='motor' else '0.16 0.19 0.23 1')
    # Virtual rotor reference link; its pose is measured to validate analytic kinematics.
    rotor=link(model,'rotor_'+name,flu(origin),.001);joint(model,'rotor_fixed_'+name,'arm_'+name,'rotor_'+name)
plugin=add(model,'plugin',filename='gz-sim-joint-state-publisher-system',name='gz::sim::systems::JointStatePublisher');add(plugin,'topic','/morph_bench/joint_states')
E.indent(sdf);E.ElementTree(sdf).write(ROOT/'bench.sdf',encoding='utf-8',xml_declaration=True)
assumptions={'status':'REFERENCE ONLY, NOT final aircraft; geometry/toolchain demonstrator','px4_connected':False,'gravity_m_s2':[0,0,0],'bench_fixed':True,
             'collisions':'disabled; no contact/propeller-clearance validation','virtual_mass_and_inertia':'numerical placeholders for joint integration only, not measured aircraft properties',
             'virtual_hinges':'FRD (+/-45,+/-45,0) mm, assumed segmentation planes; actual pivots not identified',
             'drive_model':'one demonstration input with mutually exclusive H and V branches; mirrored 25 degree endpoints are illustrative',
             'joint_controller':'ideal position via velocity commands, not calibrated actuator or elastic/rope dynamics',
             'geometry':'original STEP partitioned for visualization; X pose conserves original central-body volume'}
(ROOT/'bench_assumptions.json').write_text(json.dumps(assumptions,indent=2));print('Built',ROOT/'bench.sdf')
