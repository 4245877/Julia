"""Stage7h independent actual forces/CoP and complete native substep quadrature."""
from pathlib import Path
from collections import defaultdict
import argparse,json,math,xml.etree.ElementTree as ET
from analyze_stage7f import rows,v,vec,add,sub,mul,cross,norm

p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
support=rows(a.run/'support-v1.csv');points=defaultdict(list)
for r in rows(a.run/'support-points-v1.csv'):points[int(r['tick'])].append(r)
forces={};moments={};maxforce=maxmoment=maxcop=maxcone=0
for r in support:
    tick=int(r['tick']);g=int(r['geom']);floor=int(r['surface']);origin=v(r,'ox_m oy_m oz_m');F=M=(0.,0.,0.)
    for c in points[tick]:
        if {int(c['geom1']),int(c['geom2'])}!={g,floor} or c['active']!='1' or float(c['fn_N'])<=1e-9:continue
        sign=1 if int(c['geom2'])==g else -1
        f=mul(v(c,'f2x_N f2y_N f2z_N'),sign);m=mul(v(c,'t2x_Nm t2y_Nm t2z_Nm'),sign)
        F=add(F,f);M=add(M,add(cross(sub(v(c,'x_m y_m z_m'),origin),f),m))
        maxcone=max(maxcone,math.hypot(float(c['ft1_N'])/float(c['mu1']),float(c['ft2_N'])/float(c['mu2']))-float(c['fn_N']))
    maxforce=max(maxforce,norm(sub(F,v(r,'fx_N fy_N fz_N'))));maxmoment=max(maxmoment,norm(sub(M,v(r,'mx_Nm my_Nm mz_Nm'))))
    floorMoment=add(M,cross(origin,F))
    if r['cop_reason']=='available':
        cop=(floorMoment[2]/F[1],0.,-floorMoment[0]/F[1]);maxcop=max(maxcop,norm(sub(cop,v(r,'cop_x_m cop_y_m cop_z_m'))))
    forces[tick]=add(forces.get(tick,(0.,0.,0.)),F);moments[tick]=add(moments.get(tick,(0.,0.,0.)),floorMoment)
for r in support:
    tick=int(r['tick'])
    if r['combined_cop_reason']=='available':
        f=forces[tick];m=moments[tick];cop=(m[2]/f[1],0.,-m[0]/f[1]);maxcop=max(maxcop,norm(sub(cop,v(r,'combined_x_m combined_y_m combined_z_m'))))
assert max(maxforce,maxmoment,maxcop)<1e-10 and maxcone<1e-7
dynamics={int(x['tick']):x for x in rows(a.run/'dynamics-v1.csv')}
xml=ET.parse(a.run/'model.xml').getroot();h=float(xml.find('option').attrib['timestep']);steps=round(1/120/h)
samples=rows(a.run/'dynamics-substeps-v1.csv');assert len(samples)==(len(dynamics)-1)*steps+1
p0=vec(samples[0],'p');l0=vec(samples[0],'L');linear=angular=(0.,0.,0.);maxintegral=maxresidual=0
for i,r in enumerate(samples[1:],1):
    dt=float(r['dt_s']);assert abs(dt-h)<1e-15
    linear=add(linear,mul(add(vec(r,'force_before'),vec(r,'force')),.5*dt));angular=add(angular,mul(add(vec(r,'moment_before'),vec(r,'moment')),.5*dt))
    if i%steps==0:
        d=dynamics[int(r['tick'])]
        maxintegral=max(maxintegral,norm(sub(linear,vec(d,'impulse'))),norm(sub(angular,vec(d,'angular_impulse'))))
        maxresidual=max(maxresidual,norm(sub(sub(sub(vec(r,'p'),p0),linear),vec(d,'residual_p'))),norm(sub(sub(sub(vec(r,'L'),l0),angular),vec(d,'residual_L'))))
assert max(maxintegral,maxresidual)<1e-10
xml=ET.parse(a.run/'model.xml').getroot();assert xml.find('equality') is None
assert len(xml.findall('.//freejoint'))==1 and len(xml.find('actuator'))==41
assert all(x.attrib['joint']!='root' for x in xml.find('actuator'))
assert 'damping' not in xml.find('.//freejoint').attrib
result=dict(force_error_N=maxforce,moment_error_Nm=maxmoment,cop_error_m=maxcop,actual_friction_cone_violation_N=maxcone,
            physical_substeps=len(samples)-1,integral_error=maxintegral,residual_reconstruction_error=maxresidual,
            max_linear_quadrature_residual_Ns=max(norm(vec(x,'residual_p')) for x in dynamics.values()),
            max_angular_quadrature_residual_Nms=max(norm(vec(x,'residual_L')) for x in dynamics.values()),
            free_root=True,root_actuators=0,equality_constraints=0)
(a.output/'audit.json').write_text(json.dumps(result,indent=2));(a.output/'audit.txt').write_text('PASS independent actual contacts, CoP, friction, quadrature, free root and absence of hidden constraints\n');print(json.dumps(result,indent=2))
