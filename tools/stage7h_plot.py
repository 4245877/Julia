from pathlib import Path
import argparse,csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def read(p):
    with p.open(newline='') as f:return list(csv.DictReader(f))
def a(rs,p):return np.array([[float(r[p+x]) for x in ('x','y','z')] for r in rs])
p=argparse.ArgumentParser();p.add_argument('--run',required=True,type=Path);p.add_argument('--output',required=True,type=Path);p.add_argument('--compare',type=Path);args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
c=read(args.run/'com-v1.csv');s=read(args.run/'stand-v1.csv');t=np.array([float(r['time_s']) for r in c]);origin=a(c,'actual_')[0]
rotation=np.array([[float(s[0][f'goal_q{x}']) for x in ('w','x','y','z')]])[0]
w,x,y,z=rotation;R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
series={name:(a(c,name+'_')-origin)@R*1000 for name in ('command','reference','actual')}
plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.25})
fig,axes=plt.subplots(4,1,figsize=(12,12),sharex=True)
for axis,index in ((axes[0],0),(axes[1],2)):
    for name,color,style in (('command','#b6339b','--'),('reference','#d78212','-'),('actual','#197d8a','-')):axis.plot(t,series[name][:,index],style,c=color,label=name)
    axis.set_ylabel(('X' if index==0 else 'Z')+' COM [mm]');axis.legend(ncol=3)
axes[2].plot(t,[float(r['requested_utilization'])*100 for r in s],label='requested torque / cap [%]');axes[2].axhline(100,c='r',ls='--');axes[2].set_ylabel('cap use [%]');axes[2].legend()
for field,color in (('active_caps','#bc3737'),('active_friction','#b98218'),('active_normals','#416bb0')):axes[3].step(t,[int(r[field]) for r in c],where='post',label=field,c=color)
axes[3].set_ylabel('active inequalities');axes[3].set_xlabel('physical time [s]');axes[3].legend(ncol=3)
fig.tight_layout();fig.savefig(args.output/'com-and-constraints.png',dpi=160);fig.savefig(args.output/'com-and-constraints.svg');plt.close(fig)
fig,ax=plt.subplots(figsize=(7,6))
for name,color in (('command','#b6339b'),('reference','#d78212'),('actual','#197d8a')):ax.plot(series[name][:,0],series[name][:,2],label=name,c=color)
ax.set(xlabel='placement X [mm]',ylabel='placement Z [mm]',title='COM top view; fixed placement frame',aspect='equal');ax.legend();fig.tight_layout();fig.savefig(args.output/'top-view.png',dpi=180);fig.savefig(args.output/'top-view.svg');plt.close(fig)
fig,axes=plt.subplots(3,2,figsize=(12,10),sharex=True)
for column,side in enumerate(('left','right')):
    for row,component in enumerate(('fy','mx','mz')):
        ax=axes[row,column];ax.plot(t,[float(r[f'{side}_pred_{component}']) for r in s],label='predicted @ bottom center');ax.plot(t,[float(r[f'{side}_measured_at_pred_{component}']) for r in c],label='measured transported to same origin',alpha=.8);ax.set_ylabel(f'{side} {component} ['+('N]' if row==0 else 'Nm]'));ax.legend(fontsize=8)
axes[-1,0].set_xlabel('time [s]');axes[-1,1].set_xlabel('time [s]');fig.tight_layout();fig.savefig(args.output/'reactions.png',dpi=160);fig.savefig(args.output/'reactions.svg');plt.close(fig)
if args.compare:
    old=read(args.compare/'com-v1.csv'); os=read(args.compare/'stand-v1.csv');ot=np.array([float(r['time_s']) for r in old]);n=min(len(old),len(c))
    fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True)
    for rs,cs,ts,label in ((s,c,t,'bounded7h'),(os,old,ot,'equality7g')):
        error=np.linalg.norm((a(cs,'actual_')-a(cs,'command_'))[:,[0,2]],axis=1)*1000
        axes[0].plot(ts,error,label=label);axes[1].plot(ts,[float(r['requested_utilization'])*100 for r in rs],label=label)
    axes[0].set_ylabel('command error [mm]');axes[0].set_ylim(0,7);axes[1].set_ylabel('requested / cap [%]');axes[1].axhline(100,c='r',ls='--');axes[1].set_xlabel('time [s]')
    for ax in axes:ax.legend()
    fig.tight_layout();fig.savefig(args.output/'comparison.png',dpi=160);fig.savefig(args.output/'comparison.svg')
