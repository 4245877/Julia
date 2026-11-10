"""Standalone scientific figures from native measurements, no simulated data."""
import argparse,csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('--suite',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
def read(case,name):
    with (a.suite/case/name).open() as f:return list(csv.DictReader(f))
def col(rows,key):return np.array([float(x[key]) for x in rows])
on=read('pulse','measurements-v1.csv');off=read('feedback-off','measurements-v1.csv');control=read('pulse','stand-v1.csv')
fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True,constrained_layout=True)
t=col(on,'time')
for start,axis,color in [(5,'x','#1167b1'),(9,'z','#b34c00')]:
    baseline=col(on,'com_'+axis)[(t>=start-.5)&(t<start)].mean()
    axes[0].plot(t,1000*(col(on,'com_'+axis)-baseline),label=f'COM {axis.upper()} (mm)',color=color)
axes[1].plot(t,1000*np.hypot(col(on,'v_x'),col(on,'v_z')),color='#1167b1',label='Measured horizontal COM speed')
axes[2].plot(t,col(on,'left_fn'),label='Left measured',color='#168344');axes[2].plot(t,col(on,'right_fn'),label='Right measured',color='#1167b1')
axes[2].plot(t,col(control,'left_pred_fy'),'--',label='Left predicted',color='#ef9d21');axes[2].plot(t,col(control,'right_pred_fy'),'--',label='Right predicted',color='#965c00')
for ax in axes:
    for start in (5,9):ax.axvspan(start,start+.1,color='orange',alpha=.22)
    ax.grid(alpha=.25);ax.legend(loc='upper right',fontsize=8)
axes[0].set_ylabel('Displacement (mm)');axes[1].set_ylabel('Speed (mm/s)');axes[2].set_ylabel('Normal force (N)');axes[2].set_xlabel('Time (s)')
axes[0].set_title('Real free BodyRig: 3 N at Chest COM for 0.1 s; orange bands = applied pulses')
axes[0].set_ylim(-.1,.4);axes[0].set_xlim(4,12)
fig.savefig(a.output/'recovery.png',dpi=160);fig.savefig(a.output/'recovery.svg');plt.close(fig)
fig,ax=plt.subplots(figsize=(9,4),constrained_layout=True)
for name,rows,color in [('Balance ON',on,'#1167b1'),('Balance OFF after 4 s',off,'#b34c00')]:
    n=len(rows);error=np.hypot(col(rows,'com_x')-col(control[:n],'goal_com_x'),col(rows,'com_z')-col(control[:n],'goal_com_z'))
    ax.semilogy(col(rows,'time'),np.maximum(error*1000,1e-6),label=name,color=color)
ax.axvline(4,color='gray',linestyle='--',label='Feedback ablation');ax.axvspan(5,5.1,color='orange',alpha=.22,label='Same X pulse')
ax.set(xlim=(3,6.6),xlabel='Time (s)',ylabel='Actual COM target error (mm)',title='Identical first 480 ticks; other controller parameters unchanged')
ax.grid(alpha=.25);ax.legend();fig.savefig(a.output/'feedback-comparison.png',dpi=160);fig.savefig(a.output/'feedback-comparison.svg')
