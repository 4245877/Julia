"""Optional static figures from saved C++ CSV. Not a runtime dependency."""
import argparse
import csv
from pathlib import Path
import matplotlib.pyplot as plt

p=argparse.ArgumentParser()
p.add_argument('--cpu',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args(); a.output.mkdir(parents=True,exist_ok=True)

def read(pattern):
    paths=list(a.cpu.glob(pattern+'/contact-v1.csv'))
    assert len(paths)==1,paths
    with paths[0].open() as f:return list(csv.DictReader(f))

def values(r,k):return [float(x[k]) for x in r]

fig,ax=plt.subplots(3,2,figsize=(12,10),layout='constrained')
energy,ex=plt.subplots(1,2,figsize=(12,4),layout='constrained')
for col,side in enumerate(['left','right']):
    base=f'{side}-cycle-p0-load0.000000-off0.000000-sub8-tc0.006000-delay0-feedback1'
    r=read(base); t=values(r,'time_s')
    ax[0,col].plot(t,values(r,'normal_N'),label='solver normal force',color='#126ca6')
    ax[0,col].plot(t,values(r,'reference_N'),label='force reference',color='#c03e8e',ls='--')
    ax[0,col].set(title=f'{side.title()} / 3 N → 6 N → unload → retreat',ylabel='Normal force [N]',ylim=(-.2,9))
    for enabled,color in [(1,'#0b8571'),(0,'#d37719')]:
        r=read(f'{side}-normal-disturbance-p0-*-sub8-tc0.006000-delay0-feedback{enabled}')
        ax[1,col].plot(values(r,'time_s'),[float(x['normal_N'])-float(x['reference_N']) for x in r],label='force feedback' if enabled else 'P/I force feedback off',color=color)
    ax[1,col].set(title='Unknown +1 N normal load in each hold',ylabel='Force error [N]',ylim=(-1.5,1.5))
    r=read(f'{side}-slip-p0-*-sub8-tc0.006000-delay0-feedback1'); t=values(r,'time_s')
    ax[2,col].plot(t,values(r,'slip_speed_m_s'),label='relative point slip speed',color='#ad301d')
    ax[2,col].set(title='μ = 0.05 / 4 N tangent disturbance',ylabel='Slip speed [m/s]',xlabel='Simulation time [s]')
    ex[col].plot(t,values(r,'normal_J'),label='normal contact work')
    ex[col].plot(t,values(r,'tangent_J'),label='tangent contact work')
    ex[col].set(title=f'{side.title()} / selected pair only',xlabel='Simulation time [s]',ylabel='Contact work [J]')
for axis in [*ax.flat,*ex.flat]:
    axis.grid(alpha=.2); axis.legend(fontsize=8,loc='best')
for axis in ax.flat:axis.set_xlim(0,16)
fig.savefig(a.output/'force-feedback-and-slip.png',dpi=160)
fig.savefig(a.output/'force-feedback-and-slip.svg')
energy.savefig(a.output/'contact-work.png',dpi=160)
energy.savefig(a.output/'contact-work.svg')
