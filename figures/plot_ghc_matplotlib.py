#!/usr/bin/env python3
"""Matplotlib renderer for the GHC figures in report.tex."""
from pathlib import Path
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
T=[1,2,4,8]; names=['Few','Medium','Abundant']; colors=['#0072B2','#E69F00','#009E73']
C={'W':[[2.667520,1.311034,.669460,.346239],[9.301823,4.609844,2.395916,1.291374],[24.632119,12.329673,6.694831,4.143523]],'A':[[2.601065,1.300434,.673795,.353749],[9.725216,4.864542,2.526144,1.412920],[27.400904,13.722991,7.340459,4.284780]]}
I={'W':[[.007929,.008073,.008049,.008143],[.008686,.008523,.008427,.008660],[.011422,.011351,.011294,.011451]],'A':[[.008108,.008074,.008052,.008250],[.008520,.008413,.008463,.008702],[.011340,.011421,.011231,.011393]]}
M={'W':[[7.563553,7.592490,7.882694,7.697944],[27.484058,34.506264,35.451085,33.354328],[114.047885,145.787835,144.966270,135.904872]],'A':[[7.497534,7.509264,7.533564,7.745821],[28.873098,28.763354,29.245710,29.421326],[120.338522,117.969738,118.923504,117.885518]]}
def save(fig,n): fig.tight_layout(); fig.savefig(OUT/(n+'.pdf')); fig.savefig(OUT/(n+'.png'),dpi=220); plt.close(fig)
fig,axs=plt.subplots(2,2,figsize=(9,6.7))
for ax,(mode,total) in zip(axs.flat,[('W',0),('A',0),('W',1),('A',1)]):
 for n,c,v,i in zip(names,colors,C[mode],I[mode]):
  x=[a+b for a,b in zip(v,i)] if total else v; ax.plot(T,[x[0]/z for z in x],'o-',label=n,color=c)
 ax.plot(T,T,'k--',label='Ideal'); ax.set(xscale='log',xticks=T,xticklabels=T,ylim=(0,8.5),xlabel='Threads',ylabel='Speedup',title=f"{'Across' if mode=='A' else 'Within'}-wires: {'total' if total else 'computation'}"); ax.grid(alpha=.25)
axs[0,0].legend(); save(fig,'ghc_speedups')
fig,axs=plt.subplots(2,2,figsize=(9,6.7))
for ax,(mode,per) in zip(axs.flat,[('W',0),('A',0),('W',1),('A',1)]):
 for n,c,v in zip(names,colors,M[mode]): ax.plot(T,[z/t if per else z for z,t in zip(v,T)],'o-',label=n,color=c)
 ax.set(xscale='log',xticks=T,xticklabels=T,xlabel='Threads',ylabel='Cache misses (millions)',title=f"{'Across' if mode=='A' else 'Within'}-wires: {'mean per worker' if per else 'total misses'}"); ax.grid(alpha=.25)
axs[0,0].legend(); save(fig,'ghc_cache_misses')
fig,axs=plt.subplots(1,3,figsize=(10,3.5))
for ax,x,y,title,xlabel in zip(axs,[[.01,.1,.5],[512,2048,8192],[256,1024,4096]],[[7.394,7.398,7.387],[7.199,6.390,7.362],[7.392,7.272,6.382]],['Random-route probability','Grid-size sweep','Wire-count sweep'],['P','Grid dimension','Number of wires']):
 ax.plot(x,y,'o-',color='black'); ax.axhline(8,ls='--',color='gray'); ax.set(ylim=(0,8.5),title=title,xlabel=xlabel,ylabel='8-thread computation speedup')
save(fig,'ghc_sensitivity')

fig,ax=plt.subplots(figsize=(4.6,3.4))
ax.plot([.01,.1,.5],[7.394,7.398,7.387],'o-',color='black')
ax.axhline(8,ls='--',color='gray',label='Ideal')
ax.set(ylim=(0,8.5),xlabel='P',ylabel='8-thread computation speedup',title='Random-route probability')
ax.legend()
save(fig,'ghc_sensitivity_probability')

fig,axs=plt.subplots(1,2,figsize=(7.8,3.4))
for ax,x,y,title,xlabel in zip(
    axs,
    [[512,2048,8192],[256,1024,4096]],
    [[7.199,6.390,7.362],[7.392,7.272,6.382]],
    ['Grid-size sweep','Wire-count sweep'],
    ['Grid dimension','Number of wires'],
):
    ax.plot(x,y,'o-',color='black')
    ax.axhline(8,ls='--',color='gray',label='Ideal')
    ax.set(ylim=(0,8.5),title=title,xlabel=xlabel,ylabel='8-thread computation speedup')
axs[0].legend()
save(fig,'ghc_sensitivity_problem_size')
