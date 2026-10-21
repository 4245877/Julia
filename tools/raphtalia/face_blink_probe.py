"""Reproduce a REJECTED blink candidate for manual review; never saves a model.
The cheek/lash folds are known defects. This is a diagnostic, not a rig feature.
"""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
import bpy, numpy as np
from collections import Counter,defaultdict
from scipy.sparse import lil_matrix
from scipy.sparse.linalg import spsolve
from scipy.spatial import cKDTree
from reaudit import reset,render
bpy.ops.wm.open_mainfile(filepath=str(ROOT/'assets/models/Raphtalia_prepared.blend'))
arm=next(o for o in bpy.data.objects if o.type=='ARMATURE');reset(arm)
o=bpy.data.objects['body_tex_skin_0'];p=np.array([v.co[:] for v in o.data.vertices]);unique,inv=np.unique(np.round(p,4),axis=0,return_inverse=True)
counts=Counter();adj=defaultdict(set)
for poly in o.data.polygons:
 ids=inv[list(poly.vertices)]
 for a,b in zip(ids,np.roll(ids,-1)):
  if a!=b:counts[tuple(sorted((a,b)))]+=1;adj[a].add(b);adj[b].add(a)
ba=defaultdict(set)
for (a,b),n in counts.items():
 if n==1:ba[a].add(b);ba[b].add(a)
loops=[];seen=set()
for start in ba:
 if start in seen:continue
 path=[];prev=None;cur=start
 while cur not in path:
  path.append(cur);seen.add(cur);nxt=next(x for x in ba[cur] if x!=prev);prev,cur=cur,nxt
 loops.append(path)
o.shape_key_add(name='Basis')
for side,sign in [('L',1),('R',-1)]:
 ring=next(v for v in loops if unique[v].mean(0)[0]*sign>0);co=unique[ring];left=int(np.argmin(co[:,0]));ring=ring[left:]+ring[:left];right=int(np.argmax(unique[ring,0]))
 paths=[ring[:right+1],ring[right:]+ring[:1]];paths=[sorted(x,key=lambda i:unique[i,0]) for x in paths]
 paths.sort(key=lambda x:unique[x,2].mean());low,up=paths
 x=unique[ring,0];lo=np.interp(x,unique[low,0],unique[low,2]);hi=np.interp(x,unique[up,0],unique[up,2]);z=lo+.2*(hi-lo)
 yl=np.interp(x,unique[low,0],unique[low,1]);yu=np.interp(x,unique[up,0],unique[up,1]);y=np.minimum(yl,yu)-1.0*np.sin(np.pi*(x-x.min())/(x.max()-x.min()))
 delta=np.zeros_like(unique);delta[ring,2]=z-unique[ring,2];delta[ring,1]=y-unique[ring,1]
 distance=cKDTree(unique[ring]).query(unique)[0];region=set(np.flatnonzero((distance<8)&(unique[:,0]*sign>0)))
 free=sorted(region-set(ring));idx={v:i for i,v in enumerate(free)};A=lil_matrix((len(free),len(free)));rhs=np.zeros((len(free),3))
 for v in free:
  total=0
  for j in adj[v]:
   w=1/max(np.linalg.norm(unique[v]-unique[j]),.1);total+=w
   if j in idx:A[idx[v],idx[j]]-=w
   else:rhs[idx[v]]+=delta[j]*w
  A[idx[v],idx[v]]=total
 delta[free]=spsolve(A.tocsr(),rhs)
 key=o.shape_key_add(name='Blink.'+side)
 for i,v in enumerate(key.data):v.co=p[i]+delta[inv[i]]
 lashes=bpy.data.objects['eyelashes_tex_eyelashes_0'];lashes.shape_key_add(name='Basis') if not lashes.data.shape_keys else None
 key2=lashes.shape_key_add(name='Blink.'+side)
 tree=cKDTree(unique[ring]);near,ids=tree.query(np.array([v.co[:] for v in lashes.data.vertices]),k=3)
 for i,v in enumerate(key2.data):
  pos=np.array(v.co)
  if pos[0]*sign>0 and pos[2]<384:
   weights=1/np.maximum(near[i],.01)**2;weights/=weights.sum();v.co=pos+np.sum(delta[np.array(ring)[ids[i]]]*weights[:,None],axis=0)
 key.value=1;key2.value=1
render(ROOT/'build/raphtalia_tools/reaudit/rejected_blink.png',(0,-.12,3.78),.72)
