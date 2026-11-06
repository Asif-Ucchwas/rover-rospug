import sys,json,time; sys.path.insert(0,'py')
import numpy as np
from fpp import Planner, inflation_costmap, path_stats
from worlds import corridor_world, RES
C=json.load(open('results_corridor.json')); g=corridor_world(0)
pl=Planner(g,RES,margin=0.02); v=Planner(g,RES,margin=0.0)
cfgs={'wc0':(None,0)}
for name,(rad,sc,wc) in {'wc1':(1.0,6,1),'wc3':(1.0,6,3)}.items(): cfgs[name]=(inflation_costmap(pl,rad,sc),wc)
pairs=C['pairs']; ok_idx=[i for i,q in enumerate(C['results']['se2'] if 'results' in C else [])] if False else None
res={k:[] for k in cfgs}
for i,(s,t,h0) in enumerate(pairs):
    row={}
    for k,(cm,wc) in cfgs.items():
        pl.set_cost(cm,wc); r=pl.plan('se2',(s[0],s[1],h0),t,tight=True)
        if not r['ok']: row[k]=None; continue
        P=r['path']; row[k]=dict(len=path_stats(P)[0],bad=int(v.validate(P)),clr=min(pl.clearance(x,y) for x,y,_ in P[::3]),
          mc=float(np.mean([min(cfgs['wc1'][0][min(int(y/RES),g.shape[0]-1),min(int(x/RES),g.shape[1]-1)],252) for x,y,_ in P[::3]])),secs=r['secs'])
    for k in cfgs: res[k].append(row[k])
    print(i,end=' ',flush=True)
json.dump(res,open('results_cost.json','w'))
common=[i for i in range(len(pairs)) if all(res[k][i] for k in cfgs)]
print('\ncommon solved',len(common))
for k in cfgs:
    R=[res[k][i] for i in common]; print(k,'valid',sum(r['bad']==0 for r in R),'len',round(np.mean([r['len'] for r in R]),2),'minclr',round(np.mean([r['clr'] for r in R]),3),'meancost',round(np.mean([r['mc'] for r in R]),1),'secs',round(np.mean([r['secs'] for r in R]),2))
