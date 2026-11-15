#!/usr/bin/env python3
"""Offline comparison of detailed and conservative proxy collision scenes."""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from cad import sha256
from planner import Arm, load_config, ticks_to_joints
from collision import CollisionWorld


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--proxy',type=Path,required=True)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--poses',type=Path,required=True)
    parser.add_argument('--activate',action='store_true')
    args=parser.parse_args();root=Path(__file__).resolve().parent
    urdf=root/'.sdk/hardware/elrobot/simulation/elrobot_follower.urdf'
    config=load_config(args.config);arm=Arm(urdf,config)
    poses=json.loads(args.poses.read_text())['poses']
    named={name:ticks_to_joints(config,[pose['motors'][str(i)]['position'] for i in range(1,8)]) for name,pose in poses.items()}
    values=list(named.values());samples=list(named.values())
    for start,end in zip(values,values[1:]+values[:1]):samples.extend(np.linspace(start,end,12)[1:-1])
    outcomes=[];timings={}
    for label,path in [('detailed',args.source),('proxy',args.proxy)]:
        with CollisionWorld(urdf,arm,path) as world:
            rows=[];costs=[]
            for q in samples:
                begin=time.perf_counter();rows.append(world.collision(q));costs.append((time.perf_counter()-begin)*1000)
            outcomes.append(rows)
            timings[label]=dict(first_query_ms=costs[0],steady_p50_ms=float(np.percentile(costs[1:],50)),
                                steady_p95_ms=float(np.percentile(costs[1:],95)),maximum_ms=max(costs))
    missed=[i for i,(d,p) in enumerate(zip(*outcomes)) if d is not None and p is None]
    false_positives=[i for i,(d,p) in enumerate(zip(*outcomes)) if d is None and p is not None]
    report=dict(samples=len(samples),missed_source_collisions=missed,conservative_extra_rejections=false_positives,
                timings=timings,source_scene_sha256=sha256(args.source),
                saved_pose_results={name:dict(detailed=outcomes[0][i],proxy=outcomes[1][i]) for i,name in enumerate(named)})
    print(json.dumps(report,indent=2),flush=True)
    if missed:raise RuntimeError('Proxy missed source collisions; do not activate')
    if args.activate:
        source=json.loads(args.source.read_text());proxy=json.loads(args.proxy.read_text())
        proxy['simplification']['comparison']=report
        proxy['hardware_verified']=source.get('hardware_verified',False)
        proxy['execution_command_period_s']=.06
        proxy['execution_mode']='streamed'
        with args.proxy.open('w') as stream:json.dump(proxy,stream,indent=2)


if __name__=='__main__':main()
