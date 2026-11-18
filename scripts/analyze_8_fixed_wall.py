#!/usr/bin/env python3
"""Analyze the eight-agent fixed-wall recording without running ROS nodes.

Wall intersections use recorded world-frame CUBE markers. Pair distances use
piecewise-linear synchronized odometry, excluding interpolation over >0.2 s gaps.
This evaluates point-center trajectories, not a certified vehicle collision mesh.
"""
import argparse
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path
import re

import numpy as np
import rosbag
import yaml


def stamp(message, fallback):
    value = message.header.stamp.to_sec()
    return value if value > 0 else fallback


def track(rows):
    a = np.asarray(rows, dtype=float)
    a = a[np.argsort(a[:, 0], kind='stable')]
    # Keep the last observation when multiple messages carry the same stamp.
    return a[np.r_[np.diff(a[:, 0]) > 0, True]]


def interpolate(a, times):
    return np.column_stack([np.interp(times, a[:, 0], a[:, k]) for k in [1, 2, 3]])


def valid_intervals(a, times, limit=0.2):
    idx = np.clip(np.searchsorted(a[:, 0], (times[:-1] + times[1:]) / 2) - 1, 0, len(a) - 2)
    return (a[idx + 1, 0] - a[idx, 0]) <= limit


def slab_intersections(p, q, low, high):
    d = q - p
    enter, leave = np.zeros(len(p)), np.ones(len(p))
    valid = np.ones(len(p), dtype=bool)
    for axis in range(3):
        moving = np.abs(d[:, axis]) > 1e-12
        valid &= moving | ((p[:, axis] >= low[axis]) & (p[:, axis] <= high[axis]))
        a, b = np.full(len(p), -np.inf), np.full(len(p), np.inf)
        a[moving] = (low[axis] - p[moving, axis]) / d[moving, axis]
        b[moving] = (high[axis] - p[moving, axis]) / d[moving, axis]
        enter = np.maximum(enter, np.minimum(a, b))
        leave = np.minimum(leave, np.maximum(a, b))
    return valid & (enter < leave), enter, leave


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path, help='run.bag or its containing directory')
    args = parser.parse_args()
    bagfile = args.recording / 'run.bag' if args.recording.is_dir() else args.recording
    folder = bagfile.parent
    out = folder / 'analysis'
    out.mkdir(exist_ok=True)
    params_file = folder / 'params.yaml'
    params = yaml.safe_load(params_file.read_text()) if params_file.exists() else {}
    raw = defaultdict(list)
    commands = defaultdict(list)
    trajectories = defaultdict(list)
    raw_plans = defaultdict(list)
    heartbeats = defaultdict(list)
    finishes = defaultdict(list)
    goals, boxes, logs, leaders = [], [], [], []
    with rosbag.Bag(str(bagfile)) as bag:
        bag_start, bag_end = bag.get_start_time(), bag.get_end_time()
        for topic, m, t in bag.read_messages():
            bt = t.to_sec()
            match = re.match(r'/drone_(\d+)', topic)
            i = int(match[1]) if match else None
            if topic.endswith('/odom') and i is not None:
                if m.header.frame_id not in ('world', '/world'):
                    raise ValueError('Odometry must be in world frame: ' + topic)
                p, v = m.pose.pose.position, m.twist.twist.linear
                raw[i].append([stamp(m, bt), p.x, p.y, p.z, v.x, v.y, v.z])
            elif topic.endswith('/pos_cmd') and i is not None:
                p, v = m.position, m.velocity
                commands[i].append([stamp(m, bt), p.x, p.y, p.z, v.x, v.y, v.z])
            elif topic.endswith('/trajectory') and i is not None:
                durations = np.asarray(m.duration)
                raw_plans[i].append((durations, [list(m.coef_x), list(m.coef_y), list(m.coef_z)]))
                coeff = np.asarray(list(m.coef_x) + list(m.coef_y) + list(m.coef_z))
                trajectories[i].append({'time': bt, 'id': m.traj_id,
                    'duration': float(durations.sum()), 'pieces': len(durations),
                    'finite': bool(np.isfinite(durations).all() and np.isfinite(coeff).all()),
                    'valid_shape': m.order == 5 and len(coeff) == 18 * len(durations),
                    'positive_durations': bool(len(durations) and (durations > 0).all())})
            elif topic.endswith('/data_display') and i is not None:
                heartbeats[i].append([bt, m.a, m.b, m.c, m.d, m.e])
            elif topic.endswith('/finish') and i is not None and m.data:
                finishes[i].append(bt)
            elif topic == '/goal':
                goals.append([bt, m.pose.position.x, m.pose.position.y, m.pose.position.z])
            elif topic == '/map_generator/wall_geometry':
                boxes = []
                for marker in m.markers:
                    if marker.type != 1 or marker.header.frame_id not in ('world', '/world'):
                        raise ValueError('Expected world-frame CUBE walls')
                    q = marker.pose.orientation
                    if abs(q.x) + abs(q.y) + abs(q.z) > 1e-8:
                        raise ValueError('Rotated walls need oriented-box analysis')
                    p, s = marker.pose.position, marker.scale
                    center, size = np.array([p.x,p.y,p.z]), np.array([s.x,s.y,s.z])
                    boxes.append({'id':marker.id,'center':center.tolist(),'size':size.tolist(),
                                  'low':(center-size/2).tolist(),'high':(center+size/2).tolist()})
            elif topic == '/rosout_agg':
                logs.append({'time':bt,'node':m.name,'level':m.level,'message':m.msg})
            elif topic == '/leader_traj_from_planner':
                leaders.append(bt)
    if len(goals) != 1:
        raise ValueError('Expected one goal per trial; found %d. Split recording by goal first.' % len(goals))
    if not boxes or set(raw) != set(range(8)):
        raise ValueError('Need wall geometry and odometry for all eight agents')
    goal_time, *goal = goals[0]
    goal = np.array(goal)
    tracks = {i:track(rows) for i,rows in raw.items()}
    for i,a in tracks.items():
        if len(a) < 2 or not np.isfinite(a).all():
            raise ValueError('Invalid or insufficient odometry for agent %d' % i)
    planned_stats = {}
    for i, plans in raw_plans.items():
        hit_count, peak_speed, skipped = 0, 0.0, 0
        for durations, coeffs in plans:
            if (not len(durations) or not np.isfinite(durations).all() or
                    (durations <= 0).any() or durations.sum() > 60 or
                    any(len(x) != 6*len(durations) or not np.isfinite(x).all() for x in coeffs)):
                skipped += 1
                continue
            hit = False
            for piece, duration in enumerate(durations):
                ts = np.linspace(0, duration, max(3, int(np.ceil(duration/.02))+1))
                co = np.array([x[6*piece:6*piece+6] for x in coeffs])
                pp = np.array([np.polyval(c, ts) for c in co]).T
                vv = np.array([np.polyval(np.polyder(c), ts) for c in co]).T
                peak_speed = max(peak_speed, float(np.linalg.norm(vv, axis=1).max()))
                hit |= any(((pp > box['low']) & (pp < box['high'])).all(axis=1).any() for box in boxes)
            hit_count += int(hit)
        planned_stats[i] = {'full_horizon_sampled_wall_hits':hit_count,
                            'published_polynomial_sampled_peak_speed_mps':peak_speed,
                            'full_horizon_sampling_skipped':skipped}
    rows, crossings = [], []
    for i,a in sorted(tracks.items()):
        t0 = max(goal_time,a[0,0])
        times = np.r_[t0,a[a[:,0] > t0,0]]
        points = interpolate(a,times)
        valid = valid_intervals(a,times)
        pparams = params.get('drone_%d_ego_planner_node'%i,{})
        offsets = pparams.get('formation',{}).get('offsets')
        offset = np.array(offsets[3*i:3*i+3] if offsets else [[x,y,0] for x in [.5,-.5] for y in [-1.5,-.5,.5,1.5]][i])
        target = goal + offset
        errors = np.linalg.norm(points-target,axis=1)
        reached = errors[-1] <= 0.3
        # Stable arrival: all remaining observations stay within 0.3 m, >=1 s observed.
        persistent = np.maximum.accumulate(errors[::-1])[::-1] <= .3
        indices = np.flatnonzero(persistent & (times <= times[-1]-1))
        arrival = float(times[indices[0]]-goal_time) if len(indices) else None
        speeds = np.linalg.norm(a[:,4:7],axis=1)
        dt = np.diff(times)
        fd_speed = np.linalg.norm(np.diff(points,axis=0),axis=1)/dt
        velocity_after = a[a[:,0] >= goal_time]
        speed_peak = float(np.linalg.norm(velocity_after[:,4:7],axis=1).max())
        wall_ids=[]
        wall_distances=[]
        for box in boxes:
            low,high=np.array(box['low']),np.array(box['high'])
            wall_distances.append(np.linalg.norm(np.maximum(np.maximum(low-points,points-high),0),axis=1))
            hit,enter,leave=slab_intersections(points[:-1],points[1:],low,high)
            hit &= valid
            inside=((points>low+1e-7)&(points<high-1e-7)).all(axis=1)
            if hit.any() or inside.any():
                wall_ids.append(box['id'])
                indexes=np.flatnonzero(hit)
                crossings.append({'drone':i,'wall':box['id'],
                    'first_entry_s':float(np.min(times[:-1][hit]+enter[hit]*dt[hit])-goal_time) if len(indexes) else None,
                    'inside_samples':int(inside.sum()),
                    'interpolated_inside_seconds':float(np.sum((leave[hit]-enter[hit])*dt[hit]))})
        distances=np.vstack(wall_distances)
        nearest=np.unravel_index(distances.argmin(),distances.shape)
        steps=np.linalg.norm(np.diff(points,axis=0),axis=1)
        step_index=int(np.argmax(steps))
        tx=trajectories[i]
        rows.append({'drone':i,'path_length_m':float(np.linalg.norm(np.diff(points,axis=0)[valid],axis=1).sum()),
            'final_x':float(points[-1,0]),'final_y':float(points[-1,1]),'final_z':float(points[-1,2]),
            'sampled_min_wall_distance_m':float(distances.min()),
            'closest_wall_id':boxes[nearest[0]]['id'],'closest_wall_time_s':float(times[nearest[1]]-goal_time),
            'max_odom_position_step_m':float(steps[step_index]),'max_step_dt_s':float(dt[step_index]),
            'goal_error_m':float(errors[-1]),'within_goal_tolerance':bool(reached),'stable_arrival_s':arrival,
            'peak_odom_speed_mps':speed_peak,'peak_finite_difference_speed_mps':float(fd_speed[valid].max()),
            'nominal_speed_limit_mps':pparams.get('manager',{}).get('max_vel',.5),
            'trajectory_messages':len(tx),'last_trajectory_s':float(tx[-1]['time']-goal_time) if tx else None,
            'last_fsm_display_s':float(heartbeats[i][-1][0]-goal_time) if heartbeats[i] else None,
            'last_odom_s':float(a[-1,0]-goal_time),'max_odom_gap_s':float(np.diff(a[:,0]).max()),
            'finish_signal_s':float(finishes[i][0]-goal_time) if finishes[i] else None,
            'intersected_wall_ids':wall_ids,
            'max_local_trajectory_duration_s':max((x['duration'] for x in tx),default=None),
            'invalid_trajectory_messages':sum(not(x['finite'] and x['valid_shape'] and x['positive_durations']) for x in tx)})
    pairs=[]
    for i in range(8):
        for j in range(i+1,8):
            a,b=tracks[i],tracks[j]
            lo,hi=max(goal_time,a[0,0],b[0,0]),min(a[-1,0],b[-1,0])
            times=np.unique(np.r_[lo,hi,a[(a[:,0]>lo)&(a[:,0]<hi),0],b[(b[:,0]>lo)&(b[:,0]<hi),0]])
            r=interpolate(a,times)-interpolate(b,times)
            change=np.diff(r,axis=0);rr=r[:-1]
            denom=np.einsum('ij,ij->i',change,change)
            u=np.clip(-np.einsum('ij,ij->i',rr,change)/np.maximum(denom,1e-30),0,1)
            distances=np.linalg.norm(rr+u[:,None]*change,axis=1)
            valid=valid_intervals(a,times)&valid_intervals(b,times)
            distances[~valid]=np.inf
            k=int(np.argmin(distances))
            pairs.append({'pair':[i,j],'min_distance_m':float(distances[k]),
                          'time_s':float(times[k]+u[k]*(times[k+1]-times[k])-goal_time)})
    for agent in rows:
        agent.update(planned_stats.get(agent['drone'], {}))
    solver_ms=[float(m['message'].split(':')[-1].split()[0]) for m in logs if m['message'].startswith('ring planning time:')]
    report={'bag':str(bagfile),'bag_duration_s':bag_end-bag_start,
        'post_goal_duration_s':bag_end-goal_time,'goal':goal.tolist(),'wall_geometry':boxes,
        'configured_swarm_clearance_m':params.get('drone_0_ego_planner_node',{}).get('optimization',{}).get('swarm_clearance',.4),
        'arrival_tolerance_m':.3,'arrived_count':sum(x['within_goal_tolerance'] for x in rows),
        'agents':rows,'wall_crossings':crossings,'pair_distances':sorted(pairs,key=lambda x:x['min_distance_m']),
        'error_and_fatal_logs':[m for m in logs if m['level'] >= 8],
        'warning_messages':dict(Counter(m['message'] for m in logs if m['level']==4).most_common(12)),
        'virtual_leader_solver_ms':{'count':len(solver_ms),'median':float(np.median(solver_ms)) if solver_ms else None,
            'p95':float(np.percentile(solver_ms,95)) if solver_ms else None,'max':max(solver_ms,default=None)},
        'data_display_all_zero':all(all(not any(x[1:]) for x in h) for h in heartbeats.values()),
        'limitations':['Full-horizon polynomial checks use <=20 ms sampling and include later-superseded portions; they do not imply execution of each predicted collision.',
            'Point-center wall intersections and synchronized piecewise-linear odometry; no collision mesh or orientation-aware swept body.',
            'No interpolation across odometry gaps >0.2 s.','Local solver times are not populated in DataDisp; ring times measure the virtual-leader solver only.',
            'No console.log was recorded.' if not (folder/'console.log').exists() else 'Console log available.']}
    resource_file=folder/'resources.log'
    if resource_file.exists():
        values=[]
        for line in resource_file.read_text().splitlines():
            parts=line.split()
            if len(parts)>=19 and all(p.isdigit() for p in parts[:17]):values.append([int(x) for x in parts[:17]])
        values=np.array(values[1:]) # vmstat first line is the average since boot.
        if len(values):report['system_resources']={'samples':len(values),'cpu_busy_mean_percent':float(np.mean(100-values[:,14])),
            'cpu_busy_peak_percent':int(np.max(100-values[:,14])),'free_ram_min_mib':float(np.min(values[:,3])/1024),
            'swap_used_max_mib':float(np.max(values[:,2])/1024),'swap_in_peak_kib_s':int(values[:,6].max()),'swap_out_peak_kib_s':int(values[:,7].max())}
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    with (out/'agents.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    with (out/'wall_crossings.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(crossings[0]) if crossings else ['drone','wall']);w.writeheader();w.writerows(crossings)
    plot(out,tracks,goal_time,boxes,trajectories)
    print(json.dumps({'output':str(out),'arrived':report['arrived_count'],'wall_crossings':crossings,
        'closest_pair':report['pair_distances'][0],'agents':rows,'resources':report.get('system_resources'),
        'leader_solver':report['virtual_leader_solver_ms']},indent=2))


def plot(out,tracks,goal_time,boxes,trajectories):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig,axs=plt.subplots(1,3,figsize=(17,5))
    for box in boxes:
        low,high=np.array(box['low']),np.array(box['high'])
        axs[0].add_patch(Rectangle(low[:2],*(high-low)[:2],color='slategray',alpha=.5))
    for i,a in sorted(tracks.items()):
        a=a[a[:,0]>=goal_time];color=plt.get_cmap('tab10')(i)
        axs[0].plot(a[:,1],a[:,2],color=color,label=str(i));axs[0].scatter(a[-1,1],a[-1,2],color=color,s=35,marker='x')
        axs[1].plot(a[:,0]-goal_time,np.linalg.norm(a[:,4:7],axis=1),color=color,label=str(i))
        t=[x['time']-goal_time for x in trajectories[i]]
        axs[2].scatter(t,[i]*len(t),color=color,s=16)
    axs[0].set(title='Recorded trajectories (XY); x = last position',xlabel='x (m)',ylabel='y (m)',ylim=(-5.5,5.5));axs[0].set_aspect('equal');axs[0].legend(ncol=4,title='Drone')
    axs[1].axhline(.5,color='black',linestyle='--',label='nominal 0.5 m/s');axs[1].set(title='Odometry speed',xlabel='Seconds after goal',ylabel='Speed (m/s)');axs[1].legend(ncol=3)
    axs[2].set(title='Published local trajectories',xlabel='Seconds after goal',ylabel='Drone ID',yticks=range(8))
    for ax in axs:ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(out/'overview.png',dpi=180);plt.close(fig)


if __name__=='__main__':
    main()
