"""Analyze actual PX4 ULogs; no synthetic flight data."""
import json, pathlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pyulog import ULog

root=pathlib.Path(__file__).resolve().parent
(root/'analysis').mkdir(exist_ok=True)
results=[]
for path in sorted((root/'logs').glob('*.ulg')):
    u=ULog(str(path))
    def topic(name):
        return u.get_dataset(name).data
    lp=topic('vehicle_local_position'); vs=topic('vehicle_status'); land=topic('vehicle_land_detected')
    t0=u.start_timestamp
    def ts(d): return (d['timestamp'].astype(float)-t0)/1e6
    t=ts(lp); ground=float(np.median(lp['z'][:min(10,len(lp['z']))])); h=ground-lp['z']
    ar=vs['arming_state']==2
    armed_times=ts(vs)[ar]
    desc=(lp['vz']>0.15)&(h>1)&(h<4)
    # Restrict comparison to RTL / Land modes rather than the takeoff trajectory.
    nav=vs['nav_state'][np.clip(np.searchsorted(ts(vs),t,side='right')-1,0,len(vs['nav_state'])-1)]
    desc &= (np.isclose(nav,5)|np.isclose(nav,18))
    modes=[]
    for i in np.r_[0,np.flatnonzero(np.diff(vs['nav_state']))+1]:
        modes.append({'t':float(ts(vs)[i]),'nav_state':int(vs['nav_state'][i])})
    landed_edges=[]
    for i in np.r_[0,np.flatnonzero(np.diff(land['landed'].astype(int)))+1]:
        landed_edges.append({'t':float(ts(land)[i]),'landed':bool(land['landed'][i])})
    r={'file':path.name,'duration_s':float((u.last_timestamp-t0)/1e6),'max_altitude_m':float(np.max(h)),
       'max_horizontal_displacement_m':float(np.max(np.hypot(lp['x']-lp['x'][0],lp['y']-lp['y'][0]))),
       'final_horizontal_displacement_m':float(np.hypot(lp['x'][-1]-lp['x'][0],lp['y'][-1]-lp['y'][0])),
       'near_ground_descent_median_m_s':float(np.median(lp['vz'][desc])) if desc.any() else None,
       'near_ground_descent_p05_p95_m_s':[float(x) for x in np.percentile(lp['vz'][desc],[5,95])] if desc.any() else None,
       'mode_transitions':modes,'landed_transitions':landed_edges,
       'arming_transitions':[{'t':float(ts(vs)[i]),'state':int(vs['arming_state'][i])} for i in np.r_[0,np.flatnonzero(np.diff(vs['arming_state']))+1]],
       'initial_parameters':{k:float(u.initial_parameters[k]) for k in ['MPC_LAND_SPEED','RTL_RETURN_ALT','RTL_DESCEND_ALT','RTL_LAND_DELAY'] if k in u.initial_parameters},
       'ulog_dropout_count':len(u.dropouts)}
    att=topic('vehicle_attitude'); q=np.array([att[f'q[{i}]'] for i in range(4)])
    roll=np.arctan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]**2+q[2]**2))*180/np.pi
    pitch=np.arcsin(np.clip(2*(q[0]*q[2]-q[3]*q[1]),-1,1))*180/np.pi
    landing_times=t[(np.isclose(nav,5)|np.isclose(nav,18))&(lp['vz']>0.15)]
    if len(landing_times):
        mask=(ts(att)>=landing_times[0])&(ts(att)<=landing_times[-1])
        r['landing_max_abs_roll_pitch_deg']=[float(np.max(np.abs(v[mask]))) for v in [roll,pitch]]
    peak_t=t[np.argmax(h)]
    descent_segment=(t>peak_t)&(h>0.5)&(np.isclose(nav,5)|np.isclose(nav,18))
    r['descent_max_upward_speed_m_s']=float(max(0,-np.min(lp['vz'][descent_segment]))) if descent_segment.any() else None
    fig,axs=plt.subplots(4,1,figsize=(10,10),sharex=True)
    axs[0].plot(t,h,label='Estimated height')
    try:
        sp=topic('vehicle_local_position_setpoint'); axs[0].plot(ts(sp),ground-sp['z'],label='Height setpoint',alpha=.7)
        target=np.interp(t,ts(sp),sp['z']); valid=np.isfinite(target)&desc
        r['near_ground_height_tracking_rmse_m']=float(np.sqrt(np.mean((lp['z'][valid]-target[valid])**2))) if valid.any() else None
    except (KeyError,StopIteration,ValueError): pass
    axs[0].set_ylabel('Height (m)'); axs[0].legend()
    axs[1].plot(t,lp['vz']); axs[1].set_ylabel('Down velocity (m/s)')
    axs[2].plot(t,np.hypot(lp['x']-lp['x'][0],lp['y']-lp['y'][0])); axs[2].set_ylabel('Home distance (m)')
    axs[3].plot(ts(att),roll,label='Roll'); axs[3].plot(ts(att),pitch,label='Pitch'); axs[3].legend(); axs[3].set_ylabel('Angle (deg)'); axs[3].set_xlabel('Time since log start (s)')
    for ax in axs:
        ax.grid(alpha=.3)
        if len(armed_times): ax.set_xlim(max(0,armed_times[0]-5),armed_times[-1]+2)
        for e in modes:
            ax.axvline(e['t'],alpha=.2,color='gray')
    fig.suptitle(path.name); fig.tight_layout(); fig.savefig(root/'analysis'/f'{path.stem}.png',dpi=140); plt.close(fig)
    results.append(r)
(root/'analysis/summary.json').write_text(json.dumps(results,indent=2))
print(json.dumps(results,indent=2))
