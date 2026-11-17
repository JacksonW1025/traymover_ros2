#!/usr/bin/env python3
"""Paired real FAST-LIO vs wheel+NDT benchmark on identical synthetic sensors.

Instantaneous XYZIRT snapshots (1 us point span), not a physical spinning-LiDAR
simulator. Both NDT instances receive the same base_link cloud. No actual device
is opened. The trajectory is prescribed, not steered using either estimator.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import sys

import numpy as np
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).resolve().parent))
import option21_simulation as sim


def surface(axis, fixed, a, b, spacing=.20):
    aa, bb = np.meshgrid(np.arange(*a, spacing), np.arange(*b, spacing))
    points = np.empty((aa.size, 3), dtype=np.float32)
    points[:, axis] = fixed
    other = [i for i in range(3) if i != axis]
    points[:, other[0]], points[:, other[1]] = aa.ravel(), bb.ravel()
    return points


def box(x, y, width, depth, height):
    result = []
    for xx in (x-width/2, x+width/2):
        result.append(surface(0, xx, (y-depth/2, y+depth/2+.01), (0, height+.01)))
    for yy in (y-depth/2, y+depth/2):
        result.append(surface(1, yy, (x-width/2, x+width/2+.01), (0, height+.01)))
    return np.concatenate(result)


def hall_maps():
    floor = surface(2, 0, (-40, 40.01), (-25, 25.01))
    walls = [surface(0, x, (-25, 25.01), (0, 8.01)) for x in (-40, 40)]
    walls += [surface(1, y, (-40, 40.01), (0, 8.01)) for y in (-25, 25)]
    pillars = [box(x, y, w, w, 7.) for x, y, w in
               [(-24., -12., 1.2), (18., 13., 1.5), (31., -11., 1.0)]]
    prior = np.concatenate([floor, *walls, *pillars])
    # Remove only the former cloth frontage: x=8..20, height=0..4 m.
    frontage = (np.abs(prior[:, 1]-25) < .01) & (prior[:, 0] >= 8) & (prior[:, 0] <= 20) & (prior[:, 2] < 4)
    live = np.concatenate([
        prior[~frontage], surface(1, 28., (8, 20.01), (0, 4.01)),
        surface(0, 8., (25, 28.01), (0, 4.01)), surface(0, 20., (25, 28.01), (0, 4.01)),
        box(7., -5., 2., 1.2, 1.1), box(24., 8., 2.2, 1.5, 1.3),
    ])
    return prior.astype(np.float32), live.astype(np.float32), int(frontage.sum())


def stamp_at(seconds):
    from builtin_interfaces.msg import Time
    nanoseconds = round(seconds*1e9)
    return Time(sec=nanoseconds//1000000000, nanosec=nanoseconds % 1000000000)


class AirportModel:
    def __init__(self, case, seed, duration):
        self.case, self.seed = case, seed
        self.scan_cache = None
        self.initial_error = (0., 0., 0.)
        self.prior_map, self.changed_map, removed = hall_maps()
        self.wheel_gain = 1.01  # assumed 1% gain error in BOTH measured vx/wz
        self.max_range = 80. if case != 'stress' else 25.
        self.motion_end = 132.
        self.description = {
            'hall_m': [80, 50, 8], 'case': case, 'seed': seed,
            'removed_frontage_points': removed, 'prior_points': len(self.prior_map),
            'changed_frontage': '12m width, 4m height; new back wall 3m deeper',
            'pedestrians': 0 if case == 'static' else 18,
            'new_kiosks': 0 if case == 'static' else 2,
            'max_range_m': self.max_range,
            'sensor_model': 'instantaneous 32-ring snapshot, 1us time span; 2cm xyz noise',
            'imu_model': '200Hz, 0.02m/s^2 accel noise, 0.002rad/s gyro noise, constant .001rad/s gyro bias',
            'comparison': 'identical prescribed truth trajectory and input hashes; no estimator steers the plant',
            'initial_pose_error_xy_m_yaw_deg': [0., 0., 0.],
            'tracking_initialization': 'both frontends warm up; same initialpose at 5s; first NDT scan at 5.2s',
        }
        self.frames = []
        pose = np.zeros(3)
        velocity = np.zeros(2)
        rng = np.random.default_rng(seed+1000)
        alpha = 1-(1-.35)**.1
        for step in range(round(duration/.05)):
            samples = []
            for j in range(10):
                elapsed = step*.05-.045+j*.005
                command = self.command(elapsed)
                previous = velocity.copy()
                velocity += alpha*(command-velocity)
                average = (velocity+previous)/2
                pose = sim.plant_step(pose, *average, .005)
                acceleration = np.array([(velocity[0]-previous[0])/.005,
                                         average[0]*average[1], 9.81])
                acceleration += rng.normal(0, .02, 3)
                gyro = rng.normal(0, .002, 3)
                gyro[2] += average[1]+.001
                samples.append((10+elapsed, pose[2], acceleration, gyro))
            self.frames.append((pose.copy(), velocity.copy(), samples))
        angle, height = np.meshgrid(np.linspace(0, 2*math.pi, 20, endpoint=False),
                                   np.arange(.1, 1.8, .15))
        self.person = np.column_stack([.25*np.cos(angle.ravel()), .25*np.sin(angle.ravel()), height.ravel()])

    def prepare_scans(self):
        """Generate once offline so both estimators replay at the same wall rate."""
        rng=np.random.default_rng(self.seed)
        self.scan_cache=[]
        for step,(pose,_,_) in enumerate(self.frames):
            self.scan_cache.append(sim.make_scan(self.live_points(step*.05),pose,rng,
                                                 noise=.02,max_range=self.max_range))
            if step % 400 == 0:
                print(f'Precomputing {self.case} seed {self.seed}: {step}/{len(self.frames)}',flush=True)

    def load_scan_cache(self, bag_path):
        """Reuse the independent prescribed sequence, never an estimated cloud."""
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        reader=rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag_path),storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('',''))
        reader.set_filter(rosbag2_py.StorageFilter(topics=['/point_cloud_localization']))
        needed=min(len(self.frames),round((self.motion_end+5)/.05)+1)
        self.scan_cache=[None]*needed
        while reader.has_next():
            _,data,_=reader.read_next()
            msg=deserialize_message(data,PointCloud2)
            seconds=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            index=round((seconds-10)/.05)
            if 0 <= index < needed:
                self.scan_cache[index]=np.frombuffer(msg.data,dtype='<f4').reshape(-1,4)[:,:3].copy()
        rng=np.random.default_rng(self.seed)
        for index in range(min(104,needed)):
            pose,_=self.motion(index)
            self.scan_cache[index]=sim.make_scan(self.live_points(index*.05),pose,rng,
                                                  noise=.02,max_range=self.max_range)
        if any(points is None for points in self.scan_cache):
            raise RuntimeError('Source bag does not contain the complete prescribed sensor sequence')
        self.description['cached_sensor_bag']=str(bag_path)

    @staticmethod
    def command(elapsed):
        turn = math.pi/2/.2
        phases = [(8., (0., 0.)), (38., (.5, 0.)), (38+turn, (0., .2)),
                  (54+turn, (.5, 0.)), (54+3*turn, (0., .2)),
                  (70+3*turn, (.5, 0.)), (70+4*turn, (0., .2)),
                  (100+4*turn, (-.5, 0.))]
        for end, value in phases:
            if elapsed < end:
                return np.asarray(value)
        return np.zeros(2)

    def motion(self, step):
        pose, velocity, _ = self.frames[step]
        return pose.copy(), velocity.copy()

    def imu_messages(self, step):
        messages = []
        for stamp, yaw, acceleration, gyro in self.frames[step][2]:
            msg = Imu(header=Header(stamp=stamp_at(stamp), frame_id='imu_link'))
            msg.orientation.z, msg.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = map(float, acceleration)
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = map(float, gyro)
            messages.append(msg)
        return messages

    def live_points(self, elapsed):
        if self.case == 'static':
            return self.prior_map
        people = []
        for i in range(18):
            x = 2.+(i % 6)*3.4+.3*math.sin(.2*elapsed+i)
            y = -6.+((elapsed*(.45+.03*(i % 3))+i*1.3) % 18.)
            people.append(self.person+[x, y, 0.])
        return np.concatenate([self.changed_map, *people]).astype(np.float32)

    @staticmethod
    def raw_cloud(points, stamp):
        # Snapshot at the scan END. Every point has a nonzero 1us time offset,
        # and header is shifted 1us back, so FAST-LIO does no artificial deskew.
        sensor = points - [.435, 0., 1.365]
        data = np.zeros(len(points), dtype=np.dtype({
            'names': ['x', 'y', 'z', 'intensity', 'time', 'ring'],
            'formats': ['<f4', '<f4', '<f4', '<f4', '<f4', '<u2'],
            'offsets': [0, 4, 8, 12, 16, 20], 'itemsize': 24}))
        for i, name in enumerate(('x', 'y', 'z')):
            data[name] = sensor[:, i]
        data['intensity'], data['time'] = 1., 1e-6
        elevation = np.arctan2(sensor[:, 2], np.linalg.norm(sensor[:, :2], axis=1))
        data['ring'] = np.clip(((elevation+.5)/.95*32).astype(int), 0, 31)
        ns = stamp.sec*1000000000+stamp.nanosec-1000
        msg = PointCloud2(header=Header(stamp=stamp_at(ns/1e9), frame_id='laser'),
                          height=1, width=len(data), is_bigendian=False, is_dense=True,
                          point_step=24, row_step=len(data)*24, data=data.tobytes())
        msg.fields = [PointField(name=name, offset=offset, datatype=kind, count=1)
                      for name, offset, kind in [('x', 0, 7), ('y', 4, 7), ('z', 8, 7),
                                                ('intensity', 12, 7), ('time', 16, 7), ('ring', 20, 4)]]
        return msg


def summarize(directory, result):
    a = np.genfromtxt(directory/'metrics.csv', delimiter=',', names=True)
    moving_window = a['t'] >= 8
    raw_valid = moving_window & np.isfinite(a['odom_error_m'])
    global_valid = moving_window & np.isfinite(a['localized_x'])
    current_error = np.hypot(a['localized_x']-a['truth_x'], a['localized_y']-a['truth_y'])
    rmse = lambda v, mask: float(np.sqrt(np.mean(v[mask]**2))) if mask.any() else None
    result.update(
        cloud_noise_std_m=.02,
        note='Synthetic airport geometry/people and assumed sensor errors; not hardware accuracy evidence.',
        odom_rmse_m=rmse(a['odom_error_m'], raw_valid),
        global_current_rmse_m=rmse(current_error, global_valid),
        global_current_p95_m=float(np.quantile(current_error[global_valid], .95)) if global_valid.any() else None,
        global_current_max_m=float(np.max(current_error[global_valid])) if global_valid.any() else None,
        global_yaw_rmse_deg=math.degrees(rmse(a['localized_yaw_error_rad'], global_valid)) if global_valid.any() else None,
        localization_coverage=float(np.mean(global_valid[moving_window])),
        localization_fresh_coverage=float(np.mean(np.isfinite(a['localized_age_s'][moving_window]) & (a['localized_age_s'][moving_window] <= .3))),
        raw_odom_fresh_coverage=float(np.mean(np.isfinite(a['odom_age_s'][moving_window]) & (a['odom_age_s'][moving_window] <= .3))),
        max_localization_age_s=float(np.nanmax(a['localized_age_s'])) if global_valid.any() else None,
        first_localization_s=float(a['t'][np.isfinite(a['localized_x'])][0]) if global_valid.any() else None,
        truth_travel_distance_m=float(np.sum(np.hypot(np.diff(a['truth_x']),np.diff(a['truth_y'])))),
    )
    unreliable = moving_window & (~np.isfinite(current_error) | (current_error > .5)
                                  | (a['localized_age_s'] > .3))
    result['unavailable_stale_or_over_50cm_fraction'] = float(np.mean(unreliable[moving_window]))
    result['localization_fresh_coverage_at_nav2_600ms'] = float(np.mean(
        np.isfinite(a['localized_age_s'][moving_window]) & (a['localized_age_s'][moving_window] <= .6)))
    yaw_error=np.arctan2(np.sin(a['localized_yaw']-a['truth_yaw']),
                        np.cos(a['localized_yaw']-a['truth_yaw']))
    result['global_current_yaw_rmse_deg']=math.degrees(rmse(yaw_error,global_valid)) if global_valid.any() else None
    log=(directory/'ndt.log').read_text()
    fitness=[float(x) for x in re.findall(r'NDT aligned: fitness=([\d.]+)',log)]
    result['ndt_accepted_fitness_median']=float(np.median(fitness)) if fitness else None
    result['ndt_rejection_or_fault_lines'] = sum(
        bool(re.search(r'keeping last good TF|not converged|waiting for valid NDT|odom discontinuity', line))
        for line in log.splitlines())
    result['benchmark_checks'] = {
        'coverage_above_95pct': result['localization_coverage'] >= .95,
        'current_position_rmse_below_30cm': result['global_current_rmse_m'] is not None and result['global_current_rmse_m'] < .3,
        'p95_below_50cm': result['global_current_p95_m'] is not None and result['global_current_p95_m'] < .5,
    }
    result['passed_benchmark_checks']=all(result['benchmark_checks'].values())
    (directory/'results.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


def compare(output, results):
    pairs=[]
    for case in sorted({r['experiment']['case'] for r in results}):
        for seed in sorted({r['seed'] for r in results}):
            group={r['odometry_source']:r for r in results if r['experiment']['case']==case and r['seed']==seed}
            if len(group)!=2:continue
            wheel,lio=group['wheel'],group['fastlio']
            same=wheel['input_sensor_sha256']==lio['input_sensor_sha256']
            if not same:raise RuntimeError('Sensor inputs differ: comparison is invalid')
            wheel_dir=output/f'{case}_s{seed}_wheel'
            lio_dir=output/f'{case}_s{seed}_fastlio'
            def ndt_settings(directory):
                settings=sim.yaml.safe_load((directory/'ndt.yaml').read_text())
                settings['/**']['ros__parameters'].pop('map_path')
                return settings
            same_ndt=ndt_settings(wheel_dir)==ndt_settings(lio_dir)
            same_map=hashlib.sha256((wheel_dir/'room.pcd').read_bytes()).digest()==hashlib.sha256((lio_dir/'room.pcd').read_bytes()).digest()
            pairs.append({'case':case,'seed':seed,'identical_sensor_input':same,
                          'identical_ndt_settings':same_ndt,'identical_prior_map':same_map,
                          'wheel_rmse_m':wheel['global_current_rmse_m'],'fastlio_rmse_m':lio['global_current_rmse_m'],
                          'wheel_p95_m':wheel['global_current_p95_m'],'fastlio_p95_m':lio['global_current_p95_m'],
                          'wheel_fresh_coverage':wheel['localization_fresh_coverage'],
                          'fastlio_fresh_coverage':lio['localization_fresh_coverage'],
                          'wheel_over_50cm_or_stale_fraction':wheel.get('unavailable_stale_or_over_50cm_fraction'),
                          'fastlio_over_50cm_or_stale_fraction':lio.get('unavailable_stale_or_over_50cm_fraction')})
            if not same_ndt or not same_map:raise RuntimeError('NDT backend/map differ: comparison is invalid')
    (output/'paired_results.json').write_text(json.dumps(pairs,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(len(pairs),2,figsize=(12,4*len(pairs)),squeeze=False)
    for row,pair in enumerate(pairs):
        case,seed=pair['case'],pair['seed']
        for source,color in [('wheel','royalblue'),('fastlio','darkorange')]:
            a=np.genfromtxt(output/f'{case}_s{seed}_{source}'/'metrics.csv',delimiter=',',names=True)
            if source=='wheel':axes[row,0].plot(a['truth_x'],a['truth_y'],'g--',label='Ground truth')
            axes[row,0].plot(a['localized_x'],a['localized_y'],color=color,label=source+' + NDT')
            error=np.hypot(a['localized_x']-a['truth_x'],a['localized_y']-a['truth_y'])
            axes[row,1].plot(a['t'],error,color=color,label=source+' + NDT')
        axes[row,0].set(title=f'{case}, seed {seed}',xlabel='x (m)',ylabel='y (m)',aspect='equal')
        axes[row,1].set(xlabel='Simulation time (s)',ylabel='Current position error (m)')
        for ax in axes[row]:ax.legend();ax.grid(True)
    fig.suptitle('Synthetic 80 x 50 m hall: real estimators, identical LiDAR/IMU, not hardware calibration')
    fig.tight_layout();fig.savefig(output/'comparison.png',dpi=150);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--case',choices=['static','changed','stress','all'],default='all')
    parser.add_argument('--source',choices=['wheel','fastlio','both'],default='both')
    parser.add_argument('--seeds',default='101,202')
    parser.add_argument('--duration',type=float,default=140.)
    parser.add_argument('--rate',type=float,default=1.5)
    parser.add_argument('--domain',type=int,default=169)
    parser.add_argument('--analyze-only',action='store_true')
    parser.add_argument('--reuse-existing',action='store_true')
    parser.add_argument('--initial-yaw-deg',type=float,default=0.)
    parser.add_argument('--sensor-cache-bag',type=Path)
    args=parser.parse_args()
    args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=True)
    results=[]
    for case in (['static','changed'] if args.case=='all' else [args.case]):
        for seed in map(int,args.seeds.split(',')):
            if case=='static' and args.case=='all' and seed!=int(args.seeds.split(',')[0]):continue
            sources=['wheel','fastlio'] if args.source=='both' else [args.source]
            model=None
            if not args.analyze_only and any(not(args.reuse_existing and
                    (args.output/f'{case}_s{seed}_{source}'/'results.json').exists()) for source in sources):
                model=AirportModel(case,seed,args.duration)
                model.initial_error=(0.,0.,math.radians(args.initial_yaw_deg))
                model.description['initial_pose_error_xy_m_yaw_deg']=[0.,0.,args.initial_yaw_deg]
                if args.sensor_cache_bag:
                    model.load_scan_cache(args.sensor_cache_bag)
                else:
                    model.prepare_scans()
            for source in sources:
                name=f'{case}_s{seed}_{source}'
                args.odometry_source,args.seed=source,seed
                if args.analyze_only or (args.reuse_existing and (args.output/name/'results.json').exists()):
                    result=json.loads((args.output/name/'results.json').read_text())
                else:
                    result=sim.run_scenario(args,name,model=model)
                result=summarize(args.output/name,result)
                results.append(result)
                (args.output/'results.json').write_text(json.dumps(results,indent=2,allow_nan=False)+'\n')
                print(json.dumps({k:result[k] for k in ['scenario','odometry_source','odom_rmse_m','global_current_rmse_m','localization_coverage','ndt_accepted','passed_benchmark_checks']}),flush=True)
    sim.write_playback_qos(args.output)
    if args.source=='both':compare(args.output,results)


if __name__=='__main__':main()
