#!/usr/bin/env python3
"""
experiment_logger.py — Autonomous metric recorder for TERESA trials.

Subscribes to mission-critical topics and computes the six metrics
defined in the RA-L experimental protocol. Saves one JSON file per trial
and a cumulative CSV for cross-trial analysis.

Usage:
    ros2 run spot_control experiment_logger
    ros2 run spot_control experiment_logger --ros-args -p output_dir:=/tmp/my_trials
"""

import json
import csv
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int32
from geometry_msgs.msg import PoseStamped


class ExperimentLogger(Node):

    def __init__(self):
        super().__init__('experiment_logger')

        self._output_dir = os.path.expanduser(
            self.declare_parameter('output_dir', '~/teresa_experiments')
            .get_parameter_value().string_value
        )
        os.makedirs(self._output_dir, exist_ok=True)

        # ── subscribers ───────────────────────────────────────
        self._sub_state = self.create_subscription(
            String, '/wbc/state', self._cb_wbc_state, 10
        )
        self._sub_z1_state = self.create_subscription(
            String, '/z1_fsm/state', self._cb_z1_state, 10
        )
        self._sub_ik_done = self.create_subscription(
            Bool, '/ik_done', self._cb_ik_done, 10
        )
        self._sub_fast_ready = self.create_subscription(
            Bool, '/z1/fast_ready', self._cb_fast_ready, 10
        )
        self._sub_next_point = self.create_subscription(
            Int32, '/z1/next_point_idx', self._cb_next_point, 10
        )
        self._sub_handoff = self.create_subscription(
            Bool, '/wbc/handoff_reached', self._cb_handoff, 10
        )
        self._sub_grid_type = self.create_subscription(
            String, '/wbc/scan_grid_type', self._cb_grid_type, 10
        )
        self._sub_fast_target = self.create_subscription(
            PoseStamped, '/z1/fast_target_pose', self._cb_fast_target, 10
        )
        self._sub_torso_state = self.create_subscription(
            String, '/torso_tracker_state', self._cb_torso_state, 10
        )

        # ── trial state ───────────────────────────────────────
        self._trial_active = False
        self._trial = None
        self._csv_path = os.path.join(self._output_dir, 'all_trials.csv')
        self._init_csv()

        self.get_logger().info(f'Ready — output: {self._output_dir}')

    def _init_csv(self):
        if not os.path.exists(self._csv_path):
            with open(self._csv_path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow([
                    'trial_id', 'mode', 'timestamp',
                    'mission_time_s', 'idle_time_s',
                    'scan_overlap_pct', 'lock_time_s',
                    'probe_error_mean_mm', 'probe_error_max_mm',
                    'scan_grid_type', 'dual_sensor_lock',
            'exposure_duration_s',
            'completion', 'patient_distance_m', 'patient_angle_deg',
            'fast_reachability_pct', 'exposure_reachability_pct',
            'fast_points_total', 'fast_points_skipped',
            'exposure_points_total', 'exposure_points_skipped'
        ])

    def _start_trial(self):
        self._trial_active = True
        self._trial = {
            'trial_id': datetime.now().strftime('%Y%m%d_%H%M%S'),
            'mode': 'unknown',
            't_start': time.time(),
            'state_timeline': [],
            'wbc_state': None,
            't_lock_start': None,
            't_lock_end': None,
            't_scan_begin': None,
            't_handoff': None,
            't_scan_done': None,
            't_fast_ready_ts': None,
            't_exposure_start': None,
            't_review_start': None,
            't_mission_end': None,
            'fast_point_count': 0,
            'probe_errors_mm': [],
            'scan_grid_type': 'unknown',
            'dual_sensor_lock': False,
            'completion': False,
            'patient_distance_m': None,
            'patient_angle_deg': None,
            'idle_segments': [],
            'idle_segment_start': None,
            # Reachability tracking
            'fast_points_total': 0,
            'fast_points_skipped': 0,
            'exposure_points_total': 0,
            'exposure_points_skipped': 0,
            'in_exposure': False,   # tracks whether we're in EXPOSURE_SCANNING
            'in_fast': False,        # tracks whether we're in SCANNING (FAST)
        }
        self._record_state('SEARCHING')

    def _finish_trial(self, completed):
        if not self._trial_active:
            return
        self._trial_active = False
        t = self._trial
        t['completion'] = completed
        t['t_mission_end'] = time.time()

        mission_time = t['t_mission_end'] - t['t_start']
        idle_time = sum(s['duration'] for s in t['idle_segments'])

        lock_time = None
        if t['t_lock_start'] and t['t_lock_end']:
            lock_time = t['t_lock_end'] - t['t_lock_start']

        exposure_time = None
        if t['t_exposure_start']:
            # find next SCANNING or IDLE after EXPOSURE_SCANNING
            for i, s in enumerate(t['state_timeline']):
                if s['state'] == 'EXPOSURE_SCANNING':
                    if i + 1 < len(t['state_timeline']):
                        next_s = t['state_timeline'][i + 1]
                        exposure_time = next_s['t'] - s['t']
                    break

        scan_overlap = 0.0
        if t['t_handoff'] and t['t_fast_ready_ts']:
            if t['t_fast_ready_ts'] <= t['t_handoff']:
                scan_overlap = 100.0
            else:
                scan_dur = t['t_fast_ready_ts'] - t['t_scan_begin']
                if scan_dur > 0:
                    ov = 100.0 * (1.0 - (
                        t['t_fast_ready_ts'] - t['t_handoff']) / scan_dur)
                    scan_overlap = max(0.0, ov)

        errors = t['probe_errors_mm']
        probe_mean = sum(errors) / len(errors) if errors else None
        probe_max = max(errors) if errors else None

        # Reachability: % of points successfully reached (not skipped)
        fast_total = t['fast_points_total'] + t['fast_points_skipped']
        fast_reach = round(100.0 * t['fast_points_total'] / fast_total, 1) if fast_total > 0 else None
        exp_total = t['exposure_points_total'] + t['exposure_points_skipped']
        exp_reach = round(100.0 * t['exposure_points_total'] / exp_total, 1) if exp_total > 0 else None

        record = {
            'trial_id': t['trial_id'],
            'mode': t['mode'],
            'timestamp': datetime.now().isoformat(),
            'patient_distance_m': t['patient_distance_m'],
            'patient_angle_deg': t['patient_angle_deg'],
            'metrics': {
                'total_mission_time_s': round(mission_time, 1),
                'idle_time_s': round(idle_time, 1),
                'idle_breakdown': [
                    {'phase': s['phase'], 'duration_s': round(s['duration'], 1)}
                    for s in t['idle_segments']
                ],
                'scan_overlap_pct': round(scan_overlap, 1),
                'lock_time_s': round(lock_time, 1) if lock_time else None,
                'probe_error_mean_mm': (
                    round(probe_mean, 1) if probe_mean else None
                ),
                'probe_error_max_mm': (
                    round(probe_max, 1) if probe_max else None
                ),
                'completion': completed,
                'scan_grid_type': t['scan_grid_type'],
                'dual_sensor_lock': t['dual_sensor_lock'],
                'exposure_duration_s': (
                    round(exposure_time, 1) if exposure_time else None
                ),
                'fast_reachability_pct': fast_reach,
                'exposure_reachability_pct': exp_reach,
                'fast_points_total': t['fast_points_total'],
                'fast_points_skipped': t['fast_points_skipped'],
                'exposure_points_total': t['exposure_points_total'],
                'exposure_points_skipped': t['exposure_points_skipped'],
            },
            'state_timeline': t['state_timeline'],
        }

        json_path = os.path.join(self._output_dir, f"{t['trial_id']}.json")
        with open(json_path, 'w') as f:
            json.dump(record, f, indent=2)
        self.get_logger().info(f'Trial saved: {json_path}')

        with open(self._csv_path, 'a', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                t['trial_id'], t['mode'], record['timestamp'],
                round(mission_time, 1), round(idle_time, 1),
                round(scan_overlap, 1),
                round(lock_time, 1) if lock_time else '',
                round(probe_mean, 1) if probe_mean else '',
                round(probe_max, 1) if probe_max else '',
                t['scan_grid_type'], t['dual_sensor_lock'],
                round(exposure_time, 1) if exposure_time else '',
                completed,
                t['patient_distance_m'] or '',
                t['patient_angle_deg'] or '',
                fast_reach if fast_reach is not None else '',
                exp_reach if exp_reach is not None else '',
                t['fast_points_total'],
                t['fast_points_skipped'],
                t['exposure_points_total'],
                t['exposure_points_skipped'],
            ])

    # ── helpers ──────────────────────────────────────────────
    def _record_state(self, state):
        elapsed = time.time() - self._trial['t_start']
        self._trial['state_timeline'].append({
            'state': state, 't': round(elapsed, 1)
        })

    def _start_idle(self, phase):
        if self._trial['idle_segment_start'] is None:
            self._trial['idle_segment_start'] = (time.time(), phase)

    def _end_idle(self):
        seg = self._trial['idle_segment_start']
        if seg is not None:
            start_t, phase = seg
            dur = time.time() - start_t
            if dur > 0.1:
                self._trial['idle_segments'].append({
                    'phase': phase, 'duration': dur
                })
            self._trial['idle_segment_start'] = None

    # ── callbacks ─────────────────────────────────────────────
    def _cb_wbc_state(self, msg):
        state = msg.data
        if self._trial_active:
            prev = self._trial['wbc_state']
            if state != prev:
                self._trial['wbc_state'] = state
                self._record_state(state)

                if state == 'LOCKING':
                    self._trial['t_lock_end'] = time.time()
                elif state == 'SEMI_LOCKING':
                    self._trial['dual_sensor_lock'] = True
                elif state == 'APPROACHING':
                    self._trial['t_scan_begin'] = time.time()
                elif state == 'SCANNING':
                    self._trial['in_fast'] = True
                    self._end_idle()
                    self._start_idle('scan_settle')
                elif state == 'EXPOSURE_SCANNING':
                    self._trial['in_exposure'] = True
                    self._trial['t_exposure_start'] = time.time()
                elif state == 'EXPOSURE_REVIEW':
                    self._trial['t_review_start'] = time.time()
                elif state == 'IDLE':
                    self._end_idle()
                    self._finish_trial(
                        prev in ('SCANNING',)  # completed
                    )
        elif state == 'SEARCHING':
            self._start_trial()
            self._trial['wbc_state'] = state
            self._trial['t_lock_start'] = time.time()

    def _cb_z1_state(self, msg):
        self._trial_active and None

    def _cb_ik_done(self, msg):
        if self._trial_active and msg.data:
            self._end_idle()

    def _cb_fast_ready(self, msg):
        if self._trial_active and msg.data:
            self._trial['t_fast_ready_ts'] = time.time()
            self._trial['t_scan_done'] = time.time()

    def _cb_next_point(self, msg):
        if not self._trial_active:
            return
        if msg.data == -1:
            # Point skipped — unreachable
            if self._trial['in_fast']:
                self._trial['fast_points_skipped'] += 1
            elif self._trial['in_exposure']:
                self._trial['exposure_points_skipped'] += 1
            self._end_idle()
        elif msg.data >= 0:
            # Point being visited
            if self._trial['in_fast']:
                self._trial['fast_points_total'] += 1
            elif self._trial['in_exposure']:
                self._trial['exposure_points_total'] += 1
            self._end_idle()
            self._start_idle('fast_settle')

    def _cb_handoff(self, msg):
        if self._trial_active and msg.data:
            self._trial['t_handoff'] = time.time()
            self._end_idle()

    def _cb_grid_type(self, msg):
        if self._trial_active:
            self._trial['scan_grid_type'] = msg.data

    def _cb_fast_target(self, msg):
        pass

    def _cb_torso_state(self, msg):
        if self._trial_active:
            self._trial['_torso_state'] = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
