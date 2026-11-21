#!/usr/bin/env python3
"""Record GNSS fixes/headings and summarize stationary repeatability, not accuracy.

Run with the GNSS driver already active. Coordinates are body x-forward/y-left.
"""
import argparse
import collections
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix


def summarize(rows, main_x=-0.3, main_y=0.3):
    fixes = [r for r in rows if r['kind'] == 'fix' and r['status'] >= 0]
    headings = [r for r in rows if r['kind'] == 'heading']
    result = {'fix_count': len(fixes), 'heading_count': len(headings),
              'status_counts': dict(collections.Counter(r['status'] for r in rows if r['kind'] == 'fix'))}
    if fixes:
        lat = np.radians([r['latitude'] for r in fixes])
        lon = np.radians([r['longitude'] for r in fixes])
        phi = float(lat.mean())
        a, e2 = 6378137.0, 6.69437999014e-3
        n = a / math.sqrt(1 - e2 * math.sin(phi)**2)
        m = a * (1-e2) / (1-e2 * math.sin(phi)**2)**1.5
        east = (lon-lon.mean()) * n * math.cos(phi)
        north = (lat-lat.mean()) * m
        radius = np.hypot(east, north)
        result['position'] = {
            'east_std_m': float(np.std(east, ddof=1 if len(fixes) > 1 else 0)),
            'north_std_m': float(np.std(north, ddof=1 if len(fixes) > 1 else 0)),
            'horizontal_rms_about_mean_m': float(np.sqrt(np.mean(radius**2))),
            'radius_p95_about_mean_m': float(np.percentile(radius, 95)),
            'radius_max_about_mean_m': float(radius.max()),
            'first_to_last_m': float(np.hypot(east[-1]-east[0], north[-1]-north[0])),
            'altitude_std_m': float(np.std([r['altitude'] for r in fixes], ddof=1 if len(fixes) > 1 else 0)),
        }
    if fixes and headings:
        # Interpolate orientation at each fix; unwrap before interpolation.
        yaw_at_fix = np.interp(
            [r['elapsed_sec'] for r in fixes],
            [r['elapsed_sec'] for r in headings],
            np.unwrap([r['yaw_rad'] for r in headings]))
        center_e = east - (np.cos(yaw_at_fix)*main_x - np.sin(yaw_at_fix)*main_y)
        center_n = north - (np.sin(yaw_at_fix)*main_x + np.cos(yaw_at_fix)*main_y)
        center_e -= center_e.mean()
        center_n -= center_n.mean()
        center_radius = np.hypot(center_e, center_n)
        result['body_origin_position'] = {
            'main_xy_m': [main_x, main_y],
            'east_std_m': float(np.std(center_e, ddof=1 if len(fixes) > 1 else 0)),
            'north_std_m': float(np.std(center_n, ddof=1 if len(fixes) > 1 else 0)),
            'horizontal_rms_about_mean_m': float(np.sqrt(np.mean(center_radius**2))),
            'radius_p95_about_mean_m': float(np.percentile(center_radius, 95)),
            'radius_max_about_mean_m': float(center_radius.max()),
        }
    if headings:
        yaw = np.array([r['yaw_rad'] for r in headings])
        center = math.atan2(float(np.sin(yaw).mean()), float(np.cos(yaw).mean()))
        error = np.arctan2(np.sin(yaw-center), np.cos(yaw-center))
        result['heading'] = {
            'mean_vessel_compass_deg': (90-math.degrees(center)) % 360,
            'std_deg': float(np.degrees(error).std(ddof=1 if len(headings) > 1 else 0)),
            'absolute_deviation_p95_deg': float(np.percentile(np.abs(np.degrees(error)), 95)),
            'peak_to_peak_deg': float(np.degrees(error).max()-np.degrees(error).min()),
        }
    for kind in ('fix', 'heading'):
        stamps = [r['elapsed_sec'] for r in rows if r['kind'] == kind]
        if len(stamps) > 1:
            result[kind+'_timing'] = {'observed_hz': (len(stamps)-1)/(stamps[-1]-stamps[0]),
                                      'maximum_gap_sec': max(np.diff(stamps))}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=180)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--main-x', type=float, default=-0.3)
    parser.add_argument('--main-y', type=float, default=0.3)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rows = []
    rclpy.init()
    node = rclpy.create_node('gnss_stationary_observer')
    start = time.monotonic()
    def fix(msg):
        rows.append(dict(kind='fix', elapsed_sec=time.monotonic()-start,
                         stamp_sec=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                         status=msg.status.status, latitude=msg.latitude,
                         longitude=msg.longitude, altitude=msg.altitude,
                         covariance=list(msg.position_covariance)))
    def heading(msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        rows.append(dict(kind='heading', elapsed_sec=time.monotonic()-start,
                         stamp_sec=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                         yaw_rad=yaw, frame=msg.header.frame_id,
                         covariance=list(msg.pose.covariance)))
    node.create_subscription(NavSatFix, '/sensor/vehicle_gnss/fix/raw', fix, 100)
    node.create_subscription(PoseWithCovarianceStamped, '/sensor/vehicle_gnss/compass/raw', heading, 100)
    next_report = 30
    try:
        while time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.2)
            elapsed = time.monotonic()-start
            if elapsed >= next_report:
                print(f'{elapsed:.0f}s: {len(rows)} messages', flush=True)
                next_report += 30
    finally:
        (args.output/'samples.json').write_text(json.dumps(rows))
        result = summarize(rows, args.main_x, args.main_y)
        (args.output/'summary.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
