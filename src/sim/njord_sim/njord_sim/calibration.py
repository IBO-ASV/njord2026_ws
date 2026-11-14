"""Body-frame experiment design, scoring and identifiable planar-model fits."""

import csv
import math
from pathlib import Path

import numpy as np

AXES = ('u', 'v', 'r')
IDS = ('rr', 'rl', 'fr', 'fl')
# Physical geometry, matching the checked-in URDF; orders are explicit in CSV.
X = np.array([-.1803, -.1803, .1803, .1803])
Y = np.array([-.25, .25, -.25, .25])
ANGLES = np.array([2.356194, -2.356194, .785398, -.785398])
LEVERS = X * np.sin(ANGLES) - Y * np.cos(ANGLES)


def write_reference(path, rows, fields=('time_s', 'ref_u', 'ref_v', 'ref_r')):
    with Path(path).open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        writer.writerows(rows)


def velocity_profile(dt=.02):
    """Positive/negative axis steps, reversals, stops and a coupled holdout."""
    rows = []
    t = 0.
    for axis, amplitude in enumerate((1.0288888889, .4, .35)):
        for sign in (1, -1):
            for duration, value in ((3, 0.), (12, sign * amplitude), (5, 0.)):
                for _ in range(round(duration / dt)):
                    ref = [0., 0., 0.]
                    ref[axis] = value
                    rows.append((t, *ref)); t += dt
    for _ in range(round(35 / dt)):
        phase = t - 120
        rows.append((t, .45 * math.sin(phase * .31), .25 * math.sin(phase * .47), .22 * math.sin(phase * .73)))
        t += dt
    for _ in range(round(8 / dt)):
        rows.append((t, 0., 0., 0.)); t += dt
    return rows


def force_profile(seed=7, duration=90, dt=.02):
    """Bounded multiaxis pulses, including coasts; independent seed for holdout."""
    rng = np.random.default_rng(seed)
    rows = []
    force = np.zeros(3)
    for i in range(round(duration / dt)):
        if i % 40 == 0:
            force = rng.uniform(-1, 1, 3) * [12, 8, 3]
            if i % 200 == 0:
                force[:] = 0
        rows.append((i * dt, *force))
    return rows


def load_csv(path):
    rows = np.genfromtxt(path, delimiter=',', names=True)
    rows = np.atleast_1d(rows)
    if len(rows) < 3 or not all(np.isfinite(rows[name]).all() for name in rows.dtype.names):
        raise ValueError('need at least three finite samples')
    if np.any(np.diff(rows['time_s']) <= 0):
        raise ValueError('timestamps must be strictly increasing')
    return rows


def tracking_metrics(rows):
    t = rows['time_s']
    error = np.column_stack([rows['ref_' + a] - rows[a] for a in AXES])
    per_axis = {}
    limits = np.array([.05, .05, .03])
    # Last three seconds of each fixed step and stop; full waveform is scored separately.
    reference = np.column_stack([rows['ref_' + a] for a in AXES])
    changes = np.flatnonzero(np.any(np.abs(np.diff(reference, axis=0)) > 1e-9, axis=1)) + 1
    boundaries = np.r_[0, changes, len(rows)]
    tails = np.zeros(len(rows), dtype=bool)
    settling = []
    worst_bias = np.zeros(3)
    worst_overshoot = np.zeros(3)
    for left, right in zip(boundaries[:-1], boundaries[1:]):
        if t[right - 1] - t[left] < 2.9:
            continue
        tails[left:right] = t[left:right] >= t[right - 1] - 3
        tail_error = error[left:right][t[left:right] >= t[right-1]-3]
        worst_bias = np.maximum(worst_bias, np.abs(np.mean(tail_error,axis=0)))
        for axis in range(3):
            target = reference[left,axis]
            if abs(target)>1e-6:
                actual = rows[AXES[axis]][left:right]
                overshoot = max(0.,float(np.max(np.sign(target)*actual)-abs(target)))
                worst_overshoot[axis] = max(worst_overshoot[axis],overshoot/max(abs(target),.1))
        outside = np.flatnonzero(np.any(np.abs(error[left:right]) > limits, axis=1))
        settling.append(float(t[left + outside[-1]] - t[left]) if len(outside) else 0.)
    if not np.any(tails):raise ValueError('reference needs fixed steps/stops lasting at least three seconds')
    for i, a in enumerate(AXES):
        per_axis[a] = {'rmse': float(np.sqrt(np.mean(error[:, i] ** 2))),
                       'steady_rmse': float(np.sqrt(np.mean(error[tails, i] ** 2))),
                       'peak_error': float(np.max(np.abs(error[:, i])))}
    steady = np.array([per_axis[a]['steady_rmse'] for a in AXES])
    final = np.array([rows[a][-1] for a in AXES])
    passed = bool(np.all(steady <= limits) and np.all(worst_bias <= limits)
                  and np.all(worst_overshoot <= .2) and max(settling,default=0)<=6.
                  and np.all([per_axis[a]['rmse']<=limit for a,limit in zip(AXES,(.15,.1,.1))])
                  and np.all(np.abs(final) <= [.03, .03, .02]))
    return {'passed': passed, 'axes': per_axis, 'maximum_settling_s': max(settling, default=0),
            'worst_steady_bias':worst_bias.tolist(), 'worst_overshoot_fraction':worst_overshoot.tolist(),
            'final_velocity': final.tolist(), 'saturation_fraction': float(np.mean(rows['saturated'])) if 'saturated' in rows.dtype.names else None}


def regression_system(rows, Lpp=1., D=.25, window=.2):
    """Integral balance for x_G=0; no noisy velocity differentiation."""
    u, v, r = (rows[a] for a in AXES)
    U = np.sqrt(u*u + v*v + 1e-6)
    q = .5 * 1025 * Lpp * D
    # Unknowns Mx, My, Iz_eff, R0, Y0, Yv, Yr, N0, Nv, Nr.
    features = np.zeros((len(rows), 3, 10))
    features[:, 0, 1] = -v*r
    features[:, 0, 3] = -q*u*U
    features[:, 1, 0] = u*r
    features[:, 1, 4:7] = -q*np.column_stack((U*U, v*U, r*Lpp*U))
    features[:, 2, 7:10] = -q*Lpp*np.column_stack((U*U, v*U, r*Lpp*U))
    force = np.column_stack([rows[a] for a in ('tau_x', 'tau_y', 'tau_n')])
    t = rows['time_s']
    design, target = [], []
    left = 0
    while left < len(rows)-1:
        right = int(np.searchsorted(t, t[left]+window))
        if right >= len(rows): break
        dt = np.diff(t[left:right+1])
        if np.max(dt) > .1: raise ValueError('sample gap exceeds 0.1 s')
        block = np.einsum('i,ijk->jk', dt, features[left:right])
        block[0, 0] += u[right]-u[left]
        block[1, 1] += v[right]-v[left]
        block[2, 2] += r[right]-r[left]
        design.extend(block)
        target.extend(np.einsum('i,ij->j', dt, force[left:right]))
        left = right
    if not design:raise ValueError("log is too short for integral identification windows")
    return np.asarray(design), np.asarray(target)


def fit_mmg(training, holdout, mass_kg, I_zG, Lpp=1., D=.25):
    """Measured dry mass/inertia are anchors, never inferred separately from added mass."""
    if min(mass_kg, I_zG, Lpp, D) <= 0: raise ValueError('positive measured anchors required')
    A, b = regression_system(training, Lpp, D)
    H, y = regression_system(holdout, Lpp, D)
    scale = np.linalg.norm(A, axis=0)
    if np.any(scale < 1e-10): raise ValueError('insufficient excitation')
    normalized = A / scale
    z, _, rank, singular = np.linalg.lstsq(normalized, b, rcond=1e-7)
    if rank != 10: raise ValueError(f'parameters not identifiable: rank {rank}/10')
    coef = z / scale
    params = dict(zip(('Mx','My','Iz_eff','R_0','Y_0','Y_v','Y_r','N_0','N_v','N_r'), map(float,coef)))
    physical = bool(coef[0] >= mass_kg and coef[1] >= mass_kg and coef[2] >= I_zG
                    and coef[3] < 0 and coef[5] < 0 and coef[9] < 0)
    error = float(np.linalg.norm(H@coef-y) / max(np.linalg.norm(y),1e-9))
    mmg = {k:v for k,v in params.items() if k not in ('Mx','My','Iz_eff')}
    mmg.update(Lpp=Lpp,D=D,mass_kg=mass_kg,I_zG=I_zG,x_G=0.,
               m_x=params['Mx']-mass_kg,m_y=params['My']-mass_kg,J_z=params['Iz_eff']-I_zG)
    return {'passed': physical and error < .1, 'rank': int(rank),
            'condition_number': float(singular[0]/singular[-1]), 'holdout_relative_force_error': error,
            'effective_parameters': params, 'mmg': mmg,
            'assumptions': ['x_G=0', 'calibrated applied force in N/Nm', 'body-frame water-relative velocity',
                            'known Lpp/D and measured dry mass/inertia', 'no unmodeled current/actuator lag']}


def fit_balance(trials, wiring_reverse, holdout=None):
    """Infer relative directional actuator response from short near-rest pulse trials.

    Input each row: commanded physical force rr/rl/fr/fl and measured yaw acceleration.
    Absolute force and inertia remain confounded; normalization only reduces thrust.
    """
    A, y = [], []
    for command, acceleration in trials:
        command = np.asarray(command)
        A.append(np.r_[np.maximum(command,0)*LEVERS, np.minimum(command,0)*LEVERS, 1.])
        y.append(acceleration)
    A, y = np.asarray(A), np.asarray(y)
    z, _, rank, s = np.linalg.lstsq(A,y,rcond=1e-7)
    if rank != 9: raise ValueError(f'need varied paired pulses, both directions and coasts: rank {rank}/9')
    if np.any(z[:8] <= 0): raise ValueError('response sign inconsistent; check wiring/geometry and current')
    relative = z[:8]/z[2]  # physical FR positive anchor
    compensation = min(relative)/relative
    forward, reverse = compensation[:4].copy(), compensation[4:].copy()
    for i, flag in enumerate(wiring_reverse):
        if flag: forward[i], reverse[i] = reverse[i], forward[i]
    validation = None
    if holdout:
        prediction, actual = [], []
        for command, acceleration in holdout:
            command=np.asarray(command)
            feature=np.r_[np.maximum(command,0)*LEVERS,np.minimum(command,0)*LEVERS,1.]
            prediction.append(float(feature@z)); actual.append(acceleration)
        validation=float(np.linalg.norm(np.asarray(prediction)-actual)/max(np.linalg.norm(actual),1e-9))
    return {'passed':validation is not None and validation < .1,
            'holdout_relative_yaw_accel_error':validation,
            'rank': int(rank), 'relative_response': relative.tolist(),
            'fit_rmse_yaw_accel': float(np.sqrt(np.mean((A@z-y)**2))),
            'static_map.thrusters.forward_gain': forward.tolist(),
            'static_map.thrusters.reverse_gain': reverse.tolist(),
            'note': 'Relative balance only; dry inertia and absolute force are not identified. Gains reduce stronger directions to weakest.'}


def mission_tracking_metrics(samples):
    """Observational dynamic errors; do not relax or replace mission acceptance."""
    if not samples:return {'available':False}
    rows=np.asarray(samples)
    fresh=rows[:,7] < .2
    reference=rows[:,1:4];actual=rows[:,4:7]
    commanded=np.any(np.abs(reference)>[.01,.01,.01],axis=1)
    active=fresh & commanded
    if not np.any(active):return {'available':False,'fresh_samples':int(np.sum(fresh))}
    error=actual-reference
    return {'available':True,'frame':'base_link','active_samples':int(np.sum(active)),
            'stale_command_samples':int(np.sum(~fresh)),
            'rmse':dict(zip(AXES,map(float,np.sqrt(np.mean(error[active]**2,axis=0))))),
            'p95_absolute_error':dict(zip(AXES,map(float,np.quantile(np.abs(error[active]),.95,axis=0)))),
            'note':'Includes acceleration, braking and turns; independent step/sweep criteria qualify the low-level controller.'}


def plot_tracking(baseline, selected, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
    for axis,name in zip(axes,AXES):
        axis.plot(selected['time_s'],selected['ref_'+name],color='black',linewidth=1,label='command')
        axis.plot(baseline['time_s'],baseline[name],alpha=.7,label='original P')
        axis.plot(selected['time_s'],selected[name],linewidth=1,label='selected PI + reference filter')
        axis.set_ylabel(name+(' [rad/s]' if name=='r' else ' [m/s]'));axis.grid(alpha=.3)
    axes[0].legend(loc='upper right');axes[-1].set_xlabel('time [s]')
    figure.tight_layout();figure.savefig(output,dpi=150);plt.close(figure)


def calibrated_forces(physical_commands, relative_response, anchor_scale):
    """Convert signed command Newtons using directional balance and a measured FR+ anchor."""
    command=np.asarray(physical_commands,dtype=float)
    response=np.asarray(relative_response,dtype=float)
    if command.shape!=(4,) or response.shape!=(8,) or not np.isfinite(command).all():
        raise ValueError('four physical commands and eight directional responses required')
    if not np.isfinite(response).all() or np.any(response<=0) or not np.isfinite(anchor_scale) or anchor_scale<=0:
        raise ValueError('force responses and absolute anchor must be finite and positive')
    return command*np.where(command>=0,response[:4],response[4:])*anchor_scale
