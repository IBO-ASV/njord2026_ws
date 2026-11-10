import numpy as np
import torch
from asv_trajectory_planner import crm_torch
import math
from typing import List, Tuple, Optional
from asv_trajectory_planner.vessel_state import VesselState

class ship_torch():
    def __init__(self, loa):
        """
        classを初期化するmethod
        インスタンスを作成した際に必ず呼ばれる関数
        """
        self.x = None
        self.y = None
        self.psi = None
        self.K=0.060265768294965424
        self.T=30.249045960691753
        self.loa = loa

        self.U_min = 1*1852/3600
        self.U_max = 3*1852/3600

    def reset(self,x,y,U,psi,r):
        """
        位置(x,y)，速度U, 方位psiを初期化
        """
        self.x = x
        self.y = y
        self.U = U
        self.psi = psi
        self.r = r
        return self.data

    def move(self, r, acc, dt):
        """
        KTモデルの運動を計算
        """
        u_next = torch.clip(
            self.U + acc*dt,
            self.U_min, self.U_max
        )
        r_next = r
        psi_next = self.psi + (self.r+r_next)/2*dt
        x_next = self.x + (u_next+self.U)/2*torch.cos(torch.deg2rad((self.psi+psi_next)/2))*dt
        y_next = self.y + (u_next+self.U)/2*torch.sin(torch.deg2rad((self.psi+psi_next)/2))*dt

        self.x = x_next
        self.y = y_next
        self.psi = psi_next
        self.r = r_next
        self.U = u_next

        return self.data

    @property
    def data(self):
        """
        propoatyで定義されたmethodは，(インスタンス名).dataでreturnに書かれたデータを取得できる.
        関数を変数の様に扱うことができる．
        classの持っている変数に何か処理をしたデータが欲しい時などに便利
        """
        return [self.x, self.y, self.U, self.psi, self.r]


"for planner"

class ship():
    def __init__(self, loa):
        """
        classを初期化するmethod
        インスタンスを作成した際に必ず呼ばれる関数
        """
        self.x = None
        self.y = None
        self.psi = None
        self.K=0.060265768294965424
        self.T=30.249045960691753
        self.loa = loa

        self.U_min = 0
        self.U_max = 12*1852/3600

    def reset(self,x,y,U,psi,r):
        """
        位置(x,y)，速度U, 方位psiを初期化
        """
        self.x = x
        self.y = y
        self.U = U
        self.psi = psi
        self.r = r
        return self.data

    def move(self, r, acc, dt):
        """
        KTモデルの運動を計算
        """
        u_next = np.clip(
            self.U + acc*dt,
            self.U_min, self.U_max
        )
        r_next = r
        psi_next = self.psi + (self.r+r_next)/2*dt
        x_next = self.x + (u_next+self.U)/2*np.cos(np.radians((self.psi+psi_next)/2))*dt
        y_next = self.y + (u_next+self.U)/2*np.sin(np.radians((self.psi+psi_next)/2))*dt

        self.x = x_next
        self.y = y_next
        self.psi = psi_next
        self.r = r_next
        self.U = u_next

        return self.data

    @property
    def data(self):
        """
        propoatyで定義されたmethodは，(インスタンス名).dataでreturnに書かれたデータを取得できる.
        関数を変数の様に扱うことができる．
        classの持っている変数に何か処理をしたデータが欲しい時などに便利
        """
        return [self.x, self.y, self.U, self.psi, self.r]

def diviation2path(x,y, p0, p1):
    p0top1 = np.sqrt( (p1[0]-p0[0])**2 + (p1[1]-p0[1])**2 )

    dist2p0_square = (x-p0[0])**2+(y-p0[1])**2
    naiseki =(
        (
            (x-p0[0])*(p1[0]-p0[0]) + (y-p0[1])*(p1[1]-p0[1])
        )/p0top1
    )
    _zeros = torch.zeros_like(x)
    return (
        torch.where(
            naiseki<0, torch.sqrt(dist2p0_square), _zeros
        ) + torch.where(
            (0<=naiseki)&(naiseki<=p0top1),
            torch.sqrt( torch.clip(dist2p0_square - naiseki**2,0,None) ), _zeros
        ) + torch.where(
            naiseki>=p0top1, torch.sqrt((x-p1[0])**2+(y-p1[1])**2), _zeros
        )
    )

def diviation2infinite_line(predx, predy, wp0, wp1, eps=1e-6):
    """
    predx, predy:
        torch.Tensor
        予測位置

    wp0, wp1:
        pathを定義する2点
        例: path[0][:2], path[1][:2]

    Returns
    -------
    diviation:
        2点を通る無限直線への距離
    """

    device = predx.device
    dtype = predx.dtype

    x0 = torch.as_tensor(wp0[0], device=device, dtype=dtype)
    y0 = torch.as_tensor(wp0[1], device=device, dtype=dtype)
    x1 = torch.as_tensor(wp1[0], device=device, dtype=dtype)
    y1 = torch.as_tensor(wp1[1], device=device, dtype=dtype)

    dx = x1 - x0
    dy = y1 - y0

    length = torch.sqrt(dx * dx + dy * dy).clamp_min(eps)

    # 点と無限直線の距離
    # |(p - p0) × (p1 - p0)| / |p1 - p0|
    diviation = torch.abs(
        (predx - x0) * dy - (predy - y0) * dx
    ) / length

    return diviation

def get_traj(x, y, psi, U, r, opt_rudder, opt_acc, loa, pred_dt,
             target_speed_mps=2.0 * 1852.0 / 3600.0):
    # target_speed_mps default keeps the historical hardcoded Task2 speed (2 kn).
    task2_speed_mps = target_speed_mps
    _ship = ship(loa)
    _ship.reset(x, y, task2_speed_mps, psi, r)
    opts_traj = [_ship.data]
    for _rud, _acc in zip(opt_rudder, opt_acc):
        _ship.move(
            _rud, _acc, pred_dt
        )
        opts_traj.append(_ship.data)
    opts_traj = np.array(opts_traj)
    return opts_traj.T

def resample_traj_by_distance(
    opttraj : np.ndarray,
    spacing_m: float,
) -> List[Tuple[float, float]]:
    if spacing_m <= 0.0:
        raise ValueError("spacing_m must be positive.")

    traj = np.asarray(opttraj)

    x = traj[0, :]
    y = traj[1, :]

    if len(x) == 0:
        return []

    if len(x) == 1:
        return [(float(x[0]), float(y[0]))]

    dx = np.diff(x)
    dy = np.diff(y)

    segment_lengths = np.hypot(dx, dy)
    cumulative_s = np.concatenate(([0.0], np.cumsum(segment_lengths)))

    total_length = cumulative_s[-1]

    if total_length < 1e-9:
        return [(float(x[0]), float(y[0]))]

    keep = np.concatenate(([True], np.diff(cumulative_s) > 1e-9))

    cumulative_s = cumulative_s[keep]
    x = x[keep]
    y = y[keep]

    s_new = np.arange(0.0, total_length, spacing_m)

    if len(s_new) == 0 or s_new[-1] < total_length:
        s_new = np.append(s_new, total_length)

    x_new = np.interp(s_new, cumulative_s, x)
    y_new = np.interp(s_new, cumulative_s, y)

    points = [
        (float(xi), float(yi))
        for xi, yi in zip(x_new, y_new)
    ]

    return points

def buoy_outside_cost(
    X,
    Y,
    gps0,
    gps1,
    buoys,
    margin_m=1.0,
    longitudinal_sigma_m=8.0,
    weight=50.0,
):
    """
    GPS直線に対して、各ブイの外側に出た点へコストを与える。

    X, Y:
        MPPI予測点。torch.Tensor
    gps0, gps1:
        GPS point の座標 [x, y]
    buoys:
        ブイ座標のリスト [[x, y], ...]
    """

    if buoys is None or len(buoys) == 0:
        return torch.zeros_like(X)

    device = X.device
    dtype = X.dtype

    p0 = torch.tensor(gps0, device=device, dtype=dtype)
    p1 = torch.tensor(gps1, device=device, dtype=dtype)

    route = p1 - p0
    route_norm = torch.linalg.norm(route) + 1e-6
    e = route / route_norm

    # 左向き法線
    n = torch.stack([-e[1], e[0]])

    # 予測点のGPS直線に沿った位置と横方向位置
    px = X - p0[0]
    py = Y - p0[1]

    s = px * e[0] + py * e[1]
    d = px * n[0] + py * n[1]

    total_cost = torch.zeros_like(X)

    for buoy in buoys:
        b = torch.tensor(buoy, device=device, dtype=dtype)

        bx = b[0] - p0[0]
        by = b[1] - p0[1]

        s_b = bx * e[0] + by * e[1]
        d_b = bx * n[0] + by * n[1]

        # ブイ周辺の前後方向だけ効かせる
        longitudinal_gate = torch.exp(-((s - s_b) ** 2) / (2.0 * longitudinal_sigma_m ** 2))

        # ブイが左側なら、さらに左側が危険
        # ブイが右側なら、さらに右側が危険
        outside_amount = torch.where(
            d_b >= 0.0,
            d - (d_b - margin_m),
            (d_b + margin_m) - d,
        )

        outside_penalty = torch.relu(outside_amount) ** 2

        total_cost = total_cost + outside_penalty * longitudinal_gate

    return weight * total_cost

def make_virtual_gate_from_path(
    gps0,
    gps1,
    half_width_m=4.0,
    s_ratio=0.5,
):
    """
    GPS pointで作った直線から、左右の仮想ブイとゲート中心を作る。

    gps0, gps1:
        MPPI座標系の2点 [x, y]

    half_width_m:
        航路中心線から左右ブイまでの距離。
        つまりブイ間隔は 2 * half_width_m。

    s_ratio:
        gps0 -> gps1 のどの位置にゲートを置くか。
        0.5なら中点。
    """
    p0 = np.asarray(gps0[:2], dtype=float)
    p1 = np.asarray(gps1[:2], dtype=float)

    route = p1 - p0
    norm = np.linalg.norm(route)

    if norm < 1.0e-6:
        center = p0.copy()
        left_buoy = center + np.array([0.0, half_width_m])
        right_buoy = center - np.array([0.0, half_width_m])
    else:
        e = route / norm

        # 進行方向に対する左向き法線
        n = np.array([-e[1], e[0]])

        center = p0 + s_ratio * route
        left_buoy = center + half_width_m * n
        right_buoy = center - half_width_m * n

    gate_center = (float(center[0]), float(center[1]))
    buoys = [
        (float(left_buoy[0]), float(left_buoy[1])),
        (float(right_buoy[0]), float(right_buoy[1])),
    ]

    return gate_center, buoys

class MPPItorch():
    def __init__(self,horizon,pred_dt,nb_sample,sigma,_lambda,umin,umax,loa,targU):
        self._device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.horizon = horizon
        self.pred_dt = pred_dt
        self.nb_sample = nb_sample
        self.sigma = sigma
        self._lambda = _lambda

        self.umin = umin
        self.umax = umax

        self._caldt = 0.1
        self._nbcalloop = int(self.pred_dt/self._caldt)
        self._caldt = self.pred_dt/self._nbcalloop

        self.ships_mppi = ship_torch(loa)
        self.targU = targU

    def prediction(self, x, y, psi, U, r, rudders, accs, seed=None, straight_time = 0):
        xs = torch.full((self.nb_sample,1),x).to(self._device)
        ys = torch.full((self.nb_sample,1),y).to(self._device)
        psis = torch.full((self.nb_sample,1),psi).to(self._device)
        speeds = torch.full((self.nb_sample,1),U).to(self._device)
        rs = torch.full((self.nb_sample,1),r).to(self._device)

        self.ships_mppi.reset(
            xs, ys, speeds, psis, rs
        )
        if seed is not None:
            torch.manual_seed(seed)

        straight_time = straight_time
        pred_rudders = torch.clip(
            torch.normal(
                mean=torch.tile(
                    torch.tensor(rudders).to(self._device), (self.nb_sample,1)
                ), std=torch.where(
                    torch.arange(rudders.shape[0])*self._caldt < straight_time,
                    torch.zeros(rudders.shape), torch.ones(rudders.shape)*self.sigma[0]
                ).to(self._device)
            ), self.umin[0], self.umax[0]
        )
        pred_rudders_offset = pred_rudders.mean(axis=-1,keepdim=True)
        # pred_rudders -= pred_rudders_offset
        pred_accs = torch.clip(
            torch.normal(
                mean=torch.tile(
                    torch.tensor(accs).to(self._device), (self.nb_sample,1)
                ), std=torch.where(
                    torch.arange(accs.shape[0])*self._caldt < straight_time,
                    torch.zeros(accs.shape), torch.ones(accs.shape)*self.sigma[1]
                ).to(self._device)
            ), self.umin[1], self.umax[1]
        )


        preds = [
            [self.ships_mppi.x,],
            [self.ships_mppi.y,],
            [self.ships_mppi.U,],
            [self.ships_mppi.psi,],
            [self.ships_mppi.r,],
            [torch.zeros_like(self.ships_mppi.r)]
        ]
        sim_time = 0
        for i in range(self.horizon):
            for _ in range(self._nbcalloop):
                sim_time += self._caldt
                self.ships_mppi.move(
                    pred_rudders[:,i:i+1],
                    pred_accs[:,i:i+1],
                    self._caldt
                )
                preds[0].append(self.ships_mppi.x)
                preds[1].append(self.ships_mppi.y)
                preds[2].append(self.ships_mppi.U)
                preds[3].append(self.ships_mppi.psi)
                preds[4].append(self.ships_mppi.r)
                preds[5].append(torch.ones_like(self.ships_mppi.r)*sim_time)

        predx   = torch.concatenate(preds[0], axis=-1)
        predy   = torch.concatenate(preds[1], axis=-1)
        predU   = torch.concatenate(preds[2], axis=-1)
        predpsi = torch.concatenate(preds[3], axis=-1)
        predr   = torch.concatenate(preds[4], axis=-1)
        predtimes = torch.concatenate(preds[5], axis=-1)

        return (
            predx, predy, predU, predpsi, predr, pred_rudders, pred_accs, predtimes
        )


    def _truncate_path_behind_own(self, own_x, own_y, path, ref_tensor):
        """
        逸脱コスト計算で自船より後ろ側の基準線を使わないため、
        pathの始点を「現在位置をpath直線に射影した点」に置き換える。

        既存の逸脱コスト計算式は変更せず、pathだけを前方区間に切り詰める。
        """
        if path is None:
            return path

        device = ref_tensor.device
        dtype = ref_tensor.dtype

        if torch.is_tensor(path):
            path_t = path.to(device=device, dtype=dtype)
        else:
            path_t = torch.tensor(path, dtype=dtype, device=device)

        if path_t.numel() < 4:
            return path

        # [x0, y0, x1, y1] 形式
        if path_t.ndim == 1:
            path_xy = path_t.reshape(-1, 2)

        # [[x, y], ...] 形式
        elif path_t.ndim >= 2 and path_t.shape[-1] >= 2:
            path_xy = path_t[..., :2].reshape(-1, 2)

        else:
            return path

        if path_xy.shape[0] < 2:
            return path

        x0 = path_xy[0, 0]
        y0 = path_xy[0, 1]
        x1 = path_xy[-1, 0]
        y1 = path_xy[-1, 1]

        dx = x1 - x0
        dy = y1 - y0
        line_len2 = dx * dx + dy * dy + 1.0e-6

        own_x_t = torch.as_tensor(own_x, dtype=dtype, device=device)
        own_y_t = torch.as_tensor(own_y, dtype=dtype, device=device)

        # 自船位置を path の始点-終点直線に射影
        own_s = ((own_x_t - x0) * dx + (own_y_t - y0) * dy) / line_len2
        own_s = torch.clamp(own_s, 0.0, 1.0)

        proj_x = x0 + own_s * dx
        proj_y = y0 + own_s * dy
        proj = torch.stack([proj_x, proj_y]).reshape(1, 2)

        # 各path点の進行方向パラメータ
        s_each = ((path_xy[:, 0] - x0) * dx + (path_xy[:, 1] - y0) * dy) / line_len2

        # 自船射影点より前方だけ残す
        keep = s_each >= own_s

        forward_xy = path_xy[keep]

        # 全部消える場合は、射影点と終点だけにする
        if forward_xy.shape[0] == 0:
            new_path = torch.cat([proj, path_xy[-1:].clone()], dim=0)
        else:
            # 先頭に射影点を追加し、後方の線を確実に消す
            new_path = torch.cat([proj, forward_xy], dim=0)

        # 重複しすぎる場合の保険
        if new_path.shape[0] < 2:
            new_path = torch.cat([proj, path_xy[-1:].clone()], dim=0)

        return new_path

    def get_opt(self, x, y, psi, U, r, rudders, accs, own_loa, other_loa, others, path, col_cost_min, col_cost_max, div_cost, speed_cost, norm_cost, buoys=None, gate_center=None, buoy_cost_weight=120.0, gate_cost_weight=3.0, seed=None, straight_time=0, debug=False,
                buoy_margin_m=1.0, buoy_longitudinal_sigma_m=8.0, ax_gains=None):
        # All new keyword defaults reproduce the previously hardcoded behavior:
        #   buoy_margin_m=1.0 / buoy_longitudinal_sigma_m=8.0 were literals in the
        #   buoy_outside_cost() call, and ax_gains=None falls back to the
        #   crm_torch module constants.

        predx, predy, predU, predpsi, predr, pred_rudders, pred_accs, predtimes = self.prediction(
            x, y, psi, U, r, rudders, accs, seed=seed, straight_time = straight_time
        )

        # 自船より後ろ側の基準pathは逸脱コストに使わない
        # 既存コスト式は変えず、pathだけを現在位置射影点から前方へ切り詰める
        path = self._truncate_path_behind_own(
            own_x=x,
            own_y=y,
            path=path,
            ref_tensor=predx,
        )

        # costs for collision risks
        if len(others) == 0:
            cost_collision = torch.zeros_like(predy)
        else:
            # MPPI/ROS座標
            # x: forward
            # y: left
            # psi: ccw deg

            own_crm = [
                -y,
                x,
                U,
                (-psi) % 360.0,
                own_loa,
            ]

            others_crm = [
                [
                    -oth.y,
                    oth.x,
                    oth.u,
                    (-oth.yaw) % 360.0,
                    other_loa,
                ]
                for oth in others
            ]

            X_crm = -predy
            Y_crm =  predx
            turn_crm = torch.deg2rad(torch.remainder(-predpsi, 360.0))

            cost_collision = crm_torch.timedomaincrm(
                X_crm,
                Y_crm,
                predtimes,
                own_crm,
                others_crm,
                turn=turn_crm,
                ax_gains=ax_gains,
            )
            print('own:', [y, x, U, psi, own_loa],)
            print('other:', others)

        # cost_collision_gain = torch.exp( -predtimes**2/(600)**2/2 )
        # cost_collision_gain = torch.where(
        #     predtimes<=predtimes.max()*0.60, torch.ones_like(predtimes), torch.zeros_like(predtimes)
        # )

        cost_collision = cost_collision.sum(axis=-1, keepdim=True)
        # cost_collision = (cost_collision_gain*cost_collision).sum(axis=-1, keepdim=True)
        # cost_collision = (cost_collision).sum(axis=-1, keepdim=True)

        # cost_collision = cost_collision/(1+torch.abs(cost_collision))
        cost_collision = torch.where(
            cost_collision>0, col_cost_min+(col_cost_max-col_cost_min)*cost_collision, cost_collision
        )

        # costs for diviation from course
        targ_u = torch.full_like(predU, self.targU)

        diviation = diviation2infinite_line(
            predx,
            predy,
            path[0][:2],
            path[1][:2],
        )

        diviation = diviation.mean(axis=-1, keepdim=True)

        # costs for difference bbetween pred u and target u
        speed_diff = torch.square( targ_u - predU )
        speed_diff = speed_diff.mean(axis=-1, keepdim=True)

        # ------------------------------------------------------------
        # Buoy / gate costs
        # path[0], path[1] はMPPI座標系でのGPS直線として使う
        # buoys は同じMPPI座標系のブイ座標リスト
        # ------------------------------------------------------------
        if buoys is None:
            buoys = []

        cost_buoy_map = buoy_outside_cost(
            X=predx,
            Y=predy,
            gps0=path[0][:2],
            gps1=path[1][:2],
            buoys=buoys,
            margin_m=buoy_margin_m,
            longitudinal_sigma_m=buoy_longitudinal_sigma_m,
            weight=buoy_cost_weight,
        )

        # cost_buoy_map は [sample, time] なので、他のコストと同じ [sample, 1] にする
        cost_buoy = cost_buoy_map.mean(axis=-1, keepdim=True)

        if gate_center is not None:
            gate_x = torch.as_tensor(gate_center[0], device=predx.device, dtype=predx.dtype)
            gate_y = torch.as_tensor(gate_center[1], device=predy.device, dtype=predy.dtype)

            # 予測軌道のどこか一箇所でもゲート中心に近づけばよい
            gate_dist2 = (predx - gate_x) ** 2 + (predy - gate_y) ** 2
            cost_gate = gate_cost_weight * torch.min(gate_dist2, dim=-1, keepdim=True).values
        else:
            cost_gate = torch.zeros_like(cost_buoy)

        if torch.isnan(cost_collision).any():
            print('[MPPI] cost計算が発散しています. cost for collision',)
        if torch.isnan(diviation).any():
            print('[MPPI] cost計算が発散しています. cost for div',)
        if torch.isnan(speed_diff).any():
            print('[MPPI] cost計算が発散しています. cost for speed diff',)

        without_collision = False
        if without_collision:
            costs = (
                div_cost*diviation
                + speed_cost*speed_diff
                + cost_buoy
                + cost_gate
            )
            # norm costs
            norm_targ = ['input', 'min'][1]
            if norm_targ == 'input':
                rudders_norm = torch.tensor(rudders).to(self._device)
                accs_norm = torch.tensor(accs).to(self._device)
            elif norm_targ == 'min':
                min_idx = torch.argmin(costs.flatten())
                rudders_norm = pred_rudders[min_idx]
                accs_norm = pred_accs[min_idx]


            costs_norm = (
                norm_cost*(torch.square(pred_rudders-rudders_norm)/self.umax[0]).mean(axis=-1, keepdim=True)
                + norm_cost*(torch.square(pred_accs-accs_norm)/self.umax[1]).mean(axis=-1, keepdim=True)
            )

            # costs_norm is added to costs
            costs = costs + costs_norm

            # calculate weights for trajectories
            cost_collision_min = cost_collision.min() #.values
            cost_collision_max = cost_collision.max() #.values
            th = 0
            w = torch.where(
                cost_collision<=th,
                torch.exp(-1/self._lambda*costs),
                torch.zeros_like(costs)
            )
            if w.sum()>0:
                w = w/w.sum()
            else:
                w = torch.where(
                    cost_collision<=cost_collision_min+(cost_collision_max-cost_collision_min)*0.1,
                    torch.exp(-1/self._lambda*costs),
                    torch.zeros_like(costs)
                )

        else:
            # calculate total costs
            costs = (
                cost_collision
                + div_cost*diviation
                + speed_cost*speed_diff
                + cost_buoy
                + cost_gate
            )
            # norm costs
            norm_targ = ['input', 'min'][0]
            if norm_targ == 'input':
                rudders_norm = torch.tensor(rudders).to(self._device)
                accs_norm = torch.tensor(accs).to(self._device)
            elif norm_targ == 'min':
                min_idx = torch.argmin(costs.flatten())
                rudders_norm = pred_rudders[min_idx]
                accs_norm = pred_accs[min_idx]


            costs_norm = (
                norm_cost*(torch.square(pred_rudders-rudders_norm)/self.umax[0]).mean(axis=-1, keepdim=True)
                + norm_cost*(torch.square(pred_accs-accs_norm)/self.umax[1]).mean(axis=-1, keepdim=True)
            )

            # costs_norm is added to costs
            costs = costs + costs_norm

            # calculate weights for trajectories
            w = torch.exp(-1/self._lambda*costs)
            w = w/w.sum()

        # # calculate optimal input
        # opt_rudder = (pred_rudders*w).sum(axis=0)
        # opt_acc = (pred_accs*w).sum(axis=0)
        opt_rudder = pred_rudders[torch.argmax(w.flatten())]
        opt_acc = pred_accs[torch.argmax(w.flatten())]

        if debug:
            return (
                predx.cpu().numpy().copy(),
                predy.cpu().numpy().copy(),
                predU.cpu().numpy().copy(),
                predpsi.cpu().numpy().copy(),
                predr.cpu().numpy().copy(),
                pred_rudders.cpu().numpy().copy(),
                pred_accs.cpu().numpy().copy(),
                costs.cpu().numpy().copy(),
                w.cpu().numpy().copy(),
                opt_rudder.cpu().numpy().copy(),
                opt_acc.cpu().numpy().copy(),
                cost_collision.cpu().numpy().copy(),
            )
        else:
            return (
                opt_rudder.cpu().numpy().copy(),
                opt_acc.cpu().numpy().copy(),
            )


class MPPIPlanner():
    """
    自前の経路生成アルゴリズム本体．

    ここではすべてbase_link基準で扱う．

    入力：
        own_state  : base_link基準の自船状態
        other_state: base_link基準の他船状態
        goal       : base_link基準のゴール位置

    出力：
        points_base = [(x1, y1), (x2, y2), ...]
    """

    def __init__(
        self,
        point_spacing: float = 0.5,
        avoid_radius: float = 2.0,
        avoid_offset: float = 3.0,
        # ------------------------------------------------------------
        # MPPI core hyperparameters.
        # Every default below is the literal that used to be hardcoded in
        # this constructor / in get_traj / in the get_opt call, so a no-arg
        # MPPIPlanner() is behaviorally identical to the previous version.
        # ------------------------------------------------------------
        horizon: int = 225,
        dt: float = 0.1,                       # was: simdt = 0.1 (pred_dt)
        num_samples: int = 5000,               # was: nb_sample = 5000
        lambda_: float = 12.0,                 # was: _lambda = 12.0
        control_noise_sigma=(35.0, 0.0),       # was: sigma = [35, 0.0] (rudder deg, acc m/s^2)
        target_speed: float = 2.0 * 1852.0 / 3600.0,  # was: targU = 2 kn
        # --- cost weights (names on the left are the historical attribute names) ---
        path_cost_weight: float = 150.0,       # was: self.div_cost
        speed_cost_weight: float = 0.1,        # was: self.speed_cost
        control_cost_weight: float = 0.2,      # was: self.norm_cost
        collision_cost_min: float = 10.0,      # was: self.col_cost_min
        collision_cost_max: float = 50.0,      # was: self.col_cost_max
        buoy_cost_weight: float = 120.0,       # was: literal in get_opt call
        gate_cost_weight: float = 3.0,         # was: literal in get_opt call
        # --- safe distances / geometry ---
        own_loa_m: float = 1.2,
        other_loa_m: float = 2.0,
        gate_half_width_m: float = 4.0,        # was: literal in make_virtual_gate_from_path call
        buoy_margin_m: float = 1.0,            # was: margin_m literal in buoy_outside_cost call
        buoy_longitudinal_sigma_m: float = 8.0,  # was: longitudinal_sigma_m literal
        # CRM bumper safe distances (multiples of LOA).
        # Defaults mirror crm_torch.{RIGHT,LEFT,FORE,AFT}_AX_GAIN.
        safe_distance_right_loa: float = 3.2,
        safe_distance_left_loa: float = 1.6,
        safe_distance_fore_loa: float = 6.4,
        safe_distance_aft_loa: float = 1.6,
    ):
        self.point_spacing = point_spacing
        self.avoid_radius = avoid_radius
        self.avoid_offset = avoid_offset

        # CRM uses a length-overall entry for each vessel.  Keep them separate:
        # the own vessel is 1.2 m and the Task 2 opponent is modelled as 2.0 m.
        self.own_loa_m = own_loa_m
        self.other_loa_m = other_loa_m
        self.target_speed = target_speed

        self.speed_cost = speed_cost_weight
        self.div_cost = path_cost_weight
        self.norm_cost = control_cost_weight
        self.col_cost_min = collision_cost_min
        self.col_cost_max = collision_cost_max
        self.buoy_cost_weight = buoy_cost_weight
        self.gate_cost_weight = gate_cost_weight

        self.gate_half_width_m = gate_half_width_m
        self.buoy_margin_m = buoy_margin_m
        self.buoy_longitudinal_sigma_m = buoy_longitudinal_sigma_m

        self.ax_gains = (
            safe_distance_right_loa,
            safe_distance_left_loa,
            safe_distance_fore_loa,
            safe_distance_aft_loa,
        )

        self.path_spacing_m = 2.0 #waypoint 分割距離[m]
        simdt = dt

        self.mppi_controller = MPPItorch(
            horizon = horizon,
            pred_dt = simdt,
            # nb_sample = 10000,
            nb_sample = num_samples,
            # sigma = [20, 0.10],
            sigma = list(control_noise_sigma),
            # _lambda = 0.1,
            _lambda = lambda_,
            umin = [-20,-0.1],
            umax = [ 20, 0.1],
            loa = self.own_loa_m,
            targU = target_speed,
        )

        self.opt_rudder = np.zeros((self.mppi_controller.horizon,))
        self.opt_acc = np.zeros((self.mppi_controller.horizon,))

    def plan(
        self,
        own: VesselState,
        other: Optional[VesselState],
        waypoint1: Tuple[float, float],
        waypoint2: Tuple[float, float],
        detected_buoys: Optional[List[Tuple[float, float]]] = None,
        use_virtual_buoys: bool = False,
    ) -> List[Tuple[float, float]]:

        others = [] if other is None else [other]  # 他船がなければ衝突コストなし
        path = [waypoint1, waypoint2]

        # Normally, use measured buoy positions.  The historical GPS-derived
        # virtual gate remains available only as an explicit fallback.
        gate_center = None
        if use_virtual_buoys:
            gate_center, buoys = make_virtual_gate_from_path(
                waypoint1,
                waypoint2,
                half_width_m=self.gate_half_width_m,
                s_ratio=0.5,
            )
        else:
            buoys = detected_buoys or []

        (
            predx,
            predy,
            predU,
            predpsi,
            predr,
            pred_rudders,
            pred_accs,
            costs,
            w,
            self.opt_rudder,
            self.opt_acc,
            _,

        ) = self.mppi_controller.get_opt(
            own.x, own.y, own.yaw, own.u, own.r, self.opt_rudder, self.opt_acc,
            self.own_loa_m, self.other_loa_m, others, path,
            col_cost_min=self.col_cost_min,
            col_cost_max=self.col_cost_max,
            div_cost=self.div_cost,
            speed_cost=self.speed_cost,
            norm_cost=self.norm_cost,
            buoys=buoys,
            gate_center=gate_center,
            buoy_cost_weight=self.buoy_cost_weight,
            gate_cost_weight=self.gate_cost_weight,
            buoy_margin_m=self.buoy_margin_m,
            buoy_longitudinal_sigma_m=self.buoy_longitudinal_sigma_m,
            ax_gains=self.ax_gains,
            debug=True
        )

        opttraj = get_traj(
            own.x, own.y, own.yaw, own.u, own.r, self.opt_rudder, self.opt_acc, self.own_loa_m, self.mppi_controller.pred_dt,
            target_speed_mps=self.target_speed,
        )

        points_base = resample_traj_by_distance(
            opttraj=opttraj,
            spacing_m=self.path_spacing_m,
        )

        return points_base
