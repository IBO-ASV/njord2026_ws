from dataclasses import dataclass


@dataclass
class VesselState:
    """
    船舶状態．

    planning内部ではbase_link基準として扱う．

    x, y:
        base_link基準の位置 [m]

    yaw:
        base_link基準の方位 [deg]

    u, v:
        船体座標系速度 [m/s]

    r:
        回頭角速度 [deg/s]

    vx, vy:
        base_link基準で表した速度 [m/s]
    """

    x: float
    y: float
    yaw: float

    u: float
    v: float
    r: float

    vx: float
    vy: float