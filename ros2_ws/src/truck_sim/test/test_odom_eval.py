import math

import numpy as np

from truck_sim.odom_eval import ate


def test_ate_ignores_frame_rotation_and_offset():
    t = np.linspace(0, 2 * math.pi, 50)
    gt = np.c_[np.cos(t), np.sin(t)]
    a = math.radians(90)
    rot = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    odom = gt @ rot.T + [3.0, -1.0]          # same path, different start frame
    rms, mx, final = ate(odom, gt)
    assert rms < 1e-9 and mx < 1e-9


def test_ate_measures_real_distortion():
    # 10 % scale error on a 1 m-radius loop can't be explained by a rigid transform
    t = np.linspace(0, 2 * math.pi, 100)
    gt = np.c_[np.cos(t), np.sin(t)]
    rms, mx, final = ate(gt * 1.1, gt)
    assert 0.05 < rms < 0.15 and mx > 0.05
