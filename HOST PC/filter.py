# filter.py
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints

def quat_multiply(q, r):
    q0, q1, q2, q3 = q
    r0, r1, r2, r3 = r
    return np.array([
        q0*r0 - q1*r1 - q2*r2 - q3*r3,
        q0*r1 + q1*r0 + q2*r3 - q3*r2,
        q0*r2 - q1*r3 + q2*r0 + q3*r1,
        q0*r3 + q1*r2 - q2*r1 + q3*r0
    ])

def quat_normalize(q):
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([1., 0., 0., 0.])

def rotate_vector(q, v):
    w, x, y, z = q
    R = np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),     1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),     2*(y*z+x*w),   1-2*(x*x+y*y)]
    ])
    return R.dot(v)

def f_x(x, dt):
    q = x[0:4]; omega = x[4:7]
    dq = 0.5 * quat_multiply(q, np.hstack(([0.], omega))) * dt
    q_new = quat_normalize(q + dq)
    return np.hstack((q_new, x[4:]))

def h_x(x):
    return x

class IMUFilter:
    def __init__(self, dt=0.005):
        dim = 10
        pts = MerweScaledSigmaPoints(n=dim, alpha=0.1, beta=2., kappa=0)
        self.ukf = UnscentedKalmanFilter(
            dim_x=dim, dim_z=dim,
            fx=f_x, hx=h_x, dt=dt, points=pts
        )
        self.ukf.x = np.array([1,0,0,0, 0,0,0, 0,0,0])
        self.ukf.P *= 0.1
        self.ukf.Q = np.eye(dim) * 0.05
        self.ukf.R = np.eye(dim) * 0.1
        self.dt = dt
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)

    def update(self, z):
        """输入 10 维测量向量 z，返回滤波后状态和积分位置"""
        self.ukf.predict()
        self.ukf.update(z)
        x = self.ukf.x
        a_world = rotate_vector(x[0:4], x[7:10])
        # 简单阈值静止检测
        if np.linalg.norm(a_world) > 0.2:
            self.vel += a_world * self.dt
            self.pos += self.vel * self.dt
        return x, self.pos
