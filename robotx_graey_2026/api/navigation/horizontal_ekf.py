"""Jetson horizontal EKF: fixed-local x/y, velocity and relative yaw.

GPS position and body-frame DVL velocity update independently. VN supplies
relative yaw and yaw rate. This is not a replacement for Cube attitude control.
No Cube estimate is fed back as a measurement. Covariance uses the Joseph form.
"""
import math
import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class HorizontalEKF:
    def __init__(self):
        self.x = np.zeros(5)
        self.P = np.diag([9., 9., 1., 1., math.pi**2])
        self.time = None
        self.accepted = {name: None for name in ('gps', 'dvl', 'vn')}
        self.innovations = {}
        self.rate = 0.
        self.started = False

    def predict(self, now):
        if self.time is None:
            self.time = now
            return
        dt = now - self.time
        if dt < 0 or not math.isfinite(dt):
            raise ValueError('Estimator clock regressed')
        self.time = now
        # Do not integrate a stale angular-rate measurement indefinitely.
        rate = self.rate if self.accepted['vn'] is not None and now-self.accepted['vn'] < .5 else 0.
        self.x[:2] += self.x[2:4] * dt
        self.x[4] = wrap(self.x[4] + rate * dt)
        F = np.eye(5); F[0, 2] = F[1, 3] = dt
        Q = np.diag([.01*dt, .01*dt, .09*dt, .09*dt, math.radians(2)**2*dt])
        self.P = F @ self.P @ F.T + Q

    def update(self, name, now, residual, H, R, gate=25.):
        S = H @ self.P @ H.T + R
        nis = float(residual @ np.linalg.solve(S, residual))
        self.innovations[name] = nis
        if not math.isfinite(nis) or nis > gate:
            return False
        K = np.linalg.solve(S, H @ self.P).T
        self.x += K @ residual
        self.x[4] = wrap(self.x[4])
        A = np.eye(5)-K @ H
        self.P = A @ self.P @ A.T + K @ R @ K.T
        self.P = (self.P+self.P.T)/2
        self.accepted[name] = now
        return True

    def vn(self, now, heading, rate):
        self.predict(now)
        if not self.started:
            self.x[4] = heading
            self.started = True
        H = np.zeros((1, 5)); H[0, 4] = 1.
        good = self.update('vn', now, np.array([wrap(heading-self.x[4])]), H,
                           np.array([[math.radians(3)**2]]))
        if good: self.rate = rate
        return good

    def dvl(self, now, forward, right):
        self.predict(now)
        c, s = math.cos(self.x[4]), math.sin(self.x[4])
        vx, vy = self.x[2:4]
        predicted = np.array([c*vx+s*vy, -s*vx+c*vy])
        H = np.zeros((2, 5))
        H[0, 2:] = [c, s, -s*vx+c*vy]
        H[1, 2:] = [-s, c, -c*vx-s*vy]
        return self.update('dvl', now, np.array([forward, right])-predicted, H, np.eye(2)*.03**2)

    def gps(self, now, xy, accuracy):
        self.predict(now)
        H = np.zeros((2, 5)); H[0, 0] = H[1, 1] = 1.
        variance=max(.5,accuracy)**2
        good=self.update('gps', now, np.array(xy)-self.x[:2], H, np.eye(2)*variance)
        if good:
            # Repeated GNSS fixes share bias. Do not claim centimetric absolute
            # certainty just because many correlated metre-level fixes arrived.
            for i in (0,1):self.P[i,i]=max(self.P[i,i],variance)
        return good

    def health(self, now):
        ages = {k: None if t is None else max(0., now-t) for k, t in self.accepted.items()}
        used = {k: age is not None and age <= 2. for k, age in ages.items()}
        finite = bool(np.isfinite(self.x).all() and np.isfinite(self.P).all())
        count = sum(used.values())
        return dict(state='INITIALIZING' if not self.started else 'LOST' if not count or not finite else 'AIDED' if count == 3 else 'DEGRADED',
                    usable=bool(self.started and finite and count), all_sensors_lost=not count,
                    sensors=used, ages_s=ages, innovations=dict(self.innovations),
                    position_sigma_m=float(math.sqrt(max(self.P[0,0], self.P[1,1]))),
                    yaw_sigma_deg=float(math.degrees(math.sqrt(self.P[4,4]))))
