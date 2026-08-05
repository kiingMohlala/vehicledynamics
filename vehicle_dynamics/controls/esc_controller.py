"""ESC: understeer / oversteer correction via individual brakes + torque cut."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .sensor_model import SensorReading


@dataclass
class ESCController:
    enabled: bool = True
    kp_yaw: float = 800.0          # N·m / (rad/s) → brake bias proxy
    yaw_deadband: float = 0.05
    max_brake: float = 0.6
    understeer_gain: float = 0.4
    oversteer_gain: float = 0.5
    # Vehicle params for reference model
    L: float = 2.7
    understeer_grad: float = 0.03  # rad / (m/s^2) simplified

    def reference_yaw(self, vx: float, delta: float) -> float:
        vx = max(abs(vx), 0.5)
        # Bicycle steady-state approx: r = vx * delta / (L + Kus*vx^2)
        return vx * delta / (self.L + self.understeer_grad * vx * vx)

    def step(
        self,
        sensors: SensorReading,
        dt: float,
    ) -> tuple[np.ndarray, float, float, bool]:
        """
        Returns (brake_add[4], torque_limit_scale, tv_request, active).
        Brake indices: 0 FL, 1 FR, 2 RL, 3 RR.
        """
        brakes = np.zeros(4)
        if not self.enabled or abs(sensors.vx) < 5.0:
            return brakes, 1.0, 0.0, False

        r_ref = self.reference_yaw(sensors.vx, sensors.steer)
        e = sensors.yaw_rate - r_ref
        if abs(e) < self.yaw_deadband:
            return brakes, 1.0, 0.0, False

        active = True
        tq_lim = 1.0
        tv = 0.0

        if e * np.sign(sensors.steer + 1e-9) > 0:
            # Oversteer: yaw exceeds reference in steer direction → brake outer front
            g = self.oversteer_gain
            mag = float(np.clip(abs(e) * g, 0.0, self.max_brake))
            if sensors.steer >= 0:  # left turn, oversteer → brake FR
                brakes[1] = mag
            else:
                brakes[0] = mag
            tq_lim = float(np.clip(1.0 - 0.5 * mag, 0.4, 1.0))
            tv = -np.sign(e) * abs(e) * 50.0  # stabilize
        else:
            # Understeer: yaw less than reference → brake inner rear
            g = self.understeer_gain
            mag = float(np.clip(abs(e) * g, 0.0, self.max_brake))
            if sensors.steer >= 0:
                brakes[2] = mag  # inner rear left
            else:
                brakes[3] = mag
            tv = np.sign(sensors.steer) * abs(e) * 40.0

        return brakes, tq_lim, float(tv), active
