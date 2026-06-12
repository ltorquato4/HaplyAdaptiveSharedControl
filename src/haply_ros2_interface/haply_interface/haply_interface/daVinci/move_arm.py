#!/usr/bin/env python3
import sys
import time
import math
import argparse
import numpy as np
import PyKDL
import crtk

class ArmOps:
    def __init__(self, ral, arm_name, connection_timeout=10.0):
        self._ral = ral.create_child(arm_name)
        u = crtk.utils(self, self._ral, connection_timeout)
        u.add_operating_state()   # enable(timeout), home(timeout)
        u.add_setpoint_js()       # setpoint_jp()
        u.add_move_jp()           # move_jp(goal)
        u.add_servo_jp()          # servo_jp(pos[, vel])
        u.add_setpoint_cp()       # setpoint_cp()
        u.add_servo_cp()          # servo_cp(goal)

    def ral(self):
        return self._ral


class App:
    def __init__(self, ral, arm_name, force=False,
                 mode='joint', axis='x', amp=0.02, period=5.0, rate_hz=100.0):
        self.ral = ral
        self.arm = ArmOps(ral, arm_name)
        self.force = force
        self.mode = mode.lower()
        self.axis = axis.lower()
        self.amp = float(amp)
        self.period = float(period)
        self.rate_hz = float(rate_hz)

        if self.mode not in ('joint', 'cart'):
            raise ValueError("mode must be 'joint' or 'cart'")
        if self.axis not in ('x', 'y', 'z'):
            raise ValueError("axis must be one of: x, y, z")

    def _prepare_cartesian(self):
        ts = 0.0
        while ts == 0.0:
            jp, ts = self.arm.setpoint_jp()
            time.sleep(0.02)

        goal = np.copy(jp)
        # PSM: [yaw, pitch, insertion, roll, wrist_pitch, wrist_yaw]
        goal[0] = 0.0
        goal[1] = 0.0
        goal[2] = 0.12
        goal[3] = 0.0
        print("> moving to cartesian start")
        self.arm.move_jp(goal).wait()
        print("< ready for cartesian motion")

    def _oneshot_move_jp_test(self, delta=0.01):
        """Egylépéses ízületi teszt az insertion tengelyen (±delta méter)."""
        jp, ts = self.arm.setpoint_jp()
        while ts == 0.0:
            jp, ts = self.arm.setpoint_jp()
            time.sleep(0.02)
        start = np.copy(jp)

        goal = np.copy(start)
        goal[2] = start[2] + delta
        print(f"> move_jp sanity check: insertion +{delta:.3f} m")
        self.arm.move_jp(goal).wait()
        goal[2] = start[2]
        self.arm.move_jp(goal).wait()
        print("< move_jp sanity ok")

    def _servo_jp_sine(self):
        jp, ts = self.arm.setpoint_jp()
        while ts == 0.0:
            jp, ts = self.arm.setpoint_jp()
            time.sleep(0.02)
        center = np.copy(jp)

        omega = 2.0 * math.pi / self.period
        rate = self.arm.ral().create_rate(self.rate_hz)
        print(f"> servo_jp: joint=insertion, amp={self.amp:.3f} m, period={self.period:.2f}s, rate={self.rate_hz:.1f}Hz")

        t0 = self.arm.ral().to_sec(self.arm.ral().now())
        while True:
            t = self.arm.ral().to_sec(self.arm.ral().now()) - t0
            offset = self.amp * math.sin(omega * t)

            goal_p = np.copy(center)
            goal_p[2] = center[2] + offset  # insertion
            # opcionálisan adhatnánk sebességet is (goal_v), de nem szükséges
            self.arm.servo_jp(goal_p)
            rate.sleep()

    def _servo_cp_line(self):
        cp, ts = self.arm.setpoint_cp()
        while ts == 0.0:
            cp, ts = self.arm.setpoint_cp()
            time.sleep(0.01)

        center = PyKDL.Frame(cp)  # megtartjuk pozíciót/orientációt
        axis_idx = {'x': 0, 'y': 1, 'z': 2}[self.axis]
        omega = 2.0 * math.pi / self.period
        # csökkentett frekvencia, hogy ne telítsük a sorokat
        rate = self.arm.ral().create_rate(self.rate_hz)

        print(f"> servo_cp line: axis={self.axis}, amp={self.amp:.3f} m, period={self.period:.2f}s, rate={self.rate_hz:.1f}Hz")
        t0 = self.arm.ral().to_sec(self.arm.ral().now())

        while True:
            t = self.arm.ral().to_sec(self.arm.ral().now()) - t0
            offset = self.amp * math.sin(omega * t)

            goal = PyKDL.Frame(center)
            if axis_idx == 0:
                goal.p[0] = center.p[0] + offset
            elif axis_idx == 1:
                goal.p[1] = center.p[1] + offset
            else:
                goal.p[2] = center.p[2] + offset

            self.arm.servo_cp(goal)
            rate.sleep()

    def run(self):
        # 1) kapcsolatok
        self.ral.check_connections()

        # 2) opcionális force disable
        if self.force and hasattr(self.arm, 'disable'):
            print("> forcing: disable")
            self.arm.disable(5)

        # 3) enable + home
        print("> starting enable")
        if not self.arm.enable(10):
            raise RuntimeError("failed to enable within 10 seconds")

        print("> starting home")
        if not self.arm.home(10):
            raise RuntimeError("failed to home within 10 seconds")

        print("< system homed and tool engaged")

        # 4) sanity: egyshot move_jp (±1 cm default)
        self._oneshot_move_jp_test(delta=min(0.01, self.amp))

        # 5) mozgás üzemmód szerint
        if self.mode == 'joint':
            self._servo_jp_sine()
        else:
            self._prepare_cartesian()
            self._servo_cp_line()


def main():
    argv = crtk.ral.parse_argv(sys.argv[1:])

    parser = argparse.ArgumentParser()
    parser.add_argument("-a", "--arm", type=str, default="PSM1",
                        choices=["ECM","MTML","MTMR","PSM1","PSM2","PSM3"])
    parser.add_argument("--force", action="store_true",
                        help="Force transitions (disable -> enable -> home)")
    parser.add_argument("--mode", type=str, default="joint", choices=["joint","cart"],
                        help="Mozgás típusa: joint vagy cart")
    parser.add_argument("--axis", type=str, default="x", choices=["x","y","z"],
                        help="Cart módban a mozgás tengelye")
    parser.add_argument("--amp", type=float, default=0.02,
                        help="Amplitúdó (m) – középhez képest ±amp")
    parser.add_argument("--period", type=float, default=5.0,
                        help="Periódus (s)")
    parser.add_argument("--rate", type=float, default=100.0,
                        help="Vezérlési frekvencia (Hz)")
    args = parser.parse_args(argv)

    ral = crtk.ral("daVinci_init")
    app = App(ral, args.arm, force=args.force,
              mode=args.mode, axis=args.axis, amp=args.amp,
              period=args.period, rate_hz=args.rate)

    try:
        ral.spin_and_execute(app.run)
    except KeyboardInterrupt:
        pass
    finally:
        ral.shutdown()
        print(">> clean shutdown")


if __name__ == "__main__":
    main()
