#!/usr/bin/env python3
"""
Hold LB + X for 0.5 seconds to launch demo_walk.py.
After demo_walk.py exits, LB+Y can trigger again.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
import subprocess
import time

DEPLOY_DIR        = "/userdata/ZEROTH_MINI_DEPLOY"
MODEL_PATH        = "model/model.onnx"              # for demo_walk.py
MODEL_PATH_ESTIMATOR = "model/model_estimator.onnx" # for demo_walk_estimator.py
HOLD_SECONDS      = 0.5
LB_BUTTON_INDEX   = 4      # Left Bumper
X_BUTTON_INDEX    = 3      # X button
USE_ESTIMATOR     = False   # True: run demo_walk_estimator.py, False: run demo_walk.py


class JoyLauncher(Node):
    def __init__(self):
        super().__init__('joy_launcher')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Joy, 'joy_filter_bt', self._joy_cb, qos)
        self._press_start = None
        self._proc        = None
        script_name = "demo_walk_estimator.py" if USE_ESTIMATOR else "demo_walk.py"
        self.get_logger().info(
            f"JoyLauncher ready — hold LB+X {HOLD_SECONDS:.0f}s to launch {script_name}"
        )

    def _joy_cb(self, msg):
        # if demo_walk.py/demo_walk_estimator.py has exited, reset and allow re-launch
        if self._proc is not None and self._proc.poll() is not None:
            script_name = "demo_walk_estimator.py" if USE_ESTIMATOR else "demo_walk.py"
            self.get_logger().info(f"{script_name} exited, ready to launch again.")
            self._proc        = None
            self._press_start = None

        if self._proc is not None:
            return   # script still running, ignore triggers

        lb = msg.buttons[LB_BUTTON_INDEX]
        x  = msg.buttons[X_BUTTON_INDEX]
        both_held = lb and x

        now = time.time()
        if both_held:
            if self._press_start is None:
                self._press_start = now
                self.get_logger().info("LB+X detected — keep holding to launch...")
            elif now - self._press_start >= HOLD_SECONDS:
                self._launch()
        else:
            if self._press_start is not None:
                self.get_logger().info(f"Released before {HOLD_SECONDS}s, cancelled.")
            self._press_start = None

    def _launch(self):
        self._press_start = None
        script_name = "demo_walk_estimator.py" if USE_ESTIMATOR else "demo_walk.py"
        model_path = MODEL_PATH_ESTIMATOR if USE_ESTIMATOR else MODEL_PATH
        self.get_logger().info(f"Launching {script_name} with model {model_path}...")
        self._proc = subprocess.Popen(
            ["python3", script_name, f"--model={model_path}"],
            cwd=DEPLOY_DIR
        )
        self.get_logger().info(
            f"{script_name} launched (pid={self._proc.pid}). "
            f"After it exits, hold LB+X again to relaunch."
        )


def main():
    rclpy.init()
    node = JoyLauncher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
