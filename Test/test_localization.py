import math
import os
import sys
import importlib.util
import pytest
import rclpy

from ackermann_msgs.msg import AckermannDrive
from mocap4r2_msgs.msg import RigidBodies, RigidBody
from geometry_msgs.msg import Quaternion
from builtin_interfaces.msg import Time as RosTime



def _import_localization_module():
  

    pkg_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    candidates = [
        os.path.join(pkg_root, "localization", "localization_node.py"),
        os.path.join(pkg_root, "localization", "localization.py"),
        os.path.join(pkg_root, "localization", "node.py"),
    ]

    node_path = None
    for c in candidates:
        if os.path.isfile(c):
            node_path = c
            break

    if node_path is None:
        raise FileNotFoundError(
           
        )

    spec = importlib.util.spec_from_file_location("localization_node_under_test", node_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module, node_path


NODE_MOD, NODE_FILEPATH = _import_localization_module()


def _load_localization_class(mod):

    for cls_name in ("Localization", "LocalizationKF", "LocalizationNode"):
        cls = getattr(mod, cls_name, None)
        if cls is not None:
            return cls
    raise ImportError(
    )


LocalizationClass = _load_localization_class(NODE_MOD)


# ---------------- FIXTURES ----------------
@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(rclpy_init):
    n = LocalizationClass()
    yield n
    n.destroy_node()


# ---------------- HELPERS ----------------
def make_mocap_msg(x, y, yaw, rb_id="4", stamp_sec=1.0):
    msg = RigidBodies()

    t = RosTime()
    t.sec = int(stamp_sec)
    t.nanosec = int((stamp_sec % 1.0) * 1e9)
    msg.header.stamp = t

    rb = RigidBody()
    rb.rigid_body_name = rb_id
    rb.pose.position.x = float(x)
    rb.pose.position.y = float(y)
    rb.pose.position.z = 0.0
    rb.pose.orientation = Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )

    msg.rigidbodies.append(rb)
    return msg


# ==========================================================
# AC 3.27.1: Subscribe to ackermann_drive_feedback 
# ==========================================================
def test_ut_loc_327_01_cb_speed_updates_last_speed(node):
    msg = AckermannDrive()
    msg.speed = 2.0
    node.cb_speed(msg)
    assert node.last_speed == 2.0


# ==========================================================
# AC 3.27.2: use wheel encoder speed
# ==========================================================
def test_ut_loc_327_02_speed_to_velocity_conversion(node):
    msg_init = make_mocap_msg(0.0, 0.0, 0.0, stamp_sec=1.00)
    node.cb_mocap(msg_init)

    node.last_speed = 1.0
    node.kf.x[6, 0] = 0.0  # yaw=0

    msg_frozen = make_mocap_msg(0.0, 0.0, 0.0, stamp_sec=1.05)
    node.cb_mocap(msg_frozen)

    assert abs(float(node.kf.x[2, 0]) - 1.0) < 0.01
    assert abs(float(node.kf.x[3, 0])) < 0.01


# ==========================================================
# AC 3.27.3: switch to dead reckoning when optitrack is unavailable
# ==========================================================
def test_ut_loc_327_03_dead_reckoning_updates_pose(node):
    msg_init = make_mocap_msg(1.0, 1.0, 0.0, stamp_sec=2.00)
    node.cb_mocap(msg_init)

    node.last_speed = 1.0
    px_before = float(node.last_px)

    msg_frozen = make_mocap_msg(1.0, 1.0, 0.0, stamp_sec=2.05)
    node.cb_mocap(msg_frozen)

    assert float(node.last_px) > px_before


# ==========================================================
# AC 3.27.4: estimation shall not jump >0.20m
# ==========================================================
def test_ut_loc_327_04_switching_no_large_position_jump(node):
    msg_init = make_mocap_msg(0.0, 0.0, 0.0, stamp_sec=3.00)
    node.cb_mocap(msg_init)

    node.last_speed = 1.0
    msg_frozen = make_mocap_msg(0.0, 0.0, 0.0, stamp_sec=3.05)
    node.cb_mocap(msg_frozen)

    opt_x, opt_y = 0.15, 0.0
    msg_resume = make_mocap_msg(opt_x, opt_y, 0.0, stamp_sec=3.10)
    node.cb_mocap(msg_resume)

    dx = abs(float(node.kf.x[0, 0]) - opt_x)
    dy = abs(float(node.kf.x[1, 0]) - opt_y)
    assert math.sqrt(dx * dx + dy * dy) <= 0.20


# ==========================================================
# AC 3.27.5: publish continous pose even when optitrack is unavailable
# ==========================================================
def test_ut_loc_327_05_continuous_pose_without_mocap(node):
    msg_init = make_mocap_msg(2.0, 2.0, 0.0, stamp_sec=4.00)
    node.cb_mocap(msg_init)

    node.last_speed = 1.0
    for i in range(5):
        msg_frozen = make_mocap_msg(2.0, 2.0, 0.0, stamp_sec=4.05 + i * 0.05)
        node.cb_mocap(msg_frozen)

    assert node.last_px is not None
    assert node.last_py is not None


# ==========================================================
# AC 3.27.2: estimate pose when speed is low and optitrack is unavailable
# ==========================================================
def test_ut_loc_327_06_no_dead_reckoning_when_speed_low(node):
    msg_init = make_mocap_msg(5.0, 5.0, 0.0, stamp_sec=5.00)
    node.cb_mocap(msg_init)

    node.last_speed = 0.05  # below speed_eps (0.12)
    px_before = float(node.last_px)
    py_before = float(node.last_py)

    msg_frozen = make_mocap_msg(5.0, 5.0, 0.0, stamp_sec=5.05)
    node.cb_mocap(msg_frozen)

    assert float(node.last_px) == px_before
    assert float(node.last_py) == py_before
