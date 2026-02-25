import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Quaternion, AccelStamped
from nav_msgs.msg import Odometry
from mocap4r2_msgs.msg import RigidBodies
from ackermann_msgs.msg import AckermannDrive
from filterpy.kalman import KalmanFilter

# ---------------- UTILS ----------------
# Wrap angle to [-pi, pi]
# Prevents discontinuities in yaw when crossing ±pi
# Avoids sudden jumps when switching localization modes
def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

# Extract yaw from quaternion
def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

# Convert yaw → quaternion
def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))

# ---------------- KF NODE ----------------
class LocalizationKF(Node):
    def __init__(self):
        # Initialize ROS node
        super().__init__("localization")

        # ID of the ego rigid body (vehicle) in OptiTrack
        self.rigid_body_id = int(self.declare_parameter("rigid_body_id", 4).value)

        # Default time step if timestamp difference unavailable
        self.dt_default = 0.05

        # Thresholds for freeze detection and speed activation
        # detect loss of external positioning
        # stable estimates when sensor data is temporarily unavailable
        self.freeze_eps = 1e-5
        self.speed_eps = 0.12

        # ---------------- Linear Kalman Filter ----------------
        # 8 states: x, y, vx, vy, ax, ay, yaw, yaw_rate
        self.kf = KalmanFilter(dim_x=8, dim_z=3)
        self.kf.x = np.zeros((8, 1))
        # Initial covariance
        self.kf.P = np.diag([1, 1, 1, 1, 1, 1, 0.5, 0.3])

        # Measurement matrix: x, y, yaw from OptiTrack
        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0],  # x measurement
            [0, 1, 0, 0, 0, 0, 0, 0],  # y measurement
            [0, 0, 0, 0, 0, 0, 1, 0]   # yaw measurement
        ])

        # Measurement noise covariance
        # use OptiTrack pose input
        # prevent small sensor noise from causing sudden jumps
        self.kf.R = np.diag([0.02, 0.02, 0.001])

        # Process noise parameters
        self.q_acc = 0.7
        self.q_yaw = 0.005
        self.q_yaw_rate = 0.15

        # Internal state memory for freeze detection
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None

        # Last known good pose (used in freeze/tunnel modes)
        # continue localization without OptiTrack
        # AC 3.2.2 — stable estimates during sensor dropouts
        self.last_px = None
        self.last_py = None
        self.last_yaw = None

        # Wheel speed from AckermannDrive
        # subscribe to /ackermann_drive_feedback
        # use speed to improve motion estimation
        self.last_speed = 0.0

        # Initialization flags
        self.initialized = False
        self.prev_time = None

        # Velocity smoothing
        # prevents noise-induced velocity spikes
        self.vx_filt = 0.0
        self.vy_filt = 0.0
        self.vel_alpha = 0.7

        # Acceleration estimation 
        self.prev_vx_body = 0.0
        self.prev_vy_body = 0.0
        self.prev_pub_time = None
        self.acc_alpha = 0.7
        self.ax_filt = 0.0
        self.ay_filt = 0.0

        # Deadbands for clean standstill output
        # stable outputs at standstill
        # suppress noise near zero motion
        self.v_deadband = 0.01
        self.a_deadband = 0.05

        qos = rclpy.qos.QoSProfile(depth=10)
        self.create_subscription(RigidBodies, "/pose_modelcars", self.cb_mocap, qos)
        self.create_subscription(AckermannDrive, "/ackermann_drive_feedback", self.cb_speed, qos)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_accel = self.create_publisher(AccelStamped, "/acceleration", 10)
        self.get_logger().info("✅ Localization initialized")

    # ---------------- SPEED CALLBACK ----------------
    def cb_speed(self, msg):
        # speed input used for motion estimation
        self.last_speed = float(msg.speed)

    # ---------------- KF MATRIX FUNCTIONS ----------------
    # State transition matrix (constant acceleration model)
    def F(self, dt):
        F = np.eye(8)
        dt2 = 0.5 * dt * dt
        F[0, 2] = dt; F[1, 3] = dt
        F[0, 4] = dt2; F[1, 5] = dt2
        F[2, 4] = dt;  F[3, 5] = dt
        F[6, 7] = dt
        return F

    # Process noise covariance
    def Q(self, dt):
        dt2, dt3, dt4, dt5 = dt * dt, dt**3, dt**4, dt**5
        Q = np.zeros((8, 8))
        blk = np.array([
            [dt5 / 20, dt4 / 8, dt3 / 6],
            [dt4 / 8, dt3 / 3, dt2 / 2],
            [dt3 / 6, dt2 / 2, dt]
        ]) * self.q_acc
        Q[np.ix_([0, 2, 4], [0, 2, 4])] = blk
        Q[np.ix_([1, 3, 5], [1, 3, 5])] = blk
        Q[6, 6] += self.q_yaw * dt
        Q[7, 7] += self.q_yaw_rate * dt
        return Q

    # ---------------- OPTITRACK CALLBACK ----------------
    def cb_mocap(self, msg):
        rb = None
        for r in msg.rigidbodies:
            if str(r.rigid_body_name) == str(self.rigid_body_id):
                rb = r
                break
        if rb is None:
            return  

        x = rb.pose.position.x
        y = rb.pose.position.y
        yaw = wrap(yaw_from_quat(rb.pose.orientation))
        now = Time.from_msg(msg.header.stamp)
        dt = self.dt_default if self.prev_time is None else max((now - self.prev_time).nanoseconds * 1e-9, 1e-4)
        self.prev_time = now

        # ---------------- FIRST-TIME INITIALIZATION ----------------
        if not self.initialized:
            self.kf.x[0, 0] = x; self.kf.x[1, 0] = y; self.kf.x[6, 0] = yaw
            self.prev_x = x; self.prev_y = y; self.prev_yaw = yaw
            self.last_px = x; self.last_py = y; self.last_yaw = yaw
            self.initialized = True
            # AC 3.27.5 — initial continuous pose output
            self.publish(msg.header, x, y, yaw, 0.0, 0.0)
            return

        dx = x - self.prev_x
        dy = y - self.prev_y
        dyaw = wrap(yaw - self.prev_yaw)

        # detect loss of OptiTrack
        # temporary sensor unavailability
        mocap_frozen = abs(dx) < self.freeze_eps and abs(dy) < self.freeze_eps and abs(dyaw) < self.freeze_eps

        # ---------------- NORMAL KF UPDATE ----------------
        if not mocap_frozen:
            self.kf.F = self.F(dt); self.kf.Q = self.Q(dt)
            self.kf.predict()
            self.kf.update(np.array([[x], [y], [yaw]]))
            vx = 0.0 if abs(dx) < 1e-4 else dx / dt
            vy = 0.0 if abs(dy) < 1e-4 else dy / dt
            # AC 3.2.1 — prevent sudden velocity changes due to noise
            self.vx_filt = self.vel_alpha * self.vx_filt + (1 - self.vel_alpha) * vx
            self.vy_filt = self.vel_alpha * self.vy_filt + (1 - self.vel_alpha) * vy
            vx, vy = self.vx_filt, self.vy_filt
            self.prev_x = x; self.prev_y = y; self.prev_yaw = yaw
            self.last_px = x; self.last_py = y; self.last_yaw = yaw
            # AC 3.2.1 — avoid noise-driven acceleration
            if abs(vx) < 0.02 and abs(vy) < 0.02:
                self.kf.x[4, 0] = 0.0; self.kf.x[5, 0] = 0.0
            self.publish(msg.header, x, y, yaw, vx, vy)
            return

        # ---------------- KF FREEZE MODE ----------------
        # localization without external positioning
        # stable estimates when sensor data is unavailable
        self.kf.F = np.eye(8); self.kf.Q = np.zeros((8, 8))
        self.kf.predict()
        # avoid yaw jumps when switching localization modes
        self.kf.x[6, 0] = self.last_yaw; self.kf.x[7, 0] = 0.0

        # ---------------- TUNNEL MODE: Use wheel speed ----------------
        # speed-based motion estimation
        # pose estimation without OptiTrack
        if abs(self.last_speed) > self.speed_eps:
            yaw_use = float(self.kf.x[6, 0])
            vx = self.last_speed * math.cos(yaw_use)
            vy = self.last_speed * math.sin(yaw_use)
            self.last_px += vx * dt; self.last_py += vy * dt
            self.kf.x[2, 0] = vx; self.kf.x[3, 0] = vy
            self.publish(msg.header, self.last_px, self.last_py, yaw_use, vx, vy)
            return

        # continuous pose output at standstill
        self.publish(msg.header, self.last_px, self.last_py, self.last_yaw, 0.0, 0.0)

    # ---------------- PUBLISH FUNCTION ----------------
    def publish(self, header, px, py, yaw, vx, vy):
        cy = math.cos(yaw); sy = math.sin(yaw)
        vx_body = cy * vx + sy * vy
        vy_body = -sy * vx + cy * vy
        # stable output
        # suppress noise near zero velocity
        if abs(vx_body) < self.v_deadband:
            vx_body = 0.0
        if abs(vy_body) < self.v_deadband:
            vy_body = 0.0

        odom = Odometry()
        odom.header = header
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(px)
        odom.pose.pose.position.y = float(py)
        odom.pose.pose.orientation = yaw_to_quat(yaw)
        odom.twist.twist.linear.x = float(vx_body)
        odom.twist.twist.linear.y = float(vy_body)
        odom.twist.twist.angular.z = float(self.kf.x[7, 0])
        self.pub_odom.publish(odom)

        stopped = vx_body == 0.0 and vy_body == 0.0
        now = Time.from_msg(header.stamp)
        dt = self.dt_default if self.prev_pub_time is None else max((now - self.prev_pub_time).nanoseconds * 1e-9, 1e-4)
        self.prev_pub_time = now

        if stopped:
            # no residual acceleration at standstill
            self.prev_vx_body = 0.0; self.prev_vy_body = 0.0
            self.ax_filt = 0.0; self.ay_filt = 0.0
            accel = AccelStamped()
            accel.header = header
            accel.header.frame_id = "base_link"
            accel.accel.linear.x = 0.0
            accel.accel.linear.y = 0.0
            self.pub_accel.publish(accel)
            return

        ax_num = (vx_body - self.prev_vx_body) / dt
        ay_num = (vy_body - self.prev_vy_body) / dt
        self.prev_vx_body = vx_body; self.prev_vy_body = vy_body
        if abs(ax_num) < self.a_deadband:
            ax_num = 0.0
        if abs(ay_num) < self.a_deadband:
            ay_num = 0.0
        self.ax_filt = self.acc_alpha * self.ax_filt + (1 - self.acc_alpha) * ax_num
        self.ay_filt = self.acc_alpha * self.ay_filt + (1 - self.acc_alpha) * ay_num
        # smooth transitions between localization modes
        if vx_body > self.v_deadband and self.ax_filt < 0.0:
            self.ax_filt = 0.0
        if vx_body < -self.v_deadband and self.ax_filt > 0.0:
            self.ax_filt = 0.0

        accel = AccelStamped()
        accel.header = header
        accel.header.frame_id = "base_link"
        accel.accel.linear.x = float(self.ax_filt)
        accel.accel.linear.y = float(self.ay_filt)
        self.pub_accel.publish(accel)

# ---------------- MAIN ----------------
def main():
    rclpy.init()
    node = LocalizationKF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
