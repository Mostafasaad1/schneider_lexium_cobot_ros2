#!/usr/bin/env python3
"""
Schneider Electric Lexium Cobot L18S — Cartesian Teleoperation GUI.
Simple, flat, minimal design.
"""

import sys
import math
import signal
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import Qt, QPointF, QTimer, pyqtSignal, QRectF
from PyQt5.QtGui import QPainter, QBrush, QPen, QColor, QFont
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QSizePolicy, QStatusBar, QSlider
)

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PlanningOptions
from moveit_msgs.srv import ServoCommandType

JOINT_NAMES = [
    'l_Base_Link002__to__l_Link_012',
    'l_Link_012__to__l_Link_013',
    'l_Link_013__to__l_Link_014',
    'l_Link_014__to__l_Link_015',
    'l_Link_015__to__l_Link_016',
    'l_Link_016__to__l_Link_017',
]

BG       = QColor(18, 18, 18)
SURFACE  = QColor(30, 30, 30)
BORDER   = QColor(50, 50, 50)
FG_DIM   = QColor(120, 120, 120)
FG       = QColor(220, 220, 220)
ACCENT_T = QColor(80, 200, 120)   # mint  — translation mode
ACCENT_R = QColor(230, 160, 60)   # amber — rotation mode
ACCENT   = ACCENT_T


class Joystick2D(QWidget):
    """Simple flat 2D spring-loaded joystick, mode-aware."""
    # Emits (fwd_val, right_val) both in [-1, +1]
    changed = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._fwd = 0.0
        self._right = 0.0
        self._active = False
        self._DEADZONE = 0.04
        # Labels set externally by mode switch
        self.lbl_top   = "+X"
        self.lbl_btm   = "-X"
        self.lbl_left  = "+Y"
        self.lbl_right = "-Y"
        self.accent    = ACCENT_T

    def set_mode(self, top, btm, left, right, accent):
        self.lbl_top, self.lbl_btm = top, btm
        self.lbl_left, self.lbl_right = left, right
        self.accent = accent
        self.reset()

    def reset(self):
        self._fwd = 0.0
        self._right = 0.0
        self._active = False
        self.update()
        self.changed.emit(0.0, 0.0)

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._active = True
            self._calc(e.pos())

    def mouseMoveEvent(self, e):
        if self._active:
            self._calc(e.pos())

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton:
            self.reset()

    def _calc(self, pos):
        cx, cy = self.width() / 2, self.height() / 2
        r = min(cx, cy) - 18
        dx = pos.x() - cx
        dy = -(pos.y() - cy)          # up is positive
        dist = math.hypot(dx, dy)
        norm = min(1.0, dist / r) if r > 0 else 0.0
        if dist > 0 and norm >= self._DEADZONE:
            ang = math.atan2(dy, dx)
            s = (norm - self._DEADZONE) / (1.0 - self._DEADZONE)
            self._right = math.cos(ang) * s
            self._fwd   = math.sin(ang) * s
        else:
            self._fwd = self._right = 0.0
        self.update()
        self.changed.emit(self._fwd, self._right)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        cx, cy = self.width() / 2, self.height() / 2
        r = min(cx, cy) - 18

        # Outer disk
        p.setBrush(QBrush(SURFACE))
        p.setPen(QPen(BORDER, 1))
        p.drawEllipse(QPointF(cx, cy), r, r)

        # Crosshairs
        p.setPen(QPen(BORDER, 1))
        p.drawLine(int(cx - r + 8), int(cy), int(cx + r - 8), int(cy))
        p.drawLine(int(cx), int(cy - r + 8), int(cx), int(cy + r - 8))

        # Axis labels
        p.setFont(QFont("sans-serif", 8))
        p.setPen(QPen(FG_DIM))
        p.drawText(int(cx - 20), int(cy - r - 4), 40, 14, Qt.AlignCenter, self.lbl_top)
        p.drawText(int(cx - 20), int(cy + r - 4), 40, 14, Qt.AlignCenter, self.lbl_btm)
        p.drawText(int(cx - r - 22), int(cy - 7), 22, 14, Qt.AlignCenter, self.lbl_left)
        p.drawText(int(cx + r + 1),  int(cy - 7), 22, 14, Qt.AlignCenter, self.lbl_right)

        # Puck
        px = cx + self._right * r
        py = cy - self._fwd   * r
        puck_r = 16.0
        col = self.accent if self._active else QColor(70, 70, 70)
        p.setBrush(QBrush(col))
        p.setPen(Qt.NoPen)
        p.drawEllipse(QPointF(px, py), puck_r, puck_r)

        # White center dot
        p.setBrush(QBrush(QColor(255, 255, 255, 200)))
        p.drawEllipse(QPointF(px, py), 3.5, 3.5)


class JoystickZ(QWidget):
    """Simple flat vertical 1D joystick — Z lift or Yaw, mode-aware."""
    changed = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(60, 200)
        self.setMaximumWidth(100)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        self._val = 0.0
        self._active = False
        self._DEADZONE = 0.04
        self.lbl_top = "+Z"
        self.lbl_btm = "-Z"
        self.accent  = QColor(80, 180, 220)

    def set_mode(self, top, btm, accent):
        self.lbl_top, self.lbl_btm = top, btm
        self.accent = accent
        self.reset()

    def reset(self):
        self._val = 0.0
        self._active = False
        self.update()
        self.changed.emit(0.0)

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._active = True
            self._calc(e.pos())

    def mouseMoveEvent(self, e):
        if self._active:
            self._calc(e.pos())

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton:
            self.reset()

    def _calc(self, pos):
        cy   = self.height() / 2
        half = cy - 18
        if half <= 0:
            return
        dy = -(pos.y() - cy)
        norm = max(-1.0, min(1.0, dy / half))
        if abs(norm) < self._DEADZONE:
            self._val = 0.0
        else:
            s = 1.0 if norm > 0 else -1.0
            self._val = s * ((abs(norm) - self._DEADZONE) / (1.0 - self._DEADZONE))
        self.update()
        self.changed.emit(self._val)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        cx   = self.width()  / 2
        cy   = self.height() / 2
        half = cy - 18
        tw   = 20.0

        # Track capsule
        track = QRectF(cx - tw / 2, cy - half, tw, half * 2)
        p.setBrush(QBrush(SURFACE))
        p.setPen(QPen(BORDER, 1))
        p.drawRoundedRect(track, tw / 2, tw / 2)

        # Neutral tick
        p.setPen(QPen(BORDER, 1))
        p.drawLine(int(cx - 8), int(cy), int(cx + 8), int(cy))

        # Labels
        p.setFont(QFont("sans-serif", 8))
        p.setPen(QPen(FG_DIM))
        p.drawText(0, 2, int(self.width()), 14, Qt.AlignCenter, self.lbl_top)
        p.drawText(0, int(self.height()) - 16, int(self.width()), 14, Qt.AlignCenter, self.lbl_btm)

        # Puck
        py = cy - self._val * half
        puck_r = 15.0
        col = self.accent if self._active else QColor(70, 70, 70)
        p.setBrush(QBrush(col))
        p.setPen(Qt.NoPen)
        p.drawEllipse(QPointF(cx, py), puck_r, puck_r)

        p.setBrush(QBrush(QColor(255, 255, 255, 200)))
        p.drawEllipse(QPointF(cx, py), 3.5, 3.5)


# ──────────────────────────────────────────────────────────────────────────────
# ROS 2 Node
# ──────────────────────────────────────────────────────────────────────────────
class TeleopNode(Node):
    def __init__(self):
        super().__init__('lexium_cartesian_gui_teleop')
        self.twist_pub   = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.switch_cli  = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.frame = 'l_Base_Link002'
        self.create_subscription(JointState, '/joint_states', lambda _: None, 10)

    def switch_twist(self):
        if not self.switch_cli.wait_for_service(timeout_sec=0.5):
            return None
        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.TWIST
        return self.switch_cli.call_async(req)

    def send_twist(self, lx, ly, lz, rx=0., ry=0., rz=0.):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.twist.linear.x  = float(lx)
        msg.twist.linear.y  = float(ly)
        msg.twist.linear.z  = float(lz)
        msg.twist.angular.x = float(rx)
        msg.twist.angular.y = float(ry)
        msg.twist.angular.z = float(rz)
        self.twist_pub.publish(msg)

    def goto_joints(self, positions):
        if self.move_client.wait_for_server(timeout_sec=0.5):
            goal = MoveGroup.Goal()
            goal.request = MotionPlanRequest()
            goal.request.group_name = "lexium_arm"
            goal.request.num_planning_attempts = 5
            goal.request.allowed_planning_time = 5.0
            c = Constraints()
            for name, pos in zip(JOINT_NAMES, positions):
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = float(pos)
                jc.tolerance_above = jc.tolerance_below = 0.01
                jc.weight = 1.0
                c.joint_constraints.append(jc)
            goal.request.goal_constraints.append(c)
            goal.planning_options = PlanningOptions()
            goal.planning_options.plan_only = False
            return self.move_client.send_goal_async(goal)

        if self.traj_client.wait_for_server(timeout_sec=0.5):
            tg = FollowJointTrajectory.Goal()
            tg.trajectory.joint_names = JOINT_NAMES
            pt = JointTrajectoryPoint()
            pt.positions = [float(p) for p in positions]
            pt.time_from_start = Duration(sec=4)
            tg.trajectory.points.append(pt)
            return self.traj_client.send_goal_async(tg)

        return None


# ──────────────────────────────────────────────────────────────────────────────
# Main Window
# ──────────────────────────────────────────────────────────────────────────────
class TeleopWindow(QMainWindow):
    # Joystick axis configuration per mode
    _MODES = {
        'translate': dict(
            xy=('+X (Fwd)', '-X (Bck)', '+Y (L)', '-Y (R)'),
            z=('+Z (Up)', '-Z (Dn)'),
            xy_accent=ACCENT_T,
            z_accent=QColor(80, 180, 220),
            xy_lbl='Translation XY',
            z_lbl='Z Lift',
        ),
        'rotate': dict(
            xy=('+Pitch', '-Pitch', '+Roll', '-Roll'),
            z=('+Yaw', '-Yaw'),
            xy_accent=ACCENT_R,
            z_accent=QColor(230, 160, 60),
            xy_lbl='Pitch / Roll',
            z_lbl='Yaw',
        ),
    }

    def __init__(self, node: TeleopNode):
        super().__init__()
        self.node = node
        self.linear_speed   = 0.10
        self.angular_speed  = 0.30
        self._mode = 'translate'      # 'translate' | 'rotate'
        self.vx = self.vy = self.vz = 0.0
        self.vrx = self.vry = self.vrz = 0.0
        self._motion_stage = None
        self._motion_data  = None

        self._build_ui()
        self.node.switch_twist()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(20)

    # ── UI ──────────────────────────────────────────────────────────────────
    def _build_ui(self):
        self.setWindowTitle("L18S Cartesian Teleop")
        self.setMinimumSize(680, 520)
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{
                background: {BG.name()};
                color: {FG.name()};
                font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Inter', sans-serif;
                font-size: 13px;
            }}
            QLabel        {{ color: {FG.name()};     }}
            QLabel#dim    {{ color: {FG_DIM.name()}; font-size: 11px; }}
            QLabel#title  {{ color: {FG.name()};     font-size: 15px; font-weight: 700; }}
            QComboBox {{
                background: {SURFACE.name()};
                border: 1px solid {BORDER.name()};
                border-radius: 6px;
                padding: 4px 10px;
                color: {FG.name()};
                min-width: 180px;
            }}
            QComboBox QAbstractItemView {{
                background: {SURFACE.name()};
                border: 1px solid {BORDER.name()};
                selection-background-color: #2a2a2a;
            }}
            /* Mode toggle pill */
            QPushButton#modeT {{
                background: #1a4731;
                color: #6ee7b7;
                border-radius: 6px;
                font-size: 12px;
                font-weight: 700;
                padding: 4px 14px;
                letter-spacing: 0.8px;
                border: none;
            }}
            QPushButton#modeR {{
                background: #3d2800;
                color: #fcd34d;
                border-radius: 6px;
                font-size: 12px;
                font-weight: 700;
                padding: 4px 14px;
                letter-spacing: 0.8px;
                border: none;
            }}
            QPushButton {{
                border-radius: 7px;
                font-size: 13px;
                font-weight: 600;
                padding: 10px 20px;
                border: none;
            }}
            QPushButton#home {{
                background: #1a4731;
                color: #6ee7b7;
            }}
            QPushButton#home:hover  {{ background: #22613f; }}
            QPushButton#home:pressed {{ background: #153b28; }}
            QPushButton#ready {{
                background: #1e3560;
                color: #93c5fd;
            }}
            QPushButton#ready:hover  {{ background: #234080; }}
            QPushButton#ready:pressed {{ background: #172a50; }}
            QPushButton#stop {{
                background: #5c1a1a;
                color: #fca5a5;
                font-weight: 700;
                letter-spacing: 0.5px;
            }}
            QPushButton#stop:hover  {{ background: #7a1f1f; }}
            QPushButton#stop:pressed {{ background: #4a1414; }}
            QSlider::groove:horizontal {{
                height: 4px; background: {SURFACE.name()}; border-radius: 2px;
            }}
            QSlider::sub-page:horizontal {{
                background: {ACCENT_T.name()}; border-radius: 2px;
            }}
            QSlider::handle:horizontal {{
                background: {FG.name()};
                width: 12px; height: 12px;
                margin: -4px 0;
                border-radius: 6px;
            }}
            QStatusBar {{
                background: {BG.name()};
                color: {FG_DIM.name()};
                font-size: 11px;
                border-top: 1px solid {BORDER.name()};
            }}
        """)

        root = QWidget()
        self.setCentralWidget(root)
        main = QVBoxLayout(root)
        main.setContentsMargins(24, 20, 24, 16)
        main.setSpacing(0)

        # ── Navbar ────────────────────────────────────────────────────────
        nav = QHBoxLayout()
        nav.setSpacing(8)

        title = QLabel("L18S Teleop")
        title.setObjectName("title")
        nav.addWidget(title)

        nav.addStretch()

        # Mode toggle pill
        self.btn_mode = QPushButton("TRANSLATE")
        self.btn_mode.setObjectName("modeT")
        self.btn_mode.setCheckable(True)
        self.btn_mode.setChecked(False)
        self.btn_mode.setFixedHeight(30)
        self.btn_mode.clicked.connect(self._on_mode_toggle)
        nav.addWidget(self.btn_mode)

        nav.addSpacing(16)

        frame_lbl = QLabel("Frame")
        frame_lbl.setObjectName("dim")
        nav.addWidget(frame_lbl)

        self.frame_combo = QComboBox()
        self.frame_combo.addItems(["Base Frame", "Tool Frame"])
        self.frame_combo.currentIndexChanged.connect(self._on_frame)
        nav.addWidget(self.frame_combo)

        main.addLayout(nav)

        # Divider
        div = QWidget()
        div.setFixedHeight(1)
        div.setStyleSheet(f"background: {BORDER.name()};")
        main.addSpacing(14)
        main.addWidget(div)
        main.addSpacing(20)

        # ── Joystick Area ─────────────────────────────────────────────────
        joy_row = QHBoxLayout()
        joy_row.setSpacing(20)

        # XY column
        xy_col = QVBoxLayout()
        xy_col.setSpacing(8)
        self.lbl_xy = QLabel("Translation XY")
        self.lbl_xy.setObjectName("dim")
        self.lbl_xy.setAlignment(Qt.AlignCenter)
        self.joy_xy = Joystick2D()
        self.joy_xy.changed.connect(self._on_xy)
        xy_col.addWidget(self.lbl_xy)
        xy_col.addWidget(self.joy_xy)
        joy_row.addLayout(xy_col, stretch=3)

        # Z column
        z_col = QVBoxLayout()
        z_col.setSpacing(8)
        self.lbl_z = QLabel("Z Lift")
        self.lbl_z.setObjectName("dim")
        self.lbl_z.setAlignment(Qt.AlignCenter)
        self.joy_z = JoystickZ()
        self.joy_z.changed.connect(self._on_z)
        z_col.addWidget(self.lbl_z)
        z_col.addWidget(self.joy_z, alignment=Qt.AlignHCenter)
        joy_row.addLayout(z_col, stretch=1)

        main.addLayout(joy_row, stretch=5)
        main.addSpacing(18)

        # ── Speed Slider ──────────────────────────────────────────────────
        spd_row = QHBoxLayout()
        spd_row.setSpacing(12)
        self.spd_lbl = QLabel("Speed")
        self.spd_lbl.setObjectName("dim")
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(1, 50)
        self.slider.setValue(10)
        self.slider.valueChanged.connect(self._on_speed)
        self.spd_val = QLabel("0.10 m/s")
        self.spd_val.setObjectName("dim")
        spd_row.addWidget(self.spd_lbl)
        spd_row.addWidget(self.slider)
        spd_row.addWidget(self.spd_val)
        main.addLayout(spd_row)
        main.addSpacing(18)

        # ── Action Buttons ────────────────────────────────────────────────
        btn_row = QHBoxLayout()
        btn_row.setSpacing(12)

        self.btn_home = QPushButton("Home Position")
        self.btn_home.setObjectName("home")
        self.btn_home.clicked.connect(self._on_home)

        self.btn_ready = QPushButton("Ready Pose")
        self.btn_ready.setObjectName("ready")
        self.btn_ready.clicked.connect(self._on_ready)

        self.btn_stop = QPushButton("STOP")
        self.btn_stop.setObjectName("stop")
        self.btn_stop.clicked.connect(self._on_stop)

        btn_row.addWidget(self.btn_home)
        btn_row.addWidget(self.btn_ready)
        btn_row.addWidget(self.btn_stop)
        main.addLayout(btn_row)

        # ── Status Bar ────────────────────────────────────────────────────
        sb = QStatusBar()
        sb.showMessage("Ready — drag joysticks to move")
        self.setStatusBar(sb)
        self._sb = sb

    # ── Callbacks ────────────────────────────────────────────────────────────
    def _on_frame(self, idx):
        self.node.frame = 'l_Base_Link002' if idx == 0 else 'l_Link_017'
        self._sb.showMessage(f"Frame: {self.frame_combo.currentText()}")

    def _on_mode_toggle(self):
        is_rotate = self.btn_mode.isChecked()
        self._mode = 'rotate' if is_rotate else 'translate'
        cfg = self._MODES[self._mode]
        # Update joystick labels and colours
        self.joy_xy.set_mode(*cfg['xy'], cfg['xy_accent'])
        self.joy_z.set_mode(*cfg['z'], cfg['z_accent'])
        # Update section labels
        self.lbl_xy.setText(cfg['xy_lbl'])
        self.lbl_z.setText(cfg['z_lbl'])
        # Update the pill button appearance
        self.btn_mode.setText("ROTATE" if is_rotate else "TRANSLATE")
        self.btn_mode.setObjectName("modeR" if is_rotate else "modeT")
        # Force style re-evaluation
        self.btn_mode.style().unpolish(self.btn_mode)
        self.btn_mode.style().polish(self.btn_mode)
        # Reset velocities when switching modes
        self.vx = self.vy = self.vz = 0.0
        self.vrx = self.vry = self.vrz = 0.0
        self._sb.showMessage(f"Mode: {self._mode.upper()}")

    def _on_xy(self, fwd, right):
        if self._mode == 'translate':
            self.vx  =  fwd   * self.linear_speed
            self.vy  = -right * self.linear_speed   # ROS +Y is left
        else:
            # Pitch = Ry (up = +Ry), Roll = Rx (left = +Rx)
            self.vry =  fwd   * self.angular_speed
            self.vrx = -right * self.angular_speed

    def _on_z(self, z_norm):
        if self._mode == 'translate':
            self.vz  = z_norm * self.linear_speed
        else:
            self.vrz = z_norm * self.angular_speed

    def _on_speed(self, v):
        self.linear_speed  = v / 100.0
        self.angular_speed = v / 100.0 * 3.0   # angular 3× linear
        self.spd_val.setText(f"{self.linear_speed:.2f} m/s")

    def _on_stop(self):
        self.joy_xy.reset()
        self.joy_z.reset()
        self.vx = self.vy = self.vz = 0.0
        self.vrx = self.vry = self.vrz = 0.0
        if rclpy.ok():
            try:
                self.node.send_twist(0, 0, 0)
            except Exception:
                pass
        self._sb.showMessage("Stopped")

    def _on_home(self):
        self._goto([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "Home Position")

    def _on_ready(self):
        self._goto([0.0, -0.785, 1.57, -0.785, 1.57, 0.0], "Ready Pose")

    def _goto(self, positions, label):
        self._sb.showMessage(f"Moving to {label}…")
        self.btn_home.setEnabled(False)
        self.btn_ready.setEnabled(False)
        fut = self.node.goto_joints(positions)
        if fut:
            self._motion_stage = 'goal'
            self._motion_data  = (fut, label)
        else:
            self._sb.showMessage("Action server not available")
            self.btn_home.setEnabled(True)
            self.btn_ready.setEnabled(True)

    def closeEvent(self, e):
        self.timer.stop()
        self._on_stop()
        e.accept()

    # ── Timer Loop ───────────────────────────────────────────────────────────
    def _tick(self):
        if not rclpy.ok():
            self.timer.stop()
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception:
            self.timer.stop()
            return

        # Handle pending motion
        if self._motion_stage == 'goal':
            fut, label = self._motion_data
            if fut.done():
                gh = fut.result()
                if gh and gh.accepted:
                    self._motion_stage = 'result'
                    self._motion_data  = (gh.get_result_async(), label)
                    self._sb.showMessage(f"Executing {label}…")
                else:
                    self._sb.showMessage(f"{label}: goal rejected")
                    self._reset_btns()
        elif self._motion_stage == 'result':
            result_fut, label = self._motion_data
            if result_fut.done():
                self._motion_stage = None
                self._reset_btns()
                self._sb.showMessage(f"{label} complete")
                self.node.switch_twist()

        # Publish twist — always send all 6 DOF
        vals = [self.vx, self.vy, self.vz, self.vrx, self.vry, self.vrz]
        if any(abs(v) > 1e-4 for v in vals) and rclpy.ok():
            try:
                self.node.send_twist(*vals)
            except Exception:
                pass

    def _reset_btns(self):
        self.btn_home.setEnabled(True)
        self.btn_ready.setEnabled(True)


# ──────────────────────────────────────────────────────────────────────────────
def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    rclpy.init()
    node = TeleopNode()

    app = QApplication(sys.argv)
    win = TeleopWindow(node)
    app.aboutToQuit.connect(win.timer.stop)
    win.show()

    code = app.exec_()
    win.timer.stop()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
