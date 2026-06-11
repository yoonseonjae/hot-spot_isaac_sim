import numpy as np
import os
import json
import time
import omni.kit.commands
import omni.replicator.core as rep
import omni.usd
import omni.kit.app
from pxr import Gf, UsdLux, Sdf
from spot_policy import SpotArmFlatTerrainPolicy

# MONKEY PATCH AttributeValueHelper to suppress TypeError
try:
    from omni.graph.core._impl.attribute_values import AttributeValueHelper
    if not hasattr(AttributeValueHelper, "_patched_for_dtype_error"):
        _original_set = AttributeValueHelper.set
        def _safe_set(self, *args, **kwargs):
            try:
                _original_set(self, *args, **kwargs)
            except TypeError as e:
                if "unknown dtype" in str(e):
                    pass
                else:
                    raise e
        AttributeValueHelper.set = _safe_set
        AttributeValueHelper._patched_for_dtype_error = True
except Exception:
    pass

class SpotAgent:
    """
    robot1 (allow_grasp_trigger=True) : 소화기 Grasp + 투척 + Nav2
    robot2 (allow_grasp_trigger=False): Nav2 순찰 전용 (YOLO는 Stage 4)
    """

    def __init__(
        self,
        base_dir,
        enable_replicator_writer=False,
        namespace="robot1",
        spawn_pos=np.array([10.7, 0.5, 0.8]),
        udp_port=9876,
        allow_grasp_trigger=False,
    ):
        self.enable_replicator_writer = enable_replicator_writer
        self.namespace = namespace
        self.udp_port = udp_port
        self.allow_grasp_trigger = allow_grasp_trigger

        walking_policy_path = os.path.join(
            base_dir, "policies/spot_arm/models", "spot_arm_policy.pt"
        )
        balance_policy_path = os.path.join(
            base_dir, "policies/spot_arm/models", "model_10800.pt"
        )
        arm_balance_policy_path = None  # 76-dim 모델 없음
        policy_params_path = os.path.join(
            base_dir, "policies/spot_arm/params", "env.yaml"
        )
        usd_path = os.path.join(base_dir, "assets", "spot_arm.usd")

        self._spot = SpotArmFlatTerrainPolicy(
            prim_path=f"/World/{self.namespace}",
            name=self.namespace,
            usd_path=usd_path,
            walking_policy_path=walking_policy_path,
            balance_policy_path=balance_policy_path,
            arm_balance_policy_path=arm_balance_policy_path,
            policy_params_path=policy_params_path,
            position=spawn_pos,
            orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        )

        self.first_step = True
        self._nav_command = np.zeros(3)
        self._pose_file = f"/tmp/isaac_pose_{self.namespace}.json"
        self._robot2_plan_file = f"/tmp/{self.namespace}_plan.json"
        self._robot2_plan = []
        self._robot2_plan_mtime = 0.0
        self._robot2_plan_min_mtime = 0.0
        self._pose_publish_interval = 1.0 / max(
            1.0, float(os.environ.get("COBOT_POSE_HZ", "15.0"))
        )
        self._pose_publish_accum = self._pose_publish_interval
        self._plan_poll_interval = 1.0 / max(
            1.0, float(os.environ.get("COBOT_PLAN_POLL_HZ", "3.0"))
        )
        self._last_plan_poll_wall = 0.0
        self._last_nav_goal = None
        self._nav_goal_active = False
        self._nav_goal_started = False
        self._nav_goal_retry_t = 0.0
        self._nav_goal_retry_interval = 5.0

        import socket
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("127.0.0.1", self.udp_port))
        self.udp_sock.setblocking(False)

        # Grasp 상태머신 변수 (robot1 전용)
        self.ARM_IDX = [1, 0, 2, 7, 12, 17]
        self.GRIP_IDX = 18
        self.GRIP_CLOSE = 0.0
        self._delivery_state = "SEARCHING"
        self._is_heavy_mode = False
        self._carry_arm = None
        self._grasp_t = 0.0
        self._grabbed_cube_path = None
        self._is_thrown = False
        self._has_object = False

        # ---- robot1 자동 시나리오 변수 ----
        self._auto_fire_triggered = False
        # IDLE → NAV_TO_EXTINGUISHER → GRASPING → NAV_TO_FIRE → THROWING → DONE
        self._auto_state  = "IDLE"
        self._nav_wait_t  = 0.0
        # 맵 좌표
        self.EXTINGUISHER_POS = (9.83, 0.5)     # 소화기
        self.FIRE_POS         = (-4.93, -5.78)   # 2번방 화재
        self.EXIT_POS         = (1.568, 20.041)  # 출구
        self.FIRE_THROW_DISTANCE = 2.5
        self.FIRE_STALLED_THROW_DISTANCE = 3.0
        self.FIRE_APPROACH_STALL_TIME = 8.0
        self.FIRE_PROGRESS_EPS = 0.08
        self._fire_best_dist = float("inf")
        self._fire_stall_t = 0.0
        self._fire_goal_retry_t = 0.0

        # ---- robot2 순찰 변수 ----
        # 5→1→3→4→6
        self._patrol_waypoints = [
            (5.039,  -6.990),   # 5번방 (검증용: yolo_nice 원래 위치)
            (-1.707, -11.104),  # 1번방
            (-1.707,  2.059),   # 3번방
            (-1.707,  12.753),  # 4번방
            (5.039,   11.986),  # 6번방
        ]
        self._patrol_idx    = 0
        self._patrol_wait_t = 0.0
        self._patrol_active = False
        self.ROBOT2_WAYPOINT_ARRIVAL_TOLERANCE = 0.6
        self.ROBOT2_ROOM_SCAN_SPIN_SPEED = 0.25
        self.ROBOT2_ROOM_SCAN_SPIN_DURATION = (2.0 * np.pi) / self.ROBOT2_ROOM_SCAN_SPIN_SPEED
        self._patrol_spin_active = False
        self._patrol_spin_t = 0.0
        self._escape_reverse_t = 0.0
        self._escape_cooldown_t = 0.0
        self._stuck_watch_t = 0.0
        self._stuck_last_dist = None
        self._rescue_active = False
        self._rescue_goal_done = False
        self._rescue_goal_retry_t = 0.0
        self._rescue_no_cmd_t = 0.0
        self._last_exit_drive_log_t = -999.0
        self.RESCUE_GOAL_RETRY_INTERVAL = 4.0
        self._person2_follow_active = False
        self._person_follow_interval = 1.0 / max(
            1.0, float(os.environ.get("COBOT_PERSON_FOLLOW_HZ", "10.0"))
        )
        self._person_follow_accum = self._person_follow_interval
        self._person2_bbox_paths = []
        self.PERSON2_HOME = np.array([8.82, -9.56])
        self.PERSON2_Z = 0.01957
        self.PERSON2_PROXY_Z = 0.9
        self.PERSON2_FOLLOW_DISTANCE = float(
            os.environ.get("COBOT_PERSON_FOLLOW_DISTANCE", "2.4")
        )
        self.PERSON2_FOLLOW_SPEED = float(
            os.environ.get("COBOT_PERSON_FOLLOW_SPEED", "0.9")
        )
        self.PERSON2_CATCHUP_SPEED = float(
            os.environ.get("COBOT_PERSON_CATCHUP_SPEED", "1.8")
        )
        self.PERSON_DETECT_MAX_DISTANCE = float(
            os.environ.get("COBOT_PERSON_DETECT_MAX_DISTANCE", "5.2")
        )
        self.PERSON_APPROACH_DISTANCE = float(
            os.environ.get("COBOT_PERSON_APPROACH_DISTANCE", "1.15")
        )
        self.PERSON_APPROACH_MAX_VX = float(
            os.environ.get("COBOT_PERSON_APPROACH_MAX_VX", "0.22")
        )
        self.PERSON_APPROACH_MIN_TIME = float(
            os.environ.get("COBOT_PERSON_APPROACH_MIN_TIME", "0.8")
        )
        self.PERSON_APPROACH_TIMEOUT = float(
            os.environ.get("COBOT_PERSON_APPROACH_TIMEOUT", "8.0")
        )
        self.PERSON_ALIGN_TOLERANCE = float(
            os.environ.get("COBOT_PERSON_ALIGN_TOLERANCE", "0.10")
        )
        self.PERSON_ALIGN_DEADBAND = float(
            os.environ.get("COBOT_PERSON_ALIGN_DEADBAND", "0.03")
        )
        self.PERSON_ALIGN_MIN_WZ = float(
            os.environ.get("COBOT_PERSON_ALIGN_MIN_WZ", "0.10")
        )
        self.EXIT_FALLBACK_MAX_VX = float(
            os.environ.get("COBOT_EXIT_FALLBACK_MAX_VX", "0.26")
        )
        self.PERSON_CAMERA_MAX_DISTANCE = self.PERSON_DETECT_MAX_DISTANCE
        self.PERSON_CAMERA_HALF_FOV_RAD = np.deg2rad(42.0)

        # RL 보행 정책 안정화: 급가속/급회전과 큰 기울어짐을 한 곳에서 제한.
        self._safe_drive_command = np.zeros(3, dtype=np.float32)
        self._last_stability_log_t = 0.0
        self.DRIVE_MAX_VX = 0.45
        self.DRIVE_CARRY_MAX_VX = 0.30
        self.DRIVE_MAX_VY = 0.18
        self.DRIVE_MAX_WZ = 0.25
        self.DRIVE_SLEW = np.array([0.55, 0.45, 0.70], dtype=np.float32)
        self.TILT_SLOWDOWN_RAD = np.deg2rad(14.0)
        self.TILT_STOP_RAD = np.deg2rad(22.0)

        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = None
        self._keyboard = None
        self._sub_keyboard = None
        if self._appwindow is not None:
            import carb
            self._input = carb.input.acquire_input_interface()
            self._keyboard = self._appwindow.get_keyboard()
            self._sub_keyboard = self._input.subscribe_to_keyboard_events(
                self._keyboard, self._on_agent_keyboard
            )
        else:
            print(f"[{self.namespace}] 기본 앱 창 없음: 키보드 입력 없이 계속 실행")

    # ------------------------------------------------------------------ #
    # Nav2 목표 전송 / 위치 헬퍼
    # ------------------------------------------------------------------ #
    def _send_nav2_goal(self, x, y, yaw=0.0):
        import subprocess, math
        self._last_nav_goal = (float(x), float(y), float(yaw))
        self._nav_goal_active = True
        self._nav_goal_started = False
        self._nav_goal_retry_t = 0.0
        self._safe_drive_command[:] = 0.0
        self._robot2_plan = []
        self._robot2_plan_mtime = 0.0
        self._robot2_plan_min_mtime = time.time()
        self._last_plan_poll_wall = 0.0
        if not self.allow_grasp_trigger:
            self._reset_robot2_escape_monitor()
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal = (
            f"{{pose: {{header: {{frame_id: 'map'}}, "
            f"pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, "
            f"orientation: {{x: 0.0, y: 0.0, z: {qz:.4f}, w: {qw:.4f}}}}}}}}}"
        )
        env = self._ros_subprocess_env()
        subprocess.Popen(
            ["timeout", "8", "ros2", "action", "send_goal",
             f"/{self.namespace}/navigate_to_pose",
             "nav2_msgs/action/NavigateToPose", goal],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        print(f"[{self.namespace}] Nav2 목표: ({x:.2f}, {y:.2f})")

    def _retry_nav2_goal_until_cmd(self, step_size):
        if not self._nav_goal_active or self._nav_goal_started or self._last_nav_goal is None:
            return
        self._nav_goal_retry_t += step_size
        if self._nav_goal_retry_t < self._nav_goal_retry_interval:
            return
        x, y, yaw = self._last_nav_goal
        print(f"[{self.namespace}] Nav2 cmd_vel 대기 중 → 목표 재전송: ({x:.2f}, {y:.2f})")
        self._send_nav2_goal(x, y, yaw)

    def _mark_nav2_goal_done(self):
        self._nav_goal_active = False
        self._nav_goal_started = False
        self._last_nav_goal = None
        self._nav_goal_retry_t = 0.0
        self._safe_drive_command[:] = 0.0
        if not self.allow_grasp_trigger:
            self._reset_robot2_escape_monitor()

    def _ros_subprocess_env(self):
        env = os.environ.copy()
        if "PYTHONPATH" in env:
            env["PYTHONPATH"] = ":".join(
                p for p in env["PYTHONPATH"].split(":")
                if "isaacsim" not in p.lower()
            )
        return env

    def _clear_robot2_costmaps(self):
        if self.allow_grasp_trigger:
            return
        import subprocess

        env = self._ros_subprocess_env()
        services = [
            f"/{self.namespace}/global_costmap/clear_entirely_global_costmap",
            f"/{self.namespace}/local_costmap/clear_entirely_local_costmap",
        ]
        for service in services:
            subprocess.Popen(
                [
                    "timeout", "2", "ros2", "service", "call", service,
                    "nav2_msgs/srv/ClearEntireCostmap", "{}",
                ],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        print(f"[{self.namespace}] Nav2 costmap clear 요청")

    def _reset_robot2_escape_monitor(self):
        self._escape_reverse_t = 0.0
        self._escape_cooldown_t = 0.0
        self._stuck_watch_t = 0.0
        self._stuck_last_dist = None

    def _set_person2_collision_enabled(self, enabled):
        try:
            from pxr import Usd, UsdPhysics
            stage = omni.usd.get_context().get_stage()
            if stage is None:
                return

            roots = ["/World/Person2", "/World/Person2_LidarProxy"]
            changed = 0
            for root_path in roots:
                root = stage.GetPrimAtPath(root_path)
                if not root.IsValid():
                    continue
                for prim in Usd.PrimRange(root):
                    if not prim.HasAPI(UsdPhysics.CollisionAPI):
                        continue
                    collision_api = UsdPhysics.CollisionAPI(prim)
                    attr = collision_api.GetCollisionEnabledAttr()
                    if not attr.IsValid():
                        attr = collision_api.CreateCollisionEnabledAttr()
                    attr.Set(bool(enabled))
                    changed += 1
            state = "활성" if enabled else "비활성"
            print(f"[{self.namespace}] Person2 collision {state}: {changed}개")
        except Exception as e:
            print(f"[{self.namespace}] Person2 collision 설정 실패: {e}")

    def _start_robot2_rescue_to_exit(self):
        if self.allow_grasp_trigger:
            return False
        if self._rescue_active:
            self._set_person2_collision_enabled(False)
            self._clear_robot2_costmaps()
            self._send_nav2_goal(*self.EXIT_POS)
            return True
        print("사람이 탐지되었으므로 더 이상 탐색을 하지 않고 출구로 나갑니다.")
        self._rescue_active = True
        self._rescue_goal_done = False
        self._rescue_goal_retry_t = 0.0
        self._rescue_no_cmd_t = 0.0
        self._person2_follow_active = True
        self._person_follow_accum = self._person_follow_interval
        self._patrol_active = False
        self._patrol_spin_active = False
        self._patrol_spin_t = 0.0
        self._centered_locked = False
        self._tracking_command = np.zeros(3)
        self._nav_command = np.zeros(3)
        self._mark_nav2_goal_done()
        self._set_person2_collision_enabled(False)
        self._follow_person2_behind_robot(step_size=None)
        self._clear_robot2_costmaps()
        self._send_nav2_goal(*self.EXIT_POS)
        return True

    def _exit_fallback_command(self, nav_cmd, step_size):
        nav_cmd = np.array(nav_cmd, dtype=np.float32).copy()
        if np.linalg.norm(nav_cmd) > 0.02 and nav_cmd[0] > 0.03:
            self._rescue_no_cmd_t = 0.0
            return nav_cmd

        self._rescue_no_cmd_t += step_size
        self._rescue_goal_retry_t += step_size
        if self._rescue_goal_retry_t >= self.RESCUE_GOAL_RETRY_INTERVAL:
            self._rescue_goal_retry_t = 0.0
            self._clear_robot2_costmaps()
            self._send_nav2_goal(*self.EXIT_POS)

        pose = self._get_robot_pose_2d()
        if pose is None:
            return nav_cmd

        rx, ry, yaw = pose
        dx = self.EXIT_POS[0] - rx
        dy = self.EXIT_POS[1] - ry
        dist = (dx * dx + dy * dy) ** 0.5
        if dist < 0.8:
            return np.zeros(3, dtype=np.float32)

        desired_yaw = np.arctan2(dy, dx)
        heading_error = self._wrap_angle(desired_yaw - yaw)
        wz = np.clip(0.75 * heading_error, -self.DRIVE_MAX_WZ, self.DRIVE_MAX_WZ)

        vx = self.EXIT_FALLBACK_MAX_VX
        abs_error = abs(heading_error)
        if abs_error > 1.0:
            vx = 0.0
        elif abs_error > 0.55:
            vx = min(vx, 0.10)
        elif abs_error > 0.25:
            vx = min(vx, 0.18)
        fallback_cmd = np.array([vx, 0.0, wz], dtype=np.float32)
        now = getattr(self, "_sim_time_r2", 0.0)
        if now - getattr(self, "_last_exit_drive_log_t", -999.0) >= 1.0:
            print(
                f"[{self.namespace}] 출구 이동 fallback "
                f"(dist={dist:.2f}, heading={heading_error:.2f}, "
                f"vx={fallback_cmd[0]:.2f}, wz={fallback_cmd[2]:.2f})"
            )
            self._last_exit_drive_log_t = now
        return fallback_cmd

    def _person2_orientation_for_yaw(self, yaw):
        visual_yaw = yaw - (np.pi / 2.0)
        return np.array([
            np.cos(visual_yaw / 2.0),
            0.0,
            0.0,
            np.sin(visual_yaw / 2.0),
        ])

    def _set_person2_world_pose(self, x, y, yaw=None):
        try:
            from omni.isaac.core.prims import XFormPrim
            orient = (
                self._person2_orientation_for_yaw(yaw)
                if yaw is not None
                else np.array([0.70710678, 0.0, 0.0, -0.70710678])
            )
            XFormPrim("/World/Person2").set_world_pose(
                position=np.array([x, y, self.PERSON2_Z]),
                orientation=orient,
            )
            XFormPrim("/World/Person2_LidarProxy").set_world_pose(
                position=np.array([x, y, self.PERSON2_PROXY_Z]),
                orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            )
            self._update_person2_bbox_pose(x, y)
        except Exception as e:
            print(f"[{self.namespace}] Person2 follow 갱신 실패: {e}")

    def reset_person2_to_home(self):
        if self.allow_grasp_trigger:
            return
        self._person2_follow_active = False
        self._set_person2_collision_enabled(True)
        self._set_person2_world_pose(self.PERSON2_HOME[0], self.PERSON2_HOME[1])
        if not self._rescue_active:
            self._set_person2_bbox_visible(False)

    def _follow_person2_behind_robot(self, step_size=None):
        if not self._person2_follow_active:
            return
        if step_size is not None:
            self._person_follow_accum += step_size
            if self._person_follow_accum < self._person_follow_interval:
                return
            self._person_follow_accum = 0.0
        pose = self._get_robot_pose_2d()
        if pose is None:
            return
        rx, ry, yaw = pose
        target = np.array([
            rx - np.cos(yaw) * self.PERSON2_FOLLOW_DISTANCE,
            ry - np.sin(yaw) * self.PERSON2_FOLLOW_DISTANCE,
        ], dtype=np.float32)

        try:
            from omni.isaac.core.prims import XFormPrim
            current_pos, _ = XFormPrim("/World/Person2").get_world_pose()
            current = np.array([current_pos[0], current_pos[1]], dtype=np.float32)
        except Exception:
            current = target

        delta = target - current
        dist = float(np.linalg.norm(delta))
        if step_size is None or dist < 1e-4:
            next_xy = target
        else:
            catchup = np.clip(dist / max(self.PERSON2_FOLLOW_DISTANCE, 1e-6), 0.0, 1.0)
            follow_speed = self.PERSON2_FOLLOW_SPEED + (
                self.PERSON2_CATCHUP_SPEED - self.PERSON2_FOLLOW_SPEED
            ) * catchup
            max_step = max(0.02, follow_speed * step_size)
            next_xy = current + delta * min(1.0, max_step / max(dist, 1e-6))
        self._set_person2_world_pose(next_xy[0], next_xy[1], yaw)

    def _ensure_person2_bbox(self):
        if self._person2_bbox_paths:
            return
        try:
            from omni.isaac.core.objects import VisualCuboid
            from pxr import UsdGeom

            root_path = "/World/Person2_DetectionBox"
            stage = omni.usd.get_context().get_stage()
            UsdGeom.Xform.Define(stage, root_path)
            bars = []
            thickness = 0.035
            width = 0.8
            depth = 0.8
            height = 1.8

            def add_bar(name, scale):
                path = f"{root_path}/{name}"
                VisualCuboid(
                    prim_path=path,
                    name=name,
                    position=np.array([0.0, 0.0, -10.0]),
                    scale=np.array(scale),
                    color=np.array([1.0, 0.0, 0.0]),
                )
                prim = stage.GetPrimAtPath(path)
                if prim.IsValid():
                    UsdGeom.Imageable(prim).MakeInvisible()
                bars.append((path, scale))

            for z_name in ["bottom", "top"]:
                add_bar(f"{z_name}_front_x", [width, thickness, thickness])
                add_bar(f"{z_name}_back_x", [width, thickness, thickness])
                add_bar(f"{z_name}_left_y", [thickness, depth, thickness])
                add_bar(f"{z_name}_right_y", [thickness, depth, thickness])
            for corner in ["front_left", "front_right", "back_left", "back_right"]:
                add_bar(f"{corner}_z", [thickness, thickness, height])

            self._person2_bbox_paths = [path for path, _ in bars]
            self._person2_bbox_dims = (width, depth, height)
            self._set_person2_bbox_visible(False)
        except Exception as e:
            print(f"[{self.namespace}] Person2 바운딩 박스 생성 실패: {e}")

    def _update_person2_bbox_pose(self, x, y):
        if not self._person2_bbox_paths:
            return
        try:
            from omni.isaac.core.prims import XFormPrim
            width, depth, height = getattr(self, "_person2_bbox_dims", (0.8, 0.8, 1.8))
            zc = self.PERSON2_PROXY_Z
            z_min = zc - height / 2.0
            z_max = zc + height / 2.0
            positions = {
                "bottom_front_x": [x, y - depth / 2.0, z_min],
                "bottom_back_x": [x, y + depth / 2.0, z_min],
                "bottom_left_y": [x - width / 2.0, y, z_min],
                "bottom_right_y": [x + width / 2.0, y, z_min],
                "top_front_x": [x, y - depth / 2.0, z_max],
                "top_back_x": [x, y + depth / 2.0, z_max],
                "top_left_y": [x - width / 2.0, y, z_max],
                "top_right_y": [x + width / 2.0, y, z_max],
                "front_left_z": [x - width / 2.0, y - depth / 2.0, zc],
                "front_right_z": [x + width / 2.0, y - depth / 2.0, zc],
                "back_left_z": [x - width / 2.0, y + depth / 2.0, zc],
                "back_right_z": [x + width / 2.0, y + depth / 2.0, zc],
            }
            for name, pos in positions.items():
                XFormPrim(f"/World/Person2_DetectionBox/{name}").set_world_pose(
                    position=np.array(pos),
                    orientation=np.array([1.0, 0.0, 0.0, 0.0]),
                )
        except Exception as e:
            print(f"[{self.namespace}] Person2 바운딩 박스 갱신 실패: {e}")

    def _set_person2_bbox_visible(self, visible):
        self._ensure_person2_bbox()
        try:
            from pxr import UsdGeom
            stage = omni.usd.get_context().get_stage()
            for path in self._person2_bbox_paths:
                prim = stage.GetPrimAtPath(path)
                if not prim.IsValid():
                    continue
                imageable = UsdGeom.Imageable(prim)
                if visible:
                    imageable.MakeVisible()
                else:
                    imageable.MakeInvisible()
        except Exception:
            pass

    def _get_robot_xy(self):
        try:
            pos, _ = self._spot.robot.get_world_pose()
            return float(pos[0]), float(pos[1])
        except Exception:
            return None, None

    def _dist_to(self, tx, ty):
        rx, ry = self._get_robot_xy()
        if rx is None:
            return 999.0
        return ((rx - tx) ** 2 + (ry - ty) ** 2) ** 0.5

    def _get_robot_pose_2d(self):
        try:
            pos, q = self._spot.robot.get_world_pose()
            w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
            yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            return float(pos[0]), float(pos[1]), float(yaw)
        except Exception:
            return None

    @staticmethod
    def _wrap_angle(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def _load_robot2_plan(self):
        try:
            now = time.monotonic()
            if now - self._last_plan_poll_wall < self._plan_poll_interval:
                return self._robot2_plan
            self._last_plan_poll_wall = now

            mtime = os.path.getmtime(self._robot2_plan_file)
            if mtime < self._robot2_plan_min_mtime:
                return []
            if mtime == self._robot2_plan_mtime:
                return self._robot2_plan
            with open(self._robot2_plan_file, "r", encoding="utf-8") as f:
                payload = json.load(f)
            points = []
            for point in payload.get("points", []):
                if len(point) < 2:
                    continue
                points.append((float(point[0]), float(point[1])))
            self._robot2_plan = points
            self._robot2_plan_mtime = mtime
            return self._robot2_plan
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return self._robot2_plan

    def _robot2_plan_target(self, rx, ry, lookahead=1.4):
        plan = self._load_robot2_plan()
        if len(plan) < 2:
            return None

        closest_idx = min(
            range(len(plan)),
            key=lambda i: (plan[i][0] - rx) ** 2 + (plan[i][1] - ry) ** 2,
        )
        tx, ty = plan[closest_idx]
        traveled = 0.0
        for idx in range(closest_idx, len(plan) - 1):
            x0, y0 = plan[idx]
            x1, y1 = plan[idx + 1]
            segment = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
            if traveled + segment >= lookahead:
                ratio = (lookahead - traveled) / max(segment, 1e-6)
                return x0 + (x1 - x0) * ratio, y0 + (y1 - y0) * ratio
            traveled += segment
            tx, ty = x1, y1
        return tx, ty

    def _apply_robot2_path_tracking(self, clipped_cmd):
        pose = self._get_robot_pose_2d()
        if pose is None:
            return clipped_cmd

        rx, ry, yaw = pose
        target = self._robot2_plan_target(rx, ry)
        if target is None:
            return clipped_cmd

        tx, ty = target
        desired_yaw = np.arctan2(ty - ry, tx - rx)
        heading_error = self._wrap_angle(desired_yaw - yaw)
        abs_error = abs(heading_error)

        tracked_cmd = clipped_cmd.copy()
        tracked_cmd[1] = 0.0
        tracked_cmd[2] = np.clip(0.95 * heading_error, -self.DRIVE_MAX_WZ, self.DRIVE_MAX_WZ)

        if abs_error > 1.2:
            tracked_cmd[0] = min(tracked_cmd[0], 0.06)
        elif abs_error > 0.75:
            tracked_cmd[0] = min(max(tracked_cmd[0], 0.12), 0.20)
        elif abs_error > 0.45:
            tracked_cmd[0] = min(max(tracked_cmd[0], 0.20), 0.32)
        else:
            tracked_cmd[0] = min(max(tracked_cmd[0], 0.30), self.DRIVE_MAX_VX)
        return tracked_cmd

    def _get_base_tilt_angle(self):
        try:
            from isaacsim.core.utils.rotations import quat_to_rot_matrix
            _, q = self._spot.robot.get_world_pose()
            rot = quat_to_rot_matrix(q)
            upright_cos = float(np.clip(rot[2, 2], -1.0, 1.0))
            return float(np.arccos(upright_cos))
        except Exception:
            return 0.0

    def _zero_base_velocity(self):
        try:
            set_lin = getattr(self._spot.robot, "set_linear_velocity", None)
            if callable(set_lin):
                set_lin(np.zeros(3, dtype=np.float32))
            set_ang = getattr(self._spot.robot, "set_angular_velocity", None)
            if callable(set_ang):
                set_ang(np.zeros(3, dtype=np.float32))
        except Exception:
            pass

    def _stable_drive_command(self, command, step_size):
        cmd = np.array(command, dtype=np.float32).copy()
        max_vx = self.DRIVE_CARRY_MAX_VX if self.allow_grasp_trigger and self._has_object else self.DRIVE_MAX_VX

        cmd[0] = np.clip(cmd[0], 0.0, max_vx)
        cmd[1] = np.clip(cmd[1], -self.DRIVE_MAX_VY, self.DRIVE_MAX_VY)
        cmd[2] = np.clip(cmd[2], -self.DRIVE_MAX_WZ, self.DRIVE_MAX_WZ)

        if abs(cmd[2]) > 0.18:
            cmd[0] = min(cmd[0], 0.12)
        elif abs(cmd[2]) > 0.08:
            cmd[0] = min(cmd[0], 0.22)

        tilt = self._get_base_tilt_angle()
        if tilt >= self.TILT_STOP_RAD:
            cmd[:] = 0.0
            self._safe_drive_command[:] = 0.0
            self._zero_base_velocity()
            if hasattr(self._spot, "trigger_balance_mode"):
                self._spot.trigger_balance_mode(duration=1.5)
            now = time.time()
            if now - self._last_stability_log_t > 2.0:
                print(f"[{self.namespace}] 기울어짐 {np.rad2deg(tilt):.1f}도 → 주행 정지/균형 복구")
                self._last_stability_log_t = now
            return cmd

        if tilt >= self.TILT_SLOWDOWN_RAD:
            cmd[0] *= 0.35
            cmd[1] = 0.0
            cmd[2] *= 0.5
            if hasattr(self._spot, "trigger_balance_mode"):
                self._spot.trigger_balance_mode(duration=0.8)

        dt = max(float(step_size), 1.0 / 240.0)
        max_delta = self.DRIVE_SLEW * dt
        delta = np.clip(cmd - self._safe_drive_command, -max_delta, max_delta)
        self._safe_drive_command = self._safe_drive_command + delta
        return self._safe_drive_command.copy()

    # ------------------------------------------------------------------ #
    # robot1 자동 시나리오
    # ------------------------------------------------------------------ #
    def _pickup_extinguisher_for_delivery(self):
        self._grabbed_cube_path = None
        try:
            from omni.isaac.core.prims import XFormPrim
            from pxr import UsdGeom

            stage = omni.usd.get_context().get_stage()
            real_cube = XFormPrim("/World/Cube")
            real_cube.set_world_pose(position=np.array([0.0, 0.0, -10.0]))

            v_prim = stage.GetPrimAtPath(getattr(
                self, "_visual_cube_path", "/World/VisualGraspCube"
            ))
            if v_prim.IsValid():
                UsdGeom.Imageable(v_prim).MakeInvisible()
        except Exception as e:
            print(f"[{self.namespace}] 자동 소화기 파지 세팅 실패: {e}")

        self._has_object = True
        self._carry_arm = None
        self._delivery_state = "SEARCHING"
        self._spot.override_arm_angles = None
        self._spot.override_grip_angle = None
        print(f"[{self.namespace}] 소화기 파지 상태 세팅 완료")

    def _place_extinguisher_at_fire(self):
        place_pos = np.array([self.FIRE_POS[0], self.FIRE_POS[1], 0.18], dtype=np.float32)
        try:
            from omni.isaac.core.prims import XFormPrim
            from omni.isaac.core.prims.rigid_prim import RigidPrim
            from pxr import UsdGeom

            stage = omni.usd.get_context().get_stage()
            real_cube = RigidPrim("/World/Cube")
            real_cube.initialize()
            real_cube.set_world_pose(position=place_pos)
            real_cube.set_linear_velocity(np.zeros(3))
            try:
                real_cube.set_angular_velocity(np.zeros(3))
            except Exception:
                pass

            if self._grabbed_cube_path is not None:
                v_prim = stage.GetPrimAtPath(self._grabbed_cube_path)
                if v_prim.IsValid():
                    UsdGeom.Imageable(v_prim).MakeInvisible()
        except Exception as e:
            print(f"[{self.namespace}] 자동 소화기 놓기 실패: {e}")

        self._grabbed_cube_path = None
        self._has_object = False
        self._carry_arm = None
        self._delivery_state = "SEARCHING"
        self._spot.override_arm_angles = None
        self._spot.override_grip_angle = None
        print(f"[{self.namespace}] 2번 방에서 소화기 놓기 완료")

    def set_fire_detected(self):
        """main_simulation.py 화재 점화 시 호출"""
        if not self._auto_fire_triggered:
            self._auto_fire_triggered = True
            print(f"[{self.namespace}] 화재 신호 수신 → 자동 시나리오 시작")

    def _run_auto_scenario(self, step_size):
        if not self._auto_fire_triggered:
            return

        if self._auto_state == "IDLE":
            print(f"\n[{self.namespace}] 🔥 화재 감지! 소화기로 이동\n")
            self._send_nav2_goal(*self.EXTINGUISHER_POS)
            self._auto_state = "NAV_TO_EXTINGUISHER"
            self._nav_wait_t = 0.0

        elif self._auto_state == "NAV_TO_EXTINGUISHER":
            self._nav_wait_t += step_size
            dist = self._dist_to(*self.EXTINGUISHER_POS)
            if dist < 0.8 or self._nav_wait_t > 20.0:
                print(f"[{self.namespace}] 소화기 도착 ({dist:.2f}m) → 자동 Grasp")
                self._mark_nav2_goal_done()
                self._delivery_state = "ARRIVED"
                self._auto_state = "GRASPING"
                self._nav_wait_t = 0.0

        elif self._auto_state == "GRASPING":
            # Grasp 상태머신이 FOLD_ARM 완료 → SEARCHING + _has_object=True
            if self._delivery_state == "SEARCHING" and self._has_object:
                print(f"\n[{self.namespace}] 파지 완료 → 화재 위치로 이동\n")
                self._nav_command = np.zeros(3)  # 이전 cmd_vel 잔여값 제거
                self._send_nav2_goal(*self.FIRE_POS)
                self._auto_state = "NAV_TO_FIRE"
                self._nav_wait_t = 0.0
                self._fire_best_dist = float("inf")
                self._fire_stall_t = 0.0
                self._fire_goal_retry_t = 0.0

        elif self._auto_state == "NAV_TO_FIRE":
            self._nav_wait_t += step_size
            dist = self._dist_to(*self.FIRE_POS)
            self._fire_goal_retry_t += step_size

            if dist < self._fire_best_dist - self.FIRE_PROGRESS_EPS:
                self._fire_best_dist = dist
                self._fire_stall_t = 0.0
            else:
                self._fire_stall_t += step_size

            close_enough = dist < self.FIRE_THROW_DISTANCE
            best_reachable = (
                dist < self.FIRE_STALLED_THROW_DISTANCE
                and self._fire_stall_t > self.FIRE_APPROACH_STALL_TIME
                and self._nav_wait_t > 12.0
            )

            should_retry_goal = (
                self._fire_goal_retry_t > 15.0
                and (self._fire_stall_t > 3.0 or not self._nav_goal_started)
            )
            if not close_enough and not best_reachable and should_retry_goal:
                print(
                    f"[{self.namespace}] 화재까지 접근 중 "
                    f"(현재 {dist:.2f}m, 최단 {self._fire_best_dist:.2f}m) → 목표 재전송"
                )
                self._send_nav2_goal(*self.FIRE_POS)
                self._fire_goal_retry_t = 0.0

            if close_enough or best_reachable:
                reason = "화재 위치 근접" if close_enough else "최대한 접근 완료"
                print(f"[{self.namespace}] {reason} ({dist:.2f}m) → 자동 투척")
                self._mark_nav2_goal_done()
                self._delivery_state = "DROP_SETTLE"
                self._grasp_t = 0.0
                self._auto_state = "THROWING"
                self._nav_wait_t = 0.0

        elif self._auto_state == "THROWING":
            if self._delivery_state == "SEARCHING":
                print(f"\n[{self.namespace}] 투척 완료! 출구로 이동\n")
                self._nav_command = np.zeros(3)
                self._send_nav2_goal(*self.EXIT_POS)
                self._auto_state = "NAV_TO_EXIT"
                self._nav_wait_t = 0.0

        elif self._auto_state == "NAV_TO_EXIT":
            self._nav_wait_t += step_size
            dist = self._dist_to(*self.EXIT_POS)
            if dist < 0.8 or self._nav_wait_t > 60.0:
                print(f"[{self.namespace}] 출구 도착 ({dist:.2f}m) → 시나리오 종료")
                self._mark_nav2_goal_done()
                self._auto_state = "DONE"

    # ------------------------------------------------------------------ #
    # robot2 순찰
    # ------------------------------------------------------------------ #
    def _run_patrol_step(self, step_size):
        if self._rescue_active:
            return
        if not self._patrol_active:
            return
        # YOLO가 사람 발견 중이면 순찰 일시 정지
        if getattr(self, "_yolo_state", "SEARCHING") != "SEARCHING":
            return
        if self._patrol_spin_active:
            self._patrol_spin_t += step_size
            if self._patrol_spin_t >= self.ROBOT2_ROOM_SCAN_SPIN_DURATION:
                print(f"[{self.namespace}] 방 스캔 회전 완료 → 다음 웨이포인트")
                self._patrol_spin_active = False
                self._patrol_spin_t = 0.0
                self._patrol_idx = (self._patrol_idx + 1) % len(self._patrol_waypoints)
                nx, ny = self._patrol_waypoints[self._patrol_idx]
                self._send_nav2_goal(nx, ny)
                self._patrol_wait_t = 0.0
            return
        if not self._nav_goal_started:
            return

        self._patrol_wait_t += step_size
        tx, ty = self._patrol_waypoints[self._patrol_idx]
        dist = self._dist_to(tx, ty)

        if dist < self.ROBOT2_WAYPOINT_ARRIVAL_TOLERANCE:
            print(
                f"[{self.namespace}] 웨이포인트 {self._patrol_idx} 도착 "
                f"({dist:.2f}m < {self.ROBOT2_WAYPOINT_ARRIVAL_TOLERANCE:.2f}m) → 방 스캔"
            )
            self._mark_nav2_goal_done()
            self._nav_command = np.zeros(3)
            self._patrol_spin_active = True
            self._patrol_spin_t = 0.0
            print(
                f"[{self.namespace}] 방 스캔 시작: 제자리 1회전 "
                f"({self.ROBOT2_ROOM_SCAN_SPIN_DURATION:.1f}s)"
            )

    def _robot2_drive_command(self, total_cmd, step_size):
        clipped_cmd = total_cmd.copy()
        clipped_cmd[0] = np.clip(clipped_cmd[0], 0.0, self.DRIVE_MAX_VX)
        clipped_cmd[1] = np.clip(clipped_cmd[1], -self.DRIVE_MAX_VY, self.DRIVE_MAX_VY)
        clipped_cmd[2] = np.clip(clipped_cmd[2], -self.DRIVE_MAX_WZ, self.DRIVE_MAX_WZ)

        self._escape_cooldown_t = max(0.0, self._escape_cooldown_t - step_size)
        if not self._patrol_active or not self._nav_goal_started:
            return clipped_cmd
        if getattr(self, "_yolo_state", "SEARCHING") != "SEARCHING":
            return clipped_cmd

        clipped_cmd = self._apply_robot2_path_tracking(clipped_cmd)

        tx, ty = self._patrol_waypoints[self._patrol_idx]
        dist = self._dist_to(tx, ty)
        if self._stuck_last_dist is None:
            self._stuck_last_dist = dist
            return clipped_cmd

        self._stuck_watch_t += step_size
        if self._stuck_watch_t < 2.0:
            return clipped_cmd

        progress = self._stuck_last_dist - dist
        if progress < 0.08 and dist > 2.0:
            print(
                f"[{self.namespace}] 전진 진행 없음({progress:.2f}m/2.0s) "
                "→ 후진 없이 경로 추종 유지"
            )

        self._stuck_watch_t = 0.0
        self._stuck_last_dist = dist
        return clipped_cmd

    def _robot1_drive_command(self, total_cmd, step_size):
        clipped_cmd = total_cmd.copy()
        max_vx = self.DRIVE_CARRY_MAX_VX if self._has_object else self.DRIVE_MAX_VX
        clipped_cmd[0] = np.clip(clipped_cmd[0], 0.0, max_vx)
        clipped_cmd[1] = np.clip(clipped_cmd[1], -self.DRIVE_MAX_VY, self.DRIVE_MAX_VY)
        clipped_cmd[2] = np.clip(clipped_cmd[2], -self.DRIVE_MAX_WZ, self.DRIVE_MAX_WZ)

        if self._nav_goal_active and self._nav_goal_started:
            clipped_cmd = self._apply_robot2_path_tracking(clipped_cmd)
        return self._stable_drive_command(clipped_cmd, step_size)

    # ------------------------------------------------------------------ #
    # 포즈 파일 퍼블리시
    # ------------------------------------------------------------------ #
    def _publish_pose_file(self, step_size=None):
        if step_size is not None:
            self._pose_publish_accum += step_size
            if self._pose_publish_accum < self._pose_publish_interval:
                return
            self._pose_publish_accum = 0.0
        try:
            pos, q = self._spot.robot.get_world_pose()
            payload = {
                "namespace": self.namespace,
                "position": [float(pos[0]), float(pos[1]), float(pos[2])],
                "orientation": [float(q[0]), float(q[1]), float(q[2]), float(q[3])],
            }
            tmp = f"{self._pose_file}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(payload, f)
            os.replace(tmp, self._pose_file)
        except Exception as e:
            print(f"[{self.namespace}] pose file 쓰기 실패: {e}")

    # ------------------------------------------------------------------ #
    # 키보드: G = Grasp 시작, Q = 투척
    # ------------------------------------------------------------------ #
    def _on_agent_keyboard(self, event, *args, **kwargs) -> bool:
        import carb
        if not self.allow_grasp_trigger:
            return True
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name == "G":
                if self._delivery_state == "SEARCHING":
                    # 거리 체크
                    try:
                        from omni.isaac.core.prims import XFormPrim
                        robot_prim = XFormPrim(f"/World/{self.namespace}/body")
                        cube_prim  = XFormPrim("/World/Cube")
                        r_pos, _ = robot_prim.get_world_pose()
                        c_pos, _ = cube_prim.get_world_pose()
                        dist = np.linalg.norm(r_pos[:2] - c_pos[:2])
                        if dist > 0.8:
                            print(f"[{self.namespace}] 소화기가 너무 멉니다 ({dist:.2f}m). 더 가까이 이동하세요.")
                            return True
                    except Exception:
                        pass
                    print(f"[{self.namespace}] G키 → Grasp 시퀀스 시작")
                    self._delivery_state = "ARRIVED"

            elif event.input.name == "Q":
                if self._delivery_state == "SEARCHING" and self._has_object:
                    print(f"[{self.namespace}] Q키 → 소화기 투척")
                    self._delivery_state = "DROP_SETTLE"
                    self._grasp_t = 0.0
        return True

    # ------------------------------------------------------------------ #
    # 내부 헬퍼
    # ------------------------------------------------------------------ #
    def _dp(self):
        return np.array(self._spot.default_pos, dtype=np.float32).reshape(-1)

    def _hold(self, arm6, grip):
        from isaacsim.core.utils.types import ArticulationAction
        full = self._stance.copy()
        for k, idx in enumerate(self.ARM_IDX):
            full[idx] = arm6[k]
        full[self.GRIP_IDX] = grip
        self._spot.robot.apply_action(ArticulationAction(joint_positions=full))

    def _arm_override(self, arm6, grip):
        from isaacsim.core.utils.types import ArticulationAction
        vals = np.array(list(arm6) + [grip], dtype=np.float32)
        idxs = np.array(self.ARM_IDX + [self.GRIP_IDX])
        self._spot.robot.apply_action(
            ArticulationAction(joint_positions=vals, joint_indices=idxs)
        )

    def _set_heavy_mode(self, enable: bool):
        if getattr(self, "_is_heavy_mode", False) == enable:
            return
        from pxr import UsdPhysics
        stage = omni.usd.get_context().get_stage()
        factor = 3.0 if enable else (1.0 / 3.0)
        leg_kw = ["hip", "uleg", "lleg"]
        for prim in stage.TraverseAll():
            path = prim.GetPath().pathString
            n = prim.GetName()
            if not path.startswith(f"/World/{self.namespace}"):
                continue
            if any(k in n for k in leg_kw) or n == "body":
                mp = UsdPhysics.MassAPI.Get(stage, prim.GetPath())
                if mp:
                    cur = mp.GetMassAttr().Get()
                    if cur is not None:
                        mp.GetMassAttr().Set(float(cur * factor))
        self._is_heavy_mode = enable
        print(f"[{self.namespace}] 질량 {'x3.0 적용' if enable else '원상복구'}")

    # ------------------------------------------------------------------ #
    # 센서 셋업
    # ------------------------------------------------------------------ #
    def setup_sensors(self):
        stage = omni.usd.get_context().get_stage()
        lidar_parent = f"/World/{self.namespace}/body"
        lidar_horizontal_resolution = float(
            os.environ.get("COBOT_LIDAR_HORIZONTAL_RESOLUTION", "1.0")
        )

        success, lidar = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="Functional_Lidar",
            parent=lidar_parent,
            min_range=0.65,
            max_range=20.0,
            draw_points=False,
            draw_lines=False,
            horizontal_fov=360.0,
            vertical_fov=1.0,
            horizontal_resolution=lidar_horizontal_resolution,
            vertical_resolution=1.0,
            rotation_rate=0.0,
            high_lod=False,
            yaw_offset=0.0,
            enable_semantics=False,
        )

        if success:
            sensor_prim_path = lidar.GetPath()
            lidar.GetPrim().GetAttribute("xformOp:translate").Set(
                Gf.Vec3d(0.0, 0.0, 0.25)
            )
            print(f"[{self.namespace}] Lidar 생성: {sensor_prim_path}")
        else:
            print(f"[{self.namespace}] Lidar 생성 실패")
            sensor_prim_path = None

        self._setup_ros2_graph(sensor_prim_path)

        # YOLO 그리퍼 카메라 (robot2 전용)
        if not self.allow_grasp_trigger:
            self._setup_yolo_camera()

        # VisualGraspCube (robot1 전용)
        if self.allow_grasp_trigger:
            try:
                cube_path = "/World/VisualGraspCube"
                from omni.isaac.core.utils.stage import add_reference_to_stage
                from pxr import UsdGeom
                import os as _os
                base_dir = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
                ext_usd = _os.path.join(
                    base_dir, "map", "fire_extinguisher", "World0.usd"
                )
                if not stage.GetPrimAtPath(cube_path).IsValid():
                    add_reference_to_stage(ext_usd, cube_path)
                v_prim = stage.GetPrimAtPath(cube_path)
                if v_prim.IsValid():
                    UsdGeom.Imageable(v_prim).MakeInvisible()
                    xform = UsdGeom.Xformable(v_prim)
                    xform.ClearXformOpOrder()
                    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -10.0))
                self._visual_cube_path = cube_path
                print(f"[{self.namespace}] VisualGraspCube 생성 완료: {cube_path}")
            except Exception as e:
                print(f"[{self.namespace}] VisualGraspCube 생성 실패: {e}")

        # Headlight
        light_path = f"{lidar_parent}/Headlight"
        headlight = UsdLux.SphereLight.Define(stage, light_path)
        headlight.CreateIntensityAttr(
            float(os.environ.get("COBOT_HEADLIGHT_INTENSITY", "80000"))
        )
        headlight.GetPrim().CreateAttribute(
            "exposure", Sdf.ValueTypeNames.Float
        ).Set(5.0)
        headlight.CreateRadiusAttr(0.05)
        headlight.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.8))
        headlight.AddTranslateOp().Set(Gf.Vec3d(0.65, 0.0, 0.1))
        print(f"[{self.namespace}] 센서 셋업 완료")

    def _setup_yolo_camera(self):
        self._camera_gripper = None
        self._camera_initialized = False
        self._yolo_counter = 0
        self._yolo_interval = 1.0 / max(
            1.0, float(os.environ.get("COBOT_YOLO_HZ", "2.0"))
        )
        self._yolo_conf = float(os.environ.get("COBOT_YOLO_CONF", "0.35"))
        self._camera_width = int(os.environ.get("COBOT_CAMERA_WIDTH", "224"))
        self._camera_height = int(os.environ.get("COBOT_CAMERA_HEIGHT", "168"))
        self._yolo_accum = self._yolo_interval
        self._last_yolo_keepalive_log_t = 0.0
        self._yolo_state = "SEARCHING"
        self._tracking_command = np.zeros(3)
        self._was_person_detected = False
        self._yolo_camera_empty_warned = False
        self._last_person_best_cx = None
        self._last_person_best_depth = None
        self._last_person_seen_t = 0.0
        self._last_person_approach_log_t = -999.0
        self._last_person_align_log_t = -999.0
        self._person_approach_started_t = None

        try:
            try:
                from isaacsim.sensors.camera import Camera
                camera_api = "isaacsim.sensors.camera.Camera"
            except Exception:
                from omni.isaac.sensor import Camera
                camera_api = "omni.isaac.sensor.Camera"

            cam_path = f"/World/{self.namespace}/arm0_link_wr1/gripper_camera"
            try:
                self._camera_gripper = Camera(
                    prim_path=cam_path,
                    resolution=(self._camera_width, self._camera_height),
                    translation=np.array([0.1, 0.0, 0.0]),
                )
            except TypeError:
                self._camera_gripper = Camera(
                    prim_path=cam_path,
                    resolution=(self._camera_width, self._camera_height),
                    position=np.array([0.1, 0.0, 0.0]),
                )
            print(f"[{self.namespace}] 그리퍼 카메라 생성 완료: {camera_api}")

            if os.environ.get("COBOT_HEADLESS") == "1":
                print(f"[{self.namespace}] headless 실행: 카메라 viewport 생성 생략")
            else:
                try:
                    import omni.kit.viewport.utility as vp_util
                    vp = vp_util.create_viewport_window(
                        "robot2 Gripper View",
                        width=self._camera_width,
                        height=self._camera_height,
                    )
                    if vp:
                        vp.viewport_api.set_active_camera(cam_path)
                        print(f"[{self.namespace}] 뷰포트 창 생성: robot2 Gripper View")
                except Exception as ve:
                    print(f"[{self.namespace}] 뷰포트 창 생성 실패 (무시): {ve}")
        except Exception as e:
            print(f"[{self.namespace}] 그리퍼 카메라 생성 실패: {e}")

        self.yolo_enabled = False
        try:
            from ultralytics import YOLO
            import os as _os
            model_path = _os.path.join(
                _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))),
                "yolov8n.pt",
            )
            self.yolo_model = YOLO(model_path)
            self.yolo_enabled = True
            print(f"[{self.namespace}] YOLOv8n 로드 완료: {model_path}")
        except Exception as e:
            print(f"[{self.namespace}] YOLO 로드 실패: {e}")

    def _setup_ros2_graph(self, sensor_prim_path):
        import omni.graph.core as og
        from omni.isaac.core.utils.extensions import enable_extension

        enable_extension("isaacsim.core.nodes")
        enable_extension("isaacsim.ros2.bridge")
        enable_extension("isaacsim.sensors.physx")

        try:
            keys = og.Controller.Keys
            stage = omni.usd.get_context().get_stage()

            nodes = [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("ReadLidar", "isaacsim.sensors.physx.IsaacReadLidarBeams"),
                ("PublishScan", "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
            ]

            conns = [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("Context.outputs:context", "SubscribeTwist.inputs:context"),
                ("OnPlaybackTick.outputs:tick", "ReadLidar.inputs:execIn"),
                ("ReadLidar.outputs:execOut", "PublishScan.inputs:execIn"),
                ("Context.outputs:context", "PublishScan.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishScan.inputs:timeStamp"),
                ("ReadLidar.outputs:azimuthRange", "PublishScan.inputs:azimuthRange"),
                ("ReadLidar.outputs:depthRange", "PublishScan.inputs:depthRange"),
                ("ReadLidar.outputs:horizontalFov", "PublishScan.inputs:horizontalFov"),
                ("ReadLidar.outputs:horizontalResolution", "PublishScan.inputs:horizontalResolution"),
                ("ReadLidar.outputs:intensitiesData", "PublishScan.inputs:intensitiesData"),
                ("ReadLidar.outputs:linearDepthData", "PublishScan.inputs:linearDepthData"),
                ("ReadLidar.outputs:numCols", "PublishScan.inputs:numCols"),
                ("ReadLidar.outputs:numRows", "PublishScan.inputs:numRows"),
                ("ReadLidar.outputs:rotationRate", "PublishScan.inputs:rotationRate"),
            ]

            vals = [
                ("SubscribeTwist.inputs:topicName", f"{self.namespace}/cmd_vel"),
                ("PublishScan.inputs:topicName", f"{self.namespace}/scan"),
                ("PublishScan.inputs:frameId", f"{self.namespace}/Functional_Lidar"),
            ]

            if sensor_prim_path:
                import usdrt.Sdf
                path_str = (
                    str(sensor_prim_path.GetPath())
                    if hasattr(sensor_prim_path, "GetPath")
                    else str(sensor_prim_path)
                )
                vals.append(("ReadLidar.inputs:lidarPrim", [usdrt.Sdf.Path(path_str)]))

            og.Controller.edit(
                {
                    "graph_path": f"/ROS2_Graph_{self.namespace}",
                    "evaluator_name": "execution",
                },
                {
                    keys.CREATE_NODES: nodes,
                    keys.CONNECT: conns,
                    keys.SET_VALUES: vals,
                },
            )
            print(f"[{self.namespace}] ROS2 Graph 생성 완료")
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"[{self.namespace}] ROS2 Graph 생성 실패: {e}")

    # ------------------------------------------------------------------ #
    # 물리 스텝
    # ------------------------------------------------------------------ #
    def on_physics_step(self, step_size, base_command) -> None:
        import struct

        self._publish_pose_file(step_size)

        # 투척 후 소화기 회전 이펙트
        if self._is_thrown and hasattr(self, "_thrown_cube"):
            try:
                vel = self._thrown_cube.get_angular_velocity()
                if vel is not None:
                    vel[0] = vel[0] * 0.95
                    vel[1] = vel[1] * 0.95
                    vel[2] = 25.0
                    self._thrown_cube.set_angular_velocity(vel)
            except Exception:
                pass

        # UDP cmd_vel 수신
        try:
            while True:
                data, _ = self.udp_sock.recvfrom(1024)
                if len(data) >= 12:
                    vx, vy, wz = struct.unpack("fff", data[:12])
                    # Nav2 cmd_vel 속도 클리핑 (RL policy 안정성 확보)
                    self._nav_command[1] = np.clip(vy, -self.DRIVE_MAX_VY, self.DRIVE_MAX_VY)
                    max_vx = self.DRIVE_CARRY_MAX_VX if self.allow_grasp_trigger and self._has_object else self.DRIVE_MAX_VX
                    vx_cmd = np.clip(vx, 0.0, max_vx)
                    wz_cmd = np.clip(wz, -self.DRIVE_MAX_WZ, self.DRIVE_MAX_WZ)
                    if abs(wz_cmd) > 0.04 and vx > 0.01:
                        vx_cmd = min(vx_cmd, 0.20)
                    self._nav_command[0] = vx_cmd
                    self._nav_command[2] = wz_cmd
                    if abs(vx) + abs(vy) + abs(wz) > 0.01:
                        self._nav_goal_started = True
        except BlockingIOError:
            pass
        except Exception as e:
            print(f"[{self.namespace}] UDP 읽기 오류: {e}")

        # 첫 스텝 초기화
        if self.first_step:
            self._spot.initialize()
            self._spot.robot.set_joint_positions(self._spot.default_pos)
            self._spot.robot.set_joint_velocities(self._spot.default_vel)
            self._safe_drive_command[:] = 0.0
            self.first_step = False
            print(f"[{self.namespace}] 로봇 초기화 완료")

            # initialpose → Nav2
            try:
                import subprocess
                pos, q = self._spot.robot.get_world_pose()
                x, y = pos[0], pos[1]
                msg = (
                    f"{{header: {{frame_id: 'map'}}, pose: {{pose: {{position: "
                    f"{{x: {x}, y: {y}, z: 0.0}}, orientation: {{w: {q[0]}, "
                    f"x: {q[1]}, y: {q[2]}, z: {q[3]}}}}}, covariance: "
                    f"[0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,"
                    f"0,0,0,0,0,0,0,0,0,0,0,0.068]}}}}"
                )
                env = os.environ.copy()
                if "PYTHONPATH" in env:
                    env["PYTHONPATH"] = ":".join(
                        p for p in env["PYTHONPATH"].split(":")
                        if "isaacsim" not in p.lower()
                    )
                ns = self.namespace
                subprocess.Popen(
                    ["timeout", "2", "ros2", "topic", "pub", "--once",
                     f"/{ns}/initialpose",
                     "geometry_msgs/msg/PoseWithCovarianceStamped", msg],
                    env=env,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                print(f"[{self.namespace}] initialpose 전송: x={x:.2f}, y={y:.2f}")
            except Exception as e:
                print(f"[{self.namespace}] initialpose 전송 실패: {e}")

            if not self.allow_grasp_trigger:
                self.reset_person2_to_home()
                print(f"[{self.namespace}] Person2 원래 위치로 복귀: (8.82, -9.56)")

            # Grasp용 게인 초기화 (robot1 전용)
            if self.allow_grasp_trigger:
                try:
                    dp = self._dp()
                    names = (
                        list(self._spot.robot.dof_names)
                        if hasattr(self._spot.robot, "dof_names")
                        else self._spot.robot._articulation_view.joint_names
                    )
                    stance = dp.copy()
                    for i, nm in enumerate(names):
                        if "hx" in nm:
                            stance[i] = 0.0
                        elif "hy" in nm:
                            stance[i] = 0.8
                        elif "kn" in nm:
                            stance[i] = -1.5
                    self._stance = stance
                    self._cur_arm = stance[self.ARM_IDX].copy()
                    self._cur_grip = -1.571
                    self._kps0, self._kds0 = (
                        self._spot.robot._articulation_view.get_gains()
                    )
                    print(f"[{self.namespace}] Grasp 초기화 완료 — G키: Grasp, Q키: 투척")
                except Exception as e:
                    print(f"[{self.namespace}] Grasp 초기화 오류: {e}")
            else:
                # robot2 순찰 시작
                self._patrol_active = True
                tx, ty = self._patrol_waypoints[0]
                self._send_nav2_goal(tx, ty)
                print(f"[{self.namespace}] 순찰 시작 → 웨이포인트 0 ({tx}, {ty})")
            return

        total_cmd = base_command + self._nav_command
        self._retry_nav2_goal_until_cmd(step_size)

        # robot2: 순찰 + YOLO 상태머신
        if not self.allow_grasp_trigger:
            clipped_cmd = self._robot2_drive_command(total_cmd, step_size)
            self._run_patrol_step(step_size)
            if self._patrol_spin_active:
                clipped_cmd = np.array([0.0, 0.0, self.ROBOT2_ROOM_SCAN_SPIN_SPEED])
            self._run_yolo_step(step_size, clipped_cmd)
            return

        # robot1: 자동 시나리오
        self._run_auto_scenario(step_size)

        # -------- robot1 Grasp 상태머신 --------
        if self._delivery_state == "SEARCHING":
            drive_cmd = self._robot1_drive_command(total_cmd, step_size)
            self._spot.forward(step_size, drive_cmd)
            if self._carry_arm is not None:
                self._carry_arm += np.clip(
                    self._carry_tgt - self._carry_arm, -0.01, 0.01
                )
                self._arm_override(self._carry_arm, self.GRIP_CLOSE)
            elif self._has_object:
                self._spot.override_arm_angles = None
                self._spot.override_grip_angle = self.GRIP_CLOSE

        elif self._delivery_state == "ARRIVED":
            print(f"\n[{self.namespace}] 목적지 도착 → SETTLE\n")
            self._delivery_state = "SETTLE"
            self._grasp_t = 0.0
            try:
                nd = len(self._dp())
                kps = np.full(nd, 2000.0)
                kds = np.full(nd, 100.0)
                for idx in self.ARM_IDX:
                    kps[idx] = 5000.0
                    kds[idx] = 250.0
                kps[self.GRIP_IDX] = 5000.0
                kds[self.GRIP_IDX] = 250.0
                self._spot.robot._articulation_view.set_gains(
                    kps=kps.reshape(1, -1), kds=kds.reshape(1, -1)
                )
                self._cur_arm = self._stance[self.ARM_IDX].copy()
                self._cur_grip = -1.571
            except Exception as e:
                print(f"[{self.namespace}] 강성 부스트 오류: {e}")

        elif self._delivery_state == "SETTLE":
            self._hold(self._cur_arm, self._cur_grip)
            self._grasp_t += step_size
            if self._grasp_t > 2.5:
                self._delivery_state = "HOVER"
                self._grasp_t = 0.0
                print(f"[{self.namespace}] SETTLE 완료 → HOVER")

        elif self._delivery_state == "DROP_SETTLE":
            try:
                nd = len(self._dp())
                kps = np.full(nd, 2000.0)
                kds = np.full(nd, 100.0)
                for idx in self.ARM_IDX:
                    kps[idx] = 5000.0
                    kds[idx] = 250.0
                kps[self.GRIP_IDX] = 5000.0
                kds[self.GRIP_IDX] = 250.0
                self._spot.robot._articulation_view.set_gains(
                    kps=kps.reshape(1, -1), kds=kds.reshape(1, -1)
                )
                current_positions = self._spot.robot.get_joint_positions()
                self._cur_arm = current_positions[self.ARM_IDX].copy()
                self._cur_grip = self.GRIP_CLOSE
                print(f"\n[{self.namespace}] DROP_SETTLE → DROP_REACH\n")
            except Exception:
                pass
            self._delivery_state = "DROP_REACH"
            self._grasp_t = 0.0

        elif self._delivery_state in [
            "HOVER", "GRASP", "CLOSE", "LIFT", "DONE",
            "DROP_REACH", "DROP_OPEN", "DROP_DONE",
        ]:
            self._grasp_t += step_size

            POSES = [
                (np.array([0.0, -0.85, 1.41, 0.0, 1.05, 0.0], dtype=np.float32), -1.571),  # 0: Hover
                (np.array([0.0, -0.65, 1.41, 0.0, 0.95, 0.0], dtype=np.float32), -1.571),  # 1: Grasp
                (np.array([0.0, -2.95, 2.95, 0.0, 1.20, 0.0], dtype=np.float32), -0.5),    # 2: Lift
            ]

            targets = {
                "HOVER": (POSES[0][0], -1.571, "GRASP"),
                "GRASP": (POSES[1][0], -1.571, "CLOSE"),
                "LIFT":  (POSES[2][0], self.GRIP_CLOSE, "DONE"),
            }

            if self._delivery_state in targets:
                tgt_arm, grip, nxt = targets[self._delivery_state]
                self._cur_arm += np.clip(tgt_arm - self._cur_arm, -0.02, 0.02)
                self._hold(self._cur_arm, grip)
                if (
                    np.max(np.abs(tgt_arm - self._cur_arm)) < 0.02
                    and self._grasp_t > 1.0
                ):
                    print(f"[{self.namespace}] {self._delivery_state} → {nxt}")
                    self._delivery_state = nxt
                    self._grasp_t = 0.0

            elif self._delivery_state == "CLOSE":
                self._cur_grip = max(self.GRIP_CLOSE, self._cur_grip - 0.03)
                self._hold(POSES[1][0], self._cur_grip)
                if self._cur_grip <= self.GRIP_CLOSE + 1e-3 and self._grasp_t > 1.5:
                    self._cur_arm = POSES[1][0].copy()
                    print(f"[{self.namespace}] CLOSE 완료 — Magnetic Grasp 활성화")
                    # Magnetic Grasp: 물리 큐브 Z=-10 숨기고 시각 큐브 표시
                    self._grabbed_cube_path = getattr(
                        self, "_visual_cube_path", "/World/VisualGraspCube"
                    )
                    try:
                        from omni.isaac.core.prims import XFormPrim
                        from pxr import UsdGeom
                        stage = omni.usd.get_context().get_stage()

                        real_cube = XFormPrim("/World/Cube")
                        real_cube.set_world_pose(position=np.array([0.0, 0.0, -10.0]))

                        v_prim = stage.GetPrimAtPath(self._grabbed_cube_path)
                        if v_prim.IsValid():
                            UsdGeom.Imageable(v_prim).MakeVisible()
                        print(f"[{self.namespace}] 소화기 집기 완료!")
                    except Exception as e:
                        print(f"[{self.namespace}] Grasp 스왑 실패: {e}")
                    self._delivery_state = "LIFT"
                    self._grasp_t = 0.0

            elif self._delivery_state == "DONE":
                self._hold(POSES[2][0], self.GRIP_CLOSE)
                if self._grasp_t > 1.5:
                    print(f"[{self.namespace}] DONE → FOLD_ARM")
                    self._delivery_state = "FOLD_ARM"
                    self._grasp_t = 0.0
                    self._carry_arm = POSES[2][0].copy()
                    self._carry_tgt = self._dp()[self.ARM_IDX].copy()

            elif self._delivery_state == "DROP_REACH":
                tgt_arm = POSES[1][0]
                self._cur_arm += np.clip(tgt_arm - self._cur_arm, -0.02, 0.02)
                self._hold(self._cur_arm, self.GRIP_CLOSE)
                if (
                    np.max(np.abs(tgt_arm - self._cur_arm)) < 0.02
                    and self._grasp_t > 1.5
                ):
                    print(f"[{self.namespace}] DROP_REACH → DROP_OPEN")
                    self._delivery_state = "DROP_OPEN"
                    self._grasp_t = 0.0

            elif self._delivery_state == "DROP_OPEN":
                self._cur_grip = max(-1.571, self._cur_grip - 0.03)
                self._hold(self._cur_arm, self._cur_grip)
                if self._cur_grip <= -1.571 + 1e-3 and self._grasp_t > 1.0:
                    print(f"[{self.namespace}] DROP_OPEN → 투척!")
                    if self._grabbed_cube_path is not None:
                        try:
                            from omni.isaac.core.prims import XFormPrim
                            from omni.isaac.core.prims.rigid_prim import RigidPrim
                            from pxr import UsdGeom
                            stage = omni.usd.get_context().get_stage()

                            vis_cube = XFormPrim(self._grabbed_cube_path)
                            pos, rot = vis_cube.get_world_pose()

                            real_cube = RigidPrim("/World/Cube")
                            real_cube.initialize()

                            body_prim = XFormPrim(f"/World/{self.namespace}/body")
                            body_pos, _ = body_prim.get_world_pose()

                            dir_vec = pos[:2] - body_pos[:2]
                            dist = np.linalg.norm(dir_vec)

                            # 충돌 방지: 던지는 방향으로 25cm 오프셋
                            safe_pos = pos.copy()
                            if dist > 0.01:
                                safe_pos[0] += (dir_vec[0] / dist) * 0.25
                                safe_pos[1] += (dir_vec[1] / dist) * 0.25

                            real_cube.set_world_pose(position=safe_pos, orientation=rot)

                            self._is_thrown = True
                            self._thrown_cube = real_cube

                            if dist > 0.01:
                                dir_vec = dir_vec / dist
                            else:
                                dir_vec = np.array([1.0, 0.0])

                            throw_vel = np.array([
                                dir_vec[0] * 2.5,
                                dir_vec[1] * 2.5,
                                1.0,
                            ])
                            real_cube.set_linear_velocity(throw_vel)

                            v_prim = stage.GetPrimAtPath(self._grabbed_cube_path)
                            if v_prim.IsValid():
                                UsdGeom.Imageable(v_prim).MakeInvisible()
                            print(f"[{self.namespace}] 소화기 투척 완료!")
                        except Exception as e:
                            print(f"[{self.namespace}] 투척 실패: {e}")
                        self._grabbed_cube_path = None
                    self._has_object = False
                    self._carry_arm = None
                    self._spot.override_arm_angles = None
                    self._spot.override_grip_angle = None
                    try:
                        kps = np.array(self._kps0, dtype=np.float32).reshape(1, -1)
                        kds = np.array(self._kds0, dtype=np.float32).reshape(1, -1)
                        self._spot.robot._articulation_view.set_gains(kps=kps, kds=kds)
                    except Exception:
                        pass
                    self._set_heavy_mode(False)
                    if hasattr(self._spot, "action") and hasattr(self._spot, "_action_scale"):
                        current_positions = self._spot.robot.get_joint_positions()
                        self._spot.action = (
                            (current_positions - self._spot.default_pos)
                            / self._spot._action_scale
                        )
                        self._spot._previous_action = self._spot.action.copy()
                    self._delivery_state = "SEARCHING"
                    self._grasp_t = 0.0

            elif self._delivery_state == "DROP_DONE":
                tgt_arm = POSES[2][0]
                self._cur_arm += np.clip(tgt_arm - self._cur_arm, -0.02, 0.02)
                self._hold(self._cur_arm, -1.571)
                if (
                    np.max(np.abs(tgt_arm - self._cur_arm)) < 0.02
                    and self._grasp_t > 1.5
                ):
                    print(f"[{self.namespace}] DROP_DONE → FOLD_ARM")
                    self._delivery_state = "FOLD_ARM"
                    self._grasp_t = 0.0
                    self._carry_arm = POSES[2][0].copy()
                    self._carry_tgt = self._dp()[self.ARM_IDX].copy()

        elif self._delivery_state == "FOLD_ARM":
            self._carry_arm += np.clip(
                self._carry_tgt - self._carry_arm, -0.01, 0.01
            )
            self._hold(self._carry_arm, self.GRIP_CLOSE)
            if np.max(np.abs(self._carry_tgt - self._carry_arm)) < 0.05:
                print(f"\n[{self.namespace}] 팔 접기 완료 → SEARCHING\n")
                try:
                    kps = np.array(self._kps0, dtype=np.float32).reshape(1, -1)
                    kds = np.array(self._kds0, dtype=np.float32).reshape(1, -1)
                    kps[0, self.GRIP_IDX] = 5000.0
                    kds[0, self.GRIP_IDX] = 250.0
                    self._spot.robot._articulation_view.set_gains(kps=kps, kds=kds)
                except Exception:
                    pass
                self._set_heavy_mode(False)
                self._carry_arm = None
                self._has_object = self._grabbed_cube_path is not None
                if hasattr(self._spot, "action") and hasattr(self._spot, "_action_scale"):
                    current_positions = self._spot.robot.get_joint_positions()
                    self._spot.action = (
                        (current_positions - self._spot.default_pos)
                        / self._spot._action_scale
                    )
                    self._spot._previous_action = self._spot.action.copy()
                self._delivery_state = "SEARCHING"

        # Magnetic Grasp Follow: 시각 큐브를 그리퍼에 붙임 (robot1)
        if self.allow_grasp_trigger and self._grabbed_cube_path is not None:
            try:
                from omni.isaac.core.prims import XFormPrim
                gripper_prim = XFormPrim(f"/World/{self.namespace}/arm0_link_wr1")
                cube_prim    = XFormPrim(self._grabbed_cube_path)
                pos, rot = gripper_prim.get_world_pose()
                q_vec = rot[1:]
                q_w   = rot[0]
                v = np.array([0.15, 0.0, -0.15])
                t = 2.0 * np.cross(q_vec, v)
                offset = v + q_w * t + np.cross(q_vec, t)
                cube_prim.set_world_pose(position=pos + offset, orientation=rot)
            except Exception:
                pass

    def _camera_array_is_empty(self, value):
        if value is None:
            return True
        try:
            return np.asarray(value).size == 0
        except Exception:
            return True

    def _read_yolo_camera_frame(self):
        img = None
        depth = None
        frame = None
        frame_issue = None

        try:
            img = self._camera_gripper.get_rgba()
        except Exception as e:
            frame_issue = f"get_rgba_error={e}"

        try:
            depth = self._camera_gripper.get_depth()
        except Exception:
            depth = None

        if self._camera_array_is_empty(img) or self._camera_array_is_empty(depth):
            try:
                frame = self._camera_gripper.get_current_frame()
            except Exception as e:
                if frame_issue is None:
                    frame_issue = f"get_current_frame_error={e}"

        if self._camera_array_is_empty(img) and isinstance(frame, dict):
            for key in ("rgba", "rgb"):
                value = frame.get(key)
                if not self._camera_array_is_empty(value):
                    img = value
                    frame_issue = None
                    break

        if self._camera_array_is_empty(depth) and isinstance(frame, dict):
            for key in ("distance_to_image_plane", "distance_to_camera", "depth"):
                value = frame.get(key)
                if not self._camera_array_is_empty(value):
                    depth = value
                    break

        if self._camera_array_is_empty(img):
            frame_issue = frame_issue or "rgba_empty"

        return img, depth, frame_issue

    def _camera_raycast_person_detection(self):
        try:
            import omni.physx
            from isaacsim.core.utils.rotations import quat_to_rot_matrix
            from omni.isaac.core.prims import XFormPrim

            cam_path = f"/World/{self.namespace}/arm0_link_wr1/gripper_camera"
            cam_pos, cam_q = XFormPrim(cam_path).get_world_pose()
            cam_pos = np.array(cam_pos, dtype=np.float32)
            cam_rot = quat_to_rot_matrix(cam_q)

            proxy_pos, _ = XFormPrim("/World/Person2_LidarProxy").get_world_pose()
            person_pos = np.array(proxy_pos, dtype=np.float32)
            person_pos[2] = self.PERSON2_PROXY_Z

            to_person = person_pos - cam_pos
            dist = float(np.linalg.norm(to_person))
            if dist < 0.1 or dist > self.PERSON_CAMERA_MAX_DISTANCE:
                return None

            target_dir = to_person / max(dist, 1e-6)
            target_h = np.array([target_dir[0], target_dir[1], 0.0], dtype=np.float32)
            target_h_norm = np.linalg.norm(target_h)
            if target_h_norm < 1e-6:
                return None
            target_h /= target_h_norm

            candidates = [
                cam_rot @ np.array([1.0, 0.0, 0.0]),
                cam_rot @ np.array([0.0, 0.0, -1.0]),
            ]
            robot_pose = self._get_robot_pose_2d()
            if robot_pose is not None:
                candidates.append(np.array([np.cos(robot_pose[2]), np.sin(robot_pose[2]), 0.0]))

            best_signed_angle = None
            best_forward = None
            for forward in candidates:
                forward = np.array(forward, dtype=np.float32)
                forward_h = np.array([forward[0], forward[1], 0.0], dtype=np.float32)
                norm = np.linalg.norm(forward_h)
                if norm < 1e-6:
                    continue
                forward_h /= norm
                dot = float(np.clip(np.dot(forward_h, target_h), -1.0, 1.0))
                angle = float(np.arccos(dot))
                if angle > self.PERSON_CAMERA_HALF_FOV_RAD:
                    continue
                cross_z = float(forward_h[0] * target_h[1] - forward_h[1] * target_h[0])
                signed_angle = np.sign(cross_z) * angle
                if best_signed_angle is None or abs(signed_angle) < abs(best_signed_angle):
                    best_signed_angle = signed_angle
                    best_forward = forward_h

            if best_forward is None:
                return None

            scene_query = omni.physx.get_physx_scene_query_interface()
            target_points = [
                person_pos + np.array([0.0, 0.0, -0.35], dtype=np.float32),
                person_pos,
                person_pos + np.array([0.0, 0.0, 0.45], dtype=np.float32),
            ]

            visible = False
            for target in target_points:
                ray_vec = target - cam_pos
                ray_dist = float(np.linalg.norm(ray_vec))
                if ray_dist < 0.1:
                    continue
                ray_dir = ray_vec / ray_dist
                for offset in (0.20, 0.45):
                    origin = cam_pos + ray_dir * offset
                    cast_dist = max(0.1, ray_dist - offset + 0.15)
                    hit = scene_query.raycast_closest(
                        (float(origin[0]), float(origin[1]), float(origin[2])),
                        (float(ray_dir[0]), float(ray_dir[1]), float(ray_dir[2])),
                        float(cast_dist),
                    )
                    if not hit.get("hit", False):
                        continue
                    hit_path = str(
                        hit.get("rigidBody")
                        or hit.get("collision")
                        or hit.get("collider")
                        or hit.get("path")
                        or ""
                    )
                    if hit_path.startswith(f"/World/{self.namespace}"):
                        continue
                    if "/World/Person2" in hit_path:
                        visible = True
                        break
                    break
                if visible:
                    break

            if not visible:
                return None

            center_x = max(1.0, self._camera_width / 2.0)
            cx = int(np.clip(
                center_x - (best_signed_angle / self.PERSON_CAMERA_HALF_FOV_RAD) * center_x,
                0,
                self._camera_width - 1,
            ))
            return {
                "depth": dist,
                "cx": cx,
                "conf": None,
                "source": "CameraRaycast",
            }
        except Exception:
            return None

    def _gate_person_detection(self, found, best_depth, best_cx, best_conf, detection_source):
        if not found:
            return False, best_depth, best_cx, best_conf, detection_source

        if best_depth is None:
            camera_visible = self._camera_raycast_person_detection()
            if camera_visible is None:
                return False, best_depth, best_cx, best_conf, detection_source
            best_depth = camera_visible["depth"]
            if detection_source == "YOLO":
                detection_source = "YOLO+CameraRaycast"
            if best_conf is None:
                best_conf = camera_visible["conf"]

        if best_depth > self.PERSON_DETECT_MAX_DISTANCE:
            return False, best_depth, best_cx, best_conf, detection_source

        return True, best_depth, best_cx, best_conf, detection_source

    def _person_approach_command(self, best_cx, best_depth):
        center_x = max(1.0, self._camera_width / 2.0)
        target_turn = (center_x - best_cx) / center_x
        turn_speed = np.clip(target_turn * 0.65, -self.DRIVE_MAX_WZ, self.DRIVE_MAX_WZ)
        if abs(target_turn) >= 0.08 and abs(turn_speed) < 0.12:
            turn_speed = 0.12 if turn_speed > 0 else -0.12

        if best_depth <= self.PERSON_APPROACH_DISTANCE:
            return np.zeros(3), True

        distance_error = best_depth - self.PERSON_APPROACH_DISTANCE
        vx = np.clip(0.10 + 0.08 * distance_error, 0.10, self.PERSON_APPROACH_MAX_VX)
        if abs(target_turn) > 0.45:
            vx = min(vx, 0.07)
        return np.array([vx, 0.0, turn_speed]), False

    def _person2_world_measurement(self):
        pose = self._get_robot_pose_2d()
        if pose is None:
            return None
        try:
            from omni.isaac.core.prims import XFormPrim
            person_pos, _ = XFormPrim("/World/Person2_LidarProxy").get_world_pose()
        except Exception:
            return None

        rx, ry, yaw = pose
        dx = float(person_pos[0]) - rx
        dy = float(person_pos[1]) - ry
        depth = float((dx * dx + dy * dy) ** 0.5)
        heading_error = self._wrap_angle(np.arctan2(dy, dx) - yaw)

        center_x = max(1.0, self._camera_width / 2.0)
        cx = int(np.clip(
            center_x - (heading_error / self.PERSON_CAMERA_HALF_FOV_RAD) * center_x,
            0,
            self._camera_width - 1,
        ))
        return cx, depth, heading_error

    def _person_approach_world_command(self):
        measurement = self._person2_world_measurement()
        if measurement is None:
            if (
                self._last_person_best_cx is not None
                and self._last_person_best_depth is not None
            ):
                return self._person_approach_command(
                    self._last_person_best_cx,
                    self._last_person_best_depth,
                )
            return np.array([0.10, 0.0, 0.0], dtype=np.float32), False

        best_cx, best_depth, heading_error = measurement
        self._last_person_best_cx = best_cx
        self._last_person_best_depth = best_depth
        self._last_person_seen_t = getattr(self, "_sim_time_r2", 0.0)

        if best_depth <= self.PERSON_APPROACH_DISTANCE:
            return np.zeros(3, dtype=np.float32), True

        turn_speed = np.clip(0.75 * heading_error, -self.DRIVE_MAX_WZ, self.DRIVE_MAX_WZ)
        if abs(heading_error) >= 0.08 and abs(turn_speed) < 0.10:
            turn_speed = 0.10 if turn_speed > 0 else -0.10

        distance_error = best_depth - self.PERSON_APPROACH_DISTANCE
        vx = float(np.clip(0.12 + 0.08 * distance_error, 0.10, self.PERSON_APPROACH_MAX_VX))
        abs_error = abs(heading_error)
        if abs_error > 1.0:
            vx = 0.0
        elif abs_error > 0.55:
            vx = min(vx, 0.08)
        elif abs_error > 0.30:
            vx = min(vx, 0.14)

        return np.array([vx, 0.0, turn_speed], dtype=np.float32), False

    def _person_align_command(self, best_cx):
        center_x = max(1.0, self._camera_width / 2.0)
        target_turn = (center_x - best_cx) / center_x

        if abs(target_turn) <= self.PERSON_ALIGN_TOLERANCE:
            return np.zeros(3), True, target_turn
        if abs(target_turn) <= self.PERSON_ALIGN_DEADBAND:
            return np.zeros(3), False, target_turn

        turn_speed = np.clip(target_turn * 0.45, -self.DRIVE_MAX_WZ, self.DRIVE_MAX_WZ)
        if abs(turn_speed) < self.PERSON_ALIGN_MIN_WZ:
            turn_speed = self.PERSON_ALIGN_MIN_WZ if turn_speed > 0 else -self.PERSON_ALIGN_MIN_WZ
        return np.array([0.0, 0.0, turn_speed]), False, target_turn

    def _person_align_world_command(self):
        measurement = self._person2_world_measurement()
        if measurement is None:
            if self._last_person_best_cx is not None:
                return self._person_align_command(self._last_person_best_cx)
            return np.zeros(3, dtype=np.float32), False, 0.0

        best_cx, best_depth, heading_error = measurement
        self._last_person_best_cx = best_cx
        self._last_person_best_depth = best_depth
        self._last_person_seen_t = getattr(self, "_sim_time_r2", 0.0)

        align_tolerance_rad = max(
            np.deg2rad(4.0),
            self.PERSON_ALIGN_TOLERANCE * self.PERSON_CAMERA_HALF_FOV_RAD,
        )
        if abs(heading_error) <= align_tolerance_rad:
            return np.zeros(3, dtype=np.float32), True, heading_error

        turn_speed = np.clip(0.85 * heading_error, -self.DRIVE_MAX_WZ, self.DRIVE_MAX_WZ)
        if abs(turn_speed) < self.PERSON_ALIGN_MIN_WZ:
            turn_speed = self.PERSON_ALIGN_MIN_WZ if turn_speed > 0 else -self.PERSON_ALIGN_MIN_WZ
        return np.array([0.0, 0.0, turn_speed], dtype=np.float32), False, heading_error

    def _update_person_alignment(self):
        self._tracking_command, aligned, heading_error = self._person_align_world_command()
        sim_t = getattr(self, "_sim_time_r2", 0.0)

        if aligned:
            if not getattr(self, "_centered_locked", False):
                self._centered_locked = True
                print(
                    f"[{self.namespace}] 정렬 완료 "
                    f"(heading={heading_error:.2f}rad) → 가까이 접근"
                )
            self._begin_person_approach()
            return

        self._yolo_state = "ALIGNING"
        if sim_t - getattr(self, "_last_person_align_log_t", -999.0) >= 0.5:
            self._last_person_align_log_t = sim_t
            print(
                f"[{self.namespace}] 정렬 중 "
                f"(heading={heading_error:.2f}rad, wz={self._tracking_command[2]:.2f})"
            )

    def _begin_person_approach(self, best_cx=None, best_depth=None):
        previous_state = getattr(self, "_yolo_state", "SEARCHING")
        sim_t = getattr(self, "_sim_time_r2", 0.0)
        if previous_state != "APPROACHING" or self._person_approach_started_t is None:
            self._person_approach_started_t = sim_t

        if best_cx is not None:
            self._last_person_best_cx = best_cx
        if best_depth is not None:
            self._last_person_best_depth = best_depth

        self._tracking_command, arrived = self._person_approach_world_command()
        elapsed = sim_t - self._person_approach_started_t

        if arrived and elapsed >= self.PERSON_APPROACH_MIN_TIME:
            depth = self._last_person_best_depth
            depth_text = f"{depth:.2f}m" if depth is not None else "확인불가"
            print(
                f"[{self.namespace}] 사람 근처 도착 "
                f"({depth_text}) → 바로 출구로 이동"
            )
            self._tracking_command = np.zeros(3)
            if self._start_robot2_rescue_to_exit():
                self._yolo_state = "ESCORTING"
            return
        if elapsed >= self.PERSON_APPROACH_TIMEOUT:
            depth = self._last_person_best_depth
            depth_text = f"{depth:.2f}m" if depth is not None else "확인불가"
            print(
                f"[{self.namespace}] 접근 타임아웃 "
                f"({depth_text}) → 출구 안내로 전환"
            )
            self._tracking_command = np.zeros(3)
            if self._start_robot2_rescue_to_exit():
                self._yolo_state = "ESCORTING"
            return

        self._yolo_state = "APPROACHING"
        should_log = (
            previous_state != "APPROACHING"
            or sim_t - getattr(self, "_last_person_approach_log_t", -999.0) >= 1.0
        )
        if should_log:
            self._last_person_approach_log_t = sim_t
            depth = self._last_person_best_depth
            depth_text = f"{depth:.2f}m" if depth is not None else "확인불가"
            print(
                f"[{self.namespace}] 접근 중 "
                f"(거리={depth_text}, vx={self._tracking_command[0]:.2f}, "
                f"wz={self._tracking_command[2]:.2f})"
            )

    def _run_yolo_step(self, step_size, base_command):
        """robot2 전용: YOLO 인명탐지 상태머신 + 출구 안내."""

        if getattr(self, "_yolo_state", "SEARCHING") == "ESCORTING":
            self._sim_time_r2 = getattr(self, "_sim_time_r2", 0.0) + step_size
            self._follow_person2_behind_robot(step_size)
            if self._dist_to(*self.EXIT_POS) < 0.8 and not self._rescue_goal_done:
                print(f"[{self.namespace}] 출구 도착 — 사람 안내 완료")
                self._rescue_goal_done = True
                self._mark_nav2_goal_done()
            cmd = self._exit_fallback_command(base_command, step_size)
            self._spot.forward(step_size, self._stable_drive_command(cmd, step_size))
            return

        if not self.yolo_enabled or not hasattr(self, "_camera_gripper") or self._camera_gripper is None:
            self._spot.forward(step_size, self._stable_drive_command(base_command, step_size))
            return

        # 카메라 초기화 (첫 호출 시)
        if not self._camera_initialized:
            try:
                self._camera_gripper.initialize()
                for method_name in ("add_rgba_to_frame", "add_rgb_to_frame"):
                    method = getattr(self._camera_gripper, method_name, None)
                    if callable(method):
                        try:
                            method()
                        except Exception:
                            pass
                try:
                    self._camera_gripper.add_distance_to_image_plane_to_frame()
                except AttributeError:
                    try:
                        self._camera_gripper.add_distance_to_camera_to_frame()
                    except Exception:
                        pass
                self._camera_initialized = True
                print(f"[{self.namespace}] 그리퍼 카메라 초기화 완료")
            except Exception as e:
                print(f"[{self.namespace}] 카메라 초기화 실패: {e}")
                self._spot.forward(step_size, self._stable_drive_command(base_command, step_size))
                return

        self._yolo_counter += 1
        self._yolo_accum += step_size

        # 탐색 중에는 기본 주기, 정렬/접근 중에는 더 빠르게 비전 처리
        vision_interval = self._yolo_interval
        if getattr(self, "_yolo_state", "SEARCHING") in ("ALIGNING", "APPROACHING"):
            vision_interval = min(vision_interval, 0.25)

        if self._yolo_accum >= vision_interval:
            self._yolo_accum = 0.0
            try:
                img, depth, frame_issue = self._read_yolo_camera_frame()

                found = False
                best_depth = None
                best_cx = self._camera_width // 2
                best_conf = None
                detection_source = "YOLO"

                if img is None:
                    frame_issue = "rgba_none"
                else:
                    img = np.asarray(img)
                    if img.size == 0:
                        frame_issue = "rgba_empty"
                    else:
                        if img.ndim == 1:
                            pixel_count = self._camera_width * self._camera_height
                            if img.size == pixel_count * 4:
                                img = img.reshape((self._camera_height, self._camera_width, 4))
                            elif img.size == pixel_count * 3:
                                img = img.reshape((self._camera_height, self._camera_width, 3))
                        if img.ndim < 3 or img.shape[-1] < 3:
                            frame_issue = f"bad_rgba_shape={img.shape}"

                if frame_issue is None:
                    img_rgb = img[:, :, :3]
                    depth_np = None
                    if depth is not None:
                        depth_np = np.asarray(depth)
                        if depth_np.size == 0:
                            depth_np = None
                        elif depth_np.ndim == 1 and depth_np.size == img_rgb.shape[0] * img_rgb.shape[1]:
                            depth_np = depth_np.reshape((img_rgb.shape[0], img_rgb.shape[1]))

                    results = self.yolo_model.predict(
                        source=img_rgb,
                        conf=self._yolo_conf,
                        verbose=False,
                    )
                    max_area = -1.0
                    for r in results:
                        boxes = getattr(r, "boxes", None)
                        if boxes is None or boxes.cls is None:
                            continue
                        for i, c in enumerate(boxes.cls):
                            cls_id = int(c.item()) if hasattr(c, "item") else int(c)
                            if cls_id != 0:
                                continue

                            found = True
                            box = boxes.xyxy[i].cpu().numpy()
                            area = float((box[2] - box[0]) * (box[3] - box[1]))
                            if area <= max_area:
                                continue

                            max_area = area
                            cx = int((box[0] + box[2]) / 2.0)
                            cy = int((box[1] + box[3]) / 2.0)
                            best_cx = int(np.clip(cx, 0, img_rgb.shape[1] - 1))

                            if getattr(boxes, "conf", None) is not None:
                                conf = boxes.conf[i]
                                best_conf = float(conf.item()) if hasattr(conf, "item") else float(conf)

                            if depth_np is not None and depth_np.ndim >= 2:
                                dy = int(np.clip(cy, 0, depth_np.shape[0] - 1))
                                dx = int(np.clip(cx, 0, depth_np.shape[1] - 1))
                                d = float(depth_np[dy, dx])
                                if np.isfinite(d) and 0.01 < d < 15.0:
                                    best_depth = d
                else:
                    camera_visible = self._camera_raycast_person_detection()
                    if camera_visible is not None:
                        found = True
                        best_depth = camera_visible["depth"]
                        best_cx = camera_visible["cx"]
                        best_conf = camera_visible["conf"]
                        detection_source = camera_visible["source"]
                    elif not self._yolo_camera_empty_warned:
                        print(
                            f"[{self.namespace}] 카메라 RGB 프레임이 비어 있어 "
                            f"YOLO 사람 인식을 할 수 없습니다. ({frame_issue})",
                            flush=True,
                        )
                        self._yolo_camera_empty_warned = True

                if not found:
                    camera_visible = self._camera_raycast_person_detection()
                    if camera_visible is not None:
                        found = True
                        best_depth = camera_visible["depth"]
                        best_cx = camera_visible["cx"]
                        best_conf = camera_visible["conf"]
                        detection_source = camera_visible["source"]

                found, best_depth, best_cx, best_conf, detection_source = (
                    self._gate_person_detection(
                        found, best_depth, best_cx, best_conf, detection_source
                    )
                )

                if found:
                    self._yolo_camera_empty_warned = False
                    depth_text = f"{best_depth:.2f}m" if best_depth is not None else "확인불가"
                    conf_text = f", conf={best_conf:.2f}" if best_conf is not None else ""
                    sim_t = getattr(self, "_sim_time_r2", 0.0)
                    if not self._was_person_detected:
                        print(
                            f"\n[{self.namespace}] 사람을 찾았습니다! "
                            f"robot2 카메라에서 person 감지, 거리={depth_text}{conf_text}, "
                            f"상태={self._yolo_state}, source={detection_source}\n",
                            flush=True,
                        )
                    elif sim_t - self._last_yolo_keepalive_log_t >= 1.0:
                        print(
                            f"[{self.namespace}] 사람 감지 유지: "
                            f"거리={depth_text}{conf_text}, 상태={self._yolo_state}, "
                            f"source={detection_source}",
                            flush=True,
                        )
                        self._last_yolo_keepalive_log_t = sim_t
                    self._was_person_detected = True
                    self._person_focus_end = getattr(self, "_sim_time_r2", 0.0) + 5.0
                    self._last_person_best_cx = best_cx
                    self._last_person_best_depth = best_depth
                    self._last_person_seen_t = getattr(self, "_sim_time_r2", 0.0)
                else:
                    if self._was_person_detected:
                        print(f"[{self.namespace}] 사람 시야 이탈 — 추적 유지")
                        self._was_person_detected = False

                if self._yolo_state == "SEARCHING":
                    if found:
                        self._patrol_active = False
                        self._nav_command = np.zeros(3)
                        self._mark_nav2_goal_done()
                        self._centered_locked = False
                        print(f"\n[{self.namespace}] 👤 사람 발견 → 정렬 시작\n")

                        self._yolo_state = "ALIGNING"
                        self._update_person_alignment()
                    else:
                        self._tracking_command = np.zeros(3)

                elif self._yolo_state == "ALIGNING":
                    self._update_person_alignment()

                elif self._yolo_state == "APPROACHING":
                    if found:
                        self._begin_person_approach(best_cx, best_depth)
                    else:
                        self._begin_person_approach()

            except Exception as e:
                print(f"[{self.namespace}] YOLO 처리 오류: {e}")

        self._sim_time_r2 = getattr(self, "_sim_time_r2", 0.0) + step_size

        if self._yolo_state == "ALIGNING":
            self._update_person_alignment()

        if self._yolo_state == "APPROACHING":
            self._begin_person_approach()

        if self._yolo_state == "ESCORTING":
            self._follow_person2_behind_robot(step_size)
            if self._dist_to(*self.EXIT_POS) < 0.8 and not self._rescue_goal_done:
                print(f"[{self.namespace}] 출구 도착 — 사람 안내 완료")
                self._rescue_goal_done = True
                self._mark_nav2_goal_done()
            cmd = self._exit_fallback_command(base_command, step_size)
        elif self._yolo_state != "SEARCHING":
            cmd = self._tracking_command.copy()
        else:
            cmd = base_command if np.any(base_command != 0) else self._tracking_command.copy()

        cmd = self._stable_drive_command(cmd, step_size)
        self._spot.forward(step_size, cmd)

        # Magnetic Grasp Follow: 시각 큐브를 그리퍼에 붙임
        if self._grabbed_cube_path is not None:
            try:
                from omni.isaac.core.prims import XFormPrim
                gripper_prim = XFormPrim(f"/World/{self.namespace}/arm0_link_wr1")
                cube_prim    = XFormPrim(self._grabbed_cube_path)
                pos, rot = gripper_prim.get_world_pose()
                q_vec = rot[1:]
                q_w   = rot[0]
                v = np.array([0.15, 0.0, -0.15])
                t = 2.0 * np.cross(q_vec, v)
                offset = v + q_w * t + np.cross(q_vec, t)
                cube_prim.set_world_pose(position=pos + offset, orientation=rot)
            except Exception:
                pass
