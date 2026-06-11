import os
os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": os.environ.get("COBOT_HEADLESS") == "1"})

import carb
import numpy as np
from pathlib import Path
import omni.appwindow
import omni.timeline
import omni.replicator.core as rep
from pxr import UsdGeom, Gf

carb.settings.get_settings().set(
    "/log/loggers/isaacsim.core.nodes.impl.base_writer_node/level", "fatal"
)
carb.settings.get_settings().set(
    "/log/loggers/omni.replicator.core/level", "fatal"
)

from isaacsim.core.api import World
from omni.isaac.core.utils.extensions import enable_extension

from environment import EnvironmentLoader
from spot_agent import SpotAgent

enable_extension("isaacsim.ros2.bridge")
enable_extension("omni.replicator.isaac")
enable_extension("omni.flowusd")
enable_extension("omni.usd.schema.flow")

carb.settings.get_settings().set("/rtx/flow/enabled", True)

for _ in range(15):
    simulation_app.update()


# 화재 발생 방: 2번방 중앙 좌표
FIRE_ROOM_POS = np.array([-4.93, -5.78, 0.8])


class SpotSimulationRunner:
    def __init__(self, physics_dt, render_dt):
        self._world = World(
            stage_units_in_meters=1.0,
            physics_dt=physics_dt,
            rendering_dt=render_dt,
        )
        self.base_dir = Path(__file__).resolve().parent.parent

        # 1. 환경 로드
        self.env = EnvironmentLoader(self.base_dir)
        self.env.spawn_map()

        # 2. 화재 이펙트 생성 (맵 로드 직후, collision 적용 전)
        self._create_flow_fire(FIRE_ROOM_POS)
        self._create_extinguisher_gas()

        self.env.apply_map_collisions()
        self.env.spawn_people()

        # 화재 센서 UDP 소켓 (UI/브릿지에 FIRE_TRUE 전송용)
        import socket
        self._sensor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._bridge_sensor_address = ("127.0.0.1", 5006)
        self._fire_update_interval = 1.0 / max(
            1.0, float(os.environ.get("COBOT_FIRE_UPDATE_HZ", "12.0"))
        )
        self._fire_update_accum = self._fire_update_interval
        self._fire_sensor_interval = 1.0 / max(
            1.0, float(os.environ.get("COBOT_FIRE_SENSOR_HZ", "2.0"))
        )
        self._fire_sensor_accum = self._fire_sensor_interval
        self._prim_cache = {}
        self._attr_cache = {}
        self._translate_op_cache = {}
        self._flow_block_attrs = None
        self._flow_max_blocks = int(os.environ.get("COBOT_FLOW_MAX_BLOCKS", "8192"))
        self._fire_pos_xy = FIRE_ROOM_POS[:2].astype(np.float32)
        self._scenario_complete = False
        self.EXIT_COMPLETE_TOLERANCE = float(
            os.environ.get("COBOT_EXIT_COMPLETE_TOLERANCE", "1.2")
        )

        # 타이머/트리거 상태
        self._elapsed_time = 0.0
        self._trigger_10s_fired = False
        self._trigger_15s_fired = False
        self._last_printed_second = -1
        self.GAS_RAMP_DURATION = 1.5
        self.GAS_FULL_DURATION = 5.0
        self.GAS_FADE_DURATION = 5.0
        self.GAS_PEAK_RADIUS = 8.0
        self.GAS_PEAK_SMOKE = 500.0
        self.FIRE_EXTINGUISH_DURATION = 8.0
        self.FIRE_MIN_RADIUS = 0.0001

        # 3. robot1 (팔 있음 — Grasp 담당)
        self.agent1 = SpotAgent(
            self.base_dir,
            enable_replicator_writer=False,
            namespace="robot1",
            spawn_pos=np.array([10.735, 1.111, 0.72]),
            udp_port=9876,
            allow_grasp_trigger=True,
        )

        # 4. robot2 (순찰/인명구조 담당)
        self.agent2 = SpotAgent(
            self.base_dir,
            enable_replicator_writer=False,
            namespace="robot2",
            spawn_pos=np.array([12.7, 0.5, 0.72]),
            udp_port=9877,
            allow_grasp_trigger=False,
        )

        self._base_command = np.zeros(3)
        self._input_keyboard_mapping = {
            "NUMPAD_8": [0.8, 0.0, 0.0], "UP":    [0.8, 0.0, 0.0],
            "NUMPAD_2": [-0.8, 0.0, 0.0], "DOWN": [-0.8, 0.0, 0.0],
            "NUMPAD_4": [0.0, 0.0, 0.4],  "LEFT":  [0.0, 0.0, 0.4],
            "NUMPAD_6": [0.0, 0.0, -0.4], "RIGHT": [0.0, 0.0, -0.4],
            "N":        [0.0, 0.0, 0.4],
            "M":        [0.0, 0.0, -0.4],
        }
        self.needs_reset = False

    # ------------------------------------------------------------------ #
    # Flow 화재 이펙트 생성 (fireman 원본 그대로)
    # ------------------------------------------------------------------ #
    def _create_flow_fire(self, position):
        from pxr import Sdf, Vt
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if not stage:
            print("[Fire] Stage가 없습니다!")
            return

        base_path = "/World/Fire"
        old = stage.GetPrimAtPath(base_path)
        if old and old.IsValid():
            stage.RemovePrim(base_path)

        fire_root = UsdGeom.Xform.Define(stage, base_path)
        fire_root.AddTranslateOp().Set(
            Gf.Vec3d(float(position[0]), float(position[1]), float(position[2]))
        )

        fire_configs = [
            {
                "name": "Fire_10cm",
                "radius": 0.0001, "fuel": 3.0, "temperature": 5.0,
                "smoke": 0.2, "courant": 1.0, "burn_temp": 0.3,
                "burn_rate": 8.0, "cooling_rate": 1.0, "buoyancy": 1.5,
            },
            {
                "name": "Fire_10m",
                "radius": 0.0001, "fuel": 4.0, "temperature": 8.0,
                "smoke": 0.5, "courant": 1.5, "burn_temp": 0.3,
                "burn_rate": 10.0, "cooling_rate": 0.5, "buoyancy": 2.0,
            },
        ]

        for cfg in fire_configs:
            group_path = f"{base_path}/{cfg['name']}"
            UsdGeom.Xform.Define(stage, group_path)

            emitter_path = f"{group_path}/flowEmitterSphere"
            ep = stage.DefinePrim(emitter_path, "FlowEmitterSphere")
            ep.CreateAttribute("radius", Sdf.ValueTypeNames.Float).Set(cfg["radius"])
            ep.CreateAttribute("fuel", Sdf.ValueTypeNames.Float).Set(cfg["fuel"])
            ep.CreateAttribute("temperature", Sdf.ValueTypeNames.Float).Set(cfg["temperature"])
            ep.CreateAttribute("smoke", Sdf.ValueTypeNames.Float).Set(cfg["smoke"])
            ep.CreateAttribute("courantNumber", Sdf.ValueTypeNames.Float).Set(cfg["courant"])
            ep.CreateAttribute("layer", Sdf.ValueTypeNames.Int).Set(1)

            sim_path = f"{group_path}/flowSimulate"
            sp = stage.DefinePrim(sim_path, "FlowSimulate")
            sp.CreateAttribute("burnTemperature", Sdf.ValueTypeNames.Float).Set(cfg["burn_temp"])
            sp.CreateAttribute("burnRate", Sdf.ValueTypeNames.Float).Set(cfg["burn_rate"])
            sp.CreateAttribute("coolingRate", Sdf.ValueTypeNames.Float).Set(cfg["cooling_rate"])
            sp.CreateAttribute("buoyancyPerTemp", Sdf.ValueTypeNames.Float).Set(cfg["buoyancy"])
            sp.CreateAttribute("layer", Sdf.ValueTypeNames.Int).Set(1)

            offscreen_path = f"{group_path}/flowOffscreen"
            op = stage.DefinePrim(offscreen_path, "FlowOffscreen")
            op.CreateAttribute("layer", Sdf.ValueTypeNames.Int).Set(1)

            colormap_path = f"{offscreen_path}/colormap"
            cp = stage.DefinePrim(colormap_path, "FlowRayMarchColormapParams")
            cp.CreateAttribute("rgbaPoints", Sdf.ValueTypeNames.Float4Array).Set(
                Vt.Vec4fArray([
                    Gf.Vec4f(1.0, 1.0, 1.0, 1.0),
                    Gf.Vec4f(0.03575, 0.03575, 0.03575, 0.504902),
                    Gf.Vec4f(0.03575, 0.03575, 0.03575, 0.504902),
                    Gf.Vec4f(1.0, 0.1594, 0.0134, 0.8),
                    Gf.Vec4f(13.53, 2.99, 0.12599, 0.8),
                    Gf.Vec4f(78.0, 39.0, 6.1, 0.7),
                ])
            )
            cp.CreateAttribute("xPoints", Sdf.ValueTypeNames.FloatArray).Set(
                Vt.FloatArray([0.0, 0.05, 0.15, 0.6, 0.85, 1.0])
            )
            cp.CreateAttribute("colorScale", Sdf.ValueTypeNames.Float).Set(250.0)
            cp.CreateAttribute("colorScalePoints", Sdf.ValueTypeNames.FloatArray).Set(
                Vt.FloatArray([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
            )

            render_path = f"{group_path}/flowRender"
            rp = stage.DefinePrim(render_path, "FlowRender")
            rp.CreateAttribute("layer", Sdf.ValueTypeNames.Int).Set(1)

        self._fire_emitter_10cm_path = f"{base_path}/Fire_10cm/flowEmitterSphere"
        self._fire_emitter_10m_path  = f"{base_path}/Fire_10m/flowEmitterSphere"
        print(f"[Fire] Flow 화재 이펙트 생성 완료: {position}")

    # ------------------------------------------------------------------ #
    # 소화기 가스 이펙트 생성 (초기 비활성, 소화기 바닥 충돌 시 활성화)
    # ------------------------------------------------------------------ #
    def _create_extinguisher_gas(self):
        from pxr import Sdf
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        base_path = "/World/ExtinguisherGas"
        if stage.GetPrimAtPath(base_path).IsValid():
            stage.RemovePrim(base_path)

        gas_root = UsdGeom.Xform.Define(stage, base_path)
        gas_root.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -10.0))  # 초기엔 맵 아래 숨김

        emitter_path = f"{base_path}/flowEmitterSphere"
        ep = stage.DefinePrim(emitter_path, "FlowEmitterSphere")
        ep.CreateAttribute("radius", Sdf.ValueTypeNames.Float).Set(0.0001)
        ep.CreateAttribute("fuel", Sdf.ValueTypeNames.Float).Set(0.0)
        ep.CreateAttribute("temperature", Sdf.ValueTypeNames.Float).Set(0.0)
        ep.CreateAttribute("smoke", Sdf.ValueTypeNames.Float).Set(0.0)
        ep.CreateAttribute("coupleRateSmoke", Sdf.ValueTypeNames.Float).Set(1.0)
        ep.CreateAttribute("courantNumber", Sdf.ValueTypeNames.Float).Set(2.0)
        ep.CreateAttribute("layer", Sdf.ValueTypeNames.Int).Set(1)

        self._gas_root_path    = base_path
        self._gas_emitter_path = emitter_path
        print("[Gas] 소화기 가스 이펙트 스탠바이 완료")

    # ------------------------------------------------------------------ #
    # FlowSimulate maxBlocks 부스트 (화재 확산 품질 향상)
    # ------------------------------------------------------------------ #
    def _boost_flow_blocks(self):
        try:
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            if stage:
                if self._flow_block_attrs is None:
                    self._flow_block_attrs = []
                    for p in stage.Traverse():
                        if "flowSimulate" in p.GetName():
                            for attr_name in ["maxBlocks", "maxBlockCount"]:
                                attr = p.GetAttribute(attr_name)
                                if attr.IsValid():
                                    self._flow_block_attrs.append(attr)
                for attr in self._flow_block_attrs:
                    if attr.IsValid():
                        attr.Set(self._flow_max_blocks)
        except Exception:
            pass

    # ------------------------------------------------------------------ #
    # Xform translate/scale 직접 조작 헬퍼
    # ------------------------------------------------------------------ #
    def _apply_clean_transform(self, prim, custom_translate=None):
        if prim and prim.IsValid():
            xf = UsdGeom.Xformable(prim)
            cache_key = str(prim.GetPath())
            translate_op = self._translate_op_cache.get(cache_key)
            if translate_op is None:
                ops = xf.GetOrderedXformOps()
                translate_op = next(
                    (op for op in ops if op.GetOpType() == UsdGeom.XformOp.TypeTranslate),
                    None,
                )
                if translate_op is None:
                    translate_op = xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
                self._translate_op_cache[cache_key] = translate_op
            if custom_translate is not None:
                vec_t = Gf.Vec3d(
                    float(custom_translate[0]),
                    float(custom_translate[1]),
                    float(custom_translate[2]),
                )
                translate_op.Set(vec_t)

    def _get_cached_prim(self, cache_key, path):
        prim = self._prim_cache.get(cache_key)
        if prim and prim.IsValid():
            return prim
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(path) if stage else None
        self._prim_cache[cache_key] = prim
        return prim

    def _get_cached_attr(self, cache_key, prim, names):
        attr = self._attr_cache.get(cache_key)
        if attr is not None and attr.IsValid():
            return attr
        if prim and prim.IsValid():
            for name in names:
                attr = prim.GetAttribute(name)
                if attr.IsValid():
                    self._attr_cache[cache_key] = attr
                    return attr
        return None

    def _send_fire_sensor(self):
        if self._fire_sensor_accum < self._fire_sensor_interval:
            return
        self._fire_sensor_accum = 0.0
        self._sensor_sock.sendto(b"FIRE_TRUE", self._bridge_sensor_address)

    def _clear_cached_stage_handles(self):
        self._prim_cache.clear()
        self._attr_cache.clear()
        self._translate_op_cache.clear()
        self._flow_block_attrs = None

    # ------------------------------------------------------------------ #
    # setup
    # ------------------------------------------------------------------ #
    def setup(self):
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = None
        self._keyboard = None
        self._sub_keyboard = None
        if self._appwindow is not None:
            self._input = carb.input.acquire_input_interface()
            self._keyboard = self._appwindow.get_keyboard()
            self._sub_keyboard = self._input.subscribe_to_keyboard_events(
                self._keyboard, self._sub_keyboard_event
            )
        else:
            print("[Main] 기본 앱 창 없음: 수동 키보드 입력 없이 계속 실행")

        self.agent1.setup_sensors()
        self.agent2.setup_sensors()

        self._boost_flow_blocks()

        self._world.add_physics_callback(
            "sim_step", callback_fn=self.on_physics_step
        )

        stream = omni.timeline.get_timeline_interface().get_timeline_event_stream()
        self._timeline_sub = stream.create_subscription_to_pop(
            self._on_timeline_event
        )

    def _on_timeline_event(self, e):
        # Replicator is not used, so we don't need to start/stop the orchestrator.
        # This prevents the 'unknown dtype' error in simTimesToWrite when playing.
        pass

    def _robot_at_exit(self, agent):
        try:
            return agent._dist_to(*agent.EXIT_POS) < self.EXIT_COMPLETE_TOLERANCE
        except Exception:
            return False

    def _maybe_finish_scenario(self):
        if self._scenario_complete:
            return

        robot1_done = (
            getattr(self.agent1, "_auto_state", None) == "DONE"
            and self._robot_at_exit(self.agent1)
        )
        robot2_done = (
            getattr(self.agent2, "_rescue_goal_done", False)
            and self._robot_at_exit(self.agent2)
        )

        if not (robot1_done and robot2_done):
            return

        self._scenario_complete = True
        self.agent1._nav_command = np.zeros(3)
        self.agent2._nav_command = np.zeros(3)
        print("\n[Scenario] robot1/robot2 모두 집 밖으로 탈출 완료 — 시나리오 종료\n")
        try:
            omni.timeline.get_timeline_interface().pause()
        except Exception:
            pass

    # ------------------------------------------------------------------ #
    # 물리 스텝: 화재 타이머 + 가스 분출 + 로봇 제어
    # ------------------------------------------------------------------ #
    def on_physics_step(self, step_size):
        if self.needs_reset or not self._world.is_playing():
            return

        self._elapsed_time += step_size
        self._fire_update_accum += step_size
        self._fire_sensor_accum += step_size

        # 5초마다 경과 시간 출력
        current_second = int(self._elapsed_time)
        if current_second > self._last_printed_second:
            if current_second % 5 == 0:
                print(f"[타이머] {current_second}초 경과")
            self._last_printed_second = current_second

        # ---------- 화재 타이머 ----------
        if self._fire_update_accum >= self._fire_update_interval:
            self._fire_update_accum = 0.0
            try:
                emitter_10cm = self._get_cached_prim(
                    "fire_emitter_10cm", self._fire_emitter_10cm_path
                )
                emitter_10m = self._get_cached_prim(
                    "fire_emitter_10m", self._fire_emitter_10m_path
                )

                def set_radius(cache_key, emitter, val):
                    attr = self._get_cached_attr(cache_key, emitter, ["radius", "Radius"])
                    if attr is not None:
                        attr.Set(float(val))

                def set_smoke(cache_key, emitter, val):
                    attr = self._get_cached_attr(cache_key, emitter, ["smoke"])
                    if attr is not None:
                        attr.Set(float(val))

                def get_float_attr(cache_key, emitter, names, default=0.0):
                    attr = self._get_cached_attr(cache_key, emitter, names)
                    if attr is not None:
                        value = attr.Get()
                        if value is not None:
                            return float(value)
                    return float(default)

                fire_extinguished = hasattr(self, "_fire_extinguished")
                fire_extinguishing = hasattr(self, "_fire_extinguish_start_time")

                if self._elapsed_time < 10.0:
                    if not hasattr(self, "_init_fire_set"):
                        set_radius("fire_10cm_radius", emitter_10cm, 0.0001)
                        set_radius("fire_10m_radius", emitter_10m, 0.0001)
                        self._init_fire_set = True

                elif self._elapsed_time < 15.0:
                    if not self._trigger_10s_fired:
                        self._boost_flow_blocks()
                        print("\n[🔥 점화] 10초 경과 — 화재 발생!\n")
                        self._trigger_10s_fired = True
                        set_radius("fire_10cm_radius", emitter_10cm, 0.1)
                        set_radius("fire_10m_radius", emitter_10m, 0.0001)
                        self.agent1.set_fire_detected()
                    if not fire_extinguished:
                        self._send_fire_sensor()

                else:
                    if not self._trigger_15s_fired:
                        self._boost_flow_blocks()
                        print("\n[🔥 확산] 15초 경과 — 대형 화재 확산!\n")
                        self._trigger_15s_fired = True

                    if not fire_extinguishing and not fire_extinguished:
                        t = self._elapsed_time - 15.0
                        set_radius("fire_10cm_radius", emitter_10cm, min(0.8, 0.1 + t * 0.01))
                        set_radius("fire_10m_radius", emitter_10m, min(10.0, 0.1 + t * 0.05))
                        set_smoke("fire_10m_smoke", emitter_10m, min(30.0, 0.5 + t * 0.4))
                    if not fire_extinguished:
                        self._send_fire_sensor()

                # ---------- 소화기 가스 분출 ----------
                cube_prim = self._get_cached_prim("cube", "/World/Cube")
                if cube_prim and cube_prim.IsValid():
                    cube_xf = UsdGeom.Xformable(cube_prim)
                    time_code = omni.timeline.get_timeline_interface().get_current_time()
                    pos = cube_xf.ComputeLocalToWorldTransform(time_code).ExtractTranslation()

                    if pos is not None:
                        gas_root = self._get_cached_prim("gas_root", self._gas_root_path)
                        gas_emitter = self._get_cached_prim("gas_emitter", self._gas_emitter_path)
                        robot1_holding = getattr(self.agent1, "_grabbed_cube_path", None) is not None
                        gas_pos = [0.0, 0.0, -10.0] if robot1_holding else pos
                        self._apply_clean_transform(gas_root, custom_translate=gas_pos)

                        cube_xy = np.array([float(pos[0]), float(pos[1])], dtype=np.float32)
                        near_fire = np.linalg.norm(cube_xy - self._fire_pos_xy) < 8.0
                        gas_impact_now = (
                            -0.5 < pos[2] < 0.2
                            and self._elapsed_time > 5.0
                            and not robot1_holding
                            and near_fire
                        )
                        gas_active = gas_impact_now or hasattr(self, "_gas_trigger_time")
                        if gas_active:
                            if not hasattr(self, "_gas_trigger_time"):
                                print("\n[💨 가스 분출] 소화기 바닥 충돌!\n")
                                self._gas_trigger_time = self._elapsed_time
                            ge = self._elapsed_time - self._gas_trigger_time
                            fade_start = self.GAS_FULL_DURATION
                            fade_end = self.GAS_FULL_DURATION + self.GAS_FADE_DURATION

                            if ge < fade_start:
                                ramp = min(1.0, ge / self.GAS_RAMP_DURATION)
                                radius = 0.5 + (self.GAS_PEAK_RADIUS - 0.5) * ramp
                                smoke = self.GAS_PEAK_SMOKE
                            elif ge < fade_end:
                                fade = 1.0 - ((ge - fade_start) / self.GAS_FADE_DURATION)
                                radius = max(0.0001, self.GAS_PEAK_RADIUS * fade)
                                smoke = self.GAS_PEAK_SMOKE * fade
                            else:
                                radius = 0.0001
                                smoke = 0.0
                                if not hasattr(self, "_gas_fade_done"):
                                    print("\n[💨 가스 소멸] 소화기 연기 분출 종료\n")
                                    self._gas_fade_done = True

                            set_radius("gas_radius", gas_emitter, radius)
                            set_smoke("gas_smoke", gas_emitter, smoke)

                            if (
                                gas_impact_now
                                and not hasattr(self, "_fire_extinguish_start_time")
                                and not hasattr(self, "_fire_extinguished")
                            ):
                                print("\n[🧯 소화 시작] 가스가 화재 지점에 닿음 — 불 감소 시작\n")
                                self._fire_extinguish_start_time = self._elapsed_time
                                self._fire_extinguish_start = {
                                    "r10cm": get_float_attr("fire_10cm_radius", emitter_10cm, ["radius", "Radius"], 0.1),
                                    "r10m": get_float_attr("fire_10m_radius", emitter_10m, ["radius", "Radius"], 0.1),
                                    "smoke10cm": get_float_attr("fire_10cm_smoke", emitter_10cm, ["smoke"], 0.2),
                                    "smoke10m": get_float_attr("fire_10m_smoke", emitter_10m, ["smoke"], 0.5),
                                }
                        else:
                            if not hasattr(self, "_gas_trigger_time"):
                                set_radius("gas_radius", gas_emitter, 0.0001)
                                set_smoke("gas_smoke", gas_emitter, 0.0)

                        if hasattr(self, "_fire_extinguish_start_time"):
                            fe = self._elapsed_time - self._fire_extinguish_start_time
                            progress = min(1.0, max(0.0, fe / self.FIRE_EXTINGUISH_DURATION))
                            remain = (1.0 - progress) ** 2
                            start = getattr(self, "_fire_extinguish_start", {})

                            set_radius(
                                "fire_10cm_radius",
                                emitter_10cm,
                                max(self.FIRE_MIN_RADIUS, start.get("r10cm", 0.1) * remain),
                            )
                            set_radius(
                                "fire_10m_radius",
                                emitter_10m,
                                max(self.FIRE_MIN_RADIUS, start.get("r10m", 0.1) * remain),
                            )
                            set_smoke("fire_10cm_smoke", emitter_10cm, start.get("smoke10cm", 0.2) * remain)
                            set_smoke("fire_10m_smoke", emitter_10m, start.get("smoke10m", 0.5) * remain)

                            if progress >= 1.0 and not hasattr(self, "_fire_extinguished"):
                                set_radius("fire_10cm_radius", emitter_10cm, self.FIRE_MIN_RADIUS)
                                set_radius("fire_10m_radius", emitter_10m, self.FIRE_MIN_RADIUS)
                                set_smoke("fire_10cm_smoke", emitter_10cm, 0.0)
                                set_smoke("fire_10m_smoke", emitter_10m, 0.0)
                                print("\n[✅ 소화 완료] 화재 이펙트 종료\n")
                                self._fire_extinguished = True

            except Exception as e:
                import traceback
                traceback.print_exc()
                print(f"[Fire Error] {e}")

        # ---------- 로봇 제어 ----------
        self.agent1.on_physics_step(step_size, self._base_command)
        self.agent2.on_physics_step(step_size, np.zeros(3))
        self._maybe_finish_scenario()

    # ------------------------------------------------------------------ #
    # 메인 루프
    # ------------------------------------------------------------------ #
    def run(self):
        while simulation_app.is_running():
            if self._world.is_playing() and self.needs_reset:
                self._world.reset(True)
                self.needs_reset = False
                self.agent1.first_step = True
                self.agent1._nav_command = np.zeros(3)
                self.agent1._mark_nav2_goal_done()
                self.agent2.first_step = True
                self.agent2._nav_command = np.zeros(3)
                self.agent2._mark_nav2_goal_done()
                self.agent2._patrol_active = False
                self.agent2._rescue_active = False
                self.agent2._rescue_goal_done = False
                self.agent2._person2_follow_active = False
                if hasattr(self.agent2, "reset_person2_to_home"):
                    self.agent2.reset_person2_to_home()
                if hasattr(self.agent2, "_yolo_state"):
                    self.agent2._yolo_state = "SEARCHING"
                if hasattr(self.agent2, "_tracking_command"):
                    self.agent2._tracking_command = np.zeros(3)
                self._elapsed_time = 0.0
                self._scenario_complete = False
                self._trigger_10s_fired = False
                self._trigger_15s_fired = False
                self._last_printed_second = -1
                self._fire_update_accum = self._fire_update_interval
                self._fire_sensor_accum = self._fire_sensor_interval
                self._clear_cached_stage_handles()
                for attr in [
                    "_init_fire_set",
                    "_gas_trigger_time",
                    "_gas_fade_done",
                    "_fire_extinguish_start_time",
                    "_fire_extinguish_start",
                    "_fire_extinguished",
                ]:
                    if hasattr(self, attr):
                        delattr(self, attr)
            self._world.step(render=True)
            if self._world.is_stopped():
                self.needs_reset = True

        if hasattr(self, "_timeline_sub"):
            self._timeline_sub = None

    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command += np.array(
                    self._input_keyboard_mapping[event.input.name]
                )
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command -= np.array(
                    self._input_keyboard_mapping[event.input.name]
                )
        return True


def main():
    physics_dt = 1 / 200.0
    render_dt = 1 / max(1.0, float(os.environ.get("COBOT_RENDER_HZ", "30.0")))

    bridge_process1 = None
    bridge_process2 = None

    try:
        runner = SpotSimulationRunner(physics_dt=physics_dt, render_dt=render_dt)

        simulation_app.update()
        runner._world.reset()
        simulation_app.update()
        for _ in range(5):
            simulation_app.update()

        runner.setup()
        simulation_app.update()

        import subprocess
        bridge_path = os.path.join(runner.base_dir, "cmd_vel_udp_bridge.py")
        env = os.environ.copy()
        if "PYTHONPATH" in env:
            env["PYTHONPATH"] = ":".join(
                p for p in env["PYTHONPATH"].split(":")
                if "isaacsim" not in p.lower()
            )
        bridge_process1 = subprocess.Popen(
            ["python3", bridge_path, "--namespace", "robot1", "--port", "9876"],
            env=env,
        )
        bridge_process2 = subprocess.Popen(
            ["python3", bridge_path, "--namespace", "robot2", "--port", "9877"],
            env=env,
        )
        print(f"[Main] UDP Bridge 시작 (PID: {bridge_process1.pid}, {bridge_process2.pid})")

        runner.run()

    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        for p in [bridge_process1, bridge_process2]:
            if p is not None:
                try:
                    p.terminate()
                except Exception:
                    pass
        print("[Main] 브릿지 종료")
        try:
            if "runner" in locals():
                runner._world.stop()
                runner._world.clear_instance()
        except Exception:
            pass
        simulation_app.close()


if __name__ == "__main__":
    main()
