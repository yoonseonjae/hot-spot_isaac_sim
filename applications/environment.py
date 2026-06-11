import os
from pathlib import Path
from isaacsim.core.utils.prims import define_prim


class EnvironmentLoader:
    def __init__(self, base_dir=None):
        if base_dir is None:
            self.base_dir = Path(__file__).resolve().parent.parent
        else:
            self.base_dir = base_dir

    def spawn_map(self):
        prim = define_prim("/World/Map", "Xform")
        asset_path = os.path.join(self.base_dir, "map", "c_1_default_map.usd")
        prim.GetReferences().AddReference(asset_path)
        print(f"[Environment] Map loaded: {asset_path}")

        # 소화기(빨간 큐브 대체) 스폰 — robot1 대기방 앞
        from omni.isaac.core.objects import FixedCuboid
        from omni.isaac.core.prims import RigidPrim
        from omni.isaac.core.utils.stage import add_reference_to_stage
        import numpy as np

        cube_x = 10.7 - 0.87
        cube_y = 0.5
        cube_z = 0.48

        try:
            from omni.isaac.core.materials import PhysicsMaterial
            PhysicsMaterial(
                prim_path="/World/Physics_Materials/HighFriction",
                dynamic_friction=5.0,
                static_friction=5.0,
                restitution=0.0,
            )

            extinguisher_usd = os.path.join(
                self.base_dir, "map", "fire_extinguisher", "World0.usd"
            )
            add_reference_to_stage(extinguisher_usd, "/World/Cube")
            RigidPrim(
                prim_path="/World/Cube",
                name="cube",
                position=np.array([cube_x, cube_y, cube_z - 0.05]),
                mass=0.5,
            )

            from pxr import UsdPhysics
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            cube_prim = stage.GetPrimAtPath("/World/Cube")
            if not cube_prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(cube_prim)
                mesh_api = UsdPhysics.MeshCollisionAPI.Apply(cube_prim)
                mesh_api.CreateApproximationAttr("boundingCube")

            FixedCuboid(
                prim_path="/World/Table",
                name="table",
                position=np.array([cube_x, cube_y, cube_z - 0.025 - 0.05]),
                scale=np.array([0.2, 0.2, 0.1]),
                color=np.array([0.4, 0.4, 0.4]),
            )
            print("[Environment] 소화기 + 테이블 스폰 완료")
        except Exception as e:
            print(f"[Environment] 소화기 스폰 실패: {e}")

    def spawn_people(self):
        from omni.isaac.core.prims import XFormPrim
        from omni.isaac.core.objects import FixedCuboid
        from pxr import Usd, UsdGeom, UsdPhysics
        import numpy as np

        def add_lidar_collision_proxy(path, position, scale):
            FixedCuboid(
                prim_path=path,
                name=path.rsplit("/", 1)[-1],
                position=np.array(position, dtype=np.float32),
                scale=np.array(scale, dtype=np.float32),
                color=np.array([1.0, 0.0, 0.0]),
            )
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                print(f"[Environment] LiDAR proxy prim 생성 실패: {path}")
                return
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(prim)
            UsdGeom.Imageable(prim).MakeInvisible()

        def apply_person_collisions(root_path):
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            root = stage.GetPrimAtPath(root_path)
            if not root.IsValid():
                return
            applied = 0
            for prim in Usd.PrimRange(root):
                if prim.IsA(UsdGeom.Mesh):
                    if not prim.HasAPI(UsdPhysics.CollisionAPI):
                        UsdPhysics.CollisionAPI.Apply(prim)
                    mesh_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
                    mesh_api.CreateApproximationAttr("convexHull")
                    applied += 1
            print(f"[Environment] {root_path} 사람 mesh collision 적용: {applied}개")

        # Person2 (Female Police)
        person2_pos = np.array([8.82, -9.56, 0.01957])
        person2_prim = define_prim("/World/Person2", "Xform")
        person2_prim.GetReferences().AddReference(
            "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
            "/Assets/Isaac/5.1/Isaac/People/Characters/female_adult_police_01_new"
            "/female_adult_police_01_new.usd"
        )
        orient = np.array([0.70710678, 0.0, 0.0, -0.70710678])
        XFormPrim("/World/Person2").set_local_pose(
            translation=person2_pos,
            orientation=orient,
        )
        apply_person_collisions("/World/Person2")
        add_lidar_collision_proxy(
            "/World/Person2_LidarProxy",
            [person2_pos[0], person2_pos[1], 0.9],
            [0.45, 0.45, 1.6],
        )
        print("[Environment] Person2 스폰 완료")

    def apply_map_collisions(self):
        from pxr import Usd, UsdGeom, UsdPhysics
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        map_prim = stage.GetPrimAtPath("/World/Map")
        if map_prim:
            for p in Usd.PrimRange(map_prim):
                if p.IsA(UsdGeom.Mesh):
                    UsdPhysics.CollisionAPI.Apply(p)
                    mesh_api = UsdPhysics.MeshCollisionAPI.Apply(p)
                    mesh_api.CreateApproximationAttr("none")
            print("[Environment] 맵 충돌체 적용 완료")
