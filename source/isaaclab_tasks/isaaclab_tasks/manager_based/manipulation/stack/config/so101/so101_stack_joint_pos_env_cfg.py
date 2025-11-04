# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# This file is adapted for the SO101 robot.

from isaaclab.assets import RigidObjectCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

# --- [수정 1] Import 변경 ---
# 로봇과 무관한 mdp와 기본 환경(StackEnvCfg)은 그대로 사용합니다.
from isaaclab_tasks.manager_based.manipulation.stack import mdp
from isaaclab_tasks.manager_based.manipulation.stack.stack_env_cfg import StackEnvCfg
# Franka 이벤트 대신 우리가 만든 SO101 이벤트를 import 합니다.
from isaaclab_tasks.manager_based.manipulation.stack.mdp import so101_stack_events

##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
# Franka 설정 대신 우리가 만든 SO101 설정을 import 합니다.
# TODO: 아래 경로는 실제 SO101_CFG 파일이 있는 위치로 수정해야 합니다.
# from path.to.your.assets import SO101_CFG # isort: skip
from isaaclab_assets.robots.so101 import SO101_CFG  # isort: skip

@configclass
class EventCfg:
    """Configuration for events, adapted for SO101."""

    # --- [수정 2] 이벤트 정의 변경 ---
    # 이벤트 이름과 함수 호출을 모두 SO101에 맞게 수정합니다.
    init_so101_arm_pose = EventTerm(
        func=so101_stack_events.set_default_joint_pose,
        mode="reset",
        params={
            # SO101 로봇의 관절 수(6개)에 맞는 기본 자세를 정의합니다.
            # 이 값은 로봇이 작업을 시작하기에 적절한 '준비 자세'입니다. (필요시 튜닝)
            # 순서: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
            "default_pose": [0.0, 0.0, 0.0, 0.7071*2, 0.0, 0.0], # gripper는 닫힌 상태(0.0)로 시작
        },
    )

    randomize_so101_joint_state = EventTerm(
        func=so101_stack_events.randomize_joint_by_gaussian_offset,
        mode="reset",
        params={
            "mean": 0.0,
            "std": 0.002, # 랜덤화 강도 (라디안 단위)
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )

    # 이 이벤트는 로봇과 무관하게 큐브의 위치를 설정하므로 그대로 사용합니다.
    # 단, 호출하는 함수는 일관성을 위해 so101_stack_events 모듈의 것을 사용합니다.
    randomize_cube_positions = EventTerm(
        func=so101_stack_events.randomize_object_pose,
        mode="reset",
        params={
            "pose_range": {"x": (0.1, 0.3), "y": (-0.10, 0.10), "z": (0.0203, 0.0203), "yaw": (-1.0, 1.0)},
            "min_separation": 0.1,
            "asset_cfgs": [SceneEntityCfg("cube_1"), SceneEntityCfg("cube_2"), SceneEntityCfg("cube_3")],
        },
    )


@configclass
class SO101CubeStackEnvCfg(StackEnvCfg): # --- [수정 3] 클래스 이름 변경 ---

    def __post_init__(self):
        # 부모 클래스의 초기화는 반드시 먼저 호출해야 합니다.
        super().__post_init__()

        # --- [수정 4] 이벤트 설정 ---
        # 위에서 정의한 SO101용 EventCfg를 환경에 적용합니다.
        self.events = EventCfg()

        # --- [수정 5] 로봇 지정 ---
        # 씬의 로봇을 Franka 대신 SO101로 교체합니다.
        self.scene.robot = SO101_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot.spawn.semantic_tags = [("class", "robot")]

        # 테이블, 바닥 등 씬의 다른 요소들은 그대로 둡니다.
        self.scene.table.spawn.semantic_tags = [("class", "table")]
        self.scene.plane.semantic_tags = [("class", "ground")]

        # --- [수정 6] 행동(Action) 정의 변경 ---
        # RL 에이전트의 행동을 SO101 로봇의 관절에 맞게 재정의합니다.
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            # SO101_CFG에 정의된 팔 관절 이름을 명시합니다.
            joint_names=[
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
            ],
            scale=0.5,
            use_default_offset=True,
        )
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            # SO101_CFG에 정의된 그리퍼 관절 이름을 명시합니다.
            joint_names=["gripper"],
            # 그리퍼의 '열림'과 '닫힘' 상태에 해당하는 관절 값을 정의합니다.
            # TODO: 이 값들은 실제 로봇의 그리퍼 가동 범위에 맞게 확인 및 수정이 필요합니다.
            open_command_expr={"gripper": 1.0},  # 예: 0.05가 완전히 열린 상태
            close_command_expr={"gripper": 0.0}, # 예: 0.0이 완전히 닫힌 상태
        )
        # 그리퍼 상태를 확인하기 위한 유틸리티 변수들도 수정합니다.
        self.gripper_joint_names = ["gripper"]
        self.gripper_open_val = 1.0 # open_command_expr과 동일하게 설정
        self.gripper_threshold = 0.005 # 그리퍼가 거의 닫혔는지 확인할 때의 오차 허용 범위

        # 큐브 속성 및 스폰 설정은 로봇과 무관하므로 그대로 둡니다.
        cube_properties = RigidBodyPropertiesCfg(
            solver_position_iteration_count=32,
            solver_velocity_iteration_count=10,
            max_angular_velocity=1000.0,
            max_linear_velocity=1000.0,
            max_depenetration_velocity=5.0,
            disable_gravity=False,
        )
        self.scene.cube_1 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Cube_1",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.2, 0.0, 0.0203], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/blue_block.usd",
                # scale=(1.0, 1.0, 1.0),
                scale=(0.7, 0.7, 0.7),
                rigid_props=cube_properties,
                semantic_tags=[("class", "cube_1")],
            ),
        )
        self.scene.cube_2 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Cube_2",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.25, 0.05, 0.0203], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/red_block.usd",
                scale=(0.7, 0.7, 0.7),
                # scale=(1.0, 1.0, 1.0),
                rigid_props=cube_properties,
                semantic_tags=[("class", "cube_2")],
            ),
        )
        self.scene.cube_3 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Cube_3",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.30, -0.1, 0.0203], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/green_block.usd",
                scale=(0.7, 0.7, 0.7),
                # scale=(1.0, 1.0, 1.0),
                rigid_props=cube_properties,
                semantic_tags=[("class", "cube_3")],
            ),
        )


        # --- [수정 7] 엔드 이펙터(End-Effector) 프레임 재정의 ---
        # 로봇의 어느 부분을 '손끝'으로 간주할지 SO101 모델에 맞게 설정합니다.
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"

        self.scene.ee_frame = FrameTransformerCfg(
            # 기준이 되는 프레임. 보통 로봇의 베이스 링크(root link)로 설정합니다.
            # TODO: SO101 USD 파일에서 베이스 링크의 정확한 이름을 확인해야 합니다. (예: 'base_link')
            prim_path="{ENV_REGEX_NS}/Robot/base",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    # 추적할 '손끝'에 해당하는 링크의 경로입니다.
                    # TODO: SO101 USD 파일에서 마지막 손목 링크의 정확한 이름을 확인해야 합니다.
                    prim_path="{ENV_REGEX_NS}/Robot/wrist",
                    name="end_effector",
                    offset=OffsetCfg(
                        # 위 링크로부터 실제 TCP(Tool Center Point)까지의 거리(오프셋)입니다.
                        # TODO: 이 값은 실제 그리퍼의 형상에 맞게 정확히 측정하여 입력해야 합니다.
                        pos=[0.0, 0.0, 0.12], # 예: Z축 방향으로 12cm
                    ),
                ),
                # Franka의 양쪽 손가락 프레임은 SO101에 해당하지 않으므로 삭제합니다.
            ],
        )