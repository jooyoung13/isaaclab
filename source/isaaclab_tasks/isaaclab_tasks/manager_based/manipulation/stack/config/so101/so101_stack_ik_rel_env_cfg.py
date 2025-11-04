# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUT...
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# This file adapts the SO101 stacking environment to use Differential Inverse Kinematics (IK) control.
# It maintains the original code structure by loading a separate high-PD robot configuration for IK.

from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.devices.device_base import DevicesCfg
from isaaclab.devices.keyboard import Se3KeyboardCfg
from isaaclab.devices.openxr.openxr_device import OpenXRDevice, OpenXRDeviceCfg
from isaaclab.devices.openxr.retargeters.manipulator.gripper_retargeter import GripperRetargeterCfg
from isaaclab.devices.openxr.retargeters.manipulator.se3_rel_retargeter import Se3RelRetargeterCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.utils import configclass

# SO101의 관절 위치 제어 환경을 기반으로 상속받습니다.
from . import so101_stack_joint_pos_env_cfg

##
# Pre-defined configs
##
# --- [구조 일치] IK 제어에 최적화된 별도의 SO101 설정을 import 합니다. ---
# TODO: 아래 경로는 실제 SO101_HIGH_PD_CFG가 있는 파일 위치로 수정해야 합니다.
# from path.to.your.assets import SO101_HIGH_PD_CFG  # isort: skip
from isaaclab_assets.robots.so101 import SO101_CFG  # isort: skip

##출력용으로 추가
import omni.log

@configclass
class SO101CubeStackIKEnvCfg(so101_stack_joint_pos_env_cfg.SO101CubeStackEnvCfg):
    """Configuration for the SO101 cube stacking environment with Differential IK control."""

    def __post_init__(self):
        # 부모 클래스의 설정을 먼저 로드합니다.
        super().__post_init__()
        

        # --- [구조 일치] 씬의 로봇 설정을 IK용 고이득(High-PD) 설정으로 통째로 교체합니다. ---
        # 이 방식은 기존 설정을 코드 내에서 수정하는 대신, 준비된 설정을 불러와 대체합니다.
        self.scene.robot = SO101_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        
        # !! 바로 여기에 로봇 위치(pos) 덮어쓰기 코드를 추가 !!
        self.scene.robot.init_state.pos = [0.0, 0.0, -0.02] # <-- 원하는 X, Y, Z 값으로 수정
        
        omni.log.warn("=============================================================")
        omni.log.warn(f"[DEBUG] STIFFNESS SET TO: {self.scene.robot.actuators['arm'].stiffness}")
        omni.log.warn(f"[DEBUG] DAMPING SET TO: {self.scene.robot.actuators['arm'].damping}")
        omni.log.warn("=============================================================")

        # 팔 행동(Arm Action)을 IK 제어로 덮어씁니다.
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=[
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
            ],
            # TODO: SO101 USD 파일에서 마지막 손목 링크의 정확한 이름을 확인해야 합니다.
            body_name="wrist",
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="dls",
            ),
            scale=0.5,
            # TCP 오프셋 설정. 이 값은 so101_stack_env_cfg.py의 FrameTransformerCfg 오프셋과 일치해야 합니다.
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.12]),
        )

        # 원격 조종(Teleoperation) 장치 설정 (이 부분은 원본 구조와 동일)
        self.teleop_devices = DevicesCfg(
            devices={
                "handtracking": OpenXRDeviceCfg(
                    retargeters=[
                        Se3RelRetargeterCfg(
                            bound_hand=OpenXRDevice.TrackingTarget.HAND_RIGHT,
                            zero_out_xy_rotation=True,
                            use_wrist_rotation=False,
                            use_wrist_position=True,
                            delta_pos_scale_factor=10.0,
                            delta_rot_scale_factor=10.0,
                            sim_device=self.sim.device,
                        ),
                        GripperRetargeterCfg(
                            bound_hand=OpenXRDevice.TrackingTarget.HAND_RIGHT, sim_device=self.sim.device
                        ),
                    ],
                    sim_device=self.sim.device,
                    xr_cfg=self.xr,
                ),
                "keyboard": Se3KeyboardCfg(
                    pos_sensitivity=0.05,
                    rot_sensitivity=0.05,
                    sim_device=self.sim.device,
                ),
            }
        )