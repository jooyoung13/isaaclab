# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the SO101 robot."""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR


##
# Configuration
##

SO101_CFG = ArticulationCfg(
    # 스폰 설정: 로봇의 USD 파일 경로를 지정합니다.
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/RobotStudio/so101_new_calib/so101_new_calib.usd",
        # usd_path="/home/jy/Downloads/so101_jy.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
        ),
    ),
    # 초기 상태 설정: 제공해주신 조인트 이름을 사용해 초기 자세를 0으로 설정합니다.
    init_state=ArticulationCfg.InitialStateCfg(
        rot=[0.7071, 0, 0, 0.7071],
        joint_pos={
            "shoulder_pan": 0.0,  # shoulder_pan
            "shoulder_lift": 0.0,  # shoulder_lift
            "elbow_flex": 0.0,  # elbow_flex
            "wrist_flex": 0.0,  # wrist_flex
            "wrist_roll": 0.0,  # wrist_roll
            "gripper": 0.0,  # gripper
        },
    ),
    # 액추에이터 설정: 로봇의 관절을 움직이는 가상 모터를 정의합니다.
    # 팔과 그리퍼는 제어 특성이 다르므로 그룹을 나누는 것이 좋습니다.
    actuators={
        "arm": ImplicitActuatorCfg(
            # 1번~5번 조인트를 'arm' 그룹으로 묶습니다.
             joint_names_expr=[
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
            ],
            # 아래 값들은 시작점이며, workflow_tool로 테스트하며 튜닝해야 합니다.
            effort_limit_sim=500.0,  # 최대 토크 (단위: Nm)
            stiffness=1500.0,         # P 게인 (목표 각도에 얼마나 강하게 도달할지)
            damping=60.0,            # D 게인 (진동을 얼마나 억제할지)
        ),
        "gripper": ImplicitActuatorCfg(
            # 6번 조인트를 'gripper' 그룹으로 지정합니다.
            joint_names_expr=["gripper"],
            # 그리퍼는 보통 더 강한 힘이 필요하므로 다른 값을 설정할 수 있습니다.
            effort_limit_sim=50.0,
            stiffness=200.0,
            damping=10.0,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)