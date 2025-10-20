# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# This file registers the SO101 cube stacking environments with Gymnasium.

import gymnasium as gym

# --- [수정 1] Import할 설정 파일 변경 ---
# Franka용 설정 파일 대신 우리가 만든 SO101용 설정 파일들을 import 합니다.
from . import (
    so101_stack_joint_pos_env_cfg,
    so101_stack_ik_rel_env_cfg,
)

##
# Register Gym environments.
##

##
# Joint Position Control (관절 위치 제어)
##

gym.register(
    # --- [수정 2] 환경 ID 변경 ---
    # 환경의 고유 ID에서 'Franka'를 'SO101'로 변경합니다.
    id="Isaac-Stack-Cube-SO101-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        # --- [수정 3] 환경 설정 클래스 변경 ---
        # 이 ID로 환경을 생성할 때 사용할 설정 파일을 지정합니다.
        "env_cfg_entry_point": so101_stack_joint_pos_env_cfg.SO101CubeStackEnvCfg,
    },
    disable_env_checker=True,
)


##
# Inverse Kinematics - Relative Pose Control (IK 상대 좌표 제어)
##

gym.register(
    # --- [수정 2] 환경 ID 변경 ---
    id="Isaac-Stack-Cube-SO101-IK-Rel-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        # --- [수정 3] 환경 설정 클래스 변경 ---
        "env_cfg_entry_point": so101_stack_ik_rel_env_cfg.SO101CubeStackIKEnvCfg,
        # 참고: robomimic_bc_cfg_entry_point는 사전 훈련된 에이전트가 있을 경우 추가합니다.
        # SO101용으로 새로 학습을 시작할 것이므로 이 부분은 생략합니다.
    },
    disable_env_checker=True,
)

# 만약 추후에 시각 정보를 사용하는 환경이나 물체 위치를 랜덤화하는 환경 등
# SO101용으로 새로운 환경 설정을 추가하게 되면,
# 위와 같은 방식으로 gym.register() 블록을 추가해주면 됩니다.
#
# 예시:
# from . import so101_stack_visuomotor_env_cfg
#
# gym.register(
#     id="Isaac-Stack-Cube-SO101-IK-Rel-Visuomotor-v0",
#     entry_point="isaaclab.envs:ManagerBasedRLEnv",
#     kwargs={
#         "env_cfg_entry_point": so101_stack_visuomotor_env_cfg.SO101CubeStackVisuomotorEnvCfg,
#     },
#     disable_env_checker=True,
# )