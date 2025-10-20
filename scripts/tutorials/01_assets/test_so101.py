# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""This script tests the custom SO101 robot configuration.""" # MODIFIED

"""Launch Isaac Sim Simulator first."""
import math

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Test script for the SO101 robot.") # MODIFIED
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext

##
# Pre-defined configs
##
# MODIFIED: Cartpole 대신 SO101 설정을 불러옵니다.
# so101.py 파일이 isaaclab.assets 폴더 안에 있다고 가정합니다.
# 만약 다른 곳에 있다면 경로를 맞게 수정해야 합니다. (예: from my_robots import SO101_CFG)
from isaaclab_assets import SO101_CFG
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR

def design_scene() -> dict: # MODIFIED: 반환 타입 변경
    """Designs the scene."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # --- 1. 테이블 스폰 ---
    table_cfg = sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"
    )
    # z=1.05로 올려서 테이블 바닥이 월드 z=0보다 위로 오도록 배치
    table_cfg.func("/World/Objects/Table", table_cfg, translation=(0.0, 0.0, 1.05))
    
    # --- 2. 로봇 스폰 ---
    so101_cfg = SO101_CFG.copy()
    so101_cfg.prim_path = "/World/Robot"
    
    robot = Articulation(cfg=so101_cfg)

    # return the scene information
    scene_entities = {"so101": robot} # MODIFIED: 딕셔너리 키 이름 변경
    return scene_entities


def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation]): # MODIFIED: 파라미터 변경
    """Runs the simulation loop."""
    # Extract scene entities
    robot = entities["so101"] # MODIFIED: 딕셔너리 키 이름 변경
    
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0
    # Simulation loop
    while simulation_app.is_running():
        # Reset
        if count % 500 == 0:
            count = 0

            # ---- root state z 올리기 ----
            new_root_state = robot.data.default_root_state.clone()
            new_root_state[0, 0] = -0.6   # x 좌표
            new_root_state[0, 1] = 0.0  # y 좌표
            new_root_state[0, 2] = 1.02   # z 좌표
            yaw = math.radians(90)
            qw = math.cos(yaw / 2)
            qx = 0.0
            qy = 0.0
            qz = math.sin(yaw / 2)
            new_root_state[0, 3:7] = torch.tensor([qw, qx, qy, qz])
            
            
            robot.data.default_root_state = new_root_state

            joint_pos = robot.data.default_joint_pos.clone()
            joint_vel = robot.data.default_joint_vel.clone()
            robot.write_root_state_to_sim(robot.data.default_root_state)
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            robot.reset()
            print("[INFO]: Resetting robot state at z=1.2 ...")

        # MODIFIED: 랜덤 토크 대신 랜덤 '목표 관절 각도'를 주어 움직이게 합니다.
        # 이렇게 하면 PD 제어기가 잘 작동하는지 더 직관적으로 볼 수 있습니다.
        # 기본 자세를 기준으로 -0.5 ~ +0.5 라디안 범위 내에서 랜덤한 목표 자세를 생성합니다.
        if count==100:
            target_positions = robot.data.default_joint_pos + (torch.rand_like(robot.data.joint_pos) - 0.5)
            # 생성된 목표 자세를 로봇에 명령합니다.
            robot.set_joint_position_target(target_positions)

            # -- write data to sim
            robot.write_data_to_sim()
            
            
        # Perform step
        sim.step()
        # Increment counter
        count += 1
        # Update buffers
        robot.update(sim_dt)
        


def main():
    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    
    # MODIFIED: SO101 로봇을 보기 좋은 카메라 뷰로 설정
    # sim.set_camera_view([1.5, 1.5, 1.0], [0.0, 0.0, 0.5])
    sim.set_camera_view([1.5, 1.5, 1.3], [0.0, 0.0, 1.0])

    # Design scene
    scene_entities = design_scene() # MODIFIED: 반환값 변경
    
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene_entities) # MODIFIED: 인자 변경


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()