"""Standalone curobo MotionGen wrapper — no ROS2 dependency."""

import time
from typing import Dict, List, Optional

import numpy as np
import torch

from curobo.cuda_robot_model.util import load_robot_yaml
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import Cuboid, WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.file_path import ContentPath
from curobo.types.math import Pose
from curobo.types.state import JointState as CuJointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

from standalone.config import (
    INTERPOLATION_DT,
    JOINT_NAMES,
    MAX_ATTEMPTS,
    NUM_GRAPH_SEEDS,
    NUM_TRAJOPT_SEEDS,
    TRAJOPT_TSTEPS,
)


class StandaloneMotionPlanner:
    """GPU-accelerated motion planner using curobo MotionGen (no ROS2)."""

    def __init__(
        self,
        xrdf_path: str,
        urdf_path: str,
        add_ground_plane: bool = True,
        interpolation_dt: float = INTERPOLATION_DT,
    ):
        self.tensor_args = TensorDeviceType()

        # Load robot config from XRDF + URDF
        content_path = ContentPath(
            robot_xrdf_absolute_path=xrdf_path,
            robot_urdf_absolute_path=urdf_path,
        )
        robot_config = load_robot_yaml(content_path)
        robot_dict = robot_config["robot_cfg"]

        # World config (ground plane only, no voxels/ESDF needed)
        world_dict = {}
        if add_ground_plane:
            world_dict["cuboid"] = {
                "table": {
                    "pose": [0, 0, -0.05, 1, 0, 0, 0],
                    "dims": [2.0, 2.0, 0.1],
                }
            }
        world_file = WorldConfig.from_dict(world_dict) if world_dict else None

        # Build MotionGen config — MESH checker (no nvblox needed)
        motion_gen_config = MotionGenConfig.load_from_robot_config(
            robot_dict,
            world_file,
            self.tensor_args,
            num_graph_seeds=NUM_GRAPH_SEEDS,
            num_trajopt_seeds=NUM_TRAJOPT_SEEDS,
            trajopt_tsteps=TRAJOPT_TSTEPS,
            interpolation_dt=interpolation_dt,
            collision_checker_type=CollisionCheckerType.MESH,
        )

        self.motion_gen = MotionGen(motion_gen_config)
        self.joint_names = JOINT_NAMES
        self.ee_link = robot_dict["kinematics"].get("ee_link", "tool0")

        # Warmup
        print("[cuMotion] Warming up GPU planner...")
        t0 = time.time()
        self.motion_gen.warmup(enable_graph=True)
        print(f"[cuMotion] Ready! (warmup: {time.time() - t0:.1f}s)")

    def plan_joint(
        self,
        start_joints: List[float],
        goal_joints: List[float],
        velocity_scale: float = 0.5,
    ) -> Optional[Dict]:
        """Plan joint-space trajectory. Returns dict or None on failure."""
        start_state = CuJointState.from_position(
            position=self.tensor_args.to_device(start_joints).unsqueeze(0),
            joint_names=self.joint_names,
        )
        goal_state = CuJointState.from_position(
            position=self.tensor_args.to_device(goal_joints).unsqueeze(0),
            joint_names=self.joint_names,
        )

        plan_config = MotionGenPlanConfig(
            max_attempts=MAX_ATTEMPTS,
            enable_graph_attempt=1,
            time_dilation_factor=velocity_scale,
        )
        self.motion_gen.reset(reset_seed=False)

        t0 = time.time()
        result = self.motion_gen.plan_single_js(start_state, goal_state, plan_config)
        plan_time = time.time() - t0

        if not result.success.item():
            print(f"[cuMotion] Planning failed: {result.status}")
            return None

        return self._extract_trajectory(result, plan_time)

    def plan_cartesian(
        self,
        start_joints: List[float],
        goal_position: List[float],
        goal_quaternion: List[float],
        velocity_scale: float = 0.5,
    ) -> Optional[Dict]:
        """Plan to Cartesian pose. goal_quaternion: [qw, qx, qy, qz]. Returns dict or None."""
        start_state = CuJointState.from_position(
            position=self.tensor_args.to_device(start_joints).unsqueeze(0),
            joint_names=self.joint_names,
        )

        # curobo Pose: position [x,y,z], quaternion [qw,qx,qy,qz]
        goal_pose = Pose(
            position=self.tensor_args.to_device(goal_position).unsqueeze(0),
            quaternion=self.tensor_args.to_device(goal_quaternion).unsqueeze(0),
        )

        plan_config = MotionGenPlanConfig(
            max_attempts=MAX_ATTEMPTS,
            enable_graph_attempt=1,
            time_dilation_factor=velocity_scale,
        )
        self.motion_gen.reset(reset_seed=False)

        t0 = time.time()
        result = self.motion_gen.plan_single(start_state, goal_pose, plan_config)
        plan_time = time.time() - t0

        if not result.success.item():
            print(f"[cuMotion] Cartesian planning failed: {result.status}")
            return None

        return self._extract_trajectory(result, plan_time)

    def _extract_trajectory(self, result, plan_time: float) -> Dict:
        """Extract interpolated trajectory data from MotionGenResult."""
        # Use interpolated plan (dense waypoints at interpolation_dt) instead of
        # optimized_plan (sparse trajopt_tsteps waypoints)
        js = result.get_interpolated_plan()
        dt = result.interpolation_dt

        positions = js.position.cpu().view(-1, js.position.shape[-1]).numpy()
        velocities = js.velocity.cpu().view(-1, js.velocity.shape[-1]).numpy()
        accelerations = js.acceleration.cpu().view(-1, js.acceleration.shape[-1]).numpy()

        n_points = len(positions)
        timestamps = np.arange(n_points) * dt

        return {
            "positions": positions,       # (N, 6) ndarray
            "velocities": velocities,     # (N, 6) ndarray
            "accelerations": accelerations,  # (N, 6) ndarray
            "dt": dt,                      # float (seconds per step)
            "timestamps": timestamps,      # (N,) ndarray
            "joint_names": js.joint_names,
            "n_points": n_points,
            "total_time": timestamps[-1],
            "plan_time": plan_time,
        }

    def add_cuboid(self, name: str, pose: List[float], dims: List[float]):
        """Add a cuboid obstacle. pose: [x,y,z,qw,qx,qy,qz], dims: [dx,dy,dz]."""
        cuboid = Cuboid(name=name, pose=pose, dims=dims)
        world = WorldConfig(cuboid=[cuboid]).get_collision_check_world()
        self.motion_gen.update_world(world)
        print(f"[cuMotion] Added cuboid '{name}'")

    def clear_world(self):
        """Clear all collision objects."""
        self.motion_gen.clear_world_cache()
        print("[cuMotion] World cleared")

    def get_ee_pose(self, joint_positions: List[float]) -> Dict:
        """Forward kinematics: joints → end-effector pose."""
        q = self.tensor_args.to_device(joint_positions).unsqueeze(0)
        state = CuJointState.from_position(position=q, joint_names=self.joint_names)
        kin_state = self.motion_gen.kinematics.get_state(state.position)
        ee_pos = kin_state.ee_position.cpu().numpy()[0]
        ee_quat = kin_state.ee_quaternion.cpu().numpy()[0]
        return {
            "position": ee_pos.tolist(),      # [x, y, z]
            "quaternion": ee_quat.tolist(),    # [qw, qx, qy, qz]
        }
