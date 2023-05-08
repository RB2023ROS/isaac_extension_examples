# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.isaac.examples.base_sample import BaseSample

# This extension has franka related tasks and controllers as well
from omni.isaac.franka import Franka
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

class FrankaWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()
        # Robot specific class that provides extra functionalities
        # such as having gripper and end_effector instances.
        franka = world.scene.add(Franka(prim_path="/World/Fancy_Franka", name="fancy_franka"))
        # add a cube for franka to pick up
        world.scene.add(
            DynamicCuboid(
                prim_path="/World/random_cube",
                name="fancy_cube",
                position=np.array([0.3, 0.3, 0.3]),
                scale=np.array([0.0515, 0.0515, 0.0515]),
                color=np.array([0, 0, 1.0]),
            )
        )
        return