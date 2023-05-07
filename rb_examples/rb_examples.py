# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import numpy as np
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
import carb

# Note: checkout the required tutorials at https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html


class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):

        world = self.get_world()
        # 이게 바둑판 무늬 ground plane을 만들어주는 함수
        world.scene.add_default_ground_plane()

        # # Example1 - Add Cube
        # fancy_cube = world.scene.add(
        #     DynamicCuboid(
        #         prim_path="/World/random_cube", # The prim path of the cube in the USD stage
        #         name="fancy_cube", # The unique name used to retrieve the object from the scene later on
        #         position=np.array([0, 0, 1.0]), # Using the current stage units which is in meters by default.
        #         scale=np.array([0.5015, 0.5015, 0.5015]), # most arguments accept mainly numpy arrays.
        #         color=np.array([0, 0, 1.0]), # RGB channels, going from 0-1
        #     ))
        
        # Example2 - Add Robot
        # you configure a new server with /Isaac folder in it
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            # Use carb to log warnings, errors and infos in your application (shown on terminal)
            carb.log_error("Could not find nucleus server with /Isaac folder")
        asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
        # This will create a new XFormPrim and point it to the usd file as a reference
        # Similar to how pointers work in memory
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Fancy_Robot")
        # Wrap the jetbot prim root under a Robot class and add it to the Scene
        # to use high level api to set/ get attributes as well as initializing
        # physics handles needed..etc.
        # Note: this call doesn't create the Jetbot in the stage window, it was already
        # created with the add_reference_to_stage
        jetbot_robot = world.scene.add(Robot(prim_path="/World/Fancy_Robot", name="fancy_robot"))
        # Note: before a reset is called, we can't access information related to an Articulation
        # because physics handles are not initialized yet. setup_post_load is called after
        # the first reset so we can do so there
        print("Num of degrees of freedom before first reset: " + str(jetbot_robot.num_dof)) # prints None

        return

    # Here we assign the class's variables
    # this function is called after load button is pressed
    # regardless starting from an empty stage or not
    # this is called after setup_scene and after
    # one physics time step to propagate appropriate
    # physics handles which are needed to retrieve
    # many physical properties of the different objects
    async def setup_post_load(self):
        self._world = self.get_world()

        # # Example1 - Add Cube
        # self._cube = self._world.scene.get_object("fancy_cube")
        # position, orientation = self._cube.get_world_pose()
        # linear_velocity = self._cube.get_linear_velocity()
        # # will be shown on terminal
        # print("Cube position is : " + str(position))
        # print("Cube's orientation is : " + str(orientation))
        # print("Cube's linear velocity is : " + str(linear_velocity))

        # Example2 - Add Robot
        self._world = self.get_world()
        self._jetbot = self._world.scene.get_object("fancy_robot")
        # Print info about the jetbot after the first reset is called
        print("Num of degrees of freedom after first reset: " + str(self._jetbot.num_dof)) # prints 2
        print("Joint Positions after first reset: " + str(self._jetbot.get_joint_positions()))

        # Example3 - Move Robot
        # This is an implicit PD controller of the jetbot/ articulation
        # setting PD gains, applying actions, switching control modes..etc.
        # can be done through this controller.
        # Note: should be only called after the first reset happens to the world
        self._jetbot_articulation_controller = self._jetbot.get_articulation_controller()
        # Adding a physics callback to send the actions to apply actions with every
        # physics step executed.
        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)

        return

    def send_robot_actions(self, step_size):
        # Every articulation controller has apply_action method
        # which takes in ArticulationAction with joint_positions, joint_efforts and joint_velocities
        # as optional args. It accepts numpy arrays of floats OR lists of floats and None
        # None means that nothing is applied to this dof index in this step
        # ALTERNATIVELY, same method is called from self._jetbot.apply_action(...)
        self._jetbot_articulation_controller.apply_action(ArticulationAction(joint_positions=None,
                                                                            joint_efforts=None,
                                                                            joint_velocities=5 * np.random.rand(2,)))
        
        # print(step_size)
        return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        return
