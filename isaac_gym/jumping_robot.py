import math
from isaacgym import gymapi
from isaacgym import gymutil
from base import Base
# from logger import Logger
from utils import *
import os
import numpy as np

class Jumpingrobot(Base):
    def __init__(self):
        self.gym = None
        self.sim = None
        self.viewer = None

        self.points = [False, False, False, False, False]

    def create_scene(self):
        # add ground plane
        plane_params = gymapi.PlaneParams()
        self.gym.add_ground(self.sim, gymapi.PlaneParams())
        # set up the env grid
        num_envs = 1
        spacing = 1.5
        env_lower = gymapi.Vec3(-spacing, 0.0, -spacing)
        env_upper = gymapi.Vec3(spacing, 0.0, spacing)

        # add urdf asset
        asset_root = "assets"
        asset_file = "jumpingrobot/urdf/robot.urdf"

        # Load asset with default control type of position for all joints
        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = True
        asset_options.default_dof_drive_mode = gymapi.DOF_MODE_POS
        print("Loading asset '%s' from '%s'" % (asset_file, asset_root))
        robot_asset = self.gym.load_asset(self.sim, asset_root, asset_file, asset_options)
        
        # initial root pose for cartpole actors
        initial_pose = gymapi.Transform()
        initial_pose.p = gymapi.Vec3(0.0, 0.1, 0.0)
        initial_pose.r = gymapi.Quat(-0.707107, 0.0, 0.0, 0.707107)
        
        self.env0 = self.gym.create_env(self.sim, env_lower, env_upper, 2)
        jumping_robot = self.gym.create_actor(self.env0, robot_asset, initial_pose, 'cancellor', 0, 1)

        # Configure DOF properties
        props = self.gym.get_actor_dof_properties(self.env0, jumping_robot)
        props["driveMode"] = (gymapi.DOF_MODE_VEL, gymapi.DOF_MODE_VEL, gymapi.DOF_MODE_POS, gymapi.DOF_MODE_POS)
        props["stiffness"] = (0.0, 0.0, 5000.0, 5000.0)
        props["damping"] = (200.0, 200.0, 100.0, 100.0)
        self.gym.set_actor_dof_properties(self.env0, jumping_robot, props)

    def show(self):
        # while not self.gym.query_viewer_has_closed(self.viewer):

        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)
        self.run()
        self.gym.sync_frame_time(self.sim)

        print('Done')

        self.gym.destroy_viewer(self.viewer)
        self.gym.destroy_sim(self.sim)