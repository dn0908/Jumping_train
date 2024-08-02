import os
import numpy as np

import isaacgym
from isaacgym import gymapi
from isaacgym import gymutil

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.vec_env import VecNormalize
from stable_baselines3.common.callbacks import CheckpointCallback
import torch as th

def create_stairs(gym, sim, env, num_steps, step_height, step_depth):
    stairs = []
    for i in range(num_steps):
        step_pose = gymapi.Transform()
        step_pose.p = gymapi.Vec3(i * step_depth, 0, i * step_height)
        step_pose.r = gymapi.Quat(0, 0, 0, 1)

        box_size = gymapi.Vec3(step_depth / 2, 2, step_height / 2)  # Adjust the size accordingly
        box_asset = gym.create_box(sim, box_size.x, box_size.y, box_size.z, None)

        step_handle = gym.create_actor(env, box_asset, step_pose, f"step_{i}", 0, 1)
        stairs.append(step_handle)
    return stairs


# Initialize gym
gym = gymapi.acquire_gym()
# Parse arguments
args = gymutil.parse_arguments()

# Create sim params
sim_params = gymapi.SimParams()
sim_params.up_axis = gymapi.UP_AXIS_Z
sim_params.dt = 1.0 / 60.0
sim_params.substeps = 2
sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.8)
sim_params.physx.solver_type = 1
sim_params.physx.num_position_iterations = 4
sim_params.physx.num_velocity_iterations = 1
sim_params.physx.rest_offset = 0.001
sim_params.physx.contact_offset = 0.02

sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

# Add URDF asset
asset_root = "assets"
asset_file = "jumpingrobot/urdf/robot.urdf"

# Load the robot URDF
asset_options = gymapi.AssetOptions()
asset_options.fix_base_link = False
asset_options.default_dof_drive_mode = gymapi.DOF_MODE_POS
asset_options.collapse_fixed_joints = True

# Robot path
robot_asset = gym.load_asset(sim, asset_root, asset_file, asset_options)
print('Loading robot assets from...', asset_file)

# Create env
env = gym.create_env(sim, gymapi.Vec3(-5.0, -5.0, -5.0), gymapi.Vec3(5.0, 5.0, 5.0), 1)

# Add robot to env
pose = gymapi.Transform()
pose.p = gymapi.Vec3(0, 0, 0)
pose.r = gymapi.Quat(0, 0, 0, 1)

robot_handle = gym.create_actor(env, robot_asset, pose, "robot", 0, 1)

# Add ground plane
plane_params = gymapi.PlaneParams()
plane_params.normal = gymapi.Vec3(0, 0, 1)
plane_params.distance = 0
plane_params.static_friction = 0.5
plane_params.dynamic_friction = 0.5
plane_params.restitution = 0
gym.add_ground(sim, plane_params)

# Generate stairs
num_steps = 10
step_height = 0.2
step_depth = 0.5
stairs = create_stairs(gym, sim, env, num_steps, step_height, step_depth)

# Create cam viewer
camera_props = gymapi.CameraProperties()
viewer = gym.create_viewer(sim, camera_props)

# Set cam position & target
cam_pos = gymapi.Vec3(0.5, 0.5, 0.5)
cam_target = gymapi.Vec3(0, 0, 0)
gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

# Training setup
class JumpingRobotEnv:
    def __init__(self, gym, sim, env, robot_handle, stairs):
        self.gym = gym
        self.sim = sim
        self.env = env
        self.robot_handle = robot_handle
        self.stairs = stairs
        self.state = None

    def reset(self):
        # Reset the robot and stairs here
        self.gym.set_actor_root_state(self.env, self.robot_handle, pose)
        self.state = self.get_state()
        return self.state

    def step(self, action):
        # Apply action to the robot
        self.gym.set_actor_dof_position_targets(self.env, self.robot_handle, action)
        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)

        # Get new state and reward
        self.state = self.get_state()
        reward = self.calculate_reward()
        done = self.is_done()
        return self.state, reward, done, {}

    def get_state(self):
        # Get robot state
        state = self.gym.get_actor_dof_states(self.env, self.robot_handle, gymapi.STATE_ALL)
        return state

    def calculate_reward(self):
        # Calculate reward based on robot's state and target
        reward = 0  # Placeholder for reward calculation
        return reward

    def is_done(self):
        # Determine if the episode is done
        done = False  # Placeholder for done condition
        return done

# Create the environment
env_fn = lambda: JumpingRobotEnv(gym, sim, env, robot_handle, stairs)
env = DummyVecEnv([env_fn])
env = VecNormalize(env, norm_obs=True, norm_reward=True)

# Instantiate the agent
model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./ppo_jumping_robot_tensorboard/")


# Create a callback to save models
checkpoint_callback = CheckpointCallback(save_freq=1000, save_path='./logs/', name_prefix='ppo_model')

# Train the agent
model.learn(total_timesteps=10000, callback=checkpoint_callback)

# Main simulation loop
while not gym.query_viewer_has_closed(viewer):
    # step physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # step rendering
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)

    # sync frame
    gym.sync_frame_time(sim)

print("--- Simulation complete ---")

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
