import os
import numpy as np
import isaacgym
from isaacgym import gymapi
from isaacgym import gymutil

# Initialize gym
gym = gymapi.acquire_gym()
# Parse arguments
args = gymutil.parse_arguments()

# create sim params
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

# # create default viewer
# viewer = gym.create_viewer(sim, gymapi.CameraProperties())

# add urdf asset
asset_root = "assets"
asset_file = "jumpingrobot/urdf/robot.urdf"

# Load the robot URDF
asset_options = gymapi.AssetOptions()
asset_options.fix_base_link = False
asset_options.default_dof_drive_mode = gymapi.DOF_MODE_POS
asset_options.collapse_fixed_joints = True

# robot path
robot_asset = gym.load_asset(sim, asset_root, asset_file, asset_options)
print('loading robot assets from...', asset_file)

# create env
env = gym.create_env(sim, gymapi.Vec3(-5.0, -5.0, -5.0), gymapi.Vec3(5.0, 5.0, 5.0), 1)

# add robot to env
pose = gymapi.Transform()
pose.p = gymapi.Vec3(0, 0, 0)
pose.r = gymapi.Quat(0, 0, 0, 1)

robot_handle = gym.create_actor(env, robot_asset, pose, "robot", 0, 1)


# add ground plane
plane_params = gymapi.PlaneParams()
plane_params.normal = gymapi.Vec3(0, 0, 1)
plane_params.distance = 0
plane_params.static_friction = 0.5
plane_params.dynamic_friction = 0.5
plane_params.restitution = 0
gym.add_ground(sim, plane_params)


# create cam viewer
camera_props = gymapi.CameraProperties()
viewer = gym.create_viewer(sim, camera_props)

# set cam position & target
cam_pos = gymapi.Vec3(0.5, 0.5, 0.5)
cam_target = gymapi.Vec3(0, 0, 0)
gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)


# main sim looooooooop
while not gym.query_viewer_has_closed(viewer):
    # step physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)
    
    # step rendering
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)
    
    # sync frame
    gym.sync_frame_time(sim)


print("--- sim complete ---")

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)