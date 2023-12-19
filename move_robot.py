import numpy as np
from omni.isaac.dynamic_control import _dynamic_control


def set_robot_joints(joints:np.array):
    dc = _dynamic_control.acquire_dynamic_control_interface()

    # Get the articulation object representing the Franka robot
    art = dc.get_articulation("/robobar_with_fork")
    dc.wake_up_articulation(art)

    # Set joint positions
    joint_positions = [joints]

    dc.set_articulation_dof_position_targets(art, joint_positions)
    _dynamic_control.release_dynamic_control_interface(dc)


def reset_robot_home():
    # reset controller to home
    set_robot_joints(np.array([-0.00521, -1.456, 0.02526, -2.3688, 0.1014, 2.515, 0.712]))

def reset_robot_glass_catch():
    # reset controller to home
    set_robot_joints(np.array([0.0070539364, -0.61308086, -3.5514844e-05, -1.832153, -0.04103387, 2.8022785, 0.83300]))
