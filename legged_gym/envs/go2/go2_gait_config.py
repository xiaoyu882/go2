from legged_gym.envs.go2.go2_config import GO2RoughCfg, GO2RoughCfgPPO
from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg

class GO2WalkCfg(GO2RoughCfg):
    class env(GO2RoughCfg.env):
        episode_length_s = 20  

    class commands(GO2RoughCfg.commands):
        class ranges:
            lin_vel_x = [0.0, 0.8]   
            lin_vel_y = [-0.3, 0.3]
            ang_vel_yaw = [-0.5, 0.5]
            heading = [-3.14, 3.14]
    
    class rewards(GO2RoughCfg.rewards):
        class scales:
            tracking_lin_vel = 1.2
            orientation = -1.0
            feet_air_time = 0.5
            action_rate = -0.02

class GO2TrotCfg(GO2RoughCfg):
    class commands(GO2RoughCfg.commands):
        class ranges:
            lin_vel_x = [0.0, 1.5]
            lin_vel_y = [-0.5, 0.5]
            ang_vel_yaw = [-1.0, 1.0]
            heading = [-3.14, 3.14]

    class rewards(GO2RoughCfg.rewards):
        class scales:
            tracking_lin_vel = 1.0
            tracking_ang_vel = 0.8
            feet_air_time = 0.3

class GO2PaceCfg(GO2RoughCfg):
    class commands(GO2RoughCfg.commands):
        class ranges:
            lin_vel_x = [0.5, 2.0]
            lin_vel_y = [-0.3, 0.3]  
            ang_vel_yaw = [-0.8, 0.8]
            heading = [-3.14, 3.14]

    class rewards(GO2RoughCfg.rewards):
        class scales:
            tracking_lin_vel = 1.5  
            orientation = -0.5     
            torques = -0.0001      

class GO2BoundCfg(GO2RoughCfg):
    class commands(GO2RoughCfg.commands):
        class ranges:
            lin_vel_x = [0.8, 2.5]   
            lin_vel_y = [-0.2, 0.2] 
            ang_vel_yaw = [-0.6, 0.6]
            heading = [-3.14, 3.14]

    class rewards(GO2RoughCfg.rewards):
        class scales:
            tracking_lin_vel = 2.0
            dof_acc = -2.5e-7        
            action_rate = -0.005   

class GO2WalkCfgPPO(GO2RoughCfgPPO):
    class runner(GO2RoughCfgPPO.runner):
        experiment_name = 'go2_walk'
        run_name = 'walk_gait'

class GO2TrotCfgPPO(GO2RoughCfgPPO):
    class runner(GO2RoughCfgPPO.runner):
        experiment_name = 'go2_trot'
        run_name = 'trot_gait'

class GO2PaceCfgPPO(GO2RoughCfgPPO):
    class runner(GO2RoughCfgPPO.runner):
        experiment_name = 'go2_pace'
        run_name = 'pace_gait'

class GO2BoundCfgPPO(GO2RoughCfgPPO):
    class runner(GO2RoughCfgPPO.runner):
        experiment_name = 'go2_bound'
        run_name = 'bound_gait'