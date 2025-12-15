from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class GO2RoughCfg( LeggedRobotCfg ):
    class env( LeggedRobotCfg.env ):
        num_observations = 317  # matches go2_env observation construction (history_len=5, n_proprio=45)
        history_encoding = True
        history_len = 5
        n_proprio = 45
        contact_buf_len = 5
        include_act_obs_pair_buf = False

    class terrain( LeggedRobotCfg.terrain ):
        mesh_type = 'plane' #'trimesh' # "heightfield" # none, plane, heightfield or trimesh
        measure_heights = False #True
        terrain_length = 8.0
        terrain_width = 8.0
        env_spacing = 4.  # not used with heightfields/trimeshes
        include_act_obs_pair_buf = False


        y_range = [-1.3, 1.3]

        horizontal_scale = 0.05 # [m]
        vertical_scale = 0.005 # [m]
        border_size = 5 # [m]
        height = [0.03, 0.04]
        downsampled_scale = 0.2
        curriculum = False

        static_friction = 1.0
        dynamic_friction = 1.0
        restitution = 0.
        # rough terrain only:
        measured_points_x = [-0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8] # 1mx1.6m rectangle (without center line)
        measured_points_y = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]

        selected = False # select a unique terrain type and pass all arguments
        terrain_kwargs = None # Dict of arguments for selected terrain
        max_init_terrain_level = 5 # starting curriculum state
        num_rows= 10 # number of terrain rows (levels)
        num_cols = 10 # number of terrain cols (types)
        # # terrain types: [smooth slope, rough slope, stairs up, stairs down, discrete]
        # terrain_proportions = [0.1, 0.1, 0.35, 0.25, 0.2]

        # ! parkour definition
        terrain_dict = {"smooth slope": 0.2,
                        "rough slope": 0.0,
                        "stairs up": 0.,
                        "stairs down": 0.,
                        "discrete": 0.,
                        "stepping stones": 0.0,
                        "gaps": 0.,
                        "pit": 0.0,
                        "locomotion rough": 0.2,
                        "parkour_hurdle": 0.0,
                        "parkour_frame": 0.0 }
        terrain_proportions = list(terrain_dict.values())

        # trimesh only:
        slope_treshold = 0.75 # slopes above this threshold will be corrected to vertical surfaces
        origin_zero_z = True

        num_goals = 4

        # frame related
        combine_frame = False

    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.42] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.1,   # [rad]
            'RL_hip_joint': 0.1,   # [rad]
            'FR_hip_joint': -0.1 ,  # [rad]
            'RR_hip_joint': -0.1,   # [rad]

            'FL_thigh_joint': 0.8,     # [rad]
            'RL_thigh_joint': 1.,   # [rad]
            'FR_thigh_joint': 0.8,     # [rad]
            'RR_thigh_joint': 1.,   # [rad]

            'FL_calf_joint': -1.5,   # [rad]
            'RL_calf_joint': -1.5,    # [rad]
            'FR_calf_joint': -1.5,  # [rad]
            'RR_calf_joint': -1.5,    # [rad]
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'joint': 20.}  # [N*m/rad]
        damping = {'joint': 0.5}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        hip_scale_reduction = 1.0
        use_filter = False
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/go2/urdf/go2.urdf'
        name = "go2"
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
  
    class domain_rand( LeggedRobotCfg.domain_rand ):
        randomize_friction = True
        friction_range = [0.8, 1.2]
        randomize_base_mass = False
        added_mass_range = [-1., 1.]
        push_robots = True
        push_interval_s = 15
        max_push_vel_xy = 1.
        lag_timesteps = 1
        motor_strength_range = [0.9, 1.1]
        kp_range = [0.9, 1.1]
        kd_range = [0.9, 1.1]
        randomize_kpkd = False
  
    # class rewards( LeggedRobotCfg.rewards ):
    #     soft_dof_pos_limit = 0.9
    #     base_height_target = 0.25
    #     class scales( LeggedRobotCfg.rewards.scales ):
    #         torques = -0.0002
    #         dof_pos_limits = -10.0
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.4
        clearance_height_target = -0.2

        class scales( LeggedRobotCfg.rewards.scales ):
            termination = -0.0
            tracking_lin_vel = 1.0
            tracking_ang_vel = 0.5
            lin_vel_z = -2.0
            ang_vel_xy = -0.05
            orientation = -0.
            torques = -0.00001  # Changed from 0.0 to small penalty
            dof_vel = -0.
            dof_acc = -2.5e-7
            base_height = -0.99
            feet_air_time = 1.0
            collision = -1.
            feet_stumble = -0.0
            action_rate = -0.01
            stand_still = -0.
            dof_pos_limits = -0.05
            # trot_contact = 0.5
            # pace_contact = 0.5
            bound_contact = 0.5
            # pace_contact = 0.5
            bound_contact = 1.0

        only_positive_rewards = False
        tracking_sigma = 0.25
        soft_dof_pos_limit = 0.9  # GO2 specific - tighter than base
        soft_dof_vel_limit = 1.
        soft_torque_limit = 1.
        base_height_target = 0.30  # GO2 specific height
        max_contact_force = 100.

    # Cost configuration (used by go2_env.py)
    class cost:
        use_costs = False  # set True to enable cost functions during training
        num_costs = 0

    class costs:
        class scales:
            pass  # no cost scales defined when use_costs is False

        class d_values:
            pass  # no derivative values defined when use_costs is False

    # Depth camera configuration (defaults keep camera disabled)
    class depth:
        use_camera = False
        original = [128, 128]      # sensor output size before crop/resize
        resized = [128, 128]       # final depth image size used in policy
        buffer_len = 2             # number of frames kept per env
        update_interval = 1        # steps between captures
        horizontal_fov = 90.0
        position = [0.2, 0.0, 0.2] # camera pose in robot frame
        angle = [-15.0, -15.0]     # pitch range (deg)
        near_clip = 0.1
        far_clip = 3.0
        dis_noise = 0.0

class GO2RoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'rough_go2'


  
