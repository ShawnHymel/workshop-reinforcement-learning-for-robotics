"""
Gymnasium wrapper for MuJoCo simulator that loads and runs the environment for a simple 2-wheel
balance bot. Adds domain randomization for bumps, pushes, IMU noise, motor noise, mass randomization
etc.

Author: Shawn Hymel
Date: May 13, 2026
"""

# Standard libraries
from dataclasses import dataclass
import math
from pathlib import Path

# Third-party libraries
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import mujoco
import mujoco.viewer


@dataclass  
class DomainRandomConfig:
    """
    Configuration for domain randomization in BalanceBotEnv. Pass an instance to BalanceBotEnv's
    domain_rand parameter to enable.  All values default to disabled (0.0, False, or identity 
    ranges).
    
    Attributes:
        pitch_noise_std_dev: Standard deviation of Gaussian noise added to pitch observation. 
                             Simulates IMU noise.
        pitch_rate_noise_std_dev: Standard deviation of Gaussian noise added to pitch rate
                                  observation. Simulates IMU noise.
        yaw_rate_noise_std_dev: Standard deviation of Gaussian noise added to yaw rate
                                observation. Simulates IMU noise.
        action_delay_steps: Number of steps to delay actions (0=disabled). Simulates I2C and motor 
                            response latency.
        action_delay_random: If True, randomize delay 0..action_delay_steps each episode.
        motor_noise_scale: Uniform noise magnitude added to motor commands. Simulates motor driver 
                           noise and tire ridges.
        push_prob: Probability per step of applying a random external force. Simulates bumps, 
                   nudges, and uneven terrain effects.
        push_force_max_n: Maximum magnitude of random push force in Newtons.
        mass_scale_range: (min, max) scaling factor for chassis mass each episode. Simulates payload
                          variation and model uncertainty.
        friction_scale_range: (min, max) scaling factor for wheel-to-ground friction each episode.
                              Simulates different floor surfaces.
        motor_gain_range: (min, max) Simulate motor torque variance (e.g. battery sag) through gain
                          (gainprm in MJCF)
        ridge_prob: Probability of applying a random torque to the wheel axles to simulate the tire
                    ridges hitting the ground
        ridge_torque_max_nm: Max random torque to apply to axles (N-m)
        gravity_tilt_max_deg: Maximum angle (degrees) between the gravity vector and straight down,
                              sampled uniformly in [0, max] each episode. Simulates sloped ground.
        motor_deadband: Actions with abs() below this are zeroed
    """
    pitch_noise_std_dev: float = 0.0
    pitch_rate_noise_std_dev: float = 0.0
    yaw_rate_noise_std_dev: float = 0.0
    action_delay_steps: int = 0
    action_delay_random: bool = False
    motor_noise_scale: float = 0.0
    push_prob: float = 0.00
    push_force_max_n: float = 0.0
    mass_scale_range: tuple = (1.0, 1.0)
    friction_scale_range: tuple = (1.0, 1.0)
    motor_gain_range: tuple = (1.0, 1.0)
    ridge_prob: float = 0.0
    ridge_torque_max_nm: float = 0.0
    gravity_tilt_max_deg: float = 0.0
    motor_deadband: float = 0.0
    

class BalanceBotEnv(gym.Env):
    """
    Define our own gymnasium environment class that wraps the MuJoCo simulator so we can interact
    with it using the standard gymnasium methods (e.g. step(), reset(), action_space, etc.).
    """

    # Attribute: declare which render modes are available and set the target FPS for rendering
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}

    def __init__(
        self,
        mjcf_path="balance_bot.xml",
        max_steps=10000,
        render_mode=None,
        alpha=0.99,
        sensor_imu_accel="imu_accel",
        sensor_imu_gyro="imu_gyro",
        sensor_imu_orientation="imu_orientation",
        sensor_left_wheel_vel="left_wheel_vel",
        sensor_right_wheel_vel="right_wheel_vel",
        actuator_left_motor="left_motor",
        actuator_right_motor="right_motor",
        left_wheel_joint="left_wheel_joint",
        right_wheel_joint="right_wheel_joint",
        wheel_col_geom="wheel_left_col",
        chassis_trim_deg=0.0,
        alive_bonus=1.0,
        vel_leak=0.995,
        pos_leak=0.995,
        heading_leak=1.0,
        pitch_penalty_coef=0.5, 
        action_penalty_coef=0.01,
        vel_penalty_coef=0.0,
        vel_penalty_cap=0.25,
        cmd_pos_penalty_coef=0.0,
        heading_penalty_coef=0.0,
        tip_threshold_deg=30.0,
        domain_rand=None,
    ):
        """
        Constructor: Initialize the balance bot environment

        Args:
            mjcf_path (str or Path): Path to the MuJoCo MJCF model file (XML)
            max_steps (int): Max number of simulation steps per episode before resetting
            render_mode (str or None): Visual output, one of [None, "human", "rgb_array"]
            alpha (float): Complementary filter coefficient, higher = more gyro, lower = more accel
            sensor_imu_accel (str): MJCF name of the IMU accelerometer sensor
            sensor_imu_gyro (str): MJCF name of the IMU gyroscope sensor
            sensor_imu_orientation (str): MJCF name of the framequat orientation sensor
            sensor_left_wheel_vel (str): MJCF name of the left wheel velocity sensor
            sensor_right_wheel_vel (str): MJCF name of the right wheel velocity sensor
            actuator_left_motor (str): MJCF name of the left motor actuator
            actuator_right_motor (str): MJCF name of the right motor actuator
            left_wheel_joint (str): MJCF name of the left wheel joint
            right_wheel_joint (str): MJCF name of the right wheel joint
            wheel_col_geom (str): MJCF name of the wheel collision geometry (for looking up radius)
            chassis_trim_deg (float): Equilibrium lean angle in degrees, in the chassis frame. If 
                                    CoM is above the axle, this will be 0. If CoM is behind the 
                                    axle, this should be a positive value to denote a natural lean.
            alive_bonus (float): Reward given each step the robot stays upright.
            vel_leak (float): Leaky-integrator coefficient for velocity estimation. 0.955 is about 
                              1 sec of "memory."
            pos_leak (float): Leaky-integrator coefficient for position estimation. 0.955 is about 
                              1 sec of "memory."
            heading_leak (float): Leaky-integrator coefficient for heading estimation. 1.0 means no
                                   leak (idefinite accumulation).
            pitch_penalty_coef (float): Scales the pitch^2 penalty, encourage staying upright
            action_penalty_coef (float): Scales the action^2 penalty, discourage jittery motion
            vel_penalty_coef (float): Scales the forward/backward estimated velocity to
                                          discourage cruising to stay upright
            vel_penalty_cap (float): Bound the penalty for cruising
            cmd__pos_penalty_coef (float): Scales the penalty for moving off the origin (loose
                                           approximation of "position" using accumulated commands)
            heading_penalty_coef (float): Scales the penalty for spinning about the Z axis (heading
                                          estimated via leaky integrator from gyroscope data)
            tip_threshold_deg (float): Angle (degrees) in which the robot is considered tipped
            domain_rand (DomainRandomConfig): Configuration for performing various domain
                                              randomizations (None to disable)
        """
        # Load model into MuJoCo and get simulation state
        self.model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        self.data = mujoco.MjData(self.model)

        # Get handle to ground geometry (must match name in MJCF file)
        self._ground_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            "ground",
        )
        assert self._ground_id  != -1, "Geom 'ground' not found in MJCF"

        # Get handle to ground geometry (must match name in MJCF file)
        self._chassis_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "chassis",
        )
        assert self._chassis_id != -1, "Body 'chassis' not found in MJCF"

        # Get DOF index to left wheel joint
        left_wheel_joint_id  = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, left_wheel_joint
        )
        assert left_wheel_joint_id !=-1, "Left wheel joint not found in MJCF"
        self._left_wheel_dof_idx = self.model.jnt_dofadr[left_wheel_joint_id]

        # Get DOF index to right wheel joint
        right_wheel_joint_id  = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, right_wheel_joint
        )
        assert right_wheel_joint_id !=-1, "Right wheel joint not found in MJCF"
        self._right_wheel_dof_idx = self.model.jnt_dofadr[right_wheel_joint_id]

        # Set render mode
        self.render_mode = render_mode

        # Store complementary filter coefficient
        self.alpha = alpha

        # Store sensor names
        self.sensor_imu_accel = sensor_imu_accel
        self.sensor_imu_gyro = sensor_imu_gyro
        self.sensor_imu_orientation = sensor_imu_orientation
        self.sensor_left_wheel_vel = sensor_left_wheel_vel
        self.sensor_right_wheel_vel = sensor_right_wheel_vel

        # Get ID of actuators from MJCF names
        self.left_motor_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_ACTUATOR, 
            actuator_left_motor
        )
        self.right_motor_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            actuator_right_motor
        )

        # Save original mass, friction, and motor gain values
        self._chassis_mass_orig = float(self.model.body_mass[self._chassis_id])
        self._ground_friction_orig = float(self.model.geom_friction[self._ground_id, 0])
        self._left_gain_orig  = float(self.model.actuator_gainprm[self.left_motor_id, 0])
        self._right_gain_orig = float(self.model.actuator_gainprm[self.right_motor_id, 0])

        # Define observation space (i.e. what the agent can see) and limits
        # [pitch, pitch rate, fwd vel est, yaw rate, cmd pos est, heading est]
        obs_low  = np.array([-np.pi, -20.0, -5.0, -20.0, -10.0, -np.pi*4], dtype=np.float32)
        obs_high = np.array([ np.pi, 20.0, 5.0, 20.0, 10.0, np.pi*4], dtype=np.float32)
        self.observation_space = spaces.Box(obs_low, obs_high, dtype=np.float32)

        # Define action space (i.e. what the agent can do) and limits, normalized to [-1, 1]
        # [left_wheel_duty_cycle, right_wheel_duty_cycle]
        actions_low = np.array([-1.0, -1.0], dtype=np.float32)
        actions_high = np.array([1.0, 1.0], dtype=np.float32)
        self.action_space = spaces.Box(actions_low, actions_high, dtype=np.float32)

        # Save the chassis equilibrium pitch
        self._eq_chassis_rad = math.radians(chassis_trim_deg)

        # Set the model to the equilibrium pitch
        self.data.qpos[3:7] = [
            math.cos(self._eq_chassis_rad / 2), 0,
            math.sin(self._eq_chassis_rad / 2), 0,
        ]
        mujoco.mj_forward(self.model, self.data)

        # Read the IMU sensor and save the equilibrium pitch (from IMU perspective)
        w, x, y, z = self.data.sensor(self.sensor_imu_orientation).data
        self._pitch_trim_rad = math.atan2(1.0 - 2.0*(x*x + y*y), -2.0*(y*z + w*x))

        # Reset simulationo
        mujoco.mj_resetData(self.model, self.data)

        # Save leaky-integrator coefficients
        self._vel_leak = vel_leak
        self._pos_leak = pos_leak
        self._heading_leak = heading_leak

        # Save reward coefficients
        self.alive_bonus = alive_bonus
        self.pitch_penalty_coef = pitch_penalty_coef
        self.action_penalty_coef = action_penalty_coef
        self.vel_penalty_coef = vel_penalty_coef
        self.cmd_pos_penalty_coef = cmd_pos_penalty_coef
        self.heading_penalty_coef = heading_penalty_coef

        # Save velocity penalty cap
        self.vel_penalty_cap = vel_penalty_cap

        # Save the original gravity magnitude
        self._gravity_mag_orig = float(np.linalg.norm(self.model.opt.gravity))

        # Save tip threshold
        self.tip_threshold_deg = tip_threshold_deg

        # Number of steps to take before resetting the episode
        self.max_steps = max_steps

        # Save domain randomization
        self.dr = domain_rand

        # Action delay buffer
        self._action_delay = 0
        self._action_buffer = []

        # Set initial pitch, velocity, position, and heading estimates
        self._pitch = 0.0
        self._vel_est = 0.0
        self._heading_est = 0.0
        self._cmd_vel = 0.0
        self._cmd_pos = 0.0

        # Set forward displacement tracker
        self._fwd_disp_true = 0.0

        # Initialize the viewer
        self._viewer = None

        # Get wheel radius
        wheel_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, wheel_col_geom)
        self._wheel_radius = float(self.model.geom_size[wheel_geom_id, 0])

        # Internal step counter
        self._step = 0

    def _get_obs(self):
        """
        Read sensor data from MuJoCo and return the observation vector.

        Returns:
            np.ndarray: [pitch, pitch_rate]
        """
        # Read raw IMU sensor data. Note that +Y points down, so we negate the raw yaw rate reading.
        _, accel_y, accel_z = self.data.sensor(self.sensor_imu_accel).data
        pitch_rate = self.data.sensor(self.sensor_imu_gyro).data[0]
        yaw_rate = -1 * self.data.sensor(self.sensor_imu_gyro).data[1]

        # Accelerometer-derived pitch estimate
        accel_pitch = math.atan2(accel_z, -accel_y)

        # Copmlementary filter to estimate pitch from accelerometer and gyroscope
        # The accelerometer will pick up motion acceleration, so we mix it with the
        # gyroscope data to get a smoother "tilt" at any given moment.
        self._pitch = self.alpha * (self._pitch + pitch_rate * self.model.opt.timestep) + \
                    (1 - self.alpha) * accel_pitch

        # Get timestep
        dt = self.model.opt.timestep

        # Forward acceleration estimate: Z axis accel - gravity component (account for tilt)
        a_fwd = accel_z - self._gravity_mag_orig * math.sin(self._pitch)

        # Leaky integrator to estimate velocity
        self._vel_est = self._vel_leak * (self._vel_est + (a_fwd * dt))
 
        # Leaky integrator to estimate heading (from yaw rate IMU measurement)
        self._heading_est = self._heading_leak * (self._heading_est + (yaw_rate * dt))

        # Construct observation vector
        obs = np.array([self._pitch, 
                        pitch_rate, 
                        self._vel_est, 
                        yaw_rate, 
                        self._cmd_pos,  # Computed in step(), as it needs the action values 
                        self._heading_est], dtype=np.float32)

        # Optionally add Gaussian noise to observations
        if self.dr is not None:
            if self.dr.pitch_noise_std_dev > 0.0:
                obs[0] += self.np_random.normal(0.0, self.dr.pitch_noise_std_dev)
            if self.dr.pitch_rate_noise_std_dev > 0.0:
                obs[1] += self.np_random.normal(0.0, self.dr.pitch_rate_noise_std_dev)
            if self.dr.yaw_rate_noise_std_dev > 0.0:
                obs[3] += self.np_random.normal(0.0, self.dr.yaw_rate_noise_std_dev)
 
        return obs

    def reset(self, seed=None, options=None):
        """
        Resets the environment and bot to an initial state.

        Args:
            seed (int or None): seed for the random number generator(s)
            options (dict or None): Unused, required by the Gymnasium API

        Returns:
            observation (np.ndarray): Initial observation
            info (dict): Empty, no extra info returned on reset
        """
        super().reset(seed=seed)

        # Optionally randomize the mass
        if self.dr is not None and self.dr.mass_scale_range != (1.0, 1.0):
            scale = self.np_random.uniform(
                self.dr.mass_scale_range[0],
                self.dr.mass_scale_range[1],
            )
            self.model.body_mass[self._chassis_id] = scale * self._chassis_mass_orig

        # Optionally randomize friction
        if self.dr is not None and self.dr.friction_scale_range != (1.0, 1.0):
            scale = self.np_random.uniform(
                self.dr.friction_scale_range[0],
                self.dr.friction_scale_range[1],
            )
            self.model.geom_friction[self._ground_id, 0] = scale * self._ground_friction_orig

        # Optionally randomize motor gain
        # TODO: add per-motor DR (i.e. model motor asymmetry)
        if self.dr is not None and self.dr.motor_gain_range != (1.0, 1.0):
            scale = self.np_random.uniform(
                self.dr.motor_gain_range[0],
                self.dr.motor_gain_range[1],
            )
            self.model.actuator_gainprm[self.left_motor_id, 0]  = scale * self._left_gain_orig
            self.model.actuator_gainprm[self.right_motor_id, 0] = scale * self._right_gain_orig

        # Tilt gravity to simulate a sloped/uneven floor. Equivalent to tilting the
        # ground, but keeps contact on a clean flat plane.
        if self.dr is not None and self.dr.gravity_tilt_max_deg > 0.0:
            tilt = math.radians(self.np_random.uniform(0.0, self.dr.gravity_tilt_max_deg))
            azimuth = self.np_random.uniform(0.0, 2.0 * math.pi)
            g = self._gravity_mag_orig
            self.model.opt.gravity[:] = [
                g * math.sin(tilt) * math.cos(azimuth),
                g * math.sin(tilt) * math.sin(azimuth),
                -g * math.cos(tilt),
            ]

        # Clear any applied forces
        self.data.xfrc_applied[self._chassis_id, :] = 0.0
        self.data.qfrc_applied[self._left_wheel_dof_idx]  = 0.0
        self.data.qfrc_applied[self._right_wheel_dof_idx] = 0.0

        # Reset the simulator
        mujoco.mj_resetData(self.model, self.data)

        # Start at equilibrium lean
        self.data.qpos[3:7] = [
            math.cos(self._eq_chassis_rad / 2), 0,
            math.sin(self._eq_chassis_rad / 2), 0,
        ]

        # Reset pitch (IMU frame), velocity, position, and heading estimate
        self._pitch = self._pitch_trim_rad
        self._vel_est = 0.0
        self._heading_est = 0.0
        self._cmd_vel = 0.0
        self._cmd_pos = 0.0

        # Set forward displacement tracker
        self._fwd_disp_true = 0.0

        # Impart an initial angular velocity around the y axis so the agent learns to recover
        # Note: qvel[4] = wy (rad/s)
        self.data.qvel[4] += self.np_random.uniform(-0.5, 0.5)

        # Update the state of the robot without taking a full time step
        mujoco.mj_forward(self.model, self.data)

        # Reset the step counter
        self._step = 0

        # Reset the action delay
        if self.dr is not None:
            # Randomize action delay for this episode
            if self.dr.action_delay_steps > 0 and self.dr.action_delay_random:
                self._action_delay = self.np_random.integers(
                    0, self.dr.action_delay_steps + 1
                )
            else:
                self._action_delay = self.dr.action_delay_steps

            # Fill action buffer with zeros
            self._action_buffer = []
            for _ in range(self._action_delay):
                self._action_buffer.append(np.zeros(self.action_space.shape))

        return self._get_obs(), {}

    def step(self, action):
        """
        Advance the simulation by one step and return the result.

        Args:
            action (np.ndarray): [left_wheel_duty cycle, right_wheel_duty_cycle], 
                                 normalized to [-1, 1]

        Returns:
            obs (np.ndarray): Observation from _get_obs()
            reward (float): Reward signal for this step
            terminated (bool): True if the robot has tipped/fallen over
            truncated (bool): True if the episode has reached max_steps
            info (dict): Empty, no extra info returned
        """
        # Apply action delay: add current action to buffer and pop off previously added action
        if self.dr is not None and self._action_delay > 0:
            self._action_buffer.append(action.copy())
            action = self._action_buffer.pop(0)

        # Add random perturbation (noise) to motor commands
        if self.dr is not None and self.dr.motor_noise_scale > 0.0:
            noise = self.np_random.uniform(
                -self.dr.motor_noise_scale,
                self.dr.motor_noise_scale,
                size=action.shape
            )
            action = np.clip(action + noise, -1.0, 1.0)

        # Add motor deadband
        if self.dr is not None and self.dr.motor_deadband > 0.0:
            action = np.where(np.abs(action) < self.dr.motor_deadband, 0.0, action)

        # Set motors to given (normalized) duty cycle
        self.data.ctrl[self.left_motor_id]  = action[0]
        self.data.ctrl[self.right_motor_id] = action[1]

        # Clear any applied forces before randomly adding them (if enabled)
        self.data.xfrc_applied[self._chassis_id, :] = 0.0
        self.data.qfrc_applied[self._left_wheel_dof_idx]  = 0.0
        self.data.qfrc_applied[self._right_wheel_dof_idx] = 0.0

        # Apply random external force (push) to chassis in X and Y directions
        if self.dr is not None and self.dr.push_prob > 0.0:
            if self.np_random.random() < self.dr.push_prob:
                # Get random force in X and Y directions
                push_x = self.np_random.uniform(
                    -self.dr.push_force_max_n,
                    self.dr.push_force_max_n,
                )
                push_y = self.np_random.uniform(
                    -self.dr.push_force_max_n,
                    self.dr.push_force_max_n
                )

                # Apply push to chassis
                self.data.xfrc_applied[self._chassis_id, 0] = push_x
                self.data.xfrc_applied[self._chassis_id, 1] = push_y

        # We intend to simulate tire ridges here, but it's essentially just random axle torques,
        # which includes (but not limited to) bumps and tire ridges.
        if self.dr is not None and self.dr.ridge_prob > 0.0:
            if self.np_random.random() < self.dr.ridge_prob:
                # Get random torques between -max and +max
                ridge_torque_left  = self.np_random.uniform(
                    -self.dr.ridge_torque_max_nm,
                    self.dr.ridge_torque_max_nm
                )
                ridge_torque_right = self.np_random.uniform(
                    -self.dr.ridge_torque_max_nm,
                    self.dr.ridge_torque_max_nm
                )

                # Apply torques
                self.data.qfrc_applied[self._left_wheel_dof_idx]  = ridge_torque_left
                self.data.qfrc_applied[self._right_wheel_dof_idx] = ridge_torque_right

        # Advance simulation by one step
        mujoco.mj_step(self.model, self.data)
        self._step += 1

        # Command integration: estimate something proportional to position based on actions
        # Note: do this before _get_obs(), as the pos estimate is used in the obs vector
        dt = self.model.opt.timestep
        cmd = 0.5 * (action[0] + action[1])
        self._cmd_vel = self._pos_leak * (self._cmd_vel + cmd * dt)
        self._cmd_pos = self._pos_leak * (self._cmd_pos + self._cmd_vel * dt)

        # Get actual forward velocity (privileged info) by projecting world-frame velocity onto the
        # chassis body frame X axis.
        xmat = self.data.xmat[self._chassis_id].reshape(3, 3)
        world_vel = self.data.cvel[self._chassis_id][3:6]
        vel_actual = float(np.dot(world_vel, xmat[:, 0]))

        # Get observation
        obs = self._get_obs()

        # Integrate the true forward velocity to get the forward displacement
        self._fwd_disp_true = self._fwd_disp_true + (vel_actual * dt)

        # Ground-truth pitch from the framequat sensor (privileged info).
        w, x, y, z = self.data.sensor(self.sensor_imu_orientation).data
        up_y = 2.0 * (y * z + w * x)
        up_z = 1.0 - 2.0 * (x * x + y * y)
        pitch_true = math.atan2(up_z, -up_y)

        # Figure out how far off the natural lean (pitch trim) we are
        pitch_error = pitch_true - self._pitch_trim_rad

        # Reward function: alive - action - vel - pos - heading
        #   alive: reward for staying upright each step
        #   pitch: penalty for leaning (use privileged info, not observed pitch)
        #   action: penalty for jittery motor commands
        #   vel: penalty for moving forward or backward (capped to a limit)
        #   pos: penalty for moving off origin (estimated via command integrationo)
        #   heading: penalty for turning (estimated via leaky integrator)
        pitch_penalty = self.pitch_penalty_coef * pitch_error**2
        action_penalty = self.action_penalty_coef * np.sum(action**2)
        vel_penalty = self.vel_penalty_coef * min(self._vel_est**2, self.vel_penalty_cap)
        pos_penalty = self.cmd_pos_penalty_coef * self._cmd_pos**2
        heading_penalty = self.heading_penalty_coef * self._heading_est**2
        reward = self.alive_bonus - pitch_penalty - action_penalty - vel_penalty - \
            pos_penalty - heading_penalty

        # Termination (if robot tips or we run out of time in the episode)
        terminated = abs(pitch_error) > math.radians(self.tip_threshold_deg)
        truncated = self._step >= self.max_steps

        # Privileged ground truth: velocity as calculated from wheel motion
        wheel_vel_l = self.data.sensor(self.sensor_left_wheel_vel).data[0]
        wheel_vel_r = self.data.sensor(self.sensor_right_wheel_vel).data[0]
        true_vel = 0.5 * (wheel_vel_l + wheel_vel_r) * self._wheel_radius

        # Privileged ground truth: true heading (yaw) from orientation quaternion
        qw, qx, qy, qz = self.data.qpos[3:7]
        true_heading = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))

        # Construct info for logging
        info = {
            "estimator/cmd_pos_est": float(self._cmd_pos),
            "estimator/cmd_pos_true": float(self._fwd_disp_true),
            "estimator/vel_est": float(self._vel_est),
            "estimator/vel_true": float(true_vel),
            "estimator/heading_est": float(self._heading_est),
            "estimator/heading_true": float(true_heading),
        }

        return obs, reward, terminated, truncated, info

    def render(self):
        """
        Render the current simulation state to the MuJoCo viewer window.
        """
        if self.render_mode != "human":
            return

        # Create the viewer on the first render call
        if self._viewer is None:
            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)

            # Set up the camera
            self._viewer.cam.type     = mujoco.mjtCamera.mjCAMERA_FREE
            self._viewer.cam.lookat[:] = [0, 0, 0.05]
            self._viewer.cam.distance  = 0.8
            self._viewer.cam.azimuth   = 45
            self._viewer.cam.elevation = -25

        # Push the current simulation state to the viewer
        self._viewer.sync()

    def close(self):
        """
        Clean up the viewer and simulation resources.
        Automatically called by Gymnasium when the environment is done.
        """
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None