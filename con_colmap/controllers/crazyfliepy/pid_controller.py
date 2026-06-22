# Copyright 2022 Bitcraze AB
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#
# @file pid_controller.py
# Description: A simple PID controller for attitude,
#              height and velocity control of a quadcopter
# Author:      Kimberly McGuire (Bitcraze AB)


# ---------------------------------------------------------------------------
# Data classes  (translated from structs in pid_controller.h)
# ---------------------------------------------------------------------------

class MotorPower:
    """motor_power_t"""
    def __init__(self):
        self.m1: float = 0.0
        self.m2: float = 0.0
        self.m3: float = 0.0
        self.m4: float = 0.0


class ControlCommands:
    """control_commands_t"""
    def __init__(self):
        self.roll: float = 0.0
        self.pitch: float = 0.0
        self.yaw: float = 0.0
        self.altitude: float = 0.0


class DesiredState:
    """desired_state_t"""
    def __init__(self):
        self.roll: float = 0.0
        self.pitch: float = 0.0
        self.yaw_rate: float = 0.0
        self.altitude: float = 0.0
        self.vx: float = 0.0
        self.vy: float = 0.0


class ActualState:
    """actual_state_t"""
    def __init__(self):
        self.roll: float = 0.0
        self.pitch: float = 0.0
        self.yaw_rate: float = 0.0
        self.altitude: float = 0.0
        self.vx: float = 0.0
        self.vy: float = 0.0


class GainsPID:
    """gains_pid_t"""
    def __init__(self):
        self.kp_att_rp: float = 0.0
        self.kd_att_rp: float = 0.0
        self.kp_att_y: float = 0.0
        self.kd_att_y: float = 0.0
        self.kp_vel_xy: float = 0.0
        self.kd_vel_xy: float = 0.0
        self.kp_z: float = 0.0
        self.kd_z: float = 0.0
        self.ki_z: float = 0.0


# ---------------------------------------------------------------------------
# Module-level state  (translated from file-scope globals in pid_controller.c)
# ---------------------------------------------------------------------------

past_altitude_error: float = 0.0
past_pitch_error: float = 0.0
past_roll_error: float = 0.0
past_yaw_rate_error: float = 0.0
past_vx_error: float = 0.0
past_vy_error: float = 0.0
altitude_integrator: float = 0.0


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def constrain(value: float, min_val: float, max_val: float) -> float:
    """Equivalent to C fminf(maxVal, fmaxf(minVal, value))."""
    return min(max_val, max(min_val, value))


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def init_pid_attitude_fixed_height_controller() -> None:
    """Reset all integrators and past-error terms to zero."""
    global past_altitude_error, past_pitch_error, past_roll_error
    global past_yaw_rate_error, past_vx_error, past_vy_error
    global altitude_integrator

    past_altitude_error = 0.0
    past_yaw_rate_error = 0.0
    past_pitch_error = 0.0
    past_roll_error = 0.0
    past_vx_error = 0.0
    past_vy_error = 0.0
    altitude_integrator = 0.0


def pid_attitude_fixed_height_controller(
    actual_state: ActualState,
    desired_state: DesiredState,
    gains_pid: GainsPID,
    dt: float,
    motor_commands: MotorPower,
) -> None:
    control_commands = ControlCommands()
    pid_fixed_height_controller(actual_state, desired_state, gains_pid, dt, control_commands)
    pid_attitude_controller(actual_state, desired_state, gains_pid, dt, control_commands)
    motor_mixing(control_commands, motor_commands)


def pid_velocity_fixed_height_controller(
    actual_state: ActualState,
    desired_state: DesiredState,
    gains_pid: GainsPID,
    dt: float,
    motor_commands: MotorPower,
) -> None:
    control_commands = ControlCommands()
    pid_horizontal_velocity_controller(actual_state, desired_state, gains_pid, dt)
    pid_fixed_height_controller(actual_state, desired_state, gains_pid, dt, control_commands)
    pid_attitude_controller(actual_state, desired_state, gains_pid, dt, control_commands)
    motor_mixing(control_commands, motor_commands)


# ---------------------------------------------------------------------------
# Internal controllers
# ---------------------------------------------------------------------------

def pid_fixed_height_controller(
    actual_state: ActualState,
    desired_state: DesiredState,
    gains_pid: GainsPID,
    dt: float,
    control_commands: ControlCommands,
) -> None:
    global past_altitude_error, altitude_integrator

    altitude_error = desired_state.altitude - actual_state.altitude
    altitude_derivative_error = (altitude_error - past_altitude_error) / dt

    altitude_integrator += altitude_error * dt

    control_commands.altitude = (
        gains_pid.kp_z * constrain(altitude_error, -1, 1)
        + gains_pid.kd_z * altitude_derivative_error
        + gains_pid.ki_z * altitude_integrator
        + 48
    )

    past_altitude_error = altitude_error


def motor_mixing(
    control_commands: ControlCommands,
    motor_commands: MotorPower,
) -> None:
    motor_commands.m1 = control_commands.altitude - control_commands.roll + control_commands.pitch + control_commands.yaw
    motor_commands.m2 = control_commands.altitude - control_commands.roll - control_commands.pitch - control_commands.yaw
    motor_commands.m3 = control_commands.altitude + control_commands.roll - control_commands.pitch + control_commands.yaw
    motor_commands.m4 = control_commands.altitude + control_commands.roll + control_commands.pitch - control_commands.yaw


def pid_attitude_controller(
    actual_state: ActualState,
    desired_state: DesiredState,
    gains_pid: GainsPID,
    dt: float,
    control_commands: ControlCommands,
) -> None:
    global past_pitch_error, past_roll_error, past_yaw_rate_error

    # Calculate errors
    pitch_error = desired_state.pitch - actual_state.pitch
    pitch_derivative_error = (pitch_error - past_pitch_error) / dt

    roll_error = desired_state.roll - actual_state.roll
    roll_derivative_error = (roll_error - past_roll_error) / dt

    yaw_rate_error = desired_state.yaw_rate - actual_state.yaw_rate

    # PID control
    control_commands.roll = (
        gains_pid.kp_att_rp * constrain(roll_error, -1, 1)
        + gains_pid.kd_att_rp * roll_derivative_error
    )
    control_commands.pitch = (
        -gains_pid.kp_att_rp * constrain(pitch_error, -1, 1)
        - gains_pid.kd_att_rp * pitch_derivative_error
    )
    control_commands.yaw = gains_pid.kp_att_y * constrain(yaw_rate_error, -1, 1)

    # Save errors for the next round
    past_pitch_error = pitch_error
    past_roll_error = roll_error
    past_yaw_rate_error = yaw_rate_error


def pid_horizontal_velocity_controller(
    actual_state: ActualState,
    desired_state: DesiredState,
    gains_pid: GainsPID,
    dt: float,
) -> None:
    global past_vx_error, past_vy_error

    vx_error = desired_state.vx - actual_state.vx
    vx_derivative = (vx_error - past_vx_error) / dt

    vy_error = desired_state.vy - actual_state.vy
    vy_derivative = (vy_error - past_vy_error) / dt

    # PID control
    pitch_command = (
        gains_pid.kp_vel_xy * constrain(vx_error, -1, 1)
        + gains_pid.kd_vel_xy * vx_derivative
    )
    roll_command = (
        -gains_pid.kp_vel_xy * constrain(vy_error, -1, 1)
        - gains_pid.kd_vel_xy * vy_derivative
    )

    desired_state.pitch = pitch_command
    desired_state.roll = roll_command

    # Save errors for the next round
    past_vx_error = vx_error
    past_vy_error = vy_error
