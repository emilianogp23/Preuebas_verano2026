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

# ...........       ____  _ __
# |  ,-^-,  |      / __ )(_) /_______________ _____  ___
# | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
# | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#    +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
# @file crazyflie_controller.py
# Description: Controls the crazyflie in webots
# Author:      Kimberly McGuire (Bitcraze AB)

import math
from controller import Robot, Camera, DistanceSensor, GPS, Gyro, InertialUnit, Keyboard, Motor
import scipy.interpolate as spi

# Add external controller
from pid_controller import (
    ActualState,
    DesiredState,
    GainsPID,
    MotorPower,
    init_pid_attitude_fixed_height_controller,
    pid_velocity_fixed_height_controller,
)

FLYING_ALTITUDE = 1.0


def main():
    robot = Robot()

    timestep = int(robot.getBasicTimeStep())

    # Initialize motors
    m1_motor = robot.getDevice("m1_motor")
    m1_motor.setPosition(float("inf"))
    m1_motor.setVelocity(-1.0)

    m2_motor = robot.getDevice("m2_motor")
    m2_motor.setPosition(float("inf"))
    m2_motor.setVelocity(1.0)

    m3_motor = robot.getDevice("m3_motor")
    m3_motor.setPosition(float("inf"))
    m3_motor.setVelocity(-1.0)

    m4_motor = robot.getDevice("m4_motor")
    m4_motor.setPosition(float("inf"))
    m4_motor.setVelocity(1.0)

    # Initialize sensors
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)

    gps = robot.getDevice("gps")
    gps.enable(timestep)

    keyboard = robot.getKeyboard()
    keyboard.enable(timestep)

    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)

    camera = robot.getDevice("camera")
    camera.enable(timestep)

    range_front = robot.getDevice("range_front")
    range_front.enable(timestep)

    range_left = robot.getDevice("range_left")
    range_left.enable(timestep)

    range_back = robot.getDevice("range_back")
    range_back.enable(timestep)

    range_right = robot.getDevice("range_right")
    range_right.enable(timestep)

    # Wait for 2 seconds
    while robot.step(timestep) != -1:
        if robot.getTime() > 2.0:
            break

    # Initialize variables
    actual_state = ActualState()
    desired_state = DesiredState()
    past_x_global = 0.0
    past_y_global = 0.0
    past_time = robot.getTime()

    # Initialize PID gains
    gains_pid = GainsPID()
    gains_pid.kp_att_y = 1.0
    gains_pid.kd_att_y = 0.5
    gains_pid.kp_att_rp = 0.5
    gains_pid.kd_att_rp = 0.1
    gains_pid.kp_vel_xy = 2.0
    gains_pid.kd_vel_xy = 0.5
    gains_pid.kp_z = 10.0
    gains_pid.ki_z = 5.0
    gains_pid.kd_z = 5.0
    init_pid_attitude_fixed_height_controller()

    height_desired = FLYING_ALTITUDE

    # Initialize struct for motor power
    motor_power = MotorPower()

    print()
    print("====== Controls =======")
    print(" The Crazyflie can be controlled from your keyboard!")
    print(" All controllable movement is in body coordinates")
    print("- Use the up, back, right and left button to move in the horizontal plane")
    print("- Use Q and E to rotate around yaw")
    print("- Use W and S to go up and down")

    Kp = 5
    xvec = [0, 1, -1]
    yvec = [0, 1, 1]
    thvec = [math.pi/4, math.pi/2, math.pi]
    tvec = [0, 5, 10, 15]

    stable = False

    tini = robot.getTime()

    while robot.step(timestep) != -1:
        dt = robot.getTime() - past_time

        # Get measurements
        rpy = imu.getRollPitchYaw()
        actual_state.roll = rpy[0]
        actual_state.pitch = rpy[1]
        actual_state.yaw_rate = gyro.getValues()[2]
        actual_state.altitude = gps.getValues()[2]

        x_global = gps.getValues()[0]
        vx_global = (x_global - past_x_global) / dt
        y_global = gps.getValues()[1]
        vy_global = (y_global - past_y_global) / dt

        # Get body fixed velocities
        actual_yaw = imu.getRollPitchYaw()[2]
        cos_yaw = math.cos(actual_yaw)
        sin_yaw = math.sin(actual_yaw)
        actual_state.vx = vx_global * cos_yaw + vy_global * sin_yaw
        actual_state.vy = -vx_global * sin_yaw + vy_global * cos_yaw

        # Initialize desired state values
        desired_state.roll = 0.0
        desired_state.pitch = 0.0
        desired_state.vx = 0.0
        desired_state.vy = 0.0
        desired_state.yaw_rate = 0.0
        desired_state.altitude = 1.0

        forward_desired = 0.0
        sideways_desired = 0.0
        yaw_desired = 0.0
        height_diff_desired = 0.0

        # Read keyboard input
        #key = keyboard.getKey()
        #while key > 0:
        #    if key == Keyboard.UP:
        #        forward_desired = +0.5
        #    elif key == Keyboard.DOWN:
        #        forward_desired = -0.5
        #    elif key == Keyboard.RIGHT:
        #        sideways_desired = -0.5
        #    elif key == Keyboard.LEFT:
        #        sideways_desired = +0.5
        #    elif key == ord("Q"):
        #        yaw_desired = 1.0
        #    elif key == ord("E"):
        #        yaw_desired = -1.0
        #    elif key == ord("W"):
        #        height_diff_desired = 0.1
        #    elif key == ord("S"):
        #        height_diff_desired = -0.1

        #    key = keyboard.getKey()
        tcurr = robot.getTime()-tini

        ## Movimientos a velocidad constante
        #theta = 2*math.pi*tcurr/10.0
        #v = 0.5
        #forward_desired = v*math.cos(theta)
        #sideways_desired = v*math.sin(theta)
        #if tcurr > 5:
        #    height_diff_desired = 0.15*math.sin(2*math.pi*3*tcurr)
        #yaw_desired = 0.0

        ## Controlador tipo robot DDR
        # Kv = 0.5
        # Kh = 1.0
        # xd = 0
        # yd = 0
        # errp = math.sqrt((x_global-xd)**2 + (y_global-yd)**2)
        # ct = math.cos(rpy[2])
        # st = math.sin(rpy[2])
        # errh = math.atan2(ct*(yd-y_global)-st*(xd-x_global), st*(yd-y_global)+ct*(xd-x_global))
        # forward_desired = Kv*errp
        # yaw_desired = Kh*errh
        
        # Controlador para la orientación o yaw
        Kh = 5.0
        th = rpy[2]

        if robot.getTime() > 5:
            if stable == False:
                #inicializa interp
                xvec = [x_global] + xvec
                yvec = [y_global] + yvec
                thvec = [th] + thvec
                xc = spi.splrep(tvec, xvec)
                yc = spi.splrep(tvec, yvec)
                thc = spi.splrep(tvec, thvec)
                tini = robot.getTime()
                stable = True
            else:
                tcurr = robot.getTime()-tini
                xd = spi.splev(tcurr, xc)
                yd = spi.splev(tcurr, yc)
                vx = -Kp*(x_global-xd)
                vy = -Kp*(y_global-yd)
                forward_desired = math.cos(th)*vx+math.sin(th)*vy
                sideways_desired = -math.sin(th)*vx+math.cos(th)*vy
                height_desired = 1.0
                thd = spi.splev(tcurr, thc)
                dang = math.fmod(th-thd, 2.0*math.pi)
                yaw_desired = -Kh*dang
                if tcurr > tvec[-1]:
                    forward_desired = 0
                    sideways_desired = 0
                    yaw_desired = 0
        else:
            height_desired += height_diff_desired * dt

        # Example of how to get sensor data:
        # range_front_value = range_front.getValue()
        # image = camera.getImage()

        desired_state.yaw_rate = yaw_desired

        # PID velocity controller with fixed height
        desired_state.vy = sideways_desired
        desired_state.vx = forward_desired
        desired_state.altitude = height_desired
        pid_velocity_fixed_height_controller(actual_state, desired_state, gains_pid, dt, motor_power)

        # Set motor speeds
        m1_motor.setVelocity(-motor_power.m1)
        m2_motor.setVelocity(motor_power.m2)
        m3_motor.setVelocity(-motor_power.m3)
        m4_motor.setVelocity(motor_power.m4)

        # Save past values for next time step
        past_time = robot.getTime()
        past_x_global = x_global
        past_y_global = y_global


if __name__ == "__main__":
    main()
