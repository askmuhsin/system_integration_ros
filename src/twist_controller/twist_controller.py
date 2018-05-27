#!/usr/bin/env python

from lowpass import LowPassFilter
import yaw_controller
from pid import PID
import numpy as np

GAS_DENSITY = 2.858

class Controller(object):
    # def __init__(self, *args, **kwargs):
    def __init__(self, vehicle_mass, fuel_capacity, acceleration_limit,
                deceleration_limit, wheel_base, wheel_radius, steer_ratio,
                 max_lat_acceleration, max_steer_angle, min_speed):
        kp, ki, kd = 2.0, 0.005, 0            # pid params
        mn, mx = deceleration_limit, acceleration_limit # acceleration limit
        tau = 0.5   # 1/(2*pi*tau) = cutoff frequency
        ts = .02    # sample time

        self.vel_lpf = LowPassFilter(tau, ts)
        self.velocity_controller = PID(kp, ki, kd, mn, mx)

        self.yaw_controller = yaw_controller.YawController(wheel_base=wheel_base,
                                                           steer_ratio=steer_ratio,
                                                           min_speed=min_speed,
                                                           max_lat_accel=max_lat_acceleration,
                                                           max_steer_angle=max_steer_angle)

        self.total_mass = vehicle_mass + (fuel_capacity * GAS_DENSITY)
        self.wheel_radius = wheel_radius
        self.deceleration_limit = deceleration_limit

    def control(self, twist_cmd, current_velocity, time_span):
        # vel_filt_linear_x = self.vel_lpf.filt(current_velocity.twist.linear.x)
        # velocity_error = twist_cmd.twist.linear.x - vel_filt_linear_x
        # acceleration = self.velocity_controller.step(velocity_error, time_span)
        # steer = self.yaw_controller.get_steering(twist_cmd.twist.linear.x,
        #                                          twist_cmd.twist.angular.z,
        #                                          vel_filt_linear_x)

        velocity_error = twist_cmd.twist.linear.x - current_velocity.twist.linear.x
        acceleration = self.velocity_controller.step(velocity_error, time_span)
        steer = self.yaw_controller.get_steering(twist_cmd.twist.linear.x,
                                                 twist_cmd.twist.angular.z,
                                                 current_velocity.twist.linear.x)

        torque = acceleration * self.total_mass * self.wheel_radius
        if np.isclose(twist_cmd.twist.linear.x, 0.) and current_velocity.twist.linear.x < 0.1:
            return 0., torque, steer
        else:
            if acceleration > 0:
                return acceleration, 0., steer
            else:
                return 0., -torque, steer

    def reset(self):
        self.velocity_controller.reset()
