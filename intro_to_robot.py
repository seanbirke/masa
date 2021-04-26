#!/usr/bin/env python3

import brickpi3
# import odometry as odom
import math
# import numpy as np

class Robot:
    def __init__(self, wheelbase=4.25, radius=1.125):
        self.BP = brickpi3.BrickPi3()
        self.BP.reset_all()
        self.wheelbase = wheelbase # in inches
        self.radius = radius # in inches too
        # add to the dictionary below once we figure out the other sensor names
        self.sensorDict = {'light': self.BP.SENSOR_TYPE.NXT_LIGHT_ON, 'ultrasonic': self.BP.SENSOR_TYPE.NXT_ULTRASONIC}
 #        self.odom = odom.Odom(self.wheelbase, self.radius)
        # currently assumes that only ports B and C are used for motors
        # also assumes that the sensor is in S1
        self.portA = self.BP.PORT_A
        self.portB = self.BP.PORT_B
        self.portC = self.BP.PORT_C
        self.portD = self.BP.PORT_D
        # init sensors too
        self.sensorList = [self.BP.PORT_1, self.BP.PORT_2, self.BP.PORT_3, self.BP.PORT_4]
        # set the left and right motors
        self.motorLeft = self.portB
        self.motorRight = self.portC
        self.cons = 1.0
        # cap on the motor power
        self.motorCap = 100
        self.BP.reset_motor_encoder(self.motorLeft)
        self.BP.reset_motor_encoder(self.motorRight)
        self.x = 0
        self.y = 0
        self.theta = 0

        self.rotL = 0
        self.rotR = 0

    def get_robot_battery(self):
        return self.BP.get_voltage_battery()

    def set_sensor(self, portNumber, sType):
        sensorType = self.sensorDict[sType]
        self.BP.set_sensor_type(portNumber, sensorType)

    def get_sensor(self, portNumber):
        port = self.sensorList[portNumber-1]
        return self.BP.get_sensor(port)

    def drive_robot_power(self, powerLeft, powerRight):
        if (powerLeft > self.motorCap):
            powerLeft = self.motorCap
        elif (powerLeft < -self.motorCap):
            powerLeft = -self.motorCap
        if (powerRight > self.motorCap):
            powerRight = self.motorCap
        elif (powerRight < -self.motorCap):
            powerRight = -self.motorCap

        self.BP.set_motor_power(self.motorLeft, powerLeft)
        self.BP.set_motor_power(self.motorRight, powerRight)

    def calculate_vl_vr(self, dRot1, dRot2, dt):
        vl = dRot1 * self.radius / dt
        vr = dRot2 * self.radius / dt
        return [vl, vr]

    def calculate_V(self, dRot1, dRot2, dt):
        # vs = self.calculate_vl_vr(dRot1, dRot2, dt)
        vs = [dRot1, dRot2]
        return (vs[0] + vs[1]) / 2

    def calculate_w(self, dRot1, dRot2, dt):
        # vs = self.calculate_vl_vr(dRot1, dRot2, dt)
        vs = [dRot1, dRot2]
        return (vs[1] - vs[0]) / self.wheelbase

    def rots_to_rad(self, rotL, rotR):
        return [rotL * math.pi / 180, rotR * math.pi / 180]

    def get_encoder_readings(self):
        rot1 = self.BP.get_motor_encoder(self.motorLeft)
        rot2 = self.BP.get_motor_encoder(self.motorRight)
        return self.rots_to_rad(rot1, rot2)

    def get_wheel_displacement(self):
        rads = self.rots_to_rad(self.rotL, self.rotR)
        return [rads[0] * self.radius, rads[1] * self.radius]

    def update_odometry(self, dt):
        avg_t = dt / 6

        rot1 = self.BP.get_motor_encoder(self.motorLeft)
        rot2 = self.BP.get_motor_encoder(self.motorRight)

        rads = self.rots_to_rad(rot1, rot2)
     
        dRot1 = rads[0] - self.rotL
        dRot2 = rads[1] - self.rotR

        vlvr = self.calculate_vl_vr(dRot1, dRot2, dt)

        V = self.calculate_V(vlvr[0], vlvr[1], dt)
        w = self.calculate_w(vlvr[0], vlvr[1], dt)
        # print('rot deltas', dRot1, dRot2)
        # print('Vl, vr', vlvr)
        # print('Robot Omega:', w)
        # print(V, w)
        # print(w)
        # based on the runge-katta slides on the lab
        x0 = V * math.cos(self.theta)
        x1 = V * math.cos(self.theta + dt * w/2)
        x2 = V * math.cos(self.theta + dt * w/2)
        x3 = V * math.cos(self.theta + dt * w)

        y0 = V * math.sin(self.theta)
        y1 = V * math.sin(self.theta + dt * w/2)
        y2 = V * math.sin(self.theta + dt * w/2)
        y3 = V * math.sin(self.theta + dt * w)

        self.x = self.x + avg_t * (x0 + 2 * (x1 + x2) + x3)
        self.y = self.y + avg_t * (y0 + 2 * (y1 + y2) + y3)
        self.theta = self.theta + w * dt * self.cons

        self.rotL = rads[0]
        self.rotR = rads[1]
        
    def get_odometry(self):
        return [self.x, self.y, self.theta]

    def stop(self):
        self.drive_robot_power(0, 0)

    def print_status(self):
        print("Battery:" + str(self.get_robot_battery()))

    def ik(self, V, w):
        vr = V + self.wheelbase/2 * w
        vl = V - self.wheelbase/2 * w
        return [vl, vr]

    def pid_Vw(self, targetV, targetw, Kpl, Kpr, dt):
        rot1 = self.BP.get_motor_encoder(self.motorLeft)
        rot2 = self.BP.get_motor_encoder(self.motorRight)

        rads = self.rots_to_rad(rot1, rot2)

        dRot1 = rads[0] - self.rotL
        dRot2 = rads[1] - self.rotR

        vlvr = self.calculate_vl_vr(dRot1, dRot2, dt)

        V = self.calculate_V(vlvr[0], vlvr[1], dt)
        w = self.calculate_w(vlvr[0], vlvr[1], dt)
        # print(V)
        vError = targetV - V
        wError = targetw - w
        # print('vError', vError)

        errVlVr = self.ik(vError, wError)

        correctionPowL = Kpl * errVlVr[0]
        correctionPowR = Kpr * errVlVr[1]


        # finds the errors in degrees per second

        return [correctionPowL, correctionPowR]

    










 


        




