#!/usr/bin/env python
import math

class Motion_Planning:
    def __init__(self,speedXY=0.2,speedYaw=0.2, errorMeter=0.2, errorAngle=math.pi/6.0):
        self.target = [2.0, 0.0, 0.0]
        self.position = [0.0, 0.0, 0.0]
        self.speed_XY = speedXY
        self.speed_Yaw = speedYaw
        self.error_meter = errorMeter
        self.error_angle = errorAngle
        print "Motion_Planning start"

    def setValues(self,speedXY=0.2,speedYaw=0.2, errorMeter=0.2, errorAngle=math.pi/6.0):
        self.speed_XY = speedXY
        self.speed_Yaw = speedYaw
        self.error_meter = errorMeter
        self.error_angle = errorAngle

    def setNextTarget(self, next_target):
        self.target = next_target

    def setNowPosition(self, now_position):
        self.position = now_position

    def reach(self):
        if abs(self.target[0] - self.position[0]) <= self.error_meter and\
           abs(self.target[1] - self.position[1]) <= self.error_meter and\
           (abs(self.target[2] - self.position[2]) <=  self.error_angle or\
            abs(self.target[2] - self.position[2]-2*math.pi) <=  self.error_angle or\
            abs(self.target[2] - self.position[2]+2*math.pi) <=  self.error_angle):
            return True
        else:
            return False
    def reach_position(self):
        #if abs(self.target[0] - self.position[0]) <= self.error_meter and\
           #abs(self.target[1] - self.position[1]) <= self.error_meter :
        if abs(self.target[0] - self.position[0]) <= self.error_meter:
            return True
        else:
            return False

    def reach_angle(self):
        if (abs(self.target[2] - self.position[2]) <=  self.error_angle or\
            abs(self.target[2] - self.position[2]-2*math.pi) <=  self.error_angle or\
            abs(self.target[2] - self.position[2]+2*math.pi) <=  self.error_angle):
            return True
        else:
            return False
    def MotionPlan(self):
        flag=[0.0, 0.0, 0.0]
        if self.target and self.position :
            #use bang bang
            if abs(self.target[0] - self.position[0]) > self.error_meter:
                if self.target[0] - self.position[0] > 0:
                    #print "x speed = +1 m/s"
                    flag[0] = self.speed_XY
                else:
                    #print "x speed = -1 m/s"
                    flag[0] = -self.speed_XY
            else:
                #print "x speed = 0"
                flag[0] = 0.0
                
            if abs(self.target[1] - self.position[1]) > self.error_meter:
                if self.target[1] - self.position[1] > 0:
                    #print "y speed = +1 m/s"
                    if flag[0]>0:
                        flag[1] = self.speed_Yaw
                    else:
                        flag[1] = -self.speed_Yaw
                else:
                    #print "y speed = -1 m/s"
                    if flag[0]>0:
                        flag[1] = -self.speed_Yaw
                    else:
                        flag[1] = self.speed_Yaw
            else:
                #print "y speed = 0"
                flag[1] = 0.0

            tempYaw = self.target[2] - self.position[2]
            '''
            if tempYaw > math.pi:
                tempYaw = math.pi*2 - tempYaw
            if tempYaw < -math.pi:
                tempYaw = math.pi*2 + tempYaw
            '''
            if tempYaw > math.pi:
                tempYaw = -math.pi * 2 + tempYaw
            if tempYaw < -math.pi:
                tempYaw = math.pi * 2 + tempYaw
            if abs(tempYaw) > self.error_angle:
                if tempYaw > 0:
                    #print "yaw speed = +(math.pi/18) rad/s"
                    flag[2] = self.speed_Yaw
                else:
                    #print "yaw speed = -(math.pi/18) rad/s"
                    flag[2] = -self.speed_Yaw
            else:
                #print "yaw speed = 0"
                flag[2] = 0.0
            return flag

if __name__ == "__main__":
    body = Motion_Planning()
    print body.MotionPlan()

    print body.reach()
    

