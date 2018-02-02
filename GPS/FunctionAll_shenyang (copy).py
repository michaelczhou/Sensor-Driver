#!/usr/bin/env python
# license removed for brevity
import rospy
import Motion_Planning
import Map_Point
import TF
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from tcp2ros.msg import rtkGPSmessage



class functionAll:
     def __init__(self):
         rospy.init_node('FunctionAll', anonymous=False)
         rospy.Subscriber("joy", Joy, self.callback1)
         rospy.Subscriber("rtkGPS", rtkGPSmessage, self.callback2)
         self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
         self.rate = rospy.Rate(30)
         self.cmd = Twist()
         self.cmd.linear.x = 0.0
         self.cmd.linear.y = 0.0
         self.cmd.linear.z = 0
         self.cmd.angular.z = 0
         self.cmd.angular.z = 0
         self.cmd.angular.z = 0.0
         self.run_state = 0#0 is going to map point. 1 is going to angle

         self.MotionPlaner_speedXY = float(rospy.get_param("MotionPlaner_speedXY",'2.0'))
         self.MotionPlaner_speedYaw = float(rospy.get_param("MotionPlaner_speedYaw",'0.12'))
         self.MotionPlaner_errorMeter = float(rospy.get_param("MotionPlaner_errorMeter",'0.4'))#m
         self.MotionPlaner_errorAngle = float(rospy.get_param("MotionPlaner_errorAngle",'0.1'))#rad

         self.MotionPlaner = Motion_Planning.Motion_Planning(self.MotionPlaner_speedXY, self.MotionPlaner_speedYaw, self.MotionPlaner_errorMeter, self.MotionPlaner_errorAngle)
         self.MapPoint = Map_Point.Map_Point()
         self.TF_body = TF.TF()
         #MapPoint is in rtkGPS coordinate system

         self.North_base = float(rospy.get_param("North_base",'0.2'))
         self.East_base = float(rospy.get_param("East_base",'0.2'))
         self.Yaw_base = float(rospy.get_param("Yaw_base",'0.0'))
         print self.North_base ,self.East_base

         self.MapPoint.add_point(4630834.0,786259.6875,1.644222378)
         self.MapPoint.add_point(4630834.0,786262.8125,1.64218032)
         self.MapPoint.add_point(4630834.0,786262.8125, 0.0776322)
         self.MapPoint.add_point(4630838.0,786263.9375,0.078574724)
         self.MapPoint.add_point(4630834.0,786262.8125, 0.0776322)
         self.MapPoint.add_point(4630834.0,786262.8125,1.64218032)
         self.MapPoint.add_point(4630834.0,786259.6875,1.644222378)
         
         #self.MapPoint.add_point(3349259.83,  511222.59,3.47215795517)
      
         
         self.MapPointNum_Now = 0
         self.MapPointNum_All = len(self.MapPoint.pointlist_x)

         self.Position_now_rtkGPS = [self.North_base+0, self.East_base+0,self.Yaw_base]
         self.EnableFlag = False
         self.EnableCount = [0,0]
         self.Position_BaseLink_Target=[0,0,0]

         while not rospy.is_shutdown():
             self.rate.sleep()

     def callback1(self, data):
         print data

     def callback2(self, data):
         if self.MapPointNum_Now < self.MapPointNum_All:
             if data.flash_state == 'YAW':
                 self.Position_now_rtkGPS[2] = data.yaw_rad
                 self.EnableCount[1] = self.EnableCount[1]+1
             if data.flash_state == 'POSITION':
                 self.Position_now_rtkGPS[0] = data.north_meter
                 self.Position_now_rtkGPS[1] = data.east_meter

                 print self.Position_now_rtkGPS
                 self.EnableCount[0] = self.EnableCount[0]+1
             if data.flash_state == 'Data_Valid_Fault':
                 self.EnableFlag = False
                 self.EnableCount[0] = 0
                 self.EnableCount[1] = 0
             if self.EnableCount[0] > 10 and self.EnableCount[1] > 10 :
                 self.EnableFlag = True
                 self.EnableCount[0] = 11
                 self.EnableCount[1] = 11

             if self.EnableFlag:

                 
                 self.Position_BaseLink_Target=self.TF_body.rtkGPStoBaseLink(self.MapPoint.get_point(self.MapPointNum_Now), self.Position_now_rtkGPS)
		 print self.Position_BaseLink_Target
                 self.MotionPlaner.setNowPosition([0,0,0])
                 self.MotionPlaner.setNextTarget(self.Position_BaseLink_Target)
                 self.Motion_temp = self.MotionPlaner.MotionPlan()#get speed wanted
                 if self.run_state is 0:#go to point
                     self.cmd.linear.x = self.Motion_temp[0]
                     self.cmd.linear.y = 0
                     self.cmd.linear.z = 0
                     self.cmd.angular.z = self.Motion_temp[1]
                     self.cmd_vel_pub.publish(self.cmd)
                 if self.run_state is 1:#go to angle
                     self.cmd.linear.x = 0
                     self.cmd.linear.y = 0
                     self.cmd.linear.z = 0
                     self.cmd.angular.z = self.Motion_temp[2]
                     self.cmd_vel_pub.publish(self.cmd)
          
             else:
                 self.cmd.linear.x = 0
                 self.cmd.linear.y = 0
                 self.cmd.linear.z = 0
                 self.cmd.angular.z = 0
                 self.cmd_vel_pub.publish(self.cmd)

             if self.run_state is 0:
                 if self.MotionPlaner.reach_position():
                     self.run_state = 1

             if self.run_state is 1:
                 if self.MotionPlaner.reach_angle():
                     self.run_state = 0
                     print "MapPointNum_Now" + str(self.MapPointNum_Now)
                     self.MapPointNum_Now = self.MapPointNum_Now+1
                     if self.MapPointNum_Now == self.MapPointNum_All:
                          self.MapPointNum_Now = self.MapPointNum_All-1

          



if __name__ == "__main__":
    try:
        body = functionAll()
    except:
        rospy.logwarn("functionAll closed!")

