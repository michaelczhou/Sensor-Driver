#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import ChannelFloat32

dela_t = 1.6
def Calculate( y1, x1, y2, x2):
    if y1 == y2:
        y1 = y1+0.00001
    if (x1-x2)*(y1-y2)<0 :
        if(y1 > y2):
            th1 = math.atan(abs((x2-x1)/(y1-y2)))
            len1 = math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
            th2 = math.atan(dela_t/(len1/2.0))
            th3 = th2 - th1
            len2 = dela_t / math.sin(th2)
            x3 = x1 - len2 * math.sin(th3)
            y3 = y1 - len2 * math.cos(th3)
            x0 = (x1+x2)/2.0
            y0 = (y1+y2)/2.0
            th4 = math.atan((y0-y3)/(x0-x3))
            return[ y3,x3,th4]
        else:
            th1 = math.atan(abs((x2-x1)/(y1-y2)))
            len1 = math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
            th2 = math.atan(dela_t/(len1/2.0))
            th3 = th2 - th1
            len2 = dela_t / math.sin(th2)
            x3 = x2 - len2 * math.sin(th3)
            y3 = y2 - len2 * math.cos(th3)
            x0 = (x1+x2)/2.0
            y0 = (y1+y2)/2.0
            th4 = math.atan((y0-y3)/(x0-x3))
            return[ y3,x3,th4]
    else:
        if(y1 > y2):
            th1 = math.atan(abs((x2-x1)/(y1-y2)))
            len1 = math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
            th2 = math.atan(dela_t/(len1/2.0))
            th3 = th2 - th1
            len2 = dela_t / math.sin(th2)
            x3 = x2 - len2 * math.sin(th3)
            y3 = y2 + len2 * math.cos(th3)
            x0 = (x1+x2)/2.0
            y0 = (y1+y2)/2.0
            th4 = -math.atan((y3-y0)/(x0-x3))
            return[ y3,x3,th4]
        else:
            th1 = math.atan(abs((x2-x1)/(y1-y2)))
            len1 = math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
            th2 = math.atan(dela_t/(len1/2.0))
            th3 = th2 - th1
            len2 = dela_t / math.sin(th2)
            x3 = x1 - len2 * math.sin(th3)
            y3 = y1 + len2 * math.cos(th3)
            x0 = (x1+x2)/2.0
            y0 = (y1+y2)/2.0
            th4 = -math.atan((y3-y0)/(x0-x3))
            return[ y3,x3,th4]


class LaserProcess:
    def __init__(self):
        rospy.init_node('LaserProcess', anonymous=False)
        rospy.Subscriber("scanlms", LaserScan, self.callback1)
        self.pub = rospy.Publisher('laser_target_position', ChannelFloat32, queue_size=5)
        self.rate = rospy.Rate(50)

        self.range_find_radius = 3.0
        self.reflact_length = 1.0
        self.reach_length = 0.5

        self.reflact1_x = 0
        self.reflact1_y = 0
        self.reflact2_x = 0
        self.reflact2_y = 0
        self.reach_x = 0
        self.reach_y = 0
        self.reach_th = 0#rad

        while not rospy.is_shutdown():
            self.rate.sleep()


    def callback1(self, data):
        #print data.angle_min
        #print len(data.ranges)
        i_state = False
        i_start=0
        i_end=0
        i_num=0

        reflact_num_find = 0
        for i in range(0, len(data.ranges)):
            if data.ranges[i] < self.range_find_radius and data.intensities[i]>250:
                print data.ranges[i], data.intensities[i], data.angle_increment*i/3.1415926*180 ,i,\
                    data.ranges[i]*math.sin(data.angle_increment*i), -data.ranges[i]*math.cos(data.angle_increment*i)
                if i_state is False:
                    i_state = True
                    i_start = i
                    i_num = i_num+1
                else:
                    i_num = i_num + 1
            else:
                if i_state is True:
                    i_state = False
                    i_end = i
                    if i_num>15:#find~
                        x_temp = 0
                        y_temp = 0
                        for ii in range(i_start, i_end):
                            x_temp = x_temp + data.ranges[ii]*math.sin(data.angle_increment*ii)
                            y_temp = y_temp + -data.ranges[ii]*math.cos(data.angle_increment*ii)
                        x_temp = x_temp / i_num
                        y_temp = y_temp / i_num
                        if reflact_num_find == 0:
                            self.reflact1_x = x_temp
                            self.reflact1_y = y_temp
                            reflact_num_find = reflact_num_find + 1
                        else:
                            self.reflact2_x = x_temp
                            self.reflact2_y = y_temp
                            reflact_num_find = reflact_num_find + 1
                        print "find one"
                    else:
                        print "no use"

                    i_start = 0
                    i_end = 0
                    i_num = 0

        self.pub_data = ChannelFloat32()
        self.pub_data.name = "laser_target_position_now"

        print "//////////////"
        temp_length = math.sqrt((self.reflact1_x-self.reflact2_x)*(self.reflact1_x-self.reflact2_x)+(self.reflact1_y-self.reflact2_y)*(self.reflact1_y-self.reflact2_y))
        print reflact_num_find, self.reflact1_x, self.reflact1_y, self.reflact2_x, self.reflact2_y, temp_length

        self.pub_data = ChannelFloat32()
        self.pub_data.name = "laser_target_position_now"
        
        self.res = Calculate(self.reflact1_y, self.reflact1_x, self.reflact2_y, self.reflact2_x)
        self.pub_data.values.append(self.res[1])
        self.pub_data.values.append(self.res[0])
        self.pub_data.values.append(self.res[2])
        print self.res[1], self.res[0], self.res[2]/math.pi*180

        
        if reflact_num_find is 2 and temp_length>0.8 and temp_length<1.2:
            self.pub.publish(self.pub_data)

        print "------------"


if __name__ == "__main__":
    try:
        body = LaserProcess()
    except:
        rospy.logwarn("LaserProcess closed!")
