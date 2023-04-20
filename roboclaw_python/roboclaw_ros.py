#!/usr/bin/env python
from roboclaw import Roboclaw
import rospy
from ikh_ros_msgs.msg import FloatStamped
from std_srvs.srv import SetBool
from std_msgs.msg import String
import numpy as np

class MotorCurrents :
    def __init__(self):
        self._bufferSize = 10
        self.m1 = np.array([0.0 for i in range(self._bufferSize)])
        self.m2 = np.array([0.0 for i in range(self._bufferSize)])
    
    def setBufferSize(self, size):
        
        if (self._bufferSize == size):
            pass
        elif (self._bufferSize > size):
            self._bufferSize = size
            np.insert(self.m1, 0, np.zeros(self._bufferSize - self.m1.size))
            np.insert(self.m2, 0, np.zeros(self._bufferSize - self.m2.size))           
        else :
            self._bufferSize = size
            self.m1 = self.m1[self.m1.size-self._bufferSize:self.m1.size]
            self.m2 = self.m2[self.m2.size-self._bufferSize:self.m2.size]
            
    def appendM1(self,value):
        self.m1 = np.append(self.m1,value)
        self.m1 = np.delete(self.m1,0)
        
    def appendM2(self,value):
        self.m2= np.append(self.m2,value)
        self.m2 = np.delete(self.m2,0)
        
    def getMeanM1M2Values(self):
        return [np.mean(self.m1),np.mean(self.m2)]
    
    def getMaxOfMeans(self):
        return np.max(self.getMeanM1M2Values())
    
    def getM1(self):
        return self.m1
    
    def getM2(self):
        return self.m2
        
        

class Node:
    def __init__(self):
        rospy.init_node("roboclaw_node")
        # rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to roboclaw")
        self.dev_name = rospy.get_param("~dev", "/dev/ttyACM0")
        self.baud_rate = int(rospy.get_param("~baud", "115200"))
        self.address = int(rospy.get_param("~address", "128"))
        
        # Parameter to indicate that the motors are running.
        self.is_running = False
        
        # Check port permissions
        if (self.port_permissions(self.dev_name)):
            rospy.loginfo("Port permissions are acceptable.")
        else:
            rospy.logerr("Port permissions are not valid. (sudo chmod 777 [port name])")
            exit(-1)
        
        # Create a roboclaw instance
        self.roboclaw = Roboclaw(self.dev_name, self.baud_rate)

        # Open roboclaw port and check version
        self.open_roboclaw_port()

        # Topics
        self.m1_current_pub = rospy.Publisher('m1_current', FloatStamped, queue_size=10)
        self.m2_current_pub = rospy.Publisher('m2_current', FloatStamped, queue_size=10)
        self.deck_position_pub = rospy.Publisher('deck_position', String, queue_size=10,latch=True)
        self.m1_current_mean_pub = rospy.Publisher('m1_current_mean', FloatStamped, queue_size=10)
        self.m2_current_mean_pub = rospy.Publisher('m2_current_mean', FloatStamped, queue_size=10)
        
        self.current_msg = FloatStamped()
        
        # Motor Currents Class
        self.motorCurrents = MotorCurrents()
        
        # Services
        self.deck_control_srv = rospy.Service('move_prismatic', SetBool, self.deck_control_cb)
        
        # Timers
        timer_update_rate = rospy.Duration(1.0/20.0)
        rospy.Timer(timer_update_rate, self._read_data_callback)

        rospy.sleep(1)
        
    def _read_data_callback(self,timer):
        _none, m1_current, m2_current = self.roboclaw.ReadCurrents(self.address)
        self.motorCurrents.appendM1(m1_current)
        self.motorCurrents.appendM2(m2_current)
        # Publish Raw Current Messages
        msg = FloatStamped()
        msg.header.stamp = rospy.Time.now()
        
        # Publish Mean Current Messages
        mean_currents = self.motorCurrents.getMeanM1M2Values()
        msg.data = mean_currents[0]
        self.m1_current_pub.publish(msg)
        msg.data = mean_currents[1]
        self.m2_current_pub.publish(msg)
        
        # Get Temperature
        temp1, temp2 = self.roboclaw.ReadTemp(self.address)
        err1, err2 = self.roboclaw.ReadError(self.address)

    
    def open_roboclaw_port(self):
        rospy.loginfo('Roboclaw Node: Try to open port...')
        try:
            self.roboclaw.Open()
        except Exception as e:
            rospy.logfatal("Could not connect to Roboclaw")
            rospy.logdebug(e)
            # rospy.signal_shutdown("Could not connect to Roboclaw")
        rospy.loginfo("Roboclaw Node: Try to read version...")
        try:
            version = self.roboclaw.ReadVersion(self.address)
        except Exception as e:
            rospy.logwarn("Problem getting roboclaw version")
            rospy.logdebug(e)
            pass
        
        if not version[0]:
            rospy.logwarn("Could not get version from roboclaw")
        else:
            rospy.logdebug(repr(version[1]))

    
    def port_permissions(self,port):
        try :
            import os
            if(os.access(port,os.R_OK) and os.access(port,os.W_OK) and os.access(port, os.X_OK)):
                return True
            else:
                return False
        except:
            rospy.logerror('Roboclaw: Cannot read port permissions')
        
        
    def run(self):
        rospy.loginfo("Starting motor drive")
        #rospy.spin()
        r_time = rospy.Rate(10)
        while not rospy.is_shutdown():
            r_time.sleep()


    def deck_control_cb(self,req):
        
        res = (False, "Nothing")
        if req.data:
            res = self.roboclaw_control(1)
        else:
            res = self.roboclaw_control(0)
        return res
    
    def roboclaw_control(self,cmd):
        time_stared = rospy.Time.now().to_sec()
        vel = 85
        if (not self.is_running):
            if (cmd):
                rospy.loginfo("Forward!")
                self.is_running = True
                self.roboclaw.BackwardM1(self.address, int(vel*1.055))
                self.roboclaw.BackwardM2(self.address, vel)
            else:
                rospy.loginfo("Backword")
                self.is_running = True
                self.roboclaw.ForwardM1(self.address, int(vel*1.055))
                self.roboclaw.ForwardM2(self.address, vel)
        else: 
            return (False, "An other command is executed") 
        while(not rospy.is_shutdown()):
            # print("=====================")
            # print("Max Value: ",self.motorCurrents.getMaxOfMeans())
            # print("Cmd: ",cmd)
            duration = rospy.Time.now().to_sec()-time_stared
            # print("Duration: ",duration)
            if (duration>25):
                self.is_running = False
                return (False, "Timeout reached!")
            elif ((25>duration>3) and (self.motorCurrents.getMaxOfMeans()<-9)):
                self.roboclaw.ForwardM1(self.address, 0)
                self.roboclaw.ForwardM2(self.address, 0)
                msg=String()
                if (cmd):
                    msg.data = "up"
                    self.deck_position_pub.publish(msg)
                else:
                    msg.data = "down"
                    self.deck_position_pub.publish(msg)
                self.is_running = False
                return (True, "Position Reached")       
            rospy.sleep(0.2)
                
                
            
        


if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
