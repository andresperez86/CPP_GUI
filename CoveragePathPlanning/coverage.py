#!/usr/bin python3

import mavros
import rospy
import csv
import math
from time import time
from mavros_msgs.msg import*
from mavros_msgs.srv import*
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import TwistStamped


mavros.set_namespace()

class px4AutoFlight:
    def __init__(self):
        self.drone_in_the_air = False
        self.goal_altitude = 3.9
        self.altura_drone = 0.0
        self.velocity = 2.0
        self.wl = []
        self.velocidad_drone = 0.0
        self.tiempo_vuelo_drone = 0.0
        self.latitud_drone= 0.0
        self.longitud_drone = 0.0
        self.distancia_viaje_drone = 0.0
        self.global_position = NavSatFix()
        self.start_time = time()
        self.end_time = 0.0
        self.timeOne = 0.0
        self.timeTwo = 0.0
        self.start_measure = False
        # self.cont_time = True

         # ROS services
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/cmd/takeoff')
            rospy.wait_for_service('mavros/set_mode')
            rospy.wait_for_service('mavros/cmd/arming')
            rospy.wait_for_service('mavros/mission/push')

        except rospy.ROSException:
            self.fail("failed to connect to services")

        
        self.takeOffService = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
        self.flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.wp_push_srv = rospy.ServiceProxy('mavros/mission/push', WaypointPush)


        # Subscribe to drone state to publish mission updates
        self.wp_reached_sub = rospy.Subscriber('mavros/mission/reached', WaypointReached, self.WP_Callback)
        self.altitud_drone_sub = rospy.Subscriber('mavros/altitude', Altitude, self.altitude_callback)
        self.position_drone_sub = rospy.Subscriber('mavros/global_position/global',NavSatFix, self.global_position_callback)
        self.velo_drone_sub = rospy.Subscriber('/mavros/global_position/raw/gps_vel', TwistStamped, self.gps_vel_callback)
        self.battery_dorne_sub = rospy.Subscriber('/mavros/battery', BatteryState, self.batery_status_callback)

        

    def setTakeoff(self):
        try:
            self.takeOffService(altitude = 15.0)
        except rospy.ServiceException as e:
            rospy.logerr("Takeoff failed: %s"%e)

    
    def setAutoLand(self):
        try:
            self.flightModeService(custom_mode='AUTO.LAND')
            rospy.loginfo("Landing")

        except rospy.ServiceException as e:
            rospy.logerr("Landing failed: %s"%e)

    def setArm(self):
        try:
            self.armService(True)
            rospy.loginfo("Arming motors OK")
        except rospy.ServiceException as e:
            rospy.logerr("Arming motors failed: %s"%e)


    def readWayPoints(self, file):
        
        i = 0
        with open(file, 'r') as csvfile:
            for data in csv.reader(csvfile, delimiter = '\t'):
                i+=1
            j = i
        with open(file, 'r') as csvfile:
            for data in csv.reader(csvfile, delimiter = '\t'):
                wp = Waypoint()
                if j == i:
                    wp.is_current = True
                    wp.command = 22
                elif j == 1:
                    wp.command = 21
                    wp.is_current = False
                    wp.z_alt = float(0.0)
                elif j ==0:
                    return self.wl
                else:
                    wp.is_current = False
                    wp.command = 16

                wp.frame = int(3)
                
                wp.param1 = float(0.0)
                wp.param2 = float(0.0)
                wp.param3 = float(0.0)
                wp.param4 = float(0.0)
                wp.x_lat = float(data[0])
                wp.y_long = float(data[1])
                wp.z_alt = float(self.goal_altitude)
                wp.autocontinue = bool(int(1.0))
                self.wl.append(wp)
                j-=1
                
            
                    
        # return Waypoint



    def loadMission(self):
        wps_sent = False
        wps_verified = False
        if not wps_sent:
            try:
                
                res = self.wp_push_srv(start_index=0, waypoints=self.wl)
                wps_sent = res.success
                if wps_sent:
                    rospy.loginfo("Waypoints successfully transferred")
            except rospy.ServiceException as e:
                rospy.logerr(e)
        else:
            if len(self.wl) == len(self.wl.waypoints):
                rospy.loginfo("number of waypoints transferred: {0}".format(len(self.wl)))
                wps_verified = True
        
        if wps_sent and wps_verified:
            rospy.loginfo("send waypoints success")



    def setAutoMissionMode(self):
        try:
            self.flightModeService(custom_mode='AUTO.MISSION')
            rospy.loginfo("Entering Auto Mission mode OK")
        except rospy.ServiceException as e:
            rospy.logerr("Entering Auto Mission failed: %s. AUTO.MISSION mode could not be set."%e)

    def read_failsafe(self):
        try:
            get = rospy.ServiceProxy(mavros.get_topic('param', 'get'), ParamGet)
            DLL_param = get(param_id="NAV_DLL_ACT") # Datalink Failsafe PX4 Parameter
            RCL_param = get(param_id="NAV_RCL_ACT") # RC Failsafe PX4 Parameter 
            print (" ")
            print ("----------PX4 FAILSAFE STATUS--------------")
            print (" Present NAV_DLL_ACT value is", DLL_param.value.integer)
            print (" Present NAV_RCL_ACT value is", RCL_param.value.integer)
            print ("-------------------------------------------")
            print (" ")
            return {'DL':DLL_param.value.integer,'RC':RCL_param.value.integer}
        except rospy.ServiceException as e:
            rospy.logerr("Failsafe Status read failed: %s"%e)
        
    

    def remove_failsafe(self):
        try:
	    # Disables both Datalink and RC failsafes  
	        val = ParamValue(integer=0, real=0.0) # Int value for disabling failsafe    
	        set = rospy.ServiceProxy(mavros.get_topic('param', 'set'), ParamSet)
	        new_DLL_param = set(param_id="NAV_DLL_ACT", value=val) 
	        new_RCL_param = set(param_id="NAV_RCL_ACT", value=val)    
	        print(" ")
	        print("----------REMOVING PX4 FAILSAFES ----------")
            #print("New NAV_DLL_ACT value is", new_DLL_param.value.integer)
            #print (" New NAV_DLL_ACT value is", new_DLL_param.value.integer)
            #print (" New NAV_RCL_ACT value is", new_RCL_param.value.integer)
            #print ("--------------------------------------------")
            #print (" ")
        except rospy.ServiceException as e:
            rospy.logerr("Failsafe Status change failed: %s"%e)
    

    def WP_Callback(self, msg):
        rospy.loginfo("MISSION Waypoint #%s reached.", msg.wp_seq+1)

    def altitude_callback(self, msg):
        self.altura_drone = msg.relative
        # rospy.loginfo("Altitud: %s", self.altura_drone)

    def global_position_callback(self, msg):
        # funcional los parametros 
        latitud= msg.latitude
        longitud = msg.longitude
        #self.global_position_longitude = data.header.status.longitude
        #rospy.loginfo("Latitud: {} Longitud: {}".format(latitud, longitud)) 

    def gps_vel_callback(self, msg):
        start = 0
        if (self.altura_drone > 1.5) and (self.drone_in_the_air == False):
            self.drone_in_the_air = True
            self.start_measure = True
            # self.start_time = time()
        #elif (self.altura_drone >= 0.9*self.goal_altitude ):
        #    self.start_measure = True
            
        elif (self.drone_in_the_air) and (self.altura_drone <=0.0):
            self.drone_in_the_air = False
            self.end_time = time()
            self.start_measure = False
            

        

        elif self.drone_in_the_air:
            vel_x = msg.twist.linear.x
            vel_y = msg.twist.linear.y
            self.timeOne = msg.header.stamp.secs
            # self.velocidad_drone = math.sqrt((vel_x)**2 + (vel_y)**2)
            if (abs(vel_x) > (abs(vel_y))): 
                self.velocidad_drone= abs(vel_x)
            else:
                self.velocidad_drone = abs(vel_y)
            self.tiempo_vuelo_drone=time()-self.start_time
            self.distancia_viaje_drone+=self.velocidad_drone*(self.timeOne - self.timeTwo)
            self.timeTwo =self.timeOne
            rospy.loginfo("velocidad drone:{0:.2f}".format(self.velocidad_drone))
            rospy.loginfo("tiempo recorrido:{0:.2f}".format(self.tiempo_vuelo_drone))
            rospy.loginfo("distancia recorrido:{0:.2f}".format(self.distancia_viaje_drone))
        
        elif (self.drone_in_the_air == False) and (self.altura_drone <=0.0):
            self.tiempo_vuelo_drone = self.end_time -self.start_time



        

            
        #if (abs(vel_x) > (abs(vel_y))): 
        #    velocidad_drone = abs(vel_x)
        #else:
        #    velocidad_drone = abs(vel_y)
        
        
        



    def batery_status_callback(self, msg):
        volts = msg.voltage
        current = msg.current
        percent = msg.percentage
        # rospy.loginfo("volt: {} current: {} remaining {}".format(volts, current, percent))


def main():
    rospy.init_node('coverage', anonymous=True)
    PX4modes = px4AutoFlight()
    failsafe_status = PX4modes.read_failsafe()
    if (failsafe_status['DL'] != 0) or (failsafe_status['RC'] != 0):   
        PX4modes.remove_failsafe() 
    
    # AUTO MISSION: set mode, read WPs and Arm!  
    # print(str(path.abspath('BCD_Route4.txt')))
    PX4modes.readWayPoints('ruta1.csv')
    PX4modes.loadMission()
    PX4modes.setAutoMissionMode()
    PX4modes.setArm()


    # Keep main loop
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
        
            

        