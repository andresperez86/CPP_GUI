#!/usr/bin python3

import mavros
import rospy
import csv
import math
import utm
import numpy as np
from time import time
from mavros_msgs.msg import*
from mavros_msgs.srv import*
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import TwistStamped
#from pymavlink import mavutil


mavros.set_namespace()

class px4AutoFlight:
    def __init__(self, velocity=5, altitud = 5,latitud=0,longitud=0):
        # self.drone_in_the_air = drone_in_the_air
        self.goal_altitude = altitud
        self.altura_drone = 0.0
        self.velocity = velocity
        self.wl = []
        self.velocidad_drone = 0.0
        self.tiempo_vuelo_drone = 0.0
        self.gpsOne = (0, 0)
        self.gpsTwo = (0, 0)
        self.distancia_viaje_drone = 0.0
        self.global_position = NavSatFix()
        # self.extended_state = ExtendedState()
        self.extended_state = 0
        self.start_time = time()
        self.end_time = self.start_time
        self.timeOne = 0.0
        self.timeTwo = 0.0
        self.start_measure = False
        self.wp_actual = 0
        self.landed_state_confirmed = False
        self.voltage = 0
        self.current = 0
        self.battery_percentage = 0
        self.coordsVisisted = []
        self.repeatedCoordsVisited = 0
        self.coverageRedundancy = 0

         # ROS services
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/cmd/takeoff')
            rospy.wait_for_service('mavros/set_mode')
            rospy.wait_for_service('mavros/cmd/arming')
            rospy.wait_for_service('mavros/mission/push')

        except rospy.ROSException:
            rospy.loginfo("failed to connect to services")

        
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
        self.ext_state_sub = rospy.Subscriber('mavros/extended_state',ExtendedState,self.extended_state_callback)
        

    """def landedState(self, desired_landed_state):
        rospy.loginfo("waiting for landed state | state: {0}".
                      format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                          desired_landed_state].name))

        self.landed_state_confirmed = False
        if self.extended_state.landed_state == desired_landed_state:
                self.landed_state_confirmed = True
                rospy.loginfo("landed state confirmed")"""

    def extended_state_callback(self, data):
        """if self.extended_state.vtol_state != data.vtol_state:
            rospy.loginfo("VTOL state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE']
                [self.extended_state.vtol_state].name, mavutil.mavlink.enums[
                    'MAV_VTOL_STATE'][data.vtol_state].name))

        if self.extended_state.landed_state != data.landed_state:
            rospy.loginfo("landed state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_LANDED_STATE']
                [self.extended_state.landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][data.landed_state].name))"""

        self.extended_state = data.landed_state
        # print(data.landed_state)

    

    def setTakeoff(self):
        try:
            self.takeOffService(altitude = self.goal_altitude)
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
            self.vehicleArmed = self.armService(True)
            if self.vehicleArmed:
                rospy.loginfo("Arming motors OK")
        except rospy.ServiceException as e:
            rospy.logerr("Arming motors failed: %s"%e)

    def setVelocityOfSimulation(self):
        wp = Waypoint()
        wp.is_current  = True
        wp.command = 178
        wp.frame = 2
        wp.param1 = float(1.0)
        wp.param2 = float(self.velocity)
        wp.param3 = float(-1.0)
        wp.param4 = float(0.0)
        wp.x_lat = float(0)
        wp.y_long = float(0)
        wp.z_alt = float(0)
        wp.autocontinue = bool(int(1.0))
        self.wl.append(wp)


    def readWayPoints(self, file):
        
        i = 0
        with open(file, 'r') as csvfile:
            for data in csv.reader(csvfile, delimiter = '\t'):
                i+=1
            j = i
        del data
        self.setVelocityOfSimulation()

        with open(file, 'r') as csvfile:
            for data in csv.reader(csvfile, delimiter = '\t'):
                wp = Waypoint()
                if j == i:
                    wp.is_current = False
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
        del data
                
            
                    
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
        self.wp_actual = msg.wp_seq+1
        #rospy.loginfo("MISSION Waypoint #%s reached.", self.wp_actual)

    def altitude_callback(self, msg):
        self.altura_drone = msg.relative
        # rospy.loginfo("Altitud: %s", self.altura_drone)

    def global_position_callback(self, msg):
        #self.latitud_drone_next= msg.latitude
        # self.longitud_drone_next = msg.longitude
        self.gpsOne = (msg.latitude, msg.longitude)
                
        #self.global_position_longitude = data.header.status.longitude
        #rospy.loginfo("Latitud: {} Longitud: {}".format(latitud, longitud)) 

    def gps_vel_callback(self, msg):
        
        #vel_x = msg.twist.linear.x
        #vel_y = msg.twist.linear.y
        """if (abs(vel_x) > (abs(vel_y))): 
            self.velocidad_drone = abs(vel_x)
        else:
            self.velocidad_drone = abs(vel_y)"""
        self.velocidad_drone = math.sqrt((msg.twist.linear.x)**2 + (msg.twist.linear.y)**2)

        # when taking off
        if self.extended_state == 3:
            # self.start_time = time()
            self.tiempo_vuelo_drone = time()-self.start_time
            self.distancia_viaje_drone = 0.0
            # self.timeOne = msg.header.stamp.secs
            # self.timeTwo = self.timeOne
            self.gpsTwo = self.gpsOne

        # when reached the goal altitude
        elif self.extended_state == 2:
            # self.timeOne = msg.header.stamp.secs
            self.tiempo_vuelo_drone = time()-self.start_time
            self.distancia_viaje_drone+= self.DistGPS(self.gpsOne, self.gpsTwo,self.altura_drone, self.altura_drone)
            self.gpsTwo = self.gpsOne
            # self.distancia_viaje_drone+=self.velocidad_drone*(self.timeOne - self.timeTwo)
            #self.timeTwo = self.timeOne
        # when the UAV landing
        elif self.extended_state == 4:
            self.end_time = time()
            self.tiempo_vuelo_drone = self.end_time -self.start_time
        # when the UAV is on the ground
        elif self.extended_state == 1:
            self.tiempo_vuelo_drone = self.end_time -self.start_time
        
        else:
            self.tiempo_vuelo_drone = 0.0

        #rospy.loginfo("Tiempo de Vuelo: {0:.2f}".format(self.tiempo_vuelo_drone))
        #rospy.loginfo("Distancia Recorrida: {0:.2f}".format(self.distancia_viaje_drone))
            

            
    def batery_status_callback(self, msg):
        self.voltage = msg.voltage
        self.current = msg.current
        self.battery_percentage = msg.percentage
        #rospy.loginfo("volt: {} current: {} remaining {}".format(self.voltage, self.current, self.battery_percentage))

    def DistGPS(self, gps0, gps1, alt0 , alt1):
        e0, n0, _, _ = utm.from_latlon(*gps0)
        e1, n1, _, _ = utm.from_latlon(*gps1)
        return np.linalg.norm([e0 - e1, n0 - n1, alt0 - alt1])


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
    
        
            

        
