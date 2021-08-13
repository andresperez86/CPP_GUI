#!/usr/bin python3

import mavros
import rospy
import csv
from mavros_msgs.msg import*
from mavros_msgs.srv import*
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import TwistStamped, Twist
from time import time


mavros.set_namespace()

class px4AutoFlight:
    def __init__(self):
        self.altitude = 15.0
        self.velocity = 10.0
        self.wl = []
        self.distancia_viaje = 0.0
        self.despegue_drone = False
        self.altura_drone = 0.0
        self.velocidad_drone = 0.0
        self.latitud_drone  = 0.0
        self.longitud_drone = 0.0
        self.distancia_recorrida_drone = 0.0
        self.start_time = 0.0
        self.final_time = 0.0
        self.tiempo_recorrido_drone = 0.0
        self.timeOne = 0.0
        self.timeTwo = 0.0
        # Subscribe to drone state to publish mission updates
        self.wp_reached = rospy.Subscriber('mavros/mission/reached', 
                                            WaypointReached, self.WP_Callback)
        self.altitude_drone = rospy.Subscriber('mavros/altitude', 
                                            Altitude, self.altitude_callback)
        self.position_drone = rospy.Subscriber('mavros/global_position/global',
                                            NavSatFix, self.global_position_callback)
        self.velocity_drone = rospy.Subscriber('/mavros/global_position/raw/gps_vel', 
                                            TwistStamped, self.gps_vel_callback)
        self.battery_drone = rospy.Subscriber('/mavros/battery', 
                                            BatteryState, self.batery_status_callback)

        # Publisher for the velocity

        self.publish_vel_drone = rospy.Publisher('/mavros/gps_vel', Twist, queue_size=10)
        self.move = Twist()

        
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")

        try: 
            rospy.wait_for_service('mavros/cmd/takeoff', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.wait_for_service('mavros/mission/push', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/mission/clear', service_timeout)
        except rospy.ROSException:
            self.fail("failed to connect to services")
            
        self.takeOffService = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
        self.flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.wp_push = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
        self.wp_clear = rospy.ServiceProxy('mavros/mission/clear',WaypointClear)
            

    def setTakeoff(self):
        try:
            self.takeOffService(altitude = 5.0)
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
        del data
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
                wp.z_alt = float(self.altitude)
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
                
                res = self.wp_push(start_index=0, waypoints=self.wl)
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
        # rospy.wait_for_service('mavros/set_mode')
        try:
            # flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
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
        # self.updateDataDrone()
        # rospy.loginfo("Altitud: %s", self.altitude)

    def global_position_callback(self, msg):
        # funcional los parametros 
        self.latitud_drone= msg.latitude
        self.longitud_drone = msg.longitude
        # self.updateDataDrone()
        #self.global_position_longitude = data.header.status.longitude
        #rospy.loginfo("Latitud: {} Longitud: {}".format(latitud, longitud)) 

    def gps_vel_callback(self, msg):

        if (self.altura_drone > 0.1)  and (self.despegue_drone == False):
            self.despegue_drone = True
            self.start_time = time()
        elif (self.altura_drone <= 0.2) and (self.despegue_drone):
            self.despegue_drone = False
            self.final_time = time()
            self.setAutoLand()
            try:
                res = self.wp_clear()
                if not res.success:
                        rospy.loginfo("failed to send waypoint clear command")
            except rospy.ServiceException as e:
                rospy.logerr(e)

            self.tiempo_recorrido_drone = self.final_time - self.start_time
        elif self.despegue_drone:
            self.tiempo_recorrido_drone = time() - self.start_time

        #self.distancia_recorrida_drone = 0.0
        #vel_x = msg.twist.linear.x
        #vel_y = msg.twist.linear.y
        self.timeOne = msg.header.stamp.secs
        tiempo = self.timeOne - self.timeTwo
        
            
        if (abs(msg.twist.linear.x) > (abs(msg.twist.linear.y))): 
            self.velocidad_drone = abs(msg.twist.linear.x)
            self.move.linear.x = self.velocity
        else:
            self.velocidad_drone = abs(msg.twist.linear.y)
            self.move.linear.y = self.velocity
        
        self.distancia_recorrida_drone+= self.velocidad_drone*(tiempo)
        #self.updateDataDrone()
        self.timeTwo = self.timeOne
        rospy.loginfo("Distancia Recorrida:{0:.3f} Tiempo Recorrido:{1:.2f} ".format(self.distancia_recorrida_drone, self.tiempo_recorrido_drone))
        rospy.loginfo('Velocidad: {0:.2f}'.format(self.velocidad_drone))
        self.publish_vel_drone.publish(self.move)

    def batery_status_callback(self, msg):
        volts = msg.voltage
        current = msg.current
        percent = msg.percentage
        # rospy.loginfo("volt: {} current: {} remaining {}".format(volts, current, percent))

        #rospy.loginfo('Altura drone: {}'.format(self.altura_drone))
        #rospy.loginfo("Latitud: {} Longitud: {}".format(self.latitud_drone, self.longitud_drone))


        

def main(args):
    rospy.init_node('coverage', anonymous=True)
    PX4modes = px4AutoFlight()
    failsafe_status = PX4modes.read_failsafe()
    if (failsafe_status['DL'] != 0) or (failsafe_status['RC'] != 0):   
        PX4modes.remove_failsafe() 
    
    # AUTO MISSION: set mode, read WPs and Arm!  
    PX4modes.setTakeoff()
    PX4modes.readWayPoints('WAVE_Route5.csv')
    PX4modes.loadMission()
    PX4modes.setAutoMissionMode()
    PX4modes.setArm()

    # Drone = obtainDroneData()
    #rospy.loginfo('Altura drone: {}'.format(Drone.altura_drone))
    # rospy.loginfo("Latitud: {} Longitud: {}".format(Drone.latitud_drone, Drone.longitud_drone))
 
    try:
       rospy.spin()
    except rospy.ROSInterruptException:
        pass
  

if __name__ == '__main__':
    main(sys.argv)
    
    
        
            

        