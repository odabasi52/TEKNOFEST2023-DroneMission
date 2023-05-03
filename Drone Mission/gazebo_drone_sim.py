#!/usr/bin/env python3
# ROS python API

from google.cloud.firestore import GeoPoint
import firebase_admin as firebase
from firebase_admin import credentials
from firebase_admin import firestore

#json dosyasinin konumu
CERTIFICATE = "/home/irene/catkin_ws/src/beginner_tutorials/scripts/database.json"
cred = credentials.Certificate(CERTIFICATE)
firebase.initialize_app(cred)
database = firestore.client()
database.collection("arananlar").document("aranan").update({"bulundu":False})

#libs for image processing
import cv2
import numpy as np
from threading import Thread

#ros libs
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import NavSatFix


global first_longitude
global first_latitude
global localx,localy,localz


globalposepub=rospy.Publisher ('mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=1)
localposepub=rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped',Twist,queue_size=1)
velocity_pub =rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

rel_latitude=0
rel_longitude=0
rel_altitude=0
amsl=0

###########################################
###########################################
# callback funcs for GPS and LOCATION

def amsl_callback(message):
    global amsl
    global rel_altitude
    amsl=float ("{0:.1f}".format (message.amsl))
    rel_altitude=float ("{0:.1f}".format (message.relative))


def globalpose_callback(pose_info):
    global rel_latitude
    global rel_longitude
    global rel_altitude
    global database

    rel_latitude=pose_info.latitude
    rel_longitude=pose_info.longitude
    rel_altitude=pose_info.altitude

    database.collection("arananlar").document("aranan").update({"location":GeoPoint(rel_latitude, rel_longitude)})

def localpose_callback(pose_info):
    global localx,localy,localz
    localx=pose_info.pose.position.x
    localy=pose_info.pose.position.y
    localz=pose_info.pose.position.z

def status_callback(status):
    print("Drone's mode = "+status.mode)
    rospy.sleep(1)

######################################################
######################################################
# CLASSES FOR MODE CHANGING AND CONTROL
class controller:
    def __init__(self):
        self.information_pub=GlobalPositionTarget()
        self.information_pub.coordinate_frame=5
        self.information_pub.type_mask=256 #Plane used to arise so We have changed the type mask
        self.information_pub.latitude=0
        self.information_pub.longitude=0
        self.information_pub.altitude=0

class fcumodes:
    def setarm(self):
        rospy.wait_for_service("mavros/cmd/arming")
        try:
            print("Setting arm")
            set_arm=rospy.ServiceProxy("mavros/cmd/arming",mavros_msgs.srv.CommandBool)
            set_arm(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)
            pass
    def auto_rtl(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            rtl_service=rospy.ServiceProxy('/mavros/set_mode',SetMode)
            response=rtl_service(custom_mode="AUTO.RTL")
            print("return back to home")
            return response.mode_sent
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s" %e)
    def take_off(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            global first_latitude
            global first_longitude
            first_latitude=rel_latitude
            first_longitude=rel_longitude
            print("First position = ",format(first_latitude),format(first_longitude))
            print('Starting the take_off')
            taking_off=rospy.ServiceProxy('mavros/cmd/takeoff',mavros_msgs.srv.CommandTOL)
            response =taking_off(min_pitch=0,yaw=0,latitude=first_latitude,longitude=first_longitude,altitude=15)
            return response
        except rospy.ServiceException as e: 
            print ("Service takeoff call failed: %s" %e)
    def offboard_mode(self):
        global globalposepub
        rospy.wait_for_service('/mavros/set_mode')
        cnt=controller()
        rate = rospy.Rate(5.0)
        k=0
        while k<10:
            globalposepub.publish(cnt.information_pub)
            rate.sleep()
            k=k+1
            rospy.wait_for_service('/mavros/set_mode')
        try:
            offboard=rospy.ServiceProxy('/mavros/set_mode',SetMode)
            response=offboard(custom_mode="OFFBOARD")
            print("Offboard mode is activated")
            return response.mode_sent
        except rospy.ServiceException as e:
            print("Flight mode has not connected successfuly. The error code = %s"%e)
    def loiter_mode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            holding=rospy.ServiceProxy('mavros/set_mode',SetMode)
            response=holding(custom_mode='AUTO.LOITER')
            return response.mode_sent
        except rospy.ServiceException as e:
            print("Loiter modu bile yapamadik = %s"%e)
    def auto_landmode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
               print ("service set_mode call failed: %s. Autoland Mode could not be set."%e)


####################################
####################################
# FUNCS TO MOVE
information_pub=Twist()
vel_msg = PositionTarget()

def MoveX(length):
    global localposepub,localx, information_pub
    old_x=localx
    if(length>0):information_pub.linear.x=1.5
    else: information_pub.linear.x=-1.5
    rate=rospy.Rate(20)
    while (abs(localx-old_x)<abs(length)):
        rate.sleep()
        localposepub.publish(information_pub)
    print("Drone has just arrived first x = ",old_x," now x = ",localx)
    information_pub.linear.x=0.0
    localposepub.publish(information_pub)


def MoveY(length):
    global localposepub,localy, information_pub
    old_y=localy
    if(length>0):information_pub.linear.y=1.5
    else: information_pub.linear.y=-1.5
    rate=rospy.Rate(20)
    while (abs(localy-old_y)<abs(length)):
        rate.sleep()
        localposepub.publish(information_pub)
    print("Drone has just arrived first x = ",old_y," now x = ",localy)
    information_pub.linear.y=0.0
    localposepub.publish(information_pub)


def MoveZ(length):
    global localposepub,localz, information_pub
    old_z=localz
    
    if(length>0):information_pub.linear.z=1
    else: information_pub.linear.z=-1
    rate=rospy.Rate(20)
    while (abs(localz-old_z)<abs(length)):
        rate.sleep()
        localposepub.publish(information_pub)
    print("Drone has just arrived first x = ",old_z," now x = ",localz)
    information_pub.linear.z=0.0
    localposepub.publish(information_pub)

def moving_center():
    global konum, velocity_pub, vel_msg, rel_altitude,color_found, database, rel_latitude, rel_longitude
    modes = fcumodes()
    rate = rospy.Rate(30)
    if color_found == True:
        vel_msg.header.stamp = rospy.get_rostime()
        vel_msg.header.frame_id ="world"
        vel_msg.coordinate_frame =8
        vel_msg.type_mask = int('011111000111', 2)
        while True:
            if konum ==1:
                vel_msg.velocity.x = -0.2
                vel_msg.velocity.y = -0.2
                velocity_pub.publish (vel_msg)
                rate.sleep()
            elif konum ==2:
                vel_msg.velocity.x = -0.2
                vel_msg.velocity.y = 0
                velocity_pub.publish (vel_msg)
            elif konum ==3:
                vel_msg.velocity.x = -0.2
                vel_msg.velocity.y = 0.2
                velocity_pub.publish (vel_msg)
                rate.sleep()
            elif konum ==4:
                vel_msg.velocity.x = 0
                vel_msg.velocity.y = 0.2
                velocity_pub.publish (vel_msg)
                rate.sleep()
            elif konum ==5:
                vel_msg.velocity.x = 0.2
                vel_msg.velocity.y = 0.2
                velocity_pub.publish (vel_msg)
                rate.sleep()
            elif konum ==6:
                vel_msg.velocity.x = 0.2
                vel_msg.velocity.y = 0
                velocity_pub.publish (vel_msg)
                rate.sleep()
            elif konum ==7:
                vel_msg.velocity.x = 0.2
                vel_msg.velocity.y = -0.2
                velocity_pub.publish (vel_msg)
                rate.sleep()
            elif konum ==8:
                vel_msg.velocity.x = 0
                vel_msg.velocity.y = -0.2
                velocity_pub.publish (vel_msg)
                rate.sleep()
            elif konum ==0:
                vel_msg.velocity.x = 0
                vel_msg.velocity.y = 0
                velocity_pub.publish (vel_msg)
                rate.sleep()
                database.collection("arananlar").document("aranan").update({"bulundu_loc":GeoPoint(rel_latitude, rel_longitude), "bulundu":True})
                modes.loiter_mode()
                rospy.sleep(10)
                modes.offboard_mode()
                MoveZ(-(rel_altitude - 2))
                modes.loiter_mode()
                print("COLLECTING GARBAGE")
                rospy.sleep(30)
                modes.offboard_mode()
                MoveZ(8)
                modes.auto_rtl()
                rospy.sleep(20)
                modes.auto_landmode()
                rospy.sleep(10)
                print("MISSION DONE")
                rospy.signal_shutdown()

#########################################
#########################################
# IMAGE CONVERSION and STUFFS

#Color boundaries 
lower = np.array([114,121,255])
upper = np.array([122,250,255])
lower2 = np.array([114,121,255])
upper2 = np.array([122,250,255])

color_found = False
konum = 6 

def image_callback():
    global  lower, upper, lower2, upper2, konum, color_found
    camSet = 'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, ' \
             'format=NV12, framerate=21/1 ! nvvidconv flip-method=' + "2" + \
             ' ! video/x-raw, width=' + "1024" + ', height=' + "768" + \
             ', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
    
    cap = cv2.VideoCapture (camSet)

    while not rospy.is_shutdown():
        img = cap.read()
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        
        #mask the image (get what you want from image)
        #img_hsv because mask can be applied to hsv
        mask = cv2.inRange(img_hsv, lower, upper)
        mask2 = cv2.inRange(img_hsv, lower2, upper2)
        
        #define kernel size  
        kernel = np.ones((7,7),np.uint8)

        # Remove unnecessary noise from mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask2 = cv2.morphologyEx(mask2, cv2.MORPH_CLOSE, kernel)
        mask2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, kernel)

        #mask + mask2
        masked = cv2.bitwise_or(mask,mask2)

        # Determine if one of them exists
        if cv2.countNonZero(mask) and cv2.countNonZero(mask2):
            color_found = True
            contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            c = max(contours, key=cv2.contourArea)

            ((x, y), _) = cv2.minEnclosingCircle(c)
            centerx = int(x)
            centery = int(y)
            
            if (centerx < 350 and centery > 350 and centery < 450):
                konum = 4
            elif (centerx < 350 and centery < 350):
                konum = 5
            elif (centerx > 350 and centerx < 450 and centery < 350):
                konum = 6
            elif (centerx > 450 and centery < 350):
                konum = 7
            elif (centerx < 350 and centery > 450):
                konum = 3
            elif (centerx < 450 and centerx > 350 and centery > 450):
                konum = 2
            elif (centerx > 450 and centery > 450):
                konum = 1
            elif (centerx > 450 and centery > 350 and centery < 450):
                konum = 8
            elif (centerx > 350 and centerx < 475 and centery < 475 and centery > 350):
                konum = 0
                print ("-------------CENTER------------")
    

#############################################
############################################
##########################################

def mission():
    global rel_altitude
    global amsl
    global first_latitude
    global first_longitude
    global localy,localx,localz
    rospy.init_node('the_node',anonymous=True)
    rospy.Subscriber('mavros/global_position/raw/fix',NavSatFix,globalpose_callback)
    modes=fcumodes()
    rospy.Subscriber('mavros/state',State,status_callback)
    rospy.Subscriber('mavros/local_position/pose',PoseStamped,localpose_callback)
    rospy.Subscriber('mavros/altitude',Altitude,amsl_callback)
    rospy.sleep(2)

    #start camera as thread
    Thread(target=image_callback).start()

    modes.setarm()
    rospy.sleep(2)
    modes.take_off()
    rospy.sleep(10)
    modes.offboard_mode()

    movex = 1
    for _ in range(10):
        for _ in range(10):
            MoveX(movex)
            moving_center()

        MoveY(1)
        moving_center()
        movex = -movex

 

if __name__=="__main__":
    try: mission()
    except rospy.ROSInterruptException:
        pass
