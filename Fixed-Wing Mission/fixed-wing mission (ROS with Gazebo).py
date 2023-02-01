#!/usr/bin/env python3
# ROS python API
#FIREBASE STUFFS
from google.cloud.firestore import GeoPoint
import firebase_admin as firebase
from firebase_admin import credentials
from firebase_admin import firestore
import numpy as np
import cv2

CERTIFICATE = "/home/irene/catkin_ws/src/beginner_tutorials/scripts/database.json"
cred = credentials.Certificate(CERTIFICATE)
firebase.initialize_app(cred)
database = firestore.client()
database.collection("arananlar").document("aranan").update({"bulundu":False})

#ZMQ PUB 
import zmq

IP = "192.168.1.101"
PORT = "4141"
TCP_CONNECTION = "tcp://"+IP+":"+PORT

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind(TCP_CONNECTION)

connection_dict = {"enlem":10, "boylam":10, "hiz_y":10, "hiz_x":10, "yukseklik":10, "pusula":10, "battery":10, "durum":"LOITER"}


#Image processing libs
import cv2
import sensor_msgs.msg 
import numpy as np

#ROS libs
import rospy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import *
from std_msgs.msg import Float64
from mavros_msgs.srv import *
from sensor_msgs.msg import NavSatFix, BatteryState

globalposepub=rospy.Publisher ('mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=1)
class controller:
    def __init__(self):
        self.information_pub=GlobalPositionTarget()
        self.information_pub.coordinate_frame=5
        self.information_pub.type_mask=256 #Plane used to arise so We have changed the type mask
        self.information_pub.latitude=0
        self.information_pub.longitude=0
        self.information_pub.altitude=0

def amsl_callback(message):
    global amsl
    global rel_altitude
    global connection_dict
    amsl=float ("{0:.4f}".format (message.amsl))
    rel_altitude=float ("{0:.4f}".format (message.relative))
    connection_dict["yukseklik"] = rel_altitude

def globalpose_callback(pose_info):
    global rel_latitude
    global rel_longitude
    global rel_altitude
    global database, connection_dict

    rel_latitude=pose_info.latitude
    rel_longitude=pose_info.longitude
    rel_altitude=pose_info.altitude

    #update database -> set latitude, longtitude to updated values
    database.collection("arananlar").document("aranan").update({"location":GeoPoint(rel_latitude, rel_longitude)})
    connection_dict["enlem"]=rel_latitude
    connection_dict["boylam"]=rel_longitude

def status_callback(status):
    global connection_dict
    print("MODE of Fixed-Wing --> "+ status.mode)
    connection_dict["durum"] = status.mode
    rospy.sleep(1)

def velocity_callback(message):
    global connection_dict

    connection_dict["hiz_x"] = message.twist.linear.x
    connection_dict["hiz_y"] = message.twist.linear.y

def battery_callback(message):
    global connection_dict
    connection_dict["battery"] = message.percentage*100

def compass_callback(message):
    global socket, connection_dict
    connection_dict["pusula"] = message.data
    socket.send_pyobj(connection_dict)

#-------------------------------------------------------        
# ------------------------------------------------------
# CHANGE MODES

def offboard_mode():
    global globalposepub
    rospy.wait_for_service('/mavros/set_mode')
    cnt=controller()
    rate = rospy.Rate (5.0)
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

def loiter_mode():
    rospy.wait_for_service('mavros/set_mode')
    try:
        holding=rospy.ServiceProxy('mavros/set_mode',SetMode)
        response=holding(custom_mode='AUTO.LOITER')
        return response.mode_sent
    except rospy.ServiceException as e:
            print("Loiter modu bile yapamadik = %s"%e)
def auto_landmode():
    rospy.wait_for_service('mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
        flightModeService(custom_mode='AUTO.LAND')
    except rospy.ServiceException as e:
        print ("service set_mode call failed: %s. Autoland Mode could not be set."%e)


#-------------------------------------------------------
#-------------------------------------------------------
#-------------------------------------------------------
# DETECT IMAGE and MISSION

def imgmsg_to_cv2(img_msg):
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()

    image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)
    return image_opencv

img_msg = sensor_msgs.msg.Image
img_topic = "/iris/camera/rgb/image_raw"

def image_callback(rosmsg):
    global  lower, upper, lower2, upper2, rel_longitude, rel_latitude, database

    img = imgmsg_to_cv2(rosmsg)
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

    # Determine if the color exists on the image
    if cv2.countNonZero(mask) and cv2.countNonZero(mask2):
        database.collection("arananlar").document("aranan").update({"bulundu_loc":GeoPoint(rel_latitude, rel_longitude), "bulundu":True})
        print("-----------------------------------")
        print("-----------------------------------")
        print("-----------------------------------")
        print("CHOSEN COLOR FOUND")
        print("SERVO DROPPED")
        loiter_mode()
        rospy.sleep(60)
       
        print("MISSION SUCCESSFULL")
        rospy.signal_shutdown("MISSION HAS FINISHED")

    

def mission(choise):
    global img_topic, img_msg, amsl,first_latitude,first_longitude
    global lower, upper, lower2,upper2

    if choise == "A":   #Red lower uppper bounds
        lower = np.array([0,72,80])
        upper = np.array([0,178,250])
        lower2 = np.array([116,47,9])
        upper2 = np.array([179,218,91])
    elif choise == "B":   #Blue lower upper bounds
        lower = np.array([120,101,255])
        upper = np.array([123,255,255])
        lower2 = np.array([114,209,147])
        upper2 = np.array([119,255,255])
    elif choise == "C":   #Green lower upper bounds
        lower = np.array([60,88,255])
        upper = np.array([62,251,255])
        lower2 = np.array([61,208,99])
        upper2 = np.array([72,248,254])
    
    rospy.init_node('the_node',anonymous=True)

    #subscribers
    rospy.Subscriber('mavros/global_position/raw/fix',NavSatFix,globalpose_callback)
    rospy.Subscriber('mavros/state',State,status_callback)
    rospy.Subscriber(img_topic, img_msg, image_callback)
    rospy.Subscriber('mavros/altitude',Altitude,amsl_callback)
    rospy.Subscriber('mavros/global_position/raw/gps_vel',TwistStamped,velocity_callback)
    rospy.Subscriber('mavros/battery',BatteryState,battery_callback)
    rospy.Subscriber('mavros/global_position/compass_hdg',Float64,compass_callback)
    rospy.spin()

if __name__=="__main__":
    try: 
        #get renk information from database
        choises = database.collection("arananlar").document("aranan").get()
        choise_dict = choises.to_dict()
        choise = choise_dict["renk"] 

        #call mission
        mission(choise)
    except rospy.ROSInterruptException:
        pass
