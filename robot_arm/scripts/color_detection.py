import cv2 
import numpy as np 
import argparse
import urllib
import urllib.request
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16

# ap = argparse.ArgumentParser()
# ap.add_argument("-i", "--image", required=False, help="path to the input image")
# args = vars(ap.parse_args())

def preprocessing(img, scaling_factor):
    resized_image = cv2.resize(img, None, fx=scaling_factor, fy=scaling_factor)
    into_hsv = cv2.cvtColor(resized_image,cv2.COLOR_BGR2HSV)
    return into_hsv

def check_color(processed_img):
    red_lower = np.array([0, 50, 50], np.uint8) 
    red_upper = np.array([10, 255, 255], np.uint8) 
    red_mask = cv2.inRange(processed_img, red_lower, red_upper) 

    # green_lower = np.array([25, 52, 72], np.uint8) 
    # green_upper = np.array([102, 255, 255], np.uint8) 
    # green_mask = cv2.inRange(processed_img, green_lower, green_upper) 

    blue_lower = np.array([80, 80, 2], np.uint8) 
    blue_upper = np.array([130, 255, 255], np.uint8) 
    blue_mask = cv2.inRange(processed_img, blue_lower, blue_upper)

    light_gray_lower = np.array([0, 0, 200], np.uint8)
    light_gray_upper = np.array([180, 40, 255], np.uint8)
    light_gray_mask = cv2.inRange(processed_img, light_gray_lower, light_gray_upper)

    red = cv2.bitwise_and(processed_img, processed_img, mask=red_mask)
    # green = cv2.bitwise_and(processed_img, processed_img, mask=green_mask)
    blue = cv2.bitwise_and(processed_img, processed_img, mask=blue_mask)
    light_gray = cv2.bitwise_and(processed_img, processed_img, mask=light_gray_mask)

    red_area = cv2.countNonZero(red_mask)
    # green_area = cv2.countNonZero(green_mask)
    blue_area = cv2.countNonZero(blue_mask)
    light_gray_area = cv2.countNonZero(light_gray_mask)

    color_areas = {
        "RED": red_area,
        # "GREEN": green_area,
        "BLUE": blue_area,
        "LIGHT_GRAY": light_gray_area
    }
    
    dominant_color = max(color_areas, key=color_areas.get)
    print(f"Dominant color: {dominant_color}")

    return red, blue, light_gray, dominant_color

def callback(data):
    global ready
    if data.data == 1:
        ready = True


rospy.init_node('camera')
color = rospy.Publisher('/color', String, queue_size=10)
ready_color_detection = rospy.Subscriber('/ready_color_detection', Int16, callback)



def main():
    
    rate = rospy.Rate(5) # 10hz
    global ready
    ready = False  # Signal from robot arm
    scaling_factor = 0.5
    # image = cv2.imread(args["image"])
    url = 'http://192.168.32.181/cam-hi.jpg'

    roi_x = 300  # X-coordinate of the top-left corner of the ROI
    roi_y = 150  # Y-coordinate of the top-left corner of the ROI
    roi_width = 400  # Width of the ROI
    roi_height = 300  # Height of the ROI


    while not rospy.is_shutdown():
        # ret, frame = cap.read()

        # if not ret:
        #     print("Error: Could not read frame.")
        #     break

        imgResp=urllib.request.urlopen(url)
        imgNp=np.array(bytearray(imgResp.read()),dtype=np.uint8)
        frame=cv2.imdecode(imgNp,-1)
        #cv2.imshow('test',frame)
        roi = frame[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]

        processed_img = preprocessing(roi, scaling_factor)
        red, blue, light_gray, dominant_color = check_color(processed_img)

        cv2.imshow("Original Frame", frame)
        cv2.imshow("Red", red)
        # cv2.imshow("Green", green)
        cv2.imshow("Blue", blue)
        cv2.imshow("Grey", light_gray)

        if ready:
            obj_color = String()
            obj_color.data = dominant_color
            color.publish(obj_color)
            ready = False

        # Exit the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass