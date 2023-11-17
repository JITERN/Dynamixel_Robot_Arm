//#define sensorPin A0
//#include <ros.h>
//#include <std_msgs/String.h>
//
//ros::NodeHandle nh;
//
//std_msgs::String str_msg;
//ros::Publisher pickup_avail("pickup_avail", &str_msg);
//
//char msg[13] = "new object!";
//
//void setup()
//{
//  nh.initNode();
//  nh.advertise(pickup_avail);
//  Serial.begin(115200);
//}
//
//bool object = 0;
//int count = 0;
//void loop()
//{
//  int x = analogRead(sensorPin);
//  Serial.println(x);
//
//  if (x > 400){
//    count++;
//    }
//  else{
//    count = 0;
//    object = 0;
//    }
//   if ((count > 10) && (object==0)){
//      str_msg.data = msg;
//      pickup_avail.publish( &str_msg );
//      Serial.println(x);
//      object = 1;
//   }
//
//
//  nh.spinOnce();
//  delay(50);
//}





#include "Filter.h"
#include "MegunoLink.h" // for plotting
#include "ArduinoTimer.h" // for periodic measurement


#define sensorPin A0
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher pickup_avail("pickup_avail", &str_msg);

char msg[13] = "new object!";


// the <float> makes a filter for float numbers
// 20 is the weight (20 => 20%)
// 0 is the initial value of the filter
ExponentialFilter<float> Filtered(10, 0);
bool object = 0;
int count = 0;

void setup()
{
  nh.initNode();
  nh.advertise(pickup_avail);
//  Serial.begin(115200);
}

void loop()
{  
    float Raw = analogRead(sensorPin);
    Filtered.Filter(Raw);
    float Smooth = Filtered.Current();

//    Serial.print(Raw);
//    Serial.print(",");
//    Serial.println(Smooth);

  if (Smooth > 600){
    count++;
    }
  else{
    count = 0;
    object = 0;
    }
   if ((count > 10) && (object==0)){
      str_msg.data = msg;
      pickup_avail.publish( &str_msg );
//      Serial.println("adshjk");
      object = 1;
   }

  nh.spinOnce();
  delay(25);
}
