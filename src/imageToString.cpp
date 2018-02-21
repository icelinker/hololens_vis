#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <stdio.h>
#include <string.h>
#include <cstring>
#include <sstream>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>


void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void qualCB(const std_msgs::StringConstPtr& msg);
std::string base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len);
void sendStrIm( cv::Mat image);

//global publisher, as we need to use it in the callback function
ros::Publisher turnPub;

ros::Publisher imTextPub;
std::stringstream ssPath;
bool highQual =0;
int sentNum = 0;
ros::Time lastPub;
int main(int argc, char** argv){





    ros::init(argc, argv, "image_to_text");

    //make strings containing the paths to the test images.
    std::string path = ros::package::getPath("hololens_vis"); // this line finds where this package is on your computer without using /philip etc
    
//    ssPath << path << "/src/Image3.png";      //here we just add the ending of the path.


//    //do a check to make sure that ros found the folder on the computer.
//    if(path.empty()){
//        std::cout << "I do not know where imageTutorial is..." << std::endl;
//        return -1;
//    }

    // parse arguments
    if(argc != 3){
        std::cout << "there should be 2 args, the image topic and the output text topic" << std::endl;
        return -1;
    }
    std::cout << "image topic is " << argv[1] << "    text topic is " << argv[2] << std::endl;
    std::string imageTopicName = argv[1];
    std::string textTopicName = argv[2];


    //set up ros objects
    ros::NodeHandle node;
    image_transport::ImageTransport it_(node);


    //Subscribe to the thermal image topic.
    image_transport::Subscriber imSub = it_.subscribe(imageTopicName,1, imageCallback);
    imTextPub = node.advertise<std_msgs::String>(textTopicName,1);

    ros::Subscriber qual = node.subscribe("/qual",1,qualCB);

    cv::Mat testIm = cv::imread(ssPath.str());
    ros::Rate rate(5);
    lastPub = ros::Time::now();
    while(ros::ok()){
        ros::spinOnce();
        //sendStrIm(testIm);
        rate.sleep();
    }


}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){

    //get the image from the message
    ros::Duration sinceLast = ros::Time::now() - lastPub;
    if(sinceLast.toSec()> 0.2){
        lastPub = ros::Time::now();
        cv::Mat image =  cv_bridge::toCvShare(msg, "bgr8")->image;
        sendStrIm(image);
    }

}

void qualCB(const std_msgs::StringConstPtr& msg){
    std::string stringIn= msg->data;
    if(stringIn.compare("high") == 0){
        highQual = 1;
        sentNum = 0;
    }
    if(stringIn.compare("low") == 0){
        highQual = 0;
    }
}

void sendStrIm( cv::Mat image){
    std::vector<unsigned char> buf;
    std::vector<int> params(2);
    params[0] = CV_IMWRITE_PNG_COMPRESSION;
    params[1] = 9;
    std::string codedStr;
    if(image.rows > 0){
    if(highQual && sentNum < 2){
        sentNum ++ ;
        //cv::resize(image, image, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
        cv::imencode(".png",image,buf,params);
        codedStr = base64_encode(&buf[0], buf.size());

        std_msgs::String msg;
        msg.data = codedStr;
        imTextPub.publish(msg);
    }
    if(!highQual){
       //cv::resize(image, image, cv::Size(), 0.125, 0.125, cv::INTER_AREA);
        cv::imencode(".png",image,buf,params);
        codedStr = base64_encode(&buf[0], buf.size());
        std_msgs::String msg;
        msg.data = codedStr;
        imTextPub.publish(msg);
    }
    }

}
static const std::string base64_chars =
             "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
             "abcdefghijklmnopqrstuvwxyz"
             "0123456789+/";
std::string base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len) {
  std::string ret;
  int i = 0;
  int j = 0;
  unsigned char char_array_3[3];
  unsigned char char_array_4[4];

  while (in_len--) {
    char_array_3[i++] = *(bytes_to_encode++);
    if (i == 3) {
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] = char_array_3[2] & 0x3f;

      for(i = 0; (i <4) ; i++)
        ret += base64_chars[char_array_4[i]];
      i = 0;
    }
  }

  if (i)
  {
    for(j = i; j < 3; j++)
      char_array_3[j] = '\0';

    char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
    char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
    char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
    char_array_4[3] = char_array_3[2] & 0x3f;

    for (j = 0; (j < i + 1); j++)
      ret += base64_chars[char_array_4[j]];

    while((i++ < 3))
      ret += '=';

  }

  return ret;

}
