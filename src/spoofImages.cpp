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
#include <fstream>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

image_transport::Publisher imPub;



int main(int argc, char** argv){





    ros::init(argc, argv, "spoofImage");

    if(argc < 3){
        std::cout << "should have argument describing the topic to spoof image onto and an image to spoof" << std::endl;
        return(0);
    }
    std::stringstream ssPath;
    std::string path = ros::package::getPath("hololens_vis");
    ssPath << path << "/src/" << argv[2];

    //set up ros objects
    ros::NodeHandle node;
    image_transport::ImageTransport it_(node);



    //Subscribe to the image topic.
    std::string topicName = argv[1];
    imPub = it_.advertise(topicName,1);


    cv::Mat testIm = cv::imread(ssPath.str());
    ros::Rate rate(5);
    while(ros::ok()){
        ros::spinOnce();
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", testIm).toImageMsg();
        msg->header.stamp = ros::Time::now();
        imPub.publish(msg);
        rate.sleep();
    }


}

