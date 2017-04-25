#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

tf::TransformBroadcaster* br;
void poseCB(const geometry_msgs::PoseConstPtr &msg);
void publishPose(ros::Publisher pub, tf::StampedTransform transform);
void publishSpeech(ros::Publisher pub,std::string text);
void publishPoseArray(ros::Publisher pub);
int main(int argc, char** argv){
    ros::init(argc, argv, "holoLink");

    ros::NodeHandle node;
    br = new tf::TransformBroadcaster();
    ros::Subscriber poseSub = node.subscribe("/holoPose",1,&poseCB);
    ros::Publisher  holoWorldPub = node.advertise<geometry_msgs::Pose>("/holoWorld",1);
    ros::Publisher  manPub = node.advertise<geometry_msgs::Pose>("/manPose",1);
    ros::Publisher  originPub = node.advertise<geometry_msgs::Pose>("/origin",1);
    ros::Publisher  speechPub = node.advertise<std_msgs::String>("/speech",1);
    ros::Publisher  cameraPub = node.advertise<geometry_msgs::PoseArray>("/cameraPosArr",1);
    tf::TransformListener listener;
    ros::Rate rate(100);
    while(ros::ok()){
        try{
            tf::StampedTransform holoWorldTransform;
            listener.lookupTransform("/hlInMC", "/mocha_world",ros::Time(0), holoWorldTransform);
            publishPose(holoWorldPub,holoWorldTransform);
        }
        catch (tf::TransformException ex){

        }
        try{
            tf::StampedTransform manTransform;
            listener.lookupTransform("/mocha_world", "/man",ros::Time(0), manTransform);
            publishPose(manPub,manTransform);
        }
        catch (tf::TransformException ex){

        }
        tf::StampedTransform origin;
        origin.setOrigin(tf::Vector3(0,0,0));
        origin.setRotation(tf::Quaternion(0,0,0,1));
        publishPose(originPub,origin);
        publishPoseArray(cameraPub);
        rate.sleep();
        ros::spinOnce();
    }




}

tf::Vector3 cameraPos[8];

void publishPoseArray(ros::Publisher pub){

    cameraPos[0] = tf::Vector3(0,1,2);
    cameraPos[1] = tf::Vector3(0,1,2);
    cameraPos[2] = tf::Vector3(0,1,2);
    cameraPos[3] = tf::Vector3(0,1,2);
    cameraPos[4] = tf::Vector3(0,1,2);
    cameraPos[5] = tf::Vector3(0,1,2);
    cameraPos[6] = tf::Vector3(0,1,2);
    cameraPos[7] = tf::Vector3(0,1,2);

    geometry_msgs::PoseArray cameras;

    for(int i = 0; i < 8; i++){
        geometry_msgs::Pose aPose;
        aPose.position.x = cameraPos[i].x();
        aPose.position.y = cameraPos[i].y();
        aPose.position.z = cameraPos[i].z();
        cameras.poses.push_back(aPose);
    }
    cameras.header.frame_id = "/mocha_world";
    cameras.header.stamp = ros::Time::now();
    //pub.publish(cameras);
}

void publishPose(ros::Publisher pub, tf::StampedTransform transform){
    geometry_msgs::Pose msg;
    msg.orientation.x =transform.getRotation().getX();
    msg.orientation.y =transform.getRotation().getZ();
    msg.orientation.z =transform.getRotation().getY();
    msg.orientation.w =transform.getRotation().getW();
    msg.position.x =transform.getOrigin().getX();
    msg.position.y =transform.getOrigin().getZ();
    msg.position.z =transform.getOrigin().getY();
    pub.publish(msg);
}


void poseCB(const geometry_msgs::PoseConstPtr &msg){
    tf::StampedTransform transform;
    tf::Quaternion rotation;
    tf::Point location;
    rotation.setX(msg->orientation.x);
    rotation.setY(msg->orientation.z);
    rotation.setZ(msg->orientation.y);
    rotation.setW(msg->orientation.w);
    rotation = rotation.inverse();
    location.setX(msg->position.x);
    location.setY(msg->position.z);
    location.setZ(msg->position.y);

    transform.setOrigin(location);
    transform.setRotation(rotation);
    transform.stamp_ = ros::Time::now();


    transform.child_frame_id_ = "holoLens";
    transform.frame_id_ = "mocha_world";
    br->sendTransform(transform);
}
