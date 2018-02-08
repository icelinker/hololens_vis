
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <ros/package.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>

tf::TransformBroadcaster* br;
void poseCB(const geometry_msgs::PoseStampedConstPtr &msg);
void nextCB(const std_msgs::StringPtr &msg);
void publishPose(ros::Publisher pub, tf::StampedTransform transform);
void publishPose2(ros::Publisher pub, tf::StampedTransform transform);
void publishSpeech(ros::Publisher pub,std::string text);
void publishPoseArray(ros::Publisher pub);
void holoPingOut(ros::Publisher pub);
void holoPingRespCB(const std_msgs::TimeConstPtr &msg);
void poseOBJ1CB(const geometry_msgs::PoseStampedConstPtr &msg);

float tableH = 0.587;
float tableW = 0.43/2;
float tableL = 0.6/2;
bool shouldNext = 1;
int main(int argc, char** argv){
    ros::init(argc, argv, "holoLink");

    ros::NodeHandle node;

    if(argc != 3){
        std::cout << "the arguments should be wheelchair tf, then world frame"<< std::endl;
        return -1;
    }
    std::string WheelTFName = argv[1];
    std::string worldFrame = argv[2];

    br = new tf::TransformBroadcaster();
    ros::Subscriber poseSub = node.subscribe("/holoPose",1,&poseCB);
    ros::Subscriber nextSub = node.subscribe("/holoNext",1,&nextCB);
    ros::Subscriber obs1sub = node.subscribe("/obs1",1,&poseOBJ1CB);
    ros::Publisher  holoWorldPub = node.advertise<geometry_msgs::Pose>("/holoWorld",1);
    ros::Publisher  holoRosOffsetPub = node.advertise<geometry_msgs::Pose>("/holoRosOffset",1);
    ros::Publisher  manPub = node.advertise<geometry_msgs::Pose>("/manPose",1);
    ros::Publisher  nozzlePub = node.advertise<geometry_msgs::Pose>("/nozzlePose",1);
    ros::Publisher  wandPub = node.advertise<geometry_msgs::Pose>("/wandPose",1);
    ros::Publisher  originPub = node.advertise<geometry_msgs::Pose>("/origin",1);
    ros::Publisher  speechPub = node.advertise<std_msgs::String>("/speech",1);
    ros::Publisher  cameraPub = node.advertise<geometry_msgs::PoseArray>("/cameraPosArr",1);
    ros::Publisher timePub= node.advertise<std_msgs::Time> ("/pingOut",1);
    ros::Publisher accuracyPub = node.advertise<std_msgs::Float32MultiArray>("accuracyPub",1);
    ros::Publisher wheelChairPub = node.advertise<geometry_msgs::Pose>("wheelChairPose",1);
    tf::TransformListener listener;
    ros::Rate rate(30);

    std::string path = ros::package::getPath("hololens_vis"); // this line finds where this package is on your computer without using /philip etc
    std::stringstream ssPath;
    ssPath << path << "/data/test.txt";

    std::ofstream myfile;
    myfile.open (ssPath.str());
    myfile<< "error, angleErr" <<std::endl;
    while(ros::ok()){

        try {
            tf::StampedTransform wheelChairTransform;

            listener.lookupTransform(worldFrame, WheelTFName, ros::Time(0), wheelChairTransform);

            publishPose(wheelChairPub, wheelChairTransform);
        } catch (tf::TransformException ex) {

        }
        try{
            tf::StampedTransform holoWorldTransform;
            listener.lookupTransform("/hlInMC", "/mocha_world",ros::Time(0), holoWorldTransform);
            publishPose(holoWorldPub,holoWorldTransform);
        }
        catch (tf::TransformException ex){

        }
        try{
            tf::StampedTransform holoWorldTransform;
            listener.lookupTransform("/rosWorld", "/holoWorld",ros::Time(0), holoWorldTransform);
            publishPose(holoRosOffsetPub,holoWorldTransform);
        }
        catch (tf::TransformException ex){

        }
        try{
            tf::StampedTransform NozzleTransform;
            listener.lookupTransform("/mocha_world", "/nozzle",ros::Time(0), NozzleTransform);
            //tf::Quaternion q(NozzleTransform.getRotation().getX(), NozzleTransform.getRotation().getZ(), NozzleTransform.getRotation().getY(), 1.0 * NozzleTransform.getRotation().getW());
//            NozzleTransform.setRotation(q);
            publishPose2(nozzlePub,NozzleTransform);
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
        try{
            if(shouldNext){
                shouldNext = false;
                static tf::StampedTransform wandTransform, wandTransformFound;
                listener.lookupTransform("/mocha_world", "/wandout",ros::Time(0), wandTransformFound);
                float posErr =  wandTransformFound.getOrigin().distance( wandTransform.getOrigin());
                float angErr = wandTransformFound.getRotation().angle(wandTransform.getRotation());
                std_msgs::Float32MultiArray arrayMsg;
                arrayMsg.data.push_back(posErr);
                arrayMsg.data.push_back(angErr);
                accuracyPub.publish(arrayMsg);
                myfile  <<posErr << "," << angErr << std::endl;
                float x = -tableW + 2*tableW*rand()/RAND_MAX;
                float z = -tableL + 2*tableL* rand()/RAND_MAX;
                float angle = 3.14*2* rand()/RAND_MAX;
                wandTransform.setOrigin(tf::Vector3(x,z,tableH));
                wandTransform.setRotation(tf::Quaternion(tf::Vector3(0,0,1),angle));
                publishPose(wandPub,wandTransform);
            }
        }
        catch (tf::TransformException ex){

        }
        tf::StampedTransform origin;
        origin.setOrigin(tf::Vector3(0,0,0));
        origin.setRotation(tf::Quaternion(0,0,0,1));
        publishPose(originPub,origin);
        //publishPoseArray(cameraPub);
        rate.sleep();
        ros::spinOnce();
    }




}

tf::Vector3 cameraPos[8];
float x = 1;
float High = 2;
float low = 1;
float z = 1;

void publishPoseArray(ros::Publisher pub){

    cameraPos[0] = tf::Vector3(-x,low,z);
    cameraPos[1] = tf::Vector3(-x,High,z);
    cameraPos[2] = tf::Vector3(x,High,z);
    cameraPos[3] = tf::Vector3(x,low,z);
    cameraPos[4] = tf::Vector3(tableW,tableH,-tableL);
    cameraPos[5] = tf::Vector3(tableW,tableH,tableL);
    cameraPos[6] = tf::Vector3(-tableW,tableH,-tableL);
    cameraPos[7] = tf::Vector3(-tableW,tableH,tableL);

    geometry_msgs::PoseArray arr;

    for(int i = 0; i < 8; i++){
        geometry_msgs::Pose aPose;
        aPose.position.x = cameraPos[i].x();
        aPose.position.y = cameraPos[i].y();
        aPose.position.z = cameraPos[i].z();
        arr.poses.push_back(aPose);
    }
    arr.header.frame_id = "/mocha_world";
    arr.header.stamp = ros::Time::now();
    pub.publish(arr);
}

void nextCB(const std_msgs::StringPtr &msg){
    if(!msg->data.compare("wandNext")){
        shouldNext = 1;
    }
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

void publishPose2(ros::Publisher pub, tf::StampedTransform transform){
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


void poseCB(const geometry_msgs::PoseStampedConstPtr &msg){
    tf::StampedTransform transform;
    tf::Quaternion rotation;
    tf::Point location;
    rotation.setX(msg->pose.orientation.x);
    rotation.setY(msg->pose.orientation.z);
    rotation.setZ(msg->pose.orientation.y);
    rotation.setW(msg->pose.orientation.w);
    rotation = rotation.inverse();
    location.setX(msg->pose.position.x);
    location.setY(msg->pose.position.z);
    location.setZ(msg->pose.position.y);

    transform.setOrigin(location);
    transform.setRotation(rotation);
    transform.stamp_ = msg->header.stamp;


    transform.child_frame_id_ = "holoLens";
    transform.frame_id_ = "mocha_world";
    br->sendTransform(transform);
}

void poseOBJ1CB(const geometry_msgs::PoseStampedConstPtr &msg){
    tf::StampedTransform transform;
    tf::Quaternion rotation;
    tf::Point location;
    rotation.setX(msg->pose.orientation.x);
    rotation.setY(msg->pose.orientation.z);
    rotation.setZ(msg->pose.orientation.y);
    rotation.setW(msg->pose.orientation.w);
    rotation = rotation.inverse();
    location.setX(msg->pose.position.x);
    location.setY(msg->pose.position.z);
    location.setZ(msg->pose.position.y);

    transform.setOrigin(location);
    transform.setRotation(rotation);
    transform.stamp_ = msg->header.stamp;


    transform.child_frame_id_ = "obj1";
    transform.frame_id_ = "move_base";
    br->sendTransform(transform);
}
