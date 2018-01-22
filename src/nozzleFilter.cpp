





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
bool AreQuaternionsClose(tf::Quaternion q1, tf::Quaternion q2);
tf::Quaternion InverseSignQuaternion(tf::Quaternion q);

int main(int argc, char** argv){
    ros::init(argc, argv, "nozzleFilter");

    ros::NodeHandle node;
    br = new tf::TransformBroadcaster();
    tf::TransformListener listener;

    int filterConst = 3;
    int count = 0;

    if(argc >1){
        filterConst = atoi(argv[1]);
    }

    std::vector<tf::StampedTransform> transList;
    for(int i = 0 ; i < filterConst; i++){
        tf::StampedTransform fake;
        transList.push_back(fake);
    }




    ros::Rate rate(100);
    tf::StampedTransform NozzleTransform;
    ros::Time last = ros::Time::now();
    while(ros::ok()){

        try{

            listener.lookupTransform("/holoLens", "/nozzleRaw",ros::Time(0), NozzleTransform);
            if(NozzleTransform.stamp_!=last){
                last = NozzleTransform.stamp_;
                transList[count%filterConst]  =  NozzleTransform;

            count++;
                if(count> filterConst){

                    //find average origin, simply the average of the vector.
                    tf::Vector3 avOrigin(0,0,0);
                    for (int i = 0; i < transList.size(); i++){
                        tf::Vector3 av;
                        avOrigin = avOrigin + transList[i].getOrigin();
                    }
                    avOrigin /= transList.size();


                    //find average of the rotation quaternion, as per the unity example.
                    float w = 0.0f;
                    float x = 0.0f;
                    float y = 0.0f;
                    float z = 0.0f;
                    for(int i = 0; i < transList.size(); i++){

                        //check the next quaternion is near.
                        if(!AreQuaternionsClose(transList[0].getRotation(),transList[i].getRotation())){
                            transList[i].setRotation(InverseSignQuaternion(transList[i].getRotation()));
                        }

                        //Average the values
                        x += transList[i].getRotation().getX();
                        y += transList[i].getRotation().getY();
                        z += transList[i].getRotation().getZ();
                        w += transList[i].getRotation().getW();
                    }

                    x = x/transList.size();
                    y = y/transList.size();
                    z = z/transList.size();
                    w = w/transList.size();

                    tf::Quaternion averageQ(x,y,z,w);
                    averageQ.normalize();
                    NozzleTransform.setRotation(averageQ);
                    NozzleTransform.setOrigin(avOrigin);
                    NozzleTransform.child_frame_id_ = "nozzle";
                    br->sendTransform(NozzleTransform);


                }
            }
        }
        catch (...){

        }

        ros::spinOnce();
        rate.sleep();

    }

}

tf::Quaternion InverseSignQuaternion(tf::Quaternion q){
    tf::Quaternion res(-q.getX(), -q.getY(), -q.getZ(), -q.getW());
    return res;
}


bool AreQuaternionsClose(tf::Quaternion q1, tf::Quaternion q2){

    float dot = q1.dot(q2);

    if(dot < 0.0f){

        return false;
    }

    else{

        return true;
    }
}
