
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
#include <vector>
#include <numeric>
#include <std_msgs/Float32.h>


void holoPingOut(ros::Publisher pub);
void holoLagOut(ros::Publisher pub);
void holoPingRespCB(const std_msgs::TimeConstPtr &msg);
float averageVec(std::vector<float> in);
float devVec(std::vector<float> in, float av);
ros::Time lastPingTime;
std::vector<float> lags;
int history = 100;
float theLag = 0;
int main(int argc, char** argv){
    ros::init(argc, argv, "timeSync");
    ros::NodeHandle node;
    ros::Publisher timePub= node.advertise<std_msgs::Time> ("/pingOut",1);
    ros::Publisher lagPub= node.advertise<std_msgs::Float32> ("/lagOut",1);
    ros::Subscriber holoPingSub = node.subscribe("/holoPing",1,&holoPingRespCB);
    lastPingTime = ros::Time::now();
    ros::Duration timeSinceLastPing;
    while(ros::ok()){
        ros::Duration timeSinceLastPing = ros::Time::now() - lastPingTime;
        if(timeSinceLastPing >ros::Duration(0.1)){
            holoPingOut(timePub);
            if(lags.size() > 10){
                holoLagOut(lagPub);
            }
        }
        ros::spinOnce();
    }

}

void holoPingOut(ros::Publisher pub){
    std_msgs::Time msg;
    msg.data = ros::Time::now();
    lastPingTime = msg.data;
    pub.publish(msg);
}
void holoPingRespCB(const std_msgs::TimeConstPtr &msg){
    ros::Duration lag  = (lastPingTime - msg->data);
    static int vecPoint = 0;
    //check that we are looking at the right packet.
    if(lastPingTime.nsec == msg->data.nsec && lastPingTime.sec == msg->data.sec){
        ros::Duration lag  = (ros::Time::now() - msg->data);
        float fLag = lag.toSec()/2;
        if(lags.size() < history){
            lags.push_back(fLag);
        }
        else{
            lags.at(vecPoint%history) = fLag;
            vecPoint++;
        }
        std::vector<float> orderedLags = lags;
        std::sort (orderedLags.begin(), orderedLags.end());
        float mean = averageVec(lags);
        float stdDev = devVec(lags,mean);
        std::cout << "mean:"<<mean<<"  StdDev:"<< stdDev << std::endl;
        int low = -1;
        int  high = -1;
        for(int i = 0 ; i < orderedLags.size(); i++){
            if(orderedLags[i] > mean-stdDev && low == -1) low = i;
            if(orderedLags[i] > mean+stdDev && high == -1) high = i-1;
        }
        int mid = (high - low)/2;
        theLag = orderedLags[mid];
        std::cout << theLag << std::endl;
    }

}

float averageVec(std::vector<float> in){
    float average = accumulate( in.begin(), in.end(), 0.0)/in.size();
    return average;
}
float devVec(std::vector<float> in, float av){
    float var = 0;
    for( int n = 0; n < in.size(); n++ )
    {
      var += (in[n] - av) * (in[n] - av);
    }
    var /= in.size();
    float sd = sqrt(var);
    return sd;
}

void holoLagOut(ros::Publisher pub){
    std_msgs::Float32 msg;
    msg.data = theLag;
    pub.publish(msg);

}
