#include <iostream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

void poseCB(const geometry_msgs::PoseConstPtr &msg);

int main(int argc, char** argv){
    ros::init(argc, argv, "holoLink");

    ros::NodeHandle node;

    ros::Subscriber poseSub = node.subscribe("/holoPose",1,&poseCB);

    while(ros::ok()){
        ros::spinOnce();
    }




}
//tf::Transform transform;
//transform.setOrigin(tf::Vector3(pos.x, -pos.z, pos.y));

//tf::Quaternion q(orient.i, -orient.k, orient.j, -1.0 * orient.w);
//q = q.inverse();


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

    static tf::TransformBroadcaster br;
    transform.child_frame_id_ = "holoLens";
    transform.frame_id_ = "mocha_world";
    br.sendTransform(transform);
}
