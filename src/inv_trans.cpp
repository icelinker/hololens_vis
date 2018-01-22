#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>




int main(int argc, char** argv){
    ros::init(argc, argv, "holoLink");

    ros::NodeHandle node;
    tf::TransformBroadcaster br;
    tf::TransformListener tl;

    std::string inParent, inChild, outParent, outChild;

    ros::Rate loopRate(1000);


    if(argc != 5){
        std::cout << "you need to have 4 arguments, parent, child, newParentName, newChildName" << std::endl;
        ROS_INFO("not correct arguments");
        return -1;
    }
    inParent = argv[1];
    inChild = argv[2];
    outParent = argv[3];
    outChild = argv[4];
    ros::Time oldStamp;
    //forever find the transform and publish the invers one.
    while(ros::ok()){
        try{
            tf::StampedTransform transform;
            tf::StampedTransform invTrans;
            tl.lookupTransform(inParent,inChild,ros::Time(0),transform);
            if(transform.stamp_ != oldStamp){
                oldStamp = transform.stamp_;
                invTrans.child_frame_id_ = outChild;
                invTrans.frame_id_ = outParent;
                invTrans.stamp_ = transform.stamp_;
                invTrans.setData(transform.inverse());
                br.sendTransform(invTrans);
            }
        }
        catch(...){
            ROS_INFO("error in finding transforms");
        }
        loopRate.sleep();
        ros::spinOnce();
    }

}
