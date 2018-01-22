#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

int ready = 0;

void clickCB(std_msgs::StringConstPtr msg){
    ready = 1;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "calMarkersCam");

    ros::NodeHandle node;
    tf::TransformBroadcaster br;
    tf::TransformListener tl;
    ros::Subscriber clickSub = node.subscribe("/holoNext",1,clickCB);

    ros::Rate loopRate(100);


    //forever find the transform and publish the invers one.
    while(ros::ok()){
        try{
            tf::StampedTransform transform, transformOut;
            tf::Transform camPos;
            camPos.setOrigin(tf::Vector3(0,0,0.28));
            camPos.setRotation(tf::Quaternion(3.14,3.14/2,0));
            tf::StampedTransform invTrans;
            tl.lookupTransform("marker","camera_inv",ros::Time(0),transform);
            transformOut.mult(transform, camPos);
           //transformOut = transformOut.inverse();
            if(ready){
                while(1){
                    transformOut.frame_id_ = "marker";
                    transformOut.child_frame_id_ = "holoCentre";
                    transformOut.stamp_ = ros::Time::now();
                    br.sendTransform(transformOut);
                    tf::Transform printing;
                    printing = transformOut.inverse();
                    std::cout<< printing.getOrigin().getX() ;
                    std::cout << " " << printing.getOrigin().getY();
                    std::cout << " " << printing.getOrigin().getZ() ;
                    std::cout << " " << printing.getRotation().getX() ;
                    std::cout << " " << printing.getRotation().getY();
                    std::cout << " " << printing.getRotation().getZ() ;
                    std::cout << " " << printing.getRotation().getW() << std::endl;
                }

            }
        }
        catch(...){
            ROS_INFO("error in finding transforms");
        }
        loopRate.sleep();
        ros::spinOnce();
    }

}
