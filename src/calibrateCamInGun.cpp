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
            tf::Transform invTrans;
            camPos.setRotation(tf::Quaternion(3.142/2,3.142/2,0));
            camPos = camPos.inverse();
            tl.lookupTransform("holoLens","camera_inv",ros::Time(0),transform);
            transform.mult(camPos ,transform);
            invTrans = transform.inverse();
            invTrans.setOrigin(tf::Vector3(-0.02,0.03,-0.075));
            transformOut.setOrigin(invTrans.getOrigin());
            transformOut.setRotation(invTrans.getRotation());
            //transformOut.mult(transform,camPos );
           //transformOut = transformOut.inverse();
            if(ready){
                while(1){
                    transformOut.frame_id_ = "camera_inv";
                    transformOut.child_frame_id_ = "nozProto";
                    transformOut.stamp_ = ros::Time::now();
                    br.sendTransform(transformOut);
                    std::cout<< transformOut.getOrigin().getX() ;
                    std::cout << " " << transformOut.getOrigin().getY();
                    std::cout << " " << transformOut.getOrigin().getZ() ;
                    std::cout << " " << transformOut.getRotation().getX() ;
                    std::cout << " " << transformOut.getRotation().getY();
                    std::cout << " " << transformOut.getRotation().getZ() ;
                    std::cout << " " << transformOut.getRotation().getW() << std::endl;
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
