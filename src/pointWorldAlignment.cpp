#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <sys/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <hololens_experiment/CommonPoints.h>
float findTransform(std::vector<std::pair<cv::Mat, cv::Mat>> CP, cv::Mat &R,
                    cv::Mat &t) {
  cv::Mat ac = cv::Mat(3, 1, CV_32F, cv::Scalar(0));
  cv::Mat bc = cv::Mat(3, 1, CV_32F, cv::Scalar(0));
  int samples = CP.size();
  for (int i = 0; i < samples; i++) {
    ac += CP.at(i).first;
    // std::cout << "first" << i << " " << CP.at(i).first << std::endl;
    // std::cout << "second" << i << " " << CP.at(i).second << std::endl;
    bc += CP.at(i).second;
  }
  ac /= samples;
  bc /= samples;

  cv::Mat H = cv::Mat(3, 3, CV_32F, cv::Scalar(0));

  for (int i = 0; i < samples; i++) {
    cv::Mat bT;
    cv::transpose(CP.at(i).second - bc, bT);
    H += (CP.at(i).first - ac) * (bT);
  }

  cv::SVD svd(H);
  cv::Mat v;
  cv::transpose(svd.vt, v);
  cv::Mat ut;
  cv::transpose(svd.u, ut);
  R = v * ut;
  // std::cout << cv::determinant(v)<< std::endl;
  t = -R * ac + bc;
  float val = 0;
  for (int i = 0; i < samples; i++) {
    cv::Mat thePoint = (R * CP.at(i).first) + t;
    cv::Mat er = thePoint - CP.at(i).second;
    float thisVal = sqrt(pow(er.at<float>(0), 2) + pow(er.at<float>(1), 2) +
                         pow(er.at<float>(2), 2));
    // std::cout<< i << "  :" << thePoint << std::endl;
    val += thisVal;
  }
  val /= samples;
  // std::cout <<"averageVal :"<< val << std::endl;
  return val;
}
//cv::Mat ac = cv::Mat(3, 1, CV_32F, cv::Scalar(0));
std::vector<cv::Mat> hololensVector;
ros::Time hololensTriangleTime;
void hololensTriangleCB(hololens_experiment::CommonPointsConstPtr msg){
    hololensVector.clear();
    hololensTriangleTime = msg->stamp;
    cv::Mat pointMat = cv::Mat(3, 1, CV_32F, cv::Scalar(0));
    pointMat.at<float>(0) = msg->p1.x;
    pointMat.at<float>(1) = msg->p1.y;
    pointMat.at<float>(2) = msg->p1.z;
    hololensVector.push_back(pointMat);

    pointMat.at<float>(0) = msg->p2.x;
    pointMat.at<float>(1) = msg->p2.y;
    pointMat.at<float>(2) = msg->p2.z;
    hololensVector.push_back(pointMat);

    pointMat.at<float>(0) = msg->p3.x;
    pointMat.at<float>(1) = msg->p3.y;
    pointMat.at<float>(2) = msg->p3.z;
    hololensVector.push_back(pointMat);
}
std::vector<cv::Mat> ROSVector;
ros::Time ROSTriangleTime;
void ROSTriangleCB(hololens_experiment::CommonPointsConstPtr msg){
    ROSVector.clear();
    ROSTriangleTime = msg->stamp;
    cv::Mat pointMat = cv::Mat(3, 1, CV_32F, cv::Scalar(0));
    pointMat.at<float>(0) = msg->p1.x;
    pointMat.at<float>(1) = msg->p1.y;
    pointMat.at<float>(2) = msg->p1.z;
    hololensVector.push_back(pointMat);

    pointMat.at<float>(0) = msg->p2.x;
    pointMat.at<float>(1) = msg->p2.y;
    pointMat.at<float>(2) = msg->p2.z;
    hololensVector.push_back(pointMat);

    pointMat.at<float>(0) = msg->p3.x;
    pointMat.at<float>(1) = msg->p3.y;
    pointMat.at<float>(2) = msg->p3.z;
    ROSVector.push_back(pointMat);
}


//node that subscribes to 2 triangles and returns the corresponding transform (least squares)
int main(int argc, char **argv) {
    /// end of calibrated values.
    ros::init(argc, argv, "triangle transform node");
    if (argc < 3) {
      std::cout << "Usage: Ros triangle topic , hololens triangel topic"
                << std::endl;
      return -1;
    }

    ros::NodeHandle node;

    ros::Subscriber hololensTriangleSub = node.subscribe("/hololens/commonPoints",1,&hololensTriangleCB);
    ros::Subscriber ROSTriangleSub = node.subscribe("/hololens_experiment/commonPoints",1,&ROSTriangleCB);
    hololensTriangleTime = ros::Time::now();
    tf::TransformBroadcaster br;

    while (ros::ok()) {
        if(hololensTriangleTime == ROSTriangleTime){
            std::vector<std::pair<cv::Mat, cv::Mat>> listOfPairs;
            for(int i = 0; i < hololensVector.size(); i++ ){
                std::pair<cv::Mat,cv::Mat> thisPair;
                thisPair.first = hololensVector[i];
                thisPair.second = ROSVector[i];
                listOfPairs.push_back(thisPair);
            }
            cv::Mat R, t;
            float error  = findTransform(listOfPairs,R,t);
            ROS_INFO("Error in matching worlds: %f", error);

            tf::Transform trans;
            trans.setOrigin(tf::Vector3(1.0 * t.at<float>(0) / 1000.0,
                                      t.at<float>(1) / 1000.0,
                                      t.at<float>(2) / 1000.0));
            tf::Matrix3x3 rotMat(
                R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2),
                R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2),
                R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2));
            tf::Quaternion quat;
            rotMat.getRotation(quat);
            // quat = quat.inverse();
            trans.setRotation(quat);
            // man.setOrigin(-man.getOrigin());
            br.sendTransform(
                tf::StampedTransform(trans, ROSTriangleTime, "holoWorld", "rosWorld"));
        }
    ros::spinOnce();
    }
}
