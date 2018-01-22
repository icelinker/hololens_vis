#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

void texCB(const sensor_msgs::ImageConstPtr& msg);
void nozCB(const sensor_msgs::ImageConstPtr& msg);
void pathCB(const sensor_msgs::ImageConstPtr& msg);
void mouseCB(int event, int x, int y, int flags, void* userdata);
void formDisplay(cv::Mat im1, cv::Mat im2, cv::Mat im3, std::__cxx11::string currMode, std::__cxx11::string currRound, int targetH, int targetW);
cv::Mat tex, noz, path;
cv::Mat fb;
std::string strMode, strRound;
int winH = 800;
int winW = 1200;
ros::Publisher nextPub;
ros::Publisher calibratePub;
std::string type2str(int type);

int main(int argc, char** argv){
    ros::init(argc, argv, "exp_control");
    ros::NodeHandle node;
    image_transport::ImageTransport it_(node);

    image_transport::Subscriber texSub = it_.subscribe("/texIm",1, &texCB);
    image_transport::Subscriber nozSub = it_.subscribe("/nozView",1, &nozCB);
    image_transport::Subscriber pathSub = it_.subscribe("/pathView",1, &pathCB);

    nextPub = node.advertise<std_msgs::String>("/holoNext",1);
    calibratePub = node.advertise<std_msgs::String>("/reCalibrate",1);
    fb = cv::Mat(winH,winW,CV_8UC3,cv::Scalar(0,0,0));
    cv::namedWindow( "Experiment Control", cv::WINDOW_AUTOSIZE );
    cv::setMouseCallback("Experiment Control", mouseCB, NULL);
    ros::Rate rate(30);
    while(ros::ok()){
        if(tex.cols==0) tex = cv::Mat(100,100,CV_8UC3,cv::Scalar(20,0,0));
        if(noz.cols==0) noz = cv::Mat(100,200,CV_8UC3,cv::Scalar(0,20,0));
        if(path.cols==0) path = cv::Mat(200,100,CV_8UC3,cv::Scalar(0,0,20));
        if(strMode.length() ==0) strMode = "No mode recieved";
        if(strRound.length() ==0) strRound = "No round recieved";
        cv::imshow("path before Call",path);
        formDisplay(tex,noz,path,strMode,strRound,winH,winW);
        cv::waitKey(1);
        ros::spinOnce();
        rate.sleep();
    }

}
//this will concatonate the three images and have the 4th section with buttons
// 1 and 2 form the width and 1 and 3 form the height.
void formDisplay(cv::Mat im1, cv::Mat im2, cv::Mat im3, std::string currMode, std::string currRound, int targetH, int targetW){
    int currWidth = im1.cols;
    int currHeight = im1.rows;
    int tW,tH;

    cv::imshow("2 in",im2);
    cv::waitKey(1);
    currWidth = im2.cols;
    currHeight = im2.rows;
    cv::Mat tr;
    tW = targetW/2;
    tH = targetH/2;
    cv::Mat tl,dst_roi,dst_roi2,dst_roi3;
    // in this case the H is dominating and should be scaled to fit this.
    if(float(currHeight)/tH > float(currWidth)/tW){
        cv::resize(im2,tr,cv::Size(0,0),float(tH)/currHeight, float(tH)/currHeight);
        dst_roi2 = fb(cv::Rect(tW+(tW-tr.cols)/2, 0, tr.cols, tr.rows));
    }
    else{
        cv::resize(im2,tr,cv::Size(0,0),float(tW)/currWidth, float(tW)/currWidth);
        dst_roi2 = fb(cv::Rect(tW,(tH-tr.rows)/2, tr.cols, tr.rows));
    }
    tr.copyTo(dst_roi2);
    cv::imshow("tr",tr);
    cv::waitKey(1);


    // in this case the H is dominating and should be scaled to fit this.
    if(float(currHeight)/tH > float(currWidth)/tW){
        cv::resize(im1,tl,cv::Size(0,0),float(tH)/currHeight, float(tH)/currHeight);
        dst_roi = fb(cv::Rect((tW-tl.cols)/2, 0, tl.cols, tl.rows));
    }
    else{
        cv::resize(im1,tl,cv::Size(0,0),float(tH)/currWidth, float(tW)/currWidth);
        dst_roi = fb(cv::Rect(0, (tH-tl.rows)/2, tl.cols, tl.rows));
    }
    tl.copyTo(dst_roi);


    cv::imshow("3in",im3);
    cv::waitKey(1);
    currWidth = im3.cols;
    currHeight = im3.rows;
    cv::Mat bl;
   // in this case the H is dominating and should be scaled to fit this.
    if(float(currHeight)/tH > float(currWidth)/tW){
        cv::resize(im3,bl,cv::Size(0,0),float(tH)/currHeight, float(tH)/currHeight);
        dst_roi3 = fb(cv::Rect((tW- bl.cols)/2, tH, bl.cols, bl.rows));
    }
    else{
        cv::resize(im3,bl,cv::Size(0,0),float(tW)/currWidth, float(tW)/currWidth);
        dst_roi3 = fb(cv::Rect(0, tH+(tH-bl.rows)/2, bl.cols, bl.rows));
    }
    bl.copyTo(dst_roi3);
    cv::imshow("bl",bl);
    cv::waitKey(1);


    dst_roi = fb(cv::Rect(tW, tH, tW/2, tH/2));
    dst_roi.setTo(cv::Scalar(100,100,100));
    dst_roi = fb(cv::Rect(1.5*tW, tH, tW/2, tH/2));
    dst_roi.setTo(cv::Scalar(130,130,130));
    dst_roi = fb(cv::Rect(tW, 1.5*tH, tW/2, tH/2));
    dst_roi.setTo(cv::Scalar(150,150,150));
    dst_roi = fb(cv::Rect(1.5*tW, 1.5*tH, tW/2, tH/2));
    dst_roi.setTo(cv::Scalar(180,180,180));
    putText(fb, "Next Stage", cvPoint(tW+30, tH+30),
        cv::FONT_HERSHEY_COMPLEX_SMALL, 1.6, cvScalar(50,20,250), 1, CV_AA);
    putText(fb, "Repeat Stage", cvPoint(1.5*tW+30, tH+30),
        cv::FONT_HERSHEY_COMPLEX_SMALL, 1.6, cvScalar(50,20,250), 1, CV_AA);
    putText(fb, "Spare Button", cvPoint(tW+30, 1.5*tH+30),
        cv::FONT_HERSHEY_COMPLEX_SMALL, 1.6, cvScalar(50,20,250), 1, CV_AA);
    putText(fb, "calibrate", cvPoint(1.5*tW+30, 1.5*tH+30),
        cv::FONT_HERSHEY_COMPLEX_SMALL, 1.6, cvScalar(50,20,250), 1, CV_AA);
    cv::imshow("Experiment Control",fb);

    cv::waitKey(1);
}

std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

void mouseCB(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
          if(x > 0.75*winW && y > 0.75*winH){//bottom corner clicked
              std::cout << "Calibrate" << std::endl;
              std_msgs::String msg;
              msg.data = "Calibrate(manual)";
              calibratePub.publish(msg);
              return;
          }
          if(x > 0.75*winW && y > 0.5*winH){
              std::cout << "Repeat Stage" << std::endl;
              std_msgs::String msg;
              msg.data = "clear";
              nextPub.publish(msg);
              return;
          }
          if(x > 0.5*winW && y > 0.75*winH){
              std::cout << "spare" << std::endl;
              return;
          }
          if(x > 0.5*winW && y > 0.5*winH){
              std::cout << "Next Stage" << std::endl;
              std_msgs::String msg;
              msg.data = "next";
              nextPub.publish(msg);
              return;
          }


     }

}

void texCB(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat temp = cv_bridge::toCvShare(msg, "bgra8")->image;
    if(temp.cols != 0 && temp.rows !=0){
        cv::cvtColor(cv_bridge::toCvShare(msg, "bgra8")->image,tex,cv::COLOR_BGRA2BGR);

    }

}

void nozCB(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat temp = cv_bridge::toCvShare(msg, "bgr8")->image;
    if(temp.cols != 0 && temp.rows !=0){
        cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(noz);

    }

}

void pathCB(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat temp = cv_bridge::toCvShare(msg, "bgr8")->image;
    if(temp.cols != 0 && temp.rows !=0){
        cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(path);
        cv::imshow("pathNew", path);

    }

}
