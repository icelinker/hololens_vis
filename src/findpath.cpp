////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// STL A* Search implementation
// (C)2001 Justin Heyes-Jones
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "stlastar.h" // See header for copyright and usage information

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <chrono>  // for high_resolution_clock

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0

using namespace std;

// Global data

// The world map

int MAP_WIDTH = 400;
int MAP_HEIGHT = 400;
int initx = 0;
int inity = 0;
int endx = 0;
int endy = 0;
int scale = 2;
float* world_map;//=
//{

//// 0001020304050607080910111213141516171819
//	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,   // 00
//	1,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,1,   // 01
//	1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 02
//	1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 03
//	1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,   // 04
//	1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,   // 05
//	1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,   // 06
//	1,9,9,9,9,9,9,9,9,1,1,1,9,9,9,9,9,9,9,1,   // 07
//	1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,   // 08
//	1,9,1,9,9,9,9,9,9,9,1,1,9,9,9,9,9,9,9,1,   // 09
//	1,9,1,1,1,1,9,1,1,9,1,1,1,1,1,1,1,1,1,1,   // 10
//	1,9,9,9,9,9,1,9,1,9,1,9,9,9,9,9,1,1,1,1,   // 11
//	1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 12
//	1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 13
//	1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,   // 14
//	1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,   // 15
//	1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,   // 16
//	1,1,9,9,9,9,9,9,9,1,1,1,9,9,9,1,9,9,9,9,   // 17
//	1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,   // 18
//	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,   // 19

//};

// map helper functions

float GetMap( int x, int y )
{
    if( x < 0 ||
            x >= MAP_WIDTH ||
            y < 0 ||
            y >= MAP_HEIGHT
            )
    {
        return 1.0;
    }

    return world_map[(y*MAP_WIDTH)+x];
}

//white = navigable
//black = blocked
//red = start
//green = end

float* map;
bool imageProcessed = true;
int loadMap (std::string imageFile, float *&map, int &width, int &height){
    cv::Mat image = cv::imread(imageFile, 1);
     cv::resize(image,image,cv::Size(0,0),1.0/scale,1.0/scale,cv::INTER_NEAREST);
     cv::imshow("receivedIm",image);
    MAP_WIDTH = image.cols;
    MAP_HEIGHT = image.rows;
    width = MAP_WIDTH;
    height = MAP_HEIGHT;
    cv::Mat mapMat(width,height, CV_32F, 1.0);
    cv::Mat goodLand(width,height, CV_32F, 0.0);
    cv::Mat badLand(width,height, CV_32F, 0.0);
    cv::Mat unPassable(width,height, CV_32F, 0.0);
    if(world_map == NULL ){
       world_map = new float[width * height];
    }

    for(int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
            //start
            if(image.at<cv::Vec3b>(i,j) == cv::Vec3b(0,0,255)){
                mapMat.at<float>(i,j) = 0.001;
                world_map[width*i + j] = 1.0;
                goodLand.at<float>(i,j) =1;
                initx = j;
                inity = i;
            }

            else if(image.at<cv::Vec3b>(i,j) == cv::Vec3b(0,255,0)){
                mapMat.at<float>(i,j) = 0.1;
                world_map[width*i + j] = 1.0;
                goodLand.at<float>(i,j) =1;
                endx = j;
                endy = i;
            }
            else if(image.at<cv::Vec3b>(i,j) == cv::Vec3b(255,255,255)){
                mapMat.at<float>(i,j) = 1.0;
                unPassable.at<float>(i,j) =1;
                world_map[width*i + j] = 9;
            }
            else if(image.at<cv::Vec3b>(i,j) == cv::Vec3b(0,0,0)){
                mapMat.at<float>(i,j) = 0.99;
                badLand.at<float>(i,j) =1;
                world_map[width*i + j] = 8.8;
            }
            else if(image.at<cv::Vec3b>(i,j) == cv::Vec3b(0,51,51)){
                goodLand.at<float>(i,j) =1;
                mapMat.at<float>(i,j) = 0.001;
                world_map[width*i + j] = 1.0;
            }
            else {
                mapMat.at<float>(i,j) = 0.001;
                world_map[width*i + j] = 1.0;
            }
        }
    }
    //cv::GaussianBlur( mapMat, mapMat,cv::Size(5, 5), 0, 0 );
//    for(int i = 0; i < height; i++){
//        for(int j = 0; j < width; j++){
//            world_map[width*i + j] = mapMat.at<float>(i,j);
//        }
//    }
    int dilation_size = 1;
    int dilation_sizeBig = 3;

    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                           cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                           cv::Point( dilation_size, dilation_size ) );
    cv::Mat elementBig = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                           cv::Size( 2*dilation_sizeBig + 1, 2*dilation_sizeBig+1 ),
                                           cv::Point( dilation_sizeBig, dilation_sizeBig ) );
    cv::dilate(unPassable,unPassable,elementBig);
    cv::erode(unPassable,unPassable,elementBig);
    cv::dilate(goodLand,goodLand,element);
    cv::erode(goodLand,goodLand,element);
    cv::dilate(badLand,badLand,element);
    cv::erode(badLand,badLand,element);

    for(int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
            if (goodLand.at<float>(i,j) == 1) world_map[width*i + j] = 0.001;
            if (badLand.at<float>(i,j) == 1) world_map[width*i + j] = 0.99;
            if (unPassable.at<float>(i,j) == 1) world_map[width*i + j] = 1;

        }
    }
        for(int i = 0; i < height; i++){
            for(int j = 0; j < width; j++){
                mapMat.at<float>(i,j)= world_map[width*i + j];
            }
        }

    cv::imshow("costs",mapMat);
    cv::imshow("unpassable",unPassable);
    cv::imshow("good",goodLand);
    cv::imshow("bad",badLand);
    cv::waitKey(1);
    imageProcessed = false;

}



// Definitions

class MapSearchNode
{
public:
    int x;	 // the (x,y) positions of the node
    int y;

    MapSearchNode() { x = y = 0; }
    MapSearchNode( int px, int py ) { x=px; y=py; }

    float GoalDistanceEstimate( MapSearchNode &nodeGoal );
    bool IsGoal( MapSearchNode &nodeGoal );
    bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
    float GetCost( MapSearchNode &successor );
    bool IsSameState( MapSearchNode &rhs );

    void PrintNodeInfo();


};

bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{

    // same state in a maze search is simply when (x,y) are the same
    if( (x == rhs.x) &&
            (y == rhs.y) )
    {
        return true;
    }
    else
    {
        return false;
    }

}

void MapSearchNode::PrintNodeInfo()
{
    char str[100];
    sprintf( str, "Node position : (%d,%d)\n", x,y );

    cout << str;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal. 

float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
    //return 0.0004*(fabsf(x - nodeGoal.x) + fabsf(y - nodeGoal.y));
    return  0.001*sqrt(pow((x - nodeGoal.x),2) + pow((y - nodeGoal.y),2));
}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{

    if( (x == nodeGoal.x) &&
            (y == nodeGoal.y) )
    {
        return true;
    }

    return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
{

    int parent_x = -1;
    int parent_y = -1;

    if( parent_node )
    {
        parent_x = parent_node->x;
        parent_y = parent_node->y;
    }


    MapSearchNode NewNode;

    // push each possible move except allowing the search to go backwards

    if( (GetMap( x-1, y ) < 1)
            && !((parent_x == x-1) && (parent_y == y))
            )
    {
        NewNode = MapSearchNode( x-1, y );
        astarsearch->AddSuccessor( NewNode );
    }

    if( (GetMap( x, y-1 ) < 1)
            && !((parent_x == x) && (parent_y == y-1))
            )
    {
        NewNode = MapSearchNode( x, y-1 );
        astarsearch->AddSuccessor( NewNode );
    }

    if( (GetMap( x+1, y ) < 1)
            && !((parent_x == x+1) && (parent_y == y))
            )
    {
        NewNode = MapSearchNode( x+1, y );
        astarsearch->AddSuccessor( NewNode );
    }


    if( (GetMap( x, y+1 ) < 1)
            && !((parent_x == x) && (parent_y == y+1))
            )
    {
        NewNode = MapSearchNode( x, y+1 );
        astarsearch->AddSuccessor( NewNode );
    }

    //adding diagonals
    if( (GetMap( x-1, y-1 ) < 1)
            && !((parent_x == x-1) && (parent_y == y-1))
            )
    {
        NewNode = MapSearchNode( x-1, y-1 );
        astarsearch->AddSuccessor( NewNode );
    }

    if( (GetMap( x+1, y-1 ) < 1)
            && !((parent_x == x+1) && (parent_y == y-1))
            )
    {
        NewNode = MapSearchNode( x+1, y-1 );
        astarsearch->AddSuccessor( NewNode );
    }

    if( (GetMap( x+1, y+1 ) < 1)
            && !((parent_x == x+1) && (parent_y == y+1))
            )
    {
        NewNode = MapSearchNode( x+1, y+1 );
        astarsearch->AddSuccessor( NewNode );
    }


    if( (GetMap( x-1, y+1 ) < 1)
            && !((parent_x == x-1) && (parent_y == y+1))
            )
    {
        NewNode = MapSearchNode( x-1, y+1 );
        astarsearch->AddSuccessor( NewNode );
    }
    // adding knight moves.
//    if( (GetMap( x-2, y-1 ) < 9)
//            && !((parent_x == x-2) && (parent_y == y-1))
//            )
//    {
//        NewNode = MapSearchNode( x-2, y-1 );
//        astarsearch->AddSuccessor( NewNode );
//    }

//    if( (GetMap( x+2, y-1 ) < 9)
//            && !((parent_x == x+2) && (parent_y == y-1))
//            )
//    {
//        NewNode = MapSearchNode( x+2, y-1 );
//        astarsearch->AddSuccessor( NewNode );
//    }

//    if( (GetMap( x+2, y+1 ) < 9)
//            && !((parent_x == x+2) && (parent_y == y+1))
//            )
//    {
//        NewNode = MapSearchNode( x+2, y+1 );
//        astarsearch->AddSuccessor( NewNode );
//    }


//    if( (GetMap( x-2, y+1 ) < 9)
//            && !((parent_x == x-2) && (parent_y == y+1))
//            )
//    {
//        NewNode = MapSearchNode( x-2, y+1 );
//        astarsearch->AddSuccessor( NewNode );
//    }
//    if( (GetMap( x-1, y-2 ) < 9)
//            && !((parent_x == x-1) && (parent_y == y-2))
//            )
//    {
//        NewNode = MapSearchNode( x-1, y-2 );
//        astarsearch->AddSuccessor( NewNode );
//    }

//    if( (GetMap( x+1, y-2 ) < 9)
//            && !((parent_x == x+1) && (parent_y == y-2))
//            )
//    {
//        NewNode = MapSearchNode( x+1, y-2 );
//        astarsearch->AddSuccessor( NewNode );
//    }

//    if( (GetMap( x+1, y+2 ) < 9)
//            && !((parent_x == x+1) && (parent_y == y+2))
//            )
//    {
//        NewNode = MapSearchNode( x+1, y+2 );
//        astarsearch->AddSuccessor( NewNode );
//    }


//    if( (GetMap( x-1, y+2 ) < 9)
//            && !((parent_x == x-1) && (parent_y == y+2))
//            )
//    {
//        NewNode = MapSearchNode( x-1, y+2 );
//        astarsearch->AddSuccessor( NewNode );
//    }

    return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving

float MapSearchNode::GetCost( MapSearchNode &successor )
{

    int xChange = fabs(this->x - successor.x);
    int yChange = fabs(this->y - successor.y);
    if(xChange + yChange ==2) {
        return (1.414) *GetMap( x, y );
    }
    if(xChange + yChange ==3) {
        return (2.24)*GetMap( x, y );
    }
    return 1.0*GetMap( x, y );
    //return fabs(this->x - successor.x) + fabs(this->y - successor.y);

    //return (float) GetMap( x, y );

}


// Main

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
     cv::Mat image =  cv_bridge::toCvShare(msg, "bgr8")->image;
     cv::resize(image,image,cv::Size(0,0),1.0/scale,1.0/scale,cv::INTER_NEAREST);
     cv::imshow("receivedIm",image);
    MAP_WIDTH = image.cols;
    MAP_HEIGHT = image.rows;
    int width = MAP_WIDTH;
    int height = MAP_HEIGHT;
    cv::Mat mapMat(width,height, CV_32F, 1.0);
    cv::Mat goodLand(width,height, CV_32F, 0.0);
    cv::Mat badLand(width,height, CV_32F, 0.0);
    cv::Mat unPassable(width,height, CV_32F, 0.0);
    if(world_map == NULL ){
       world_map = new float[width * height];
    }

    for(int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
            //start
            if(image.at<cv::Vec3b>(i,j) == cv::Vec3b(0,0,255)){
                mapMat.at<float>(i,j) = 0.001;
                world_map[width*i + j] = 1.0;
                goodLand.at<float>(i,j) =1;
                initx = j;
                inity = i;
            }

            else if(image.at<cv::Vec3b>(i,j) == cv::Vec3b(0,255,0)){
                mapMat.at<float>(i,j) = 0.1;
                world_map[width*i + j] = 1.0;
                goodLand.at<float>(i,j) =1;
                endx = j;
                endy = i;
            }
            else if(image.at<cv::Vec3b>(i,j) == cv::Vec3b(255,255,255)){
                mapMat.at<float>(i,j) = 1.0;
                unPassable.at<float>(i,j) =1;
                world_map[width*i + j] = 9;
            }
            else if(image.at<cv::Vec3b>(i,j) == cv::Vec3b(0,0,0)){
                mapMat.at<float>(i,j) = 0.99;
                badLand.at<float>(i,j) =1;
                world_map[width*i + j] = 8.8;
            }
            else if(image.at<cv::Vec3b>(i,j) == cv::Vec3b(0,51,51)){
                goodLand.at<float>(i,j) =1;
                mapMat.at<float>(i,j) = 0.001;
                world_map[width*i + j] = 1.0;
            }
            else {
                mapMat.at<float>(i,j) = 0.001;
                world_map[width*i + j] = 1.0;
            }
        }
    }
    //cv::GaussianBlur( mapMat, mapMat,cv::Size(5, 5), 0, 0 );
//    for(int i = 0; i < height; i++){
//        for(int j = 0; j < width; j++){
//            world_map[width*i + j] = mapMat.at<float>(i,j);
//        }
//    }
    int dilation_size = 1;
    int dilation_sizeBig = 3;

    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                           cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                           cv::Point( dilation_size, dilation_size ) );
    cv::Mat elementBig = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                           cv::Size( 2*dilation_sizeBig + 1, 2*dilation_sizeBig+1 ),
                                           cv::Point( dilation_sizeBig, dilation_sizeBig ) );
    cv::dilate(unPassable,unPassable,element);
    cv::erode(unPassable,unPassable,element);
//    cv::erode(unPassable,unPassable,element);
//    cv::dilate(unPassable,unPassable,elementBig);
    cv::dilate(goodLand,goodLand,element);
    cv::erode(goodLand,goodLand,element);
     cv::erode(goodLand,goodLand,element);
     cv::dilate(goodLand,goodLand,element);
        cv::Mat margin;
    cv::dilate(badLand,margin,elementBig);
    cv::dilate(badLand,badLand,element);
   // badLand.copyTo(margin);
    cv::erode(badLand,badLand,element);
    cv::erode(badLand,badLand,element);
    cv::dilate(badLand,badLand,element);

    for(int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
            if (goodLand.at<float>(i,j) == 1) world_map[width*i + j] = 0.001;
            if(margin.at<float>(i,j)==1) world_map[width*i + j] = 0.003;
            if (badLand.at<float>(i,j) == 1) world_map[width*i + j] = 0.99;
            if (unPassable.at<float>(i,j) == 1) world_map[width*i + j] = 1;

        }
    }
        for(int i = 0; i < height; i++){
            for(int j = 0; j < width; j++){
                mapMat.at<float>(i,j)= world_map[width*i + j];
            }
        }

    cv::imshow("costs",mapMat);
    cv::imshow("unpassable",unPassable);
    cv::imshow("good",goodLand);
    cv::imshow("bad",badLand);
    cv::waitKey(1);
    imageProcessed = false;
}

int main( int argc, char *argv[] )
{

    ros::init(argc, argv, "findPath");
    //do ros subs
    ros::NodeHandle node;
    image_transport::ImageTransport it_(node);
    image_transport::Subscriber imSub = it_.subscribe("/mapImRaw",1, imageCallback);
    image_transport::Publisher imPub = it_.advertise("/solvedMap",1);
    cv::Mat resultMat;
    //main loop
    ros::Rate rate(0.5);
    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();
        //loadMap ("/home/josh/catkin_ws/src/hololens_vis/src/map.png", world_map, MAP_WIDTH, MAP_HEIGHT);
        if(!imageProcessed){

        //    for(int i =  0; i < MAP_HEIGHT; i ++){
        //        for(int j = 0 ; j < MAP_WIDTH; j++){
        //            if(i == inity && j == initx){
        //                cout << "s";
        //            }
        //            else if( i == endy && j == endx){
        //                cout << "e";
        //            }
        //            else{
        //                cout << GetMap( j,i );
        //            }
        //        }
        //        cout << endl;
        //    }
        cv::Mat M(MAP_WIDTH,MAP_HEIGHT, CV_8UC3, cv::Scalar(0,0,0));

        AStarSearch<MapSearchNode> astarsearch;

        unsigned int SearchCount = 0;

        const unsigned int NumSearches = 1;
        auto start = std::chrono::high_resolution_clock::now();



        // Create a start state
        MapSearchNode nodeStart;
        nodeStart.x = initx;
        nodeStart.y = inity;

        // Define the goal state
        MapSearchNode nodeEnd;
        nodeEnd.x = endx;
        nodeEnd.y = endy;

        // Set Start and goal states

        astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );

        unsigned int SearchState;
        unsigned int SearchSteps = 0;

        do
        {
//                        if(SearchSteps%1 ==0){
//                        for(int d = 0; d < astarsearch.m_OpenList.size(); d++){
//                            M.at<cv::Vec3b>(astarsearch.m_OpenList[d]->m_UserState.x,astarsearch.m_OpenList[d]->m_UserState.y) = cv::Vec3b(0,astarsearch.m_OpenList[d]->g,255-astarsearch.m_OpenList[d]->g);
//                        }
//                        cv::imshow("debug",M);
//                        cv::waitKey(1);
//                        }
            SearchState = astarsearch.SearchStep();

            SearchSteps++;
            //#define DEBUG_LISTS 1
#if DEBUG_LISTS

            cout << "Steps:" << SearchSteps << "\n";

            int len = 0;

            cout << "Open:\n";
            MapSearchNode *p = astarsearch.GetOpenListStart();
            while( p )
            {
                len++;
#if !DEBUG_LIST_LENGTHS_ONLY
                ((MapSearchNode *)p)->PrintNodeInfo();
#endif
                p = astarsearch.GetOpenListNext();

            }

            cout << "Open list has " << len << " nodes\n";

            len = 0;

            cout << "Closed:\n";
            p = astarsearch.GetClosedListStart();
            while( p )
            {
                len++;
#if !DEBUG_LIST_LENGTHS_ONLY
                p->PrintNodeInfo();
#endif
                p = astarsearch.GetClosedListNext();
            }

            cout << "Closed list has " << len << " nodes\n";
#endif

        }
        while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );

        if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
        {
            auto finish = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = finish - start;
            std::cout << "Elapsed time: " << elapsed.count() << std::endl;
            cout << "Search found goal state\n";

            MapSearchNode *node = astarsearch.GetSolutionStart();

#if DISPLAY_SOLUTION
            cout << "Displaying solution\n";
#endif
            int steps = 0;
            resultMat = cv::Mat(MAP_WIDTH,MAP_HEIGHT, CV_8UC3, cv::Scalar(0,0,0)); //
            //cv::Mat resultMat = cv::imread("/home/josh/catkin_ws/src/hololens_vis/src/map.png",1);
            cv::Point2d first,second;
            //node->PrintNodeInfo();
            for( ;; )
            {
                first = cv::Point2d(node->x,node->y);
                //resultMat.at<cv::Vec3b>(node->y,node->x) = cv::Vec3b(255,0,0);
                node = astarsearch.GetSolutionNext();

                if( !node )
                {
                    break;
                }
                second = cv::Point2d(node->x,node->y);
                cv::line(resultMat, first, second, cv::Vec3b(0,255,0));

                //node->PrintNodeInfo();
                steps ++;

            };
            second = cv::Point2d(endx,endy);
            cv::line(resultMat, first, second, cv::Vec3b(0,255,0));
            cv::GaussianBlur( resultMat, resultMat,cv::Size(1, 1), 0, 0 );
            resultMat *= 5;
            cv::imshow("result",resultMat);

            cout << "Solution steps " << steps << endl;

            // Once you're done with the solution you can free the nodes up
            astarsearch.FreeSolutionNodes();


        }
        else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED )
        {
            cout << "Search terminated. Did not find goal state\n";

        }

        // Display the number of loops the search went through


        SearchCount ++;

        astarsearch.EnsureMemoryFreed();

        cv::waitKey(1);
        //publish image to ros.
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resultMat).toImageMsg();
        imPub.publish(msg);
        }


    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
