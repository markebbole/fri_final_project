#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <cstdio>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Twist.h"
#include "angles/angles.h"
#include <cmath>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "LearningController.h"
#include "ValueLearner.h"
#include "geometry_msgs/Twist.h"
using namespace std;

ros::Publisher velocity_pub;

ros::ServiceClient client;
LearningController *controller;

//double target_x = 8;
//double target_y = 2;
//double target_v = 0;

vector<double> lastState;
geometry_msgs::Twist lastAction;
//const turtlesim::Pose::ConstPtr &lastPose;
//geometry_msgs:
double rate = .8;
ros::Time last_command;

double ball_x = 7.;
double ball_y = 4.;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool new_cloud_available_flag = false;
double THRESHOLD_DISTANCE = 0.1; //below this distance, the two hats are 'on top of eachother'
double MOVE_DISTANCE = 1.8; //past this distance the robot will move towards the target
double MOVE_BACK_DISTANCE = 1.7; //less than this distance the robot will move away from the target
double ROBOT_MOVE_SPEED = .15;
double ROBOT_TURN_SPEED = .6;

// General point cloud to store the whole image
PointCloudT::Ptr cloud (new PointCloudT);

//Point Cloud to store out neon cap
PointCloudT::Ptr neon_cloud (new PointCloudT);

// Message required to publish the cloud - Convert from pcl to msg
sensor_msgs::PointCloud2 cloud_ros;
ros::Publisher cmd_pub;


void cloud_sub(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    //convert the msg to PCL format
    pcl::fromROSMsg (*msg, *cloud);

    //state that a new cloud is available
    new_cloud_available_flag = true;
}

PointCloudT::Ptr computeNeonVoxels(PointCloudT::Ptr in) {
    int total_neon = 0;
    
    //Point Cloud to store out neon cap
  PointCloudT::Ptr temp_neon_cloud (new PointCloudT);

    for (int i = 0; i < in->points.size(); i++) {
        unsigned int r, g, b;
        r = in->points[i].r;
        g = in->points[i].g;
        b = in->points[i].b;
        // Look for mostly neon value points
        if (g > 150 && (r + b) < 220) {
      temp_neon_cloud->push_back(in->points[i]);
      }
    }

    return temp_neon_cloud;
}

double r(const vector<double>& s, geometry_msgs::Twist& a,  const vector<double>& s_prime) {
  


  //double ball_distance_x = poseX - ball_x;
  //double ball_distance_y = poseY - ball_y;
  //double ball_distance = sqrt(ball_distance_x * ball_distance_x + ball_distance_y * ball_distance_y);

  /*if(ball_distance < 1) {
    ROS_INFO_STREAM("HIT BARRIER: REWARD = -2000");
    return -2000;
  }*/
  //first approximation of the reward: distance to the target
  
  double reward;
  
  if(s_prime[0] < 2)
    reward = 100;
  else
    reward = -s_prime[0];
    
  ROS_INFO_STREAM("vec.x: " << s_prime[0]);
  ROS_INFO_STREAM("reward: " << reward << " " /*<< target_x << " " << target_y*/);

  return reward;
  
//   double x = s_prime[0] - target_x;
//   double y = s_prime[1] - target_y;
//   double v = s_prime[2] - target_v;
//   
//   return -sqrt(x*x + y*y);
}

/*void reset() { 
  std_srvs::Empty c;
  client.call(c);
  //target_x = (rand() % 4) + 3;
  //target_y = (rand() % 4) + 3;
}
*/
/*void receivePose(const turtlesim::Pose::ConstPtr &pose){
  
  if((ros::Time::now() - last_command).toSec() < 1./rate)
    return; //wait for the effect of our action
    
   //double diff_x = target_x - pose->x;
   //double diff_y = target_y - pose->y;
  
   vector<double> state(2,0.);
   state[0] = sqrt(diff_x * diff_x + diff_y * diff_y);
  // state[1] = target_v - pose->linear_velocity;
   
   double angle = atan2(diff_y,diff_x);
   
   state[1] = angles::shortest_angular_distance(pose->theta,angle);
   
   
   geometry_msgs::Twist action = controller->computeAction(state);
   
   if(!lastState.empty()) {
     double reward = r(pose->x, pose->y, lastState,lastAction,state);
     controller->learn(lastState,lastAction,reward,state, action);
     if(reward == 100) {
       lastState.clear();
       reset();
     }
   }
   
   lastState = state;
   lastAction = action;
   
   
   last_command = ros::Time::now();
   velocity_pub.publish(action);
  
}*/


/*bool setTarget(turtle_controller::TargetPose::Request &req, turtle_controller::TargetPose::Response &res) {
   target_x = req.x;
   target_y = req.y;
   target_v = req.v;

   return true;
}*/


/*

int main(int argc, char** cc) {
  
   srand(time(NULL));
   
   ros::init(argc, cc, "turtlecontroller");
   
   vector<pargo::BoundsPair> bounds;
   bounds.push_back(make_pair(-5.,50.));
  // bounds.push_back(make_pair(-15.,15.));
   bounds.push_back(make_pair(-M_PI,M_PI));
   
   controller = new ValueLearner(bounds,5);
   
   ros::NodeHandle n;
   ros::Subscriber ros_pose = n.subscribe("/turtle1/pose", 1000, receivePose);
   client = n.serviceClient<std_srvs::Empty>("reset");
   
   ros::ServiceServer service = n.advertiseService("target_pose", setTarget);
   
   velocity_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
   
   last_command = ros::Time::now(); //initialize the variable

   ros::spin();
   
   delete controller;
   
}*/





void processDistance(tf::Vector3 vec) {
  if((ros::Time::now() - last_command).toSec() < 1./rate)
    return; //wait for the effect of our action
    
   //double diff_x = target_x - pose->x;
   //double diff_y = target_y - pose->y;
   
   vector<double> state(1,0.);
   state[0] = vec[2];
  // state[1] = target_v - pose->linear_velocity;
   
   //double angle = atan2(diff_y,diff_x);
   
   //state[1] = angles::shortest_angular_distance(pose->theta,angle);
   ROS_INFO("state: %f", state[0]);
   
   geometry_msgs::Twist action = controller->computeAction(state);
   
   if(!lastState.empty()) {
     double reward = r(lastState,lastAction,state);
     controller->learn(lastState,lastAction,reward,state, action);
     /*if(reward == 100) {
       lastState.clear();
       reset();
     }*/
   }
   
   lastState = state;
   lastAction = action;
   
   
   last_command = ros::Time::now();
   velocity_pub.publish(action);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "linear_robot");
  ros::NodeHandle nh;
  ROS_INFO("Node initialized");

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/nav_kinect/depth_registered/points", 1000, cloud_sub);
  
  //debugging publisher --> can create your own topic and then subscribe to it through rviz
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("linear_robot/cloud", 10);

  velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  // TODO: pls fix
  vector<pargo::BoundsPair> bounds;
  // bounds.push_back(make_pair(-.,50.));
  bounds.push_back(make_pair(-1.,10.));
  bounds.push_back(make_pair(-.5, .5));
   //bounds.push_back(make_pair(-M_PI,M_PI));
  controller = new ValueLearner(bounds,5);

  //refresh rate
  double ros_rate = 5.0;
  ros::Rate r(ros_rate);
  tf::TransformListener listener;
  
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
    //velocity_pub.publish(vel);

    if (new_cloud_available_flag){

      new_cloud_available_flag = false;

      // Voxel Grid reduces the computation time. Its a good idea to do it if you will be doing
      //sequential processing or frame-by-frame
      // Create the filtering object: downsample the dataset using a leaf size of 1cm
      pcl::VoxelGrid<PointT> vg;
      pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
      vg.setInputCloud (cloud);
      vg.setLeafSize (0.01f, 0.01f, 0.01f);
      vg.filter (*cloud_filtered);

      
      //ROS_INFO("After voxel grid filter: %i points",(int)cloud_filtered->points.size());
      
      int max_num_neon = 0;
      
      //Send the filtered point cloud to be processed in order to get the neon blob
      neon_cloud = computeNeonVoxels(cloud_filtered);
      
      
      //------------euclidean clustering---------
      // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<PointT> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (50);
        seg.setDistanceThreshold (0.1);
        
        
      if(neon_cloud->points.size() < 20) {
        ROS_INFO("NOT ENOUGH POINTS");
        continue;
        }

        // Creating the KdTree object for the search method of the extraction
      pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
      tree->setInputCloud (neon_cloud);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<PointT> ec;
      ec.setClusterTolerance (0.02); // 2cm
      ec.setMinClusterSize (20);
      ec.setMaxClusterSize (2800);
      ec.setSearchMethod (tree);
      ec.setInputCloud (neon_cloud);
      ec.extract (cluster_indices);

      int j = 0;
      int size = 0;
      if(cluster_indices.size() < 1) {
        ROS_INFO("NOT ENOUGH HATS!!!");
        continue;
      }

      tf::Vector3 firstHat;
      bool foundOneHat = false;
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end(); ++it)
      {
        pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (neon_cloud->points[*pit]); 
          cloud_cluster->width = cloud_cluster->points.size ();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;
          //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
          j++;
          //compute a centroid for each found cluster.
          Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);
        //ROS_INFO("The centroid of this neon cap is: (%f, %f, %f)", centroid(0), centroid(1), centroid(2));
        
        tf::StampedTransform transform;
        tf::Vector3 transf;
        tf::Vector3 vec(centroid(0), centroid(1), centroid(2));
      
        //transform each found cluster
        try {
          listener.waitForTransform("base_footprint", "nav_kinect_depth_frame", ros::Time(0), ros::Duration(3.0));
          listener.lookupTransform("base_footprint", "nav_kinect_depth_frame", ros::Time(0), transform);
          transf = transform*vec;
        } catch(tf::TransformException &ex) {
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }
      
        processDistance(transf);

        /*         
        if(!foundOneHat) { //haven't found a hat yet, store this one so that we can compare others with it
          firstHat = transf;
          foundOneHat = true;
        } else {
          //otherwise, we check to see if the two hats have nearly the same x,z coordinates.
          if(sqrt(pow(transf[0]-firstHat[0], 2) + pow(transf[2]-firstHat[2], 2)) < THRESHOLD_DISTANCE) {
            ROS_INFO("TWO HATS ON TOP OF EACHOTHER");
            
            //if they do, move twoards the last found one.
            geometry_msgs::Twist twst;
            double target_theta =-atan2(transf[0], transf[2]);// angles::shortest_angular_distance(0.0, atan2(transf[0], transf[2]));
            double distance = sqrt(transf[2]*transf[2] + transf[0]*transf[0]);
            ROS_INFO("distance %f", distance);

            if(distance > MOVE_DISTANCE && distance < 6) {
              twst.linear.x = ROBOT_MOVE_SPEED;
              twst.angular.z = ROBOT_TURN_SPEED * target_theta;
            }else if(distance < MOVE_BACK_DISTANCE) {
              twst.linear.x = -ROBOT_MOVE_SPEED; 
              twst.angular.z = ROBOT_TURN_SPEED * target_theta;
            }
            //ROS_INFO("twist info linear: %f angular: %f", twst.linear.x, twst.angular.z);
            cmd_pub.publish(twst);
          } else {
            //otherwise it sees two hats but they aren't close enough in the x,z coordinates
            ROS_INFO("TWO HATS, BUT NOT ON TOP OF EACHOTHER");
          }
        }*/
      //ROS_INFO("Translated to base_footprint: (%f, %f, %f)", transf[0], transf[1], transf[2]);
        
      }
      
      pcl::toROSMsg(*neon_cloud,cloud_ros);
      
      //Set the frame ID to the first cloud we took in coz we want to replace that one
      cloud_ros.header.frame_id = cloud->header.frame_id;
      cloud_pub.publish(cloud_ros);
      
      
    }
  }
  
  return 0;
}
