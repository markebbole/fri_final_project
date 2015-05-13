#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Twist.h"
#include "angles/angles.h"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "LearningController.h"
#include "ValueLearner.h"
#include <cmath>
#include <iostream>
#include <cstdio>
#include "ColorConversion.h"

using namespace std;

ros::Publisher velocity_pub;

ros::ServiceClient client;
LearningController *controller;

vector<double> lastState;
geometry_msgs::Twist lastAction;

double rate = .9;
ros::Time last_command;
ros::Time last_cloud_received_at;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool new_cloud_available_flag = false;

Node* currentPosition;
Node* endPosition;

// General point cloud to store the whole image
PointCloudT::Ptr cloud (new PointCloudT);

// Message required to publish the cloud - Convert from pcl to msg
sensor_msgs::PointCloud2 cloud_ros;

void cloud_sub(const sensor_msgs::PointCloud2ConstPtr& msg) {
  //convert the msg to PCL format
  pcl::fromROSMsg (*msg, *cloud);
  last_cloud_received_at = ros::Time::now();
  //state that a new cloud is available
  new_cloud_available_flag = true;
}

PointCloudT::Ptr computeNeonVoxels(PointCloudT::Ptr in, int color) {
  int THRESHOLD_H = 15;
  int THRESHOLD_S = 20;
  int THRESHOLD_V = 20;

  rgb my_rgb;
  rgb test_rgb;
  //Point Cloud to store out neon cap
  PointCloudT::Ptr temp_neon_cloud (new PointCloudT);
  unsigned int r, g, b;

  int col = color;
  int filterR = (col >> 16);
  int filterG = (col >> 8) & 0xff;
  int filterB = col & 0xff;

  for (int i = 0; i < in->points.size(); i++) {
      r = in->points[i].r;
      g = in->points[i].g;
      b = in->points[i].b;
      
      my_rgb.r = r;
      my_rgb.g = g;
      my_rgb.b = b;

      test_rgb.r = filterR;
      test_rgb.g = filterG;
      test_rgb.b = filterB;
        
      hsv c1 = rgb2hsv(my_rgb);
      hsv c2 = rgb2hsv(test_rgb);
        // Look for mostly neon value points
      if (abs(c2.h - c1.h) < THRESHOLD_H && abs(c2.s - c1.s) < THRESHOLD_S && abs(c2.v - c1.v) < THRESHOLD_V) {
        temp_neon_cloud->push_back(in->points[i]);
      }
      
  }

  return temp_neon_cloud;
}

double r(const vector<double>& s, geometry_msgs::Twist& a,  const vector<double>& s_prime) {

  double reward; 
  if(s_prime[0] < 2) {
    reward = 100;
  }else{
	if(s[0] == 9) {
		reward = 0;
	} else {
		reward = -10*(s_prime[0] - s[0]);
	}
  }
  ROS_INFO_STREAM("reward: went from state " << s[0] << " to state " << s_prime[0] << " with reward " << reward);

  return reward;
}

void processDistances(vector<tf::Vector3> markers) {
  //wait for the effect of our action
  if((ros::Time::now() - last_command).toSec() < 1./rate) {
    return; 
  }

  vector<double> state(8,0.);
  
  for(int i = 0; i < markers.size(); i++) {
    state[i] = markers[i] == NULL ? -1 : markers[i][0]; //distance
    state[i+4] = markers[i] == NULL ? -3.5 : atan2(markers[i][0], markers[i][2]); //angle
  }

  //state[0] = markers[0][2]; //right now we're just looking at the first marker's z distance
  //state[1] = atan2(markers[0][0], markers[0][2]);
  geometry_msgs::Twist action = controller->computeAction(state, currentPosition);
   
  if(!lastState.empty()) {
    //ROS_INFO_STREAM("last command sent at " << last_command << " from state " << lastState[0]); 
    //ROS_INFO_STREAM("distance from timestamp " << last_cloud_received_at << ": " << markers[0][2]);
    double reward = r(lastState,lastAction,state);
    controller->learn(lastState,lastAction,reward,state, action);
    if(reward == 100) {
		ROS_INFO_STREAM("done with episode. about to take a nap.");
		ros::Duration d = ros::Duration(5, 0);
		d.sleep();
		ROS_INFO_STREAM("done napping! :)");
	}
  }
   
  lastState = state;
  lastAction = action;

  last_command = ros::Time::now();
  velocity_pub.publish(action);
}


vector<tf::Vector3> getClusters(vector<PointCloudT::Ptr> clouds) {
  
  vector<tf::Vector3> centroids;
  for(int i = 0; i < clouds.size(); i++) {
    PointCloudT::Ptr cloud = clouds[i];
    
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
      
      
    if(cloud->points.size() < 20) {
      ROS_INFO("NOT ENOUGH POINTS");
      centroids.push_back(NULL);
      continue;
      //return centroids;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (20);
    ec.setMaxClusterSize (2800);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    /*  if(cluster_indices.size() < 1) {
      ROS_INFO("NOT ENOUGH HATS!!!");
      continue;
    }*/

    tf::TransformListener listener;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.begin()+1;/*cluster_indices.end();*/ ++it)
    {
      pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud->points[*pit]); 
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

      //compute a centroid for each found cluster.
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cloud_cluster, centroid);
      
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
      centroids.push_back(transf);
    }

  }
  return centroids;
  
}
int main (int argc, char** argv)
{

  Node* p = new Node(-100, 0xff9900, 0);
  Node* p2 = new Node(100, 0xff0000, 1);
  Node* p3 = new Node(-100, 0x0000ff, 2);
  Node* p4 = new Node(500, 0x00ff00, 3);

  p->addNeighbor(p2);

  p2->addNeighbor(p);
  p2->addNeighbor(p4);
  p2->addNeighbor(p3);

  p3->addNeighbor(p2);

  p4->addNeighbor(p2);
  
  currentPosition = p;
  endPosition = p4;

  // Initialize ROS
  ros::init (argc, argv, "linear_robot");
  ros::NodeHandle nh;
  ROS_INFO("Node initialized");

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/nav_kinect/depth_registered/points", 1000, cloud_sub);
  
  //debugging publisher --> can create your own topic and then subscribe to it through rviz
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("linear_robot/cloud", 10);

  velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  vector<pargo::BoundsPair> bounds;
  bounds.push_back(make_pair(-2., 10.); //orange
  bounds.push_back(make_pair(-2., 10.); //red
  bounds.push_back(make_pair(-2., 10.); //blue
  bounds.push_back(make_pair(-2., 10.); //green
  bounds.push_back(make_pair(-5., 5.));
  bounds.push_back(make_pair(-5., 5.));
  bounds.push_back(make_pair(-5., 5.));
  bounds.push_back(make_pair(-5., 5.));
    
  controller = new ValueLearner(bounds,2);

  //refresh rate
  double ros_rate = 12.0;
  ros::Rate r(ros_rate);
  
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();

    if (new_cloud_available_flag){
      //cloud contains a new image to process.
      new_cloud_available_flag = false; //reset this

      // Create the filtering object: downsample the dataset using a leaf size of 1cm
      pcl::VoxelGrid<PointT> vg;
      pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
      vg.setInputCloud (cloud);
      vg.setLeafSize (0.01f, 0.01f, 0.01f);
      vg.filter (*cloud_filtered);
      
      
      vector<PointerCloudT::Ptr> clouds;
      /*for(int i = 0; i < currentPosition.neighbors.size(); i++) {
        PointCloudT::Ptr tmp = computeNeonVoxels(currentPosition.neighbors[i], 
          currentPosition.neighbors[i].color);

        clouds.push_back(tmp);
      }*/
      PointCloudT::Ptr orange_cloud = computeNeonVoxels(cloud_filtered, 0xff9900);
      PointCloudT::Ptr red_cloud = computeNeonVoxels(cloud_filtered, 0xff0000);
      PointCloudT::Ptr blue_cloud = computeNeonVoxels(cloud_filtered, 0x0000ff);
      PointCloudT::Ptr green_cloud = computeNeonVoxels(cloud_filtered, 0x00ff00);

      clouds.push_back(orange_cloud);
      clouds.push_back(red_cloud);
      clouds.push_back(blue_cloud);
      clouds.push_back(green_cloud);

      vector<tf::Vector3> clusterCentroids = getClusters(clouds);
      //currentPosition.neighbors
      for(int i = 0; i < clusterCentroids.size(); i++) {
        bool found = false;
        for(int j = 0; j < currentPosition.neighbors.size(); j++) {
          if(currentPosition.neighbors[j].index == i) {
            found = true;
            break;
          }
        }
        if(!found) {
          clusterCentroids[i] = NULL;
        }
      }
      /*if(clusterCentroids.size() < 1) {
        ROS_INFO("NOT ENOUGH MARKERS");
        //continue;
      }*/

      processDistances(clusterCentroids); //right now there's only one marker but there will be more later
      
      pcl::toROSMsg(*neon_cloud,cloud_ros);
      
      //Set the frame ID to the first cloud we took in cause we want to replace that one
      cloud_ros.header.frame_id = cloud->header.frame_id;
      cloud_pub.publish(cloud_ros);
      
    }
  }
  
  return 0;
}
