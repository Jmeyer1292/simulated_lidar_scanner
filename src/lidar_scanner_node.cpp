#include <ros/ros.h>
#include <ros/package.h>
#include <simulated_lidar_scanner/lidar_scanner_simulator.h>
#include <simulated_lidar_scanner/scene_builder.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkAppendPolyData.h>
#include <urdf/model.h>

const static double TF_TIMEOUT = 10.0; // seconds

static bool distanceComparator(const tf::StampedTransform& a,
                               const tf::StampedTransform& b,
                               const double dist_sqr,
                               const double angle)
{
  const tf::Transform diff = a.inverseTimes(b);
  const double d2 = diff.getOrigin().length2();
  const tfScalar q = diff.getRotation().getAngle();
  return (d2 > dist_sqr || q > angle);
}

bool getScannerParameters(ros::NodeHandle& nh,
                   scanner_params& sim)
{
  if(!nh.getParam("scanner/theta_span", sim.theta_span))
  {
    ROS_ERROR("scanner/theta_span' parameter must be set");
    return false;
  }
  sim.theta_span *= M_PI / 180.0;
  if(!nh.getParam("scanner/phi_span", sim.phi_span))
  {
    ROS_ERROR("'scanner/phi_span' parameter must be set");
    return false;
  }
  sim.phi_span *= M_PI / 180.0;
  if(!nh.getParam("scanner/theta_points", sim.theta_points))
  {
    ROS_ERROR("'scanner/theta_points' parameter must be set");
    return false;
  }
  if(!nh.getParam("scanner/phi_points", sim.phi_points))
  {
    ROS_ERROR("'scanner/phi_points' parameter must be set");
    return false;
  }

  // Get the optional parameters
  if(!nh.getParam("scanner/los_variance", sim.los_variance))
  {
    ROS_INFO("'scanner/los_variance' parameter defaulting to 0.0");
  }
  if(!nh.getParam("scanner/orthogonal_variance", sim.orthogonal_variance))
  {
    ROS_INFO("'scanner/orthogonal_variance' parameter defaulting to 0.0");
  }
  if(!nh.getParam("scanner/max_incidence_angle", sim.max_incidence_angle))
  {
    ROS_INFO("'scanner/max_incidence_angle' parameter not set; disabling incidence check");
  }
  if(!nh.getParam("scanner/max_distance", sim.max_distance))
  {
    ROS_INFO("'scanner/max_distance' parameter not set; disabling distance filter");
  }

  return true;
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "lidar_sim_node");

  // Set up ROS node handle
  ros::NodeHandle pnh("~");

  // Get scanner parameters
  scanner_params sim;
  if(!getScannerParameters(pnh, sim))
  {
    return -1;
  }

  std::string fixed_frame;
  if(!pnh.getParam("fixed_frame", fixed_frame))
  {
    ROS_ERROR("'fixed_frame' parameter must be set");
    return -1;
  }

  std::string scanner_frame;
  if(!pnh.getParam("scanner_frame", scanner_frame))
  {
    ROS_ERROR("'scanner_frame' parameter must be set");
    return -1;
  }

  double tf_filter_angle;
  if(!pnh.getParam("tf_filter_angle", tf_filter_angle))
  {
    ROS_ERROR("'tf_filter_angle' parameter not set; defaulting to 0.0");
    tf_filter_angle = 0.0;
  }
  tf_filter_angle *= M_PI / 180.0;

  double tf_filter_distance;
  if(!pnh.getParam("tf_filter_distance", tf_filter_distance))
  {
    ROS_ERROR("'tf_filter_distance' parameter not set; defaulting to 0.0");
    tf_filter_distance = 0.0;
  }
  double tf_filter_distance_sqr = tf_filter_distance * tf_filter_distance;

  double scan_frequency;
  if(!pnh.getParam("scanner/scan_frequency", scan_frequency))
  {
    ROS_ERROR("'scanner/scan_frequency' parameter must be set");
    return -1;
  }

  // Create a ROS tf listener to get the scanner frame
  tf::TransformListener listener;
  if(!listener.waitForTransform(fixed_frame, scanner_frame, ros::Time(0), ros::Duration(TF_TIMEOUT)))
  {
    ROS_ERROR("TF listener timeout for '%s' to '%s' transform", fixed_frame.c_str(), scanner_frame.c_str());
    return -1;
  }

  // Create scanner class
  LidarScannerSim scanner(sim);

  // Set static scene
  SceneBuilder builder;
  if(!builder.createVTKScene())
  {
    ROS_ERROR("Unable to create VTK scene");
    return -1;
  }

  scanner.setScannerScene(builder.getVTKScene());

  ROS_INFO("Simulated LIDAR scanner initialized. Beginning data acquisition...");

  // Create a ROS publisher to publish the scan data
  ros::Publisher scan_pub = pnh.advertise<sensor_msgs::PointCloud2>("/sensor_data/" + scanner_frame, 1, false);

  // Set initial previous transform to identity matrix
  tf::StampedTransform previous_transform;
  previous_transform.setIdentity();

  sensor_msgs::PointCloud2 scan_data_msg;

  ros::Rate loop_rate(scan_frequency);
  while(pnh.ok())
  {
    // Get TF frame from absolute static world origin to scanner focal point
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform(fixed_frame, scanner_frame, ros::Time(0), transform);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      continue;
    }

    // Update scan information if TF frame has moved significantly
    if(distanceComparator(previous_transform, transform, tf_filter_distance_sqr, tf_filter_angle))
    {
      // Get a new scan based on the current location of the scanner
      scanner.getNewScanData(transform);
    }

    // Add noise to the scanner data
    scanner.addScannerNoise();

    // Convert scan data from private class object to ROS msg
    pcl::PointCloud<pcl::PointNormal>::Ptr data = scanner.getScanDataPointCloud();
    pcl::toROSMsg(*data, scan_data_msg);
    scan_data_msg.header.stamp = ros::Time::now();
    scan_data_msg.header.frame_id = scanner_frame;
    scan_pub.publish(scan_data_msg);

    previous_transform = transform;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
