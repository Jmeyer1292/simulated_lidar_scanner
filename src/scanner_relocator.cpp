#include <ros/ros.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

const static double FREQUENCY = 10.0f;
const static std::string SERVER_TOPIC = "scanner_relocator";

visualization_msgs::Marker makeVisualMarker(std::string& scanner_frame)
{
  static int idx = 0;
  visualization_msgs::Marker marker;
  marker.header.frame_id = scanner_frame;
  marker.header.stamp = ros::Time::now();
  marker.id = idx++;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.075;
  marker.scale.y = 0.075;
  marker.scale.z = 0.075;

  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;

  return marker;
}

void make6DOFControl(visualization_msgs::InteractiveMarker& m)
{
  visualization_msgs::InteractiveMarkerControl control;

  // Move and rotate X-axis
  control.orientation.w = 1.0;
  control.orientation.x = 1.0;
  control.orientation.y = 0.0;
  control.orientation.z = 0.0;

  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  m.controls.push_back(control);

  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  m. controls.push_back(control);

  // Move and rotate Y-axis

  control.orientation.w = 1.0;
  control.orientation.x = 0.0;
  control.orientation.y = 1.0;
  control.orientation.z = 0.0;

  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  m.controls.push_back(control);

  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  m. controls.push_back(control);

  // Move and rotate Z-axis

  control.orientation.w = 1.0;
  control.orientation.x = 0.0;
  control.orientation.y = 0.0;
  control.orientation.z = 1.0;

  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  m.controls.push_back(control);

  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  m. controls.push_back(control);
}

visualization_msgs::InteractiveMarker makeInteractiveMarker(std::string& scanner_frame,
                                                            std::string& scanner_parent_frame)
{
  visualization_msgs::InteractiveMarker m;
  m.header.frame_id = scanner_parent_frame;
  m.scale = 0.5;
  m.name = scanner_frame;
  m.pose.position.x = m.pose.position.y = m.pose.position.z = 0.0;
  m.pose.orientation.x = m.pose.orientation.y = m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;

  // Visual marker
  visualization_msgs::Marker visual = makeVisualMarker(scanner_frame);

  // Controls
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  control.markers.push_back(visual);
  m.controls.push_back(control);
  make6DOFControl(m);

  return m;
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "scanner_relocator");

  // Set up ROS node handle
  ros::NodeHandle pnh("~");

  // Get ROS parameters
  std::string scanner_parent_frame;
  if(!pnh.getParam("scanner_parent_frame", scanner_parent_frame))
  {
    ROS_FATAL("'scanner_parent_frame' parameter must be set");
    return -1;
  }

  std::vector<std::string> scanner_frames;
  if(!pnh.getParam("scanner_frames", scanner_frames))
  {
    ROS_FATAL("'scanner_frames' parameter must be set");
    return -1;
  }

  // Set up TF broadcaster for changing scanner frames
  tf::TransformBroadcaster broadcaster;

  // Set up interactive marker server
  interactive_markers::InteractiveMarkerServer server(SERVER_TOPIC, "", false);
  server.applyChanges();

  // Create an interactive marker for each of the remaining scanners
  for(size_t i = 0; i < scanner_frames.size(); ++i)
  {
    visualization_msgs::InteractiveMarker int_marker = makeInteractiveMarker(scanner_frames[i], scanner_parent_frame);
    server.insert(int_marker);
    server.applyChanges();
  }

  // Loop
  ros::Rate loop(FREQUENCY);
  while(ros::ok())
  {
    for(size_t i = 0; i < scanner_frames.size(); ++i)
    {
      visualization_msgs::InteractiveMarker int_marker;
      if(!server.get(scanner_frames[i], int_marker))
      {
        ROS_ERROR("Could not get interactive marker #%lu", i);
        continue;
      }

      tf::StampedTransform transform;
      const tf::Vector3 pos (int_marker.pose.position.x, int_marker.pose.position.y, int_marker.pose.position.z);
      const tf::Quaternion quat (int_marker.pose.orientation.x, int_marker.pose.orientation.y, int_marker.pose.orientation.z, int_marker.pose.orientation.w);

      transform.setOrigin(pos);
      transform.setRotation(quat);
      broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), scanner_parent_frame, scanner_frames[i]));
    }

    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}
