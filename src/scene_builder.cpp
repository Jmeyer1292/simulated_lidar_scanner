#include <simulated_lidar_scanner/scene_builder.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <vtkAppendPolyData.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkSTLReader.h>
#include <urdf/model.h>
#include <boost/filesystem.hpp>

namespace
{
  vtkSmartPointer<vtkTransform> urdfPoseToVTKTransform(const urdf::Pose& pose)
  {
    urdf::Rotation q = pose.rotation;
    urdf::Vector3 p = pose.position;
    double w, x, y, z, denom;

    if(q.w > 1) {q.normalize();}
    w = 2*std::acos(q.w);
    denom = std::sqrt(1 - q.w*q.w);
    if(denom < 0.001)
    {
      // Choose an arbitrary axis since the rotation angle ~= 0
      x = 0.0;
      y = 0.0;
      z = 1.0;
    }
    else
    {
      x = q.x / denom;
      y = q.y / denom;
      z = q.z / denom;
    }

    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Translate(p.x, p.y, p.z);
    transform->RotateWXYZ(w*180/M_PI, x, y, z);

    return transform;
  }

  vtkSmartPointer<vtkPolyData> readSTLFile(std::string file)
  {
    if(!boost::filesystem::exists(file.c_str()))
    {
      ROS_ERROR("File doesn't exist: %s", file.c_str());
    }

    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(file.c_str());
    reader->SetMerging(1);
    reader->Update();

    return reader->GetOutput();
  }
}

SceneBuilder::SceneBuilder()
{
  scene_ = vtkSmartPointer<vtkPolyData>::New();
  if(!urdf_model_.initParam("/robot_description"))
  {
    ROS_FATAL("'robot_description' parameter must be set");
  }
}

bool SceneBuilder::createVTKScene()
{
  // Get filenames for the meshes of all static links
  if(!getSceneGeometry())
  {
    ROS_FATAL("'/robot_description' parameter must be set");
    return false;
  }

  // Concatenate all vtkPolyData objects into one if multiple static meshes exist
  if(scene_data_.size() == 0)
  {
    ROS_INFO("No static geometry present in URDF");
    return false;
  }

  if(!vtkSceneFromMeshFiles())
  {
    ROS_FATAL("Unable to load scene into VTK");
    return false;
  }
  return true;
}

bool SceneBuilder::getSceneGeometry()
{
  // Get the root link of the URDF
  urdf::LinkConstSharedPtr root_link = urdf_model_.getRoot();
  if(!root_link)
  {
    ROS_FATAL("No root link set in URDF");
    return false;
  }

  // Check if the root link has any mesh geometry
  std::vector<urdf::VisualSharedPtr> root_link_visuals = root_link->visual_array;
  urdf::Pose base_pose;
  base_pose.position.x = base_pose.position.y = base_pose.position.z = 0.0f;
  base_pose.rotation.w = base_pose.rotation.x = base_pose.rotation.z = base_pose.rotation.w = 0.0f;
  getLinkGeometry(root_link_visuals, base_pose);

  // Check joints of the root link for static links
  std::vector<urdf::JointSharedPtr> root_joints = root_link->child_joints;
  getLinkChildGeometry(root_joints);

  // Resolve package:// URI in filenames to get full file paths
  changeFilenames();

  return true;
}

void SceneBuilder::getLinkGeometry(std::vector<urdf::VisualSharedPtr>& visuals,
                                   urdf::Pose& joint_pose)
{
  for(auto it = visuals.begin(); it != visuals.end(); ++it)
  {
    urdf::VisualConstSharedPtr vis = *it;
    if(vis->geometry->type == urdf::Geometry::MESH)
    {
      scene_object obj;
      obj.filename = boost::dynamic_pointer_cast<const urdf::Mesh>(vis->geometry)->filename;
      obj.link_pose = vis->origin;
      obj.joint_pose = joint_pose;
      scene_data_.push_back(obj);
    }
  }
}

void SceneBuilder::getLinkChildGeometry(std::vector<urdf::JointSharedPtr>& joints)
{
  for(auto it = joints.begin(); it != joints.end(); ++it)
  {
    urdf::JointSharedPtr joint = *it;
    if(joint->type == urdf::Joint::FIXED)
    {
      urdf::LinkSharedPtr link;
      urdf_model_.getLink(joint->child_link_name, link);
      getLinkGeometry(link->visual_array, joint->parent_to_joint_origin_transform);

      // Check if this static link has static link children
      if(link->child_joints.size() > 0)
      {
        getLinkChildGeometry(link->child_joints);
      }
    }
  }
}

void SceneBuilder::changeFilenames()
{
  // Create a vector of iterators to filenames that aren't in the correct format
  std::vector<std::vector<scene_object>::iterator> erase_its;

  // Define the word to search for
  const std::string prefix = "package://";

  // Iterate through all filenames to find the defined prefix
  for(auto it = scene_data_.begin(); it != scene_data_.end(); ++it)
  {
    // Create a reference to the current file name
    scene_object &obj = *it;

    // Find the prefix to remove in the current filename
    size_t pos = obj.filename.find(prefix);
    if(pos != std::string::npos)
    {
      // Erase prefix from filename
      obj.filename.erase(pos, prefix.length());

      // Find the first "/" in the filename (indicating end of package name)
      size_t start_pos = obj.filename.find_first_of("/", 0);

      // Create package name string and get the full package name file path
      std::string pkg_name = obj.filename.substr(pos, start_pos);
      std::string pkg_path = ros::package::getPath(pkg_name);
      if(pkg_path.length() == 0)
      {
        // Save the iterator to the filename if the ROS package where it came from is not found
        ROS_INFO("Package not found: %s", pkg_name.c_str());
        auto erase_it = std::find_if(scene_data_.begin(), scene_data_.end(),
                                 [&obj](const scene_object& x){return x.filename == obj.filename;});
        erase_its.push_back(erase_it);
        continue;
      }

      // Erase the package name from the front of the filename and replace with full package path
      obj.filename.erase(0, start_pos);
      obj.filename.insert(obj.filename.begin(), pkg_path.begin(), pkg_path.end());
    }
    else
    {
      // Save the iterator to the filename if it doesn't start with the defined prefix
      ROS_INFO("Filename not in correct format: %s", obj.filename.c_str());
      auto erase_it = std::find_if(scene_data_.begin(), scene_data_.end(),
                               [&obj](const scene_object& x){return x.filename == obj.filename;});
      erase_its.push_back(erase_it);
    }
  }

  // Erase all filenames that weren't in the correct format
  for(auto it = erase_its.begin(); it != erase_its.end(); ++it)
  {
    scene_data_.erase(*it);
  }
}

bool SceneBuilder::vtkSceneFromMeshFiles()
{
  // Create a vtkAppendPolyData filter to combine multiple vtkPolyData objects
  vtkSmartPointer<vtkAppendPolyData> append_filter = vtkSmartPointer<vtkAppendPolyData>::New();

  for(size_t i = 0; i < scene_data_.size(); ++i)
  {
    // Create the vtkPolyData object from the stl mesh filename
    vtkSmartPointer<vtkTransformPolyDataFilter> transform_filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    vtkSmartPointer<vtkPolyData> vtk_poly = readSTLFile(scene_data_[i].filename);
    if(!vtk_poly) {return false;}

    // Apply transform specified by URDF Pose
    vtkSmartPointer<vtkTransform> link_transform = urdfPoseToVTKTransform(scene_data_[i].link_pose);
    vtkSmartPointer<vtkTransform> joint_transform = urdfPoseToVTKTransform(scene_data_[i].joint_pose);
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Concatenate(joint_transform->GetMatrix());
    transform->Concatenate(link_transform->GetMatrix());

    transform_filter->SetInputData(vtk_poly);
    transform_filter->SetTransform(transform);
    transform_filter->Update();

    append_filter->AddInputData(transform_filter->GetOutput());
  }
  append_filter->Update();
  scene_ = append_filter->GetOutput();

  return true;
}
