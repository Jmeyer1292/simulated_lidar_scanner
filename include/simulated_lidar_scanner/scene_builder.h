#ifndef SCENE_BUILDER_H
#define SCENE_BUILDER_H

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <urdf/model.h>

/**
 * @brief The SceneBuilder class identifies all permanently static mesh geometry in a URDF (i.e. links with visual mesh geometry
 * attached by fixed joints) published on the 'robot_description' topic, and then creates a VTK PolyData scene containing all identified meshes
 */
class SceneBuilder
{
public:
  /**
   * @brief Default class constructor
   */
  SceneBuilder();

  /**
   * @brief Identifies permanently static mesh geometry in a URDF and creates a VTK PolyData scene containing those meshes
   * @return True on success, false on failure
   */
  bool createVTKScene();

  /**
   * @brief Returns the created VTK scene
   * @return Pointer to the VTK scene
   */
  vtkSmartPointer<vtkPolyData> getVTKScene() const {return scene_;}

private:
  bool getSceneGeometry();
  void getLinkGeometry(std::vector<urdf::VisualSharedPtr>& root_link_visuals,
                       urdf::Pose& joint_pose);
  void getLinkChildGeometry(std::vector<urdf::JointSharedPtr>& joints);
  void changeFilenames();
  bool vtkSceneFromMeshFiles();

  struct scene_object
  {
    std::string filename;
    urdf::Pose link_pose;
    urdf::Pose joint_pose;
  };

  std::vector<scene_object> scene_data_;
  urdf::Model urdf_model_;
  vtkSmartPointer<vtkPolyData> scene_;
};

#endif // SCENE_BUILDER_H
