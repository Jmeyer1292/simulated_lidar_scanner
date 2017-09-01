#ifndef LIDAR_SCANNER_SIMULATOR_H
#define LIDAR_SCANNER_SIMULATOR_H

#include <simulated_lidar_scanner/synthetic_lidar_scanner/vtkLidarScanner.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief Container for the parameters defining the operation of the simulated lidar scanner
 */
struct scanner_params
{
  /**
   * @brief Horizontal scanner field of view angle (radians; -2*pi to 2*pi)
   */
  double theta_span;

  /**
   * @brief Vertical scanner field of view angle (radians; -pi to pi)
   */
  double phi_span;

  /**
   * @brief Scanner noise in the direction of the cast ray (line-of-sight) (meters). Default is 0.0.
   */
  double los_variance = 0.0;

  /**
   * @brief Scanner noise in the direction orthogonal to the cast ray (meters). Default is 0.0.
   */
  double orthogonal_variance = 0.0;

  /**
   * @brief Maximum angle of incidence of the cast ray to the surface before loss of data (radians). Default is 0.0.
   */
  double max_incidence_angle = 0.0;

  /**
   * @brief Maximum scanner sensing distance (meters). Default is 0.0.
   */
  double max_distance = 0.0;

  /**
   * @brief Number of rays to be cast in the horizontal direction
   */
  int theta_points;

  /**
   * @brief Number of rays to be cast in the vertical direction
   */
  double phi_points;
};

/**
 * @brief The LidarScannerSim class simulates the operation of a LiDAR sensor or Time-of-Flight camera per a set of user input parameters. The simulated
 * sensor casts rays into a scene of static VTK PolyData geometry and records the position of the ray intersection with the mesh. The sensor returns can
 * then be output as a VTK PolyData object or a PCL PointCloud.
 */
class LidarScannerSim
{
public:
  /**
   * @brief LidarScannerSim class constructor; takes a scanner_params structure defining various parameters
   * @param sim
   */
  LidarScannerSim(const scanner_params& sim);

  /**
   * @brief Set the static scene geometry which the simulated scanner will observe
   * @param scene
   */
  void setScannerScene(const vtkSmartPointer<vtkPolyData>& scene) {scanner_->SetScene(scene);}

  /**
   * @brief Set the transform of the scanner relative to the fixed world frame
   * @param frame
   */
  void setScannerTransform(const tf::StampedTransform& frame);

  /**
   * @brief Add line-of-sight and orthogonal noise based on a normal distribution to the sensor data.
   * This function is a re-implementation of a similar function in vtkLidarScanner.cxx; however this function can be called outside the ray-casting
   * operation and without the need of converting vtkPolyData or PCL point objects into vtkLidarPoint objects
   */
  void addScannerNoise();

  /**
   * @brief Perform a new scan with the simulated scanner's position defined by the input transformation relative to the fixed world frame.
   * The acquired scan data is saved to an internal class variable which can be accessed by the getScanPolyData() or getScanDataPointCloud() members.
   * @param scanner_transform
   */
  void getNewScanData(const tf::StampedTransform& scanner_transform);

  /**
   * @brief Returns the last acquired scan data from an internal class variable to a VTK PolyData object
   * @return A smart pointer to a VTK PolyData object of the scanner return points
   */
  vtkSmartPointer<vtkPolyData> getScanPolyData() const {return scan_data_vtk_;}

  /**
   * @brief Returns the last acquired scan data from an internal class variable to a PCL Point Cloud
   * @return A pointer to a PCL point cloud of the scanner return points
   */
  pcl::PointCloud<pcl::PointNormal>::Ptr getScanDataPointCloud() const {return noisy_scan_data_cloud_;}

private:
  vtkSmartPointer<vtkLidarScanner> scanner_;
  vtkSmartPointer<vtkPolyData> scan_data_vtk_;
  pcl::PointCloud<pcl::PointNormal>::Ptr scan_data_cloud_;
  pcl::PointCloud<pcl::PointNormal>::Ptr noisy_scan_data_cloud_;
  tf::StampedTransform scanner_frame_;
  double max_incidence_angle_ = 0.0;
  double max_distance_ = 0.0;
};

#endif // LIDAR_SCANNER_SIMULATOR_H
