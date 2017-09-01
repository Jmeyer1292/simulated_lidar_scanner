#include <simulated_lidar_scanner/lidar_scanner_simulator.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkTransform.h>

namespace
{

  bool incidenceFilter(const pcl::PointNormal& pt,
                       const double max_angle)
  {
    Eigen::Vector3d ray (pt.x, pt.y, pt.z);
    Eigen::Vector3d normal (pt.normal_x, pt.normal_y, pt.normal_z);
    const double c_theta = normal.dot(ray.normalized());
    return (c_theta > max_angle);
  }

  bool distanceFilter(const pcl::PointNormal& pt,
                      const double max_dist)
  {
    const double dist2 = pt.x*pt.x + pt.y*pt.y + pt.z*pt.z;
    return (dist2 > max_dist*max_dist);
  }

  void VTKtoPCL(vtkPolyData* const pdata, pcl::PointCloud<pcl::PointNormal> &cloud)
  {
    for (int i = 0; i < pdata->GetPoints()->GetNumberOfPoints(); ++i)
    {
      pcl::PointNormal pt;
      double* ptr = pdata->GetPoints()->GetPoint(i);
      pt.x = ptr[0];
      pt.y = ptr[1];
      pt.z = ptr[2];

      double* norm = pdata->GetPointData()->GetNormals()->GetTuple(i);
      pt.normal_x = norm[0];
      pt.normal_y = norm[1];
      pt.normal_z = norm[2];

      cloud.push_back(pt);
    }
  }

}

LidarScannerSim::LidarScannerSim(const scanner_params& sim)
{
  // Initialize scanner and scan data
  scanner_ = vtkSmartPointer<vtkLidarScanner>::New();
  scan_data_vtk_ = vtkSmartPointer<vtkPolyData>::New();
  scan_data_cloud_.reset(new pcl::PointCloud<pcl::PointNormal> ());
  noisy_scan_data_cloud_.reset(new pcl::PointCloud<pcl::PointNormal> ());

  // Set scanner parameters
  scanner_->SetPhiSpan(sim.phi_span);
  scanner_->SetThetaSpan(sim.theta_span);
  scanner_->SetNumberOfThetaPoints(sim.theta_points);
  scanner_->SetNumberOfPhiPoints(sim.phi_points);
  scanner_->SetLOSVariance(sim.los_variance);
  scanner_->SetOrthogonalVariance(sim.orthogonal_variance);
  scanner_->SetStoreRays(false);
  scanner_->SetCreateMesh(false);

  if(sim.max_incidence_angle > 0.0)
  {
    max_incidence_angle_ = -std::cos(sim.max_incidence_angle * M_PI / 180.0);
  }

  if(sim.max_distance > 0.0)
  {
    max_distance_ = sim.max_distance;
  }
}

void LidarScannerSim::setScannerTransform(const tf::StampedTransform& frame)
{
  tf::Vector3 origin = frame.getOrigin();
  tf::Matrix3x3 orientation_mat = frame.getBasis();

  vtkSmartPointer<vtkMatrix4x4> vtk_matrix = vtkSmartPointer<vtkMatrix4x4>::New();
  for(unsigned int j = 0; j < 4; ++j)
  {
    if(j != 3)
    {
      vtk_matrix->SetElement(0, j, orientation_mat.getColumn(j).getX());
      vtk_matrix->SetElement(1, j, orientation_mat.getColumn(j).getY());
      vtk_matrix->SetElement(2, j, orientation_mat.getColumn(j).getZ());
    }
    else
    {
      vtk_matrix->SetElement(0, j, origin.getX());
      vtk_matrix->SetElement(1, j, origin.getY());
      vtk_matrix->SetElement(2, j, origin.getZ());
    }
  }

  // Create transform from matrix; rotate such that sensor axis is Z axis
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->SetMatrix(vtk_matrix);
  transform->RotateX(90.0f);

  // Set and update transform in scanner object
  scanner_->SetTransform(transform);
  scanner_frame_ = frame;
}

void LidarScannerSim::addScannerNoise()
{
  double los_variance = scanner_->GetLOSVariance();
  double orthogonal_variance = scanner_->GetOrthogonalVariance();

  std::default_random_engine generator;
  std::normal_distribution<double> los_dist (0.0, los_variance);
  std::normal_distribution<double> orthogonal_dist(0.0, orthogonal_variance);

  noisy_scan_data_cloud_->points.clear();

  for(auto pt = scan_data_cloud_->points.begin(); pt != scan_data_cloud_->points.end(); ++pt)
  {
    pcl::PointNormal noisy_pt;
    noisy_pt = *pt;

    // Add Line-of-sight noise
    if(los_variance > 0.0)
    {
      double los_noise = los_dist(generator);
      noisy_pt.x += pt->normal_x * los_noise;
      noisy_pt.y += pt->normal_y * los_noise;
      noisy_pt.z += pt->normal_z * los_noise;
    }

    // Add orthogonal noise
    if(orthogonal_variance > 0.0)
    {
      // Create a random normalized vector that is orthogonal to the point's current normal vector
      Eigen::Vector3d norm (pt->normal_x, pt->normal_y, pt->normal_z);
      Eigen::Vector3d rand_vec = Eigen::Vector3d::Random();
      Eigen::Vector3d orthogonal_vec = norm.cross(rand_vec);
      orthogonal_vec.normalize();

      double orthogonal_noise = orthogonal_dist(generator);
      noisy_pt.x += orthogonal_vec[0] * orthogonal_noise;
      noisy_pt.y += orthogonal_vec[1] * orthogonal_noise;
      noisy_pt.z += orthogonal_vec[2] * orthogonal_noise;
    }

    noisy_scan_data_cloud_->points.push_back(noisy_pt);
  }
}

void LidarScannerSim::getNewScanData(const tf::StampedTransform& scanner_transform)
{
  // Set the scanner transform
  setScannerTransform(scanner_transform);

  // Perform the scan and get the points
  scanner_->PerformScan();
  scanner_->GetValidOutputPoints(scan_data_vtk_);

  pcl::PointCloud<pcl::PointNormal> cloud;
  VTKtoPCL(scan_data_vtk_, cloud);

  // Transform point cloud into scanner frame
  Eigen::Affine3d transform;
  tf::transformTFToEigen(scanner_frame_.inverse(), transform);
  pcl::transformPointCloudWithNormals(cloud, *scan_data_cloud_, transform);

  // Cull points whose angle of incidence is greater than the specified tolerance
  if(max_incidence_angle_ > 0.0)
  {
    scan_data_cloud_->erase(std::remove_if(scan_data_cloud_->points.begin(),
                                          scan_data_cloud_->points.end(),
                                          boost::bind(incidenceFilter, _1, max_incidence_angle_)),
                            scan_data_cloud_->end());
  }

  if(max_distance_ > 0.0)
  {
    scan_data_cloud_->erase(std::remove_if(scan_data_cloud_->points.begin(),
                                          scan_data_cloud_->points.end(),
                                          boost::bind(distanceFilter, _1, max_distance_)),
                            scan_data_cloud_->end());
  }
}
