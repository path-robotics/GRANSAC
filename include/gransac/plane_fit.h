#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "gransac/GRANSAC.hpp"
#include "gransac/PlaneModel.hpp"

class PlaneFit {

 private:
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > m_data_points;
  GRANSAC::RANSAC<Plane3DModel, 3> m_estimator;

 public:
  PlaneFit();
  PlaneFit(const cv::Mat3d& data_points);
  PlaneFit(const pcl::PointCloud<pcl::PointXYZ>& data_points);
  ~PlaneFit();

  void load_data_points(const cv::Mat3d& data_points);
  void load_data_points(const pcl::PointCloud<pcl::PointXYZ>& data_points);
  void clear_data_points();
  void get_best_inliers(cv::Mat3d& best_inliers);
  void get_best_inliers(pcl::PointCloud<pcl::PointXYZ>& best_inliers);
  bool get_best_model(cv::Mat3d& best_model);
  bool get_best_model(pcl::PointCloud<pcl::PointXYZ>& best_model);
  bool estimate(GRANSAC::VPFloat threshold, int max_iterations = 1000);
};
