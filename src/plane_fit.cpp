#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <random>

#include "gransac/GRANSAC.hpp"
#include "gransac/PlaneModel.hpp"
#include "gransac/plane_fit.h"

PlaneFit::PlaneFit() { }

PlaneFit::~PlaneFit() { }

PlaneFit::PlaneFit(const cv::Mat3d& data_points) {
  load_data_points(data_points);
}

PlaneFit::PlaneFit(const pcl::PointCloud<pcl::PointXYZ>& data_points) {
  load_data_points(data_points);
}

void PlaneFit::load_data_points(const cv::Mat3d& data_points) {
  int N = data_points.cols;
  for (int i = 0; i < N; ++i) {
    cv::Vec3d cv_point = data_points.at<cv::Vec3d>(0, i);
    std::shared_ptr<GRANSAC::AbstractParameter> data_point = std::make_shared<Point3D>(cv_point.val[0], cv_point.val[1], cv_point.val[2]);
    m_data_points.push_back(data_point);
  }
}

void PlaneFit::load_data_points(const pcl::PointCloud<pcl::PointXYZ>& data_points) {
  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = data_points.begin(); it != data_points.end(); ++it) {
    std::shared_ptr<GRANSAC::AbstractParameter> data_point = std::make_shared<Point3D>(it->x, it->y, it->z);
    m_data_points.push_back(data_point);
  }
}

void PlaneFit::clear_data_points() {
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> >().swap(m_data_points);
}

void PlaneFit::get_best_inliers(cv::Mat3d& best_inliers) {
  auto r_best_inliers = m_estimator.GetBestInliers();
  int N = r_best_inliers.size();
  best_inliers = cv::Mat3d(1,N);
  if (r_best_inliers.size() > 0) {
    int i = 0;
    for (auto& inlier : r_best_inliers) {
      auto r_pt = std::dynamic_pointer_cast<Point3D>(inlier);
      cv::Vec3d pt(r_pt->m_Point3D[0], r_pt->m_Point3D[1], r_pt->m_Point3D[2]);
      best_inliers.at<cv::Vec3d>(0,i) = pt;
      i++;
    }
  }
}

void PlaneFit::get_best_inliers(pcl::PointCloud<pcl::PointXYZ>& best_inliers) {
  best_inliers.clear();
  auto r_best_inliers = m_estimator.GetBestInliers();
  int N = r_best_inliers.size();
  if (r_best_inliers.size() > 0) {
    for (auto& inlier : r_best_inliers) {
      auto r_pt = std::dynamic_pointer_cast<Point3D>(inlier);
      pcl::PointXYZ pt(r_pt->m_Point3D[0], r_pt->m_Point3D[1], r_pt->m_Point3D[2]);
      best_inliers.push_back(pt);
    }
  }
}

bool PlaneFit::get_best_model(cv::Mat3d& best_model) {
  bool success = false;
  cv::Size mat_size(1,3);
  best_model = cv::Mat3d(mat_size);
  auto best_plane = m_estimator.GetBestModel();
  if (best_plane) {
    auto best_plane_pt1 = std::dynamic_pointer_cast<Point3D>(best_plane->GetModelParams()[0]);
    auto best_plane_pt2 = std::dynamic_pointer_cast<Point3D>(best_plane->GetModelParams()[1]);
    auto best_plane_pt3 = std::dynamic_pointer_cast<Point3D>(best_plane->GetModelParams()[2]);
    if (best_plane_pt1 && best_plane_pt2 && best_plane_pt3) {
      cv::Vec3d vec1(best_plane_pt1->m_Point3D[0], best_plane_pt1->m_Point3D[1], best_plane_pt1->m_Point3D[2]);
      cv::Vec3d vec2(best_plane_pt2->m_Point3D[0], best_plane_pt2->m_Point3D[1], best_plane_pt2->m_Point3D[2]);
      cv::Vec3d vec3(best_plane_pt3->m_Point3D[0], best_plane_pt3->m_Point3D[1], best_plane_pt3->m_Point3D[2]);
      best_model.at<cv::Vec3d>(0,0) = vec1;
      best_model.at<cv::Vec3d>(0,1) = vec2;
      best_model.at<cv::Vec3d>(0,2) = vec3;
      success = true;
    }
  }
  return success;
}

bool PlaneFit::get_best_model(pcl::PointCloud<pcl::PointXYZ>& best_model) {
  bool success = false;
  best_model.clear();
  auto best_plane = m_estimator.GetBestModel();
  if (best_plane) {
    auto best_plane_pt1 = std::dynamic_pointer_cast<Point3D>(best_plane->GetModelParams()[0]);
    auto best_plane_pt2 = std::dynamic_pointer_cast<Point3D>(best_plane->GetModelParams()[1]);
    auto best_plane_pt3 = std::dynamic_pointer_cast<Point3D>(best_plane->GetModelParams()[2]);
    if (best_plane_pt1 && best_plane_pt2 && best_plane_pt3) {
      pcl::PointXYZ pt1(best_plane_pt1->m_Point3D[0], best_plane_pt1->m_Point3D[1], best_plane_pt1->m_Point3D[2]);
      pcl::PointXYZ pt2(best_plane_pt2->m_Point3D[0], best_plane_pt2->m_Point3D[1], best_plane_pt2->m_Point3D[2]);
      pcl::PointXYZ pt3(best_plane_pt3->m_Point3D[0], best_plane_pt3->m_Point3D[1], best_plane_pt3->m_Point3D[2]);
      best_model.push_back(pt1);
      best_model.push_back(pt2);
      best_model.push_back(pt3);
      success = true;
    }
  }
  return success;
}

bool PlaneFit::estimate(GRANSAC::VPFloat threshold, int max_iterations /* = 1000 */) {
  m_estimator.Initialize(threshold, max_iterations); // Threshold, iterations
  m_estimator.Estimate(m_data_points);
}
