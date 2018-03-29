#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <random>

#include "gransac/GRANSAC.hpp"
#include "gransac/LineModel.hpp"
#include "gransac/line_fit.h"

LineFit::LineFit() { }

LineFit::~LineFit() { }

LineFit::LineFit(const cv::Mat2f& data_points) {
  load_data_points(data_points);
}

void LineFit::load_data_points(const cv::Mat2f& data_points) {
  int N = data_points.cols;
  for (int i = 0; i < N; ++i) {
    cv::Vec2f cv_point = data_points.at<cv::Vec2f>(0, i);
    std::shared_ptr<GRANSAC::AbstractParameter> data_point = std::make_shared<Point2D>(cv_point.val[0], cv_point.val[1]);
    m_data_points.push_back(data_point);
  }
}

void LineFit::clear_data_points() {
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> >().swap(m_data_points);
}

void LineFit::get_best_inliers(cv::Mat2f& best_inliers) {
  auto r_best_inliers = m_estimator.GetBestInliers();
  int N = r_best_inliers.size();
  best_inliers = cv::Mat2f(1,N);
  if (r_best_inliers.size() > 0) {
    int i = 0;
    for (auto& inlier : r_best_inliers) {
      auto r_pt = std::dynamic_pointer_cast<Point2D>(inlier);
      cv::Vec2f pt(r_pt->m_Point2D[0], r_pt->m_Point2D[1]);
      best_inliers.at<cv::Vec2f>(0,i) = pt;
      i++;
    }
  }
}

bool LineFit::get_best_model(cv::Mat2f& best_model) {
  bool success = false;
  cv::Size mat_size(1,2);
  best_model = cv::Mat2f(mat_size);
  auto best_line = m_estimator.GetBestModel();
  if (best_line) {
    auto best_line_pt1 = std::dynamic_pointer_cast<Point2D>(best_line->GetModelParams()[0]);
    auto best_line_pt2 = std::dynamic_pointer_cast<Point2D>(best_line->GetModelParams()[1]);
    if (best_line_pt1 && best_line_pt2) {
      cv::Vec2f vec1(best_line_pt1->m_Point2D[0], best_line_pt1->m_Point2D[1]);
      cv::Vec2f vec2(best_line_pt2->m_Point2D[0], best_line_pt2->m_Point2D[1]);
      best_model.at<cv::Vec2f>(0,0) = vec1;
      best_model.at<cv::Vec2f>(0,1) = vec2;
      success = true;
    }
  }
  return success;
}

bool LineFit::estimate(GRANSAC::VPFloat threshold, int max_iterations /* = 1000 */) {
  m_estimator.Initialize(threshold, max_iterations); // Threshold, iterations
  m_estimator.Estimate(m_data_points);
}

void LineFit::draw_best_inliers(cv::Mat& canvas, int radius /* = 0 */) {
  // If no radius paramter is set, or it is set to 0, set a default radius
  // relative to the minimum image dimension.
  if (radius == 0) {
    int min_dim = (canvas.cols < canvas.rows) ? canvas.cols : canvas.rows;
    radius = floor(min_dim/100);
  }
  auto best_inliers = m_estimator.GetBestInliers();
  if (best_inliers.size() > 0) {
    for (auto& inlier : best_inliers) {
      auto r_pt = std::dynamic_pointer_cast<Point2D>(inlier);
      cv::Point pt(floor(r_pt->m_Point2D[0]), floor(r_pt->m_Point2D[1]));
      cv::circle(canvas, pt, radius, cv::Scalar(0, 255, 0), -1);
    }
  }
}

void LineFit::draw_best_model(cv::Mat& canvas, int line_width /* = 2 */) {
  auto best_line = m_estimator.GetBestModel();
  if (best_line) {
    auto best_line_pt1 = std::dynamic_pointer_cast<Point2D>(best_line->GetModelParams()[0]);
    auto best_line_pt2 = std::dynamic_pointer_cast<Point2D>(best_line->GetModelParams()[1]);
    if (best_line_pt1 && best_line_pt2) {
      cv::Point pt1(best_line_pt1->m_Point2D[0], best_line_pt1->m_Point2D[1]);
      cv::Point pt2(best_line_pt2->m_Point2D[0], best_line_pt2->m_Point2D[1]);
      DrawFullLine(canvas, pt1, pt2, cv::Scalar(0, 0, 255), 2);
    }
  }
}


GRANSAC::VPFloat Slope(int x0, int y0, int x1, int y1)
{
  return (GRANSAC::VPFloat)(y1 - y0) / (x1 - x0);
}

void DrawFullLine(cv::Mat& img, cv::Point a, cv::Point b, cv::Scalar color, int LineWidth)
{
  GRANSAC::VPFloat slope = Slope(a.x, a.y, b.x, b.y);

  cv::Point p(0, 0), q(img.cols, img.rows);

  p.y = -(a.x - p.x) * slope + a.y;
  q.y = -(b.x - q.x) * slope + b.y;

  cv::line(img, p, q, color, LineWidth, 8, 0);
}
