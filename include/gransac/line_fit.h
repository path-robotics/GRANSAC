#include <opencv2/opencv.hpp>

#include "gransac/GRANSAC.hpp"
#include "gransac/LineModel.hpp"

class LineFit {

 private:
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter> > m_data_points;
  GRANSAC::RANSAC<Line2DModel, 2> m_estimator;

 public:
  LineFit();
  LineFit(const cv::Mat2f& data_points);
  ~LineFit();

  void load_data_points(const cv::Mat2f& data_points);
  void clear_data_points();
  void get_best_inliers(cv::Mat2f& best_inliers);
  bool get_best_model(cv::Mat2f& best_model);
  bool estimate(GRANSAC::VPFloat threshold, int max_iterations = 1000);

  void draw_best_inliers(cv::Mat& canvas, int radius = 0);
  void draw_best_model(cv::Mat& canvas, int line_width = 2);
};

GRANSAC::VPFloat Slope(int x0, int y0, int x1, int y1);

void DrawFullLine(cv::Mat& img, cv::Point a, cv::Point b, cv::Scalar color, int LineWidth);
