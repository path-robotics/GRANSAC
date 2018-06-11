#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cmath>
#include <random>

#include "gransac/plane_fit.h"

std::string Mat3d_to_csv(const cv::Mat3d mat) {
    std::ostringstream oss;
    for (int col = 0; col < mat.cols; ++col) {
        cv::Vec3d vec = mat.at<cv::Vec3d>(0,col);
        oss << std::setprecision(18)
            << vec[0] << "," << vec[1] << "," << vec[2] << "\n";
    }
    return oss.str();
}

int main(int argc, char * argv[])
{
	if (argc != 1 && argc != 3)
	{
		std::cout << "[ USAGE ]: " << argv[0] << " [<Image Size> = 1000] [<nPoints> = 500]" << std::endl;
		return -1;
	}

	int Side = 1000;
	int nPoints = 500;
	if (argc == 3)
	{
		Side = std::atoi(argv[1]);
		nPoints = std::atoi(argv[2]);
	}

	cv::Mat Canvas(Side, Side, CV_8UC3);
	Canvas.setTo(255);

    // Randomly generate points in a 3D plane roughly aligned in a plane for testing
	std::random_device SeedDevice;
    std::mt19937 RNG(SeedDevice());

    std::uniform_real_distribution<> UniDist(-Side, Side); // [Incl, Incl]
    double Perturb = 200;
	std::normal_distribution<GRANSAC::VPFloat> PerturbDist(0, Perturb);

    pcl::PointCloud<pcl::PointXYZ> pcl_CandPoints;
    cv::Mat3d cv_CandPoints(1, nPoints);
	for (int i = 0; i < nPoints; ++i)
    {
        double s = UniDist(RNG);
        double t = UniDist(RNG);

        double x = 6 - 6 * t - 6 * s;
        double y = -3 * t;
        double z = 2 * s;

        // Also populate a Mat3d to test like the glugun_laserscanner
        cv::Vec3d vec(x + PerturbDist(RNG), y + PerturbDist(RNG), z + PerturbDist(RNG));
        cv_CandPoints.at<cv::Vec3d>(0,i) = vec;

        pcl::PointXYZ pt(vec.val[0], vec.val[1], vec.val[2]);
        pcl_CandPoints.push_back(pt);
	}

    // Fit the line and find the inliers
    int threshold = 100;
    int iterations = 2000;
    PlaneFit pf(cv_CandPoints);
    pf.estimate(threshold,iterations);

    cv::Mat3d inliers = cv_CandPoints.clone();
    pf.get_best_inliers(inliers);
    cv::Mat3d best_model;
    pf.get_best_inliers(best_model);

    std::cout << "cv_CandPoints.size() = " << cv_CandPoints.size() << std::endl;
    std::cout << "inliers.size() = " << inliers.size() << std::endl;

    std::ofstream all_data_csv("all_data.csv");
    all_data_csv << Mat3d_to_csv(cv_CandPoints);

    std::ofstream inliers_csv("inliers.csv");
    inliers_csv << Mat3d_to_csv(inliers);

    // Fit the line and find the inliers
    PlaneFit pf_pcl(pcl_CandPoints);
    pf_pcl.estimate(threshold,iterations);
    pcl::PointCloud<pcl::PointXYZ> inliers_pcl;
    pf_pcl.get_best_inliers(inliers_pcl);
    pcl::PointCloud<pcl::PointXYZ> best_model_pcl;
    pf_pcl.get_best_inliers(best_model_pcl);

    std::cout << "pcl_CandPoints.size() = " << pcl_CandPoints.size() << std::endl;
    std::cout << "inliers_pcl.size() = " << inliers_pcl.size() << std::endl;

	return 0;
}
