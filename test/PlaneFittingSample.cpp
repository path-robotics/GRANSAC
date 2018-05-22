#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <opencv2/opencv.hpp>
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
    double Perturb = 25;
	std::normal_distribution<GRANSAC::VPFloat> PerturbDist(0, Perturb);

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
	}

    // Fit the line and find the inliers
    PlaneFit pf(cv_CandPoints);
    pf.estimate(5,1000);

    cv::Mat3d inliers = cv_CandPoints.clone();
    pf.get_best_inliers(inliers);
    cv::Mat3d best_model;
    pf.get_best_inliers(best_model);

    std::cout << "best_model =\n";
    for (int col = 0; col < best_model.cols; ++col) {
        cv::Vec3d vec = best_model.at<cv::Vec3d>(0,col);
        std::cout << std::setprecision(6)
                  << vec[0] << "," << vec[1] << "," << vec[2] << "\n";
    }


    std::cout << "cv_CandPoints.size() = " << cv_CandPoints.size() << std::endl;
    std::cout << "inliers.size() = " << inliers.size() << std::endl;

    std::ofstream all_data_csv("all_data.csv");
    all_data_csv << Mat3d_to_csv(cv_CandPoints);

    std::ofstream inliers_csv("inliers.csv");
    inliers_csv << Mat3d_to_csv(inliers);

	return 0;
}
