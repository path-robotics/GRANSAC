#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <random>

#include "gransac/line_fit.h"

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

	// Randomly generate points in a 2D plane roughly aligned in a line for testing
	std::random_device SeedDevice;
	std::mt19937 RNG = std::mt19937(SeedDevice());

	std::uniform_int_distribution<int> UniDist(0, Side - 1); // [Incl, Incl]
	int Perturb = 25;
	std::normal_distribution<GRANSAC::VPFloat> PerturbDist(0, Perturb);

	std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;
    cv::Mat2f cv_CandPoints(1, nPoints);
	for (int i = 0; i < nPoints; ++i)
	{
		int Diag = UniDist(RNG);
		cv::Point Pt(floor(Diag + PerturbDist(RNG)), floor(Diag + PerturbDist(RNG)));
		cv::circle(Canvas, Pt, floor(Side / 100), cv::Scalar(0, 0, 0), -1);

		std::shared_ptr<GRANSAC::AbstractParameter> CandPt = std::make_shared<Point2D>(Pt.x, Pt.y);
		CandPoints.push_back(CandPt);

        // Also populate a Mat2f to test like the glugun_laserscanner
        cv::Vec2f vec(floor(Diag + PerturbDist(RNG)), floor(Diag + PerturbDist(RNG)));
        cv_CandPoints.at<cv::Vec2f>(0,i) = vec;
	}

    // Fit the line and find the inliers
    LineFit lf(cv_CandPoints);
    lf.estimate(20,100);
    lf.draw_best_inliers(Canvas);
    lf.draw_best_model(Canvas);

	while (true)
	{
		cv::imshow("RANSAC Example", Canvas);

		char Key = cv::waitKey(1);
		if (Key == 27)
			return 0;
		if (Key == ' ')
			cv::imwrite("LineFitting.png", Canvas);
	}

	return 0;
}
