#include<string>
#include<iostream>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

#include"fitCurves.h"
#include"utils.h"

using namespace std;
int main()
{
	string imgPath1 = "E:\\ransac\\data\\05151640_0419.MP4\\00000_prob_1.jpg";
	string imgPath2 = "E:\\ransac\\data\\05151640_0419.MP4\\00000_prob_2.jpg";
	cv::Mat leftImg;
	cv::Mat rightImg;
	cv::Mat img;
	ReadImgs(imgPath1, imgPath2, leftImg, rightImg, img);

	vector<cv::Point2d> leftPoints = getNonZeroPoints(leftImg, 0, 100);
	//cout << "Left Points" << endl;
	//cout << leftPoints << endl;

	vector<cv::Point2d> rightPoints = getNonZeroPoints(rightImg, 25, 100);
	//cout << "Right Points" << endl;
	//cout << rightPoints << endl;

	double error = 10;
	vector<BezierCurve> lbezCurves = FitCurve(leftPoints, 0, (int)leftPoints.size(), error);
	vector<BezierCurve> rbezCurves = FitCurve(rightPoints, 0, (int)rightPoints.size(), error);
	
	//cout << "The contorl points of Bezier curve in leftImg" << endl;
	//for (auto i : lbezCurves)
	//{
	//	cout << i.pt1 << " " << i.c1 << " " << i.c2 << " "<< i.pt2 << endl;
	//}
	//cout << "The contorl points of Bezier curve in rightImg" << endl;
	//for (auto i : rbezCurves)
	//{
	//	cout << i.pt1 << " " << i.c1 << " " << i.c2 << " " << i.pt2 << endl;
	//}
	cv::resize(img, img, cv::Size(420, 240));
	cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
	
	mulBezier(lbezCurves, 8.0);
	mulBezier(rbezCurves, 8.0);

	drawControlPoints(img, lbezCurves);
	drawControlPoints(img, rbezCurves);

	drawBezier(img, lbezCurves, 50);
	drawBezier(img, rbezCurves, 50);
	cv::imshow("Fitting Curve", img);
	cv::waitKey(0);
	cv::destroyAllWindows();
	system("pause");
	return 0;
}