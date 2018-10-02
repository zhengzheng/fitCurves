#include<string>
#include<iostream>
#include<algorithm>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include"fitCurves.h"
#include"utils.h"

using namespace std;

/*
*  ReadImgs:
*  	Read Img1 and Img2, then merge them and split according to boundary
*   (the value lower than threshold will be zero).
*/
void ReadImgs(string imgPath1, string imgPath2, cv::Mat& leftImg, cv::Mat& rightImg, cv::Mat& img, int boundary)
{
	cv::Mat img1 = cv::imread(imgPath1, 0);
	cv::Mat img2 = cv::imread(imgPath2, 0);
	if (img1.empty()) throw "Img1 is not existed";
	if (img2.empty()) throw "Img2 is not existed";

	if (img1.size() != img2.size()) throw "Img1 and Img2 must have the same size";
	img = img1 + img2;

	int width = img.cols;
	int height = img.rows;
	//cout << img.size() << endl;
	cv::Rect leftRect(0, 0, boundary, height);
	cv::Rect rightRect(width / 2, 0, width - boundary - 1, height);

	leftImg = img(leftRect);
	rightImg = img(rightRect);
}

vector<cv::Point2d> getNonZeroPoints(cv::Mat& img, int offset, int threshold)
{
	vector<cv::Point2d> d;
	cv::Point2d p;
	// cout << img.size << endl;
	for (int j = 0; j < img.rows; ++j) {
		for (int i = 0; i < img.cols; ++i) {
			if (img.at<uchar>(j, i) >= threshold) {
				p = { (double)j,double(i+offset) };
				// cout << p << endl;
				d.push_back(p);
			}
		}
	}
	/*reverse(d.begin(), d.end());*/
	return d;
}

void drawBezier(cv::Mat img, vector<BezierCurve> bezCurves, int nPts)
{
	vector<cv::Point> points;
	cv::Point2d tmp;	
	for (auto bezCurve : bezCurves)
	{
		//cout << bezCurve.pt1 << " " << bezCurve.c1 << " " << bezCurve.c2 << " " << bezCurve.pt2 << endl;
		for (int i = 0; i <= nPts; ++i)
		{
			double t = double(i) / (double)nPts;
			tmp = q(bezCurve, t);
			swap(tmp.x, tmp.y);
			points.push_back(tmp);
		}
	}
	
	cv::polylines(img, points, false, cv::Scalar(255, 255, 0), 2);
}

void drawControlPoints(cv::Mat& img, vector<BezierCurve> bezCurves)
{
	for (auto i : bezCurves)
	{
		cv::circle(img, cv::Point((int)i.pt1.y, (int)i.pt1.x), 3, cv::Scalar(0, 0, 255), -1);
		cv::circle(img, cv::Point((int)i.pt2.y, (int)i.pt2.x), 3, cv::Scalar(0, 0, 255), -1);
		cv::circle(img, cv::Point((int)i.c1.y, (int)i.c1.x), 3, cv::Scalar(0, 0, 255), -1);
		cv::circle(img, cv::Point((int)i.c2.y, (int)i.c2.x), 3, cv::Scalar(0, 0, 255), -1);
	}
}

cv::Point2d q(BezierCurve bezCurve, double t)
{
	//cout << bezCurve.pt1 << " " << bezCurve.c1 << " " << bezCurve.c2 << " " << bezCurve.pt2 << endl;
	return B0(t) * bezCurve.pt1 + B1(t) * bezCurve.c1 + B2(t) * bezCurve.c2 + B3(t) * bezCurve.pt2;
}

void mulBezier(vector<BezierCurve>& bezCurves, double mul)
{
	for (auto& bezCurve : bezCurves)
	{
		bezCurve.pt1 *= mul;
		bezCurve.pt2 *= mul;
		bezCurve.c1 *= mul;
		bezCurve.c2 *= mul;
	}
}