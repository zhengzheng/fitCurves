#pragma once
#include<string>
#include<opencv2/core/core.hpp>

using namespace std;

void ReadImgs(string imgPath1, string imgPath2, cv::Mat& leftImg, cv::Mat& rightImg, cv::Mat& img, int boundary = 25);
vector<cv::Point2d> getNonZeroPoints(cv::Mat& img, int offset, int threshold = 100);
void drawBezier(cv::Mat img, vector<BezierCurve> bezCurves, int nPts);
void drawControlPoints(cv::Mat& img, vector<BezierCurve> bezCurves);
cv::Point2d q(BezierCurve bezCurve, double t);
void mulBezier(vector<BezierCurve>& bezCurves, double mul);