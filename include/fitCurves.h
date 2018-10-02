#pragma once
#include<opencv2/core/core.hpp>
using namespace std;

class BezierCurve
{
	public:
		cv::Point2d pt1;
		cv::Point2d pt2;
		cv::Point2d c1;
		cv::Point2d c2;
};
//typedef vector<double, double> vec2d;

vector<BezierCurve> FitCurve(vector<cv::Point2d> const &d, int startIndex, int endIndex, double error);
void addBezierCurve(vector<cv::Vec2d> const &bezCurve, vector<BezierCurve> *bezCurves);
void FitCubic(vector<cv::Point2d> const &d, vector<BezierCurve> *bezCurves,
	int first, int last, cv::Vec2d tHat1, cv::Vec2d tHat2, double error);
void generateBezier(vector<cv::Point2d> const &d, vector<cv::Vec2d> *bezCurve,
	int first, int last, vector<double> const &uPrime, cv::Vec2d tHat1, cv::Vec2d tHat2);

vector<double>* reparameterize(vector<cv::Point2d> const &d, int first, int last, vector<double> const &u, vector<cv::Vec2d> *bezCurve);
double newtonRaphsonRootFind(vector<cv::Vec2d> const &Q, cv::Point2d P, double u);
cv::Point2d bezierII(int degree, vector<cv::Vec2d> const &V, double t);

double B0(double u);
double B1(double u);
double B2(double u);
double B3(double u);

cv::Vec2d computeLeftTangent(vector<cv::Point2d> const &d, int end);
cv::Vec2d computeRightTangent(vector<cv::Point2d> const &d, int end);
cv::Vec2d computeCenterTangent(vector<cv::Point2d> const &d, int center);
void chordLengthParameterize(vector<cv::Point2d> const &d, int first, int last, vector<double> *u);

double computeMaxError(vector<cv::Point2d> const &d, int first, int last,
	vector<cv::Vec2d> *bezCurve, vector<double> const &u, int *splitPoint);
double getDistance(cv::Point2d p0, cv::Point p1);
cv::Vec2d scaleVec(cv::Vec2d v, double newlen);
cv::Vec2d negateVec(cv::Vec2d v);


