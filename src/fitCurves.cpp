/*
C++ implementation of
An Algorithm for Automatically Fitting Digitized Curves
by Philip J. Schneider
from "Graphics Gems", Academic Press, 1990
*/

#include<string>
#include<iostream>
#include<cmath>
#include<opencv2/core/core.hpp>

#include"fitCurves.h"

#define MAXPOINTS 1000

using namespace std;

/*
*  FitCurve :
*  	Fit a Bezier curve to a set of points.
*/
vector<BezierCurve> FitCurve(vector<cv::Point2d> const &d, int startIndex, int endIndex, double error)
{
	if (error <= 0.0) throw "error value must be greater than 0.0";

	cv::Point2d tHat1, tHat2;	   // Unit tangent vectors at endpoints.
	vector<BezierCurve> bezCurves; // The vector that will store the BezierCurve 
								   // values to be returned once the curve fitting 
								   // is complete.
	// if startIndex is the beginning of the curve.
	tHat1 = computeLeftTangent(d, startIndex);
	// if endIndex is the end of the curve.
	tHat2 = computeRightTangent(d, endIndex - 1);
	FitCubic(d, &bezCurves, startIndex, endIndex - 1, tHat1, tHat2, error);

	return bezCurves;
}

void addBezierCurve(vector<cv::Vec2d> const &bezCurve, vector<BezierCurve> *bezCurves)
{
	// add curve to an array of curves which will then be returned on FitCurve.
	BezierCurve newBezier;
	newBezier.pt1 = bezCurve[0];
	newBezier.pt2 = bezCurve[3];
	newBezier.c1 = bezCurve[1];
	newBezier.c2 = bezCurve[2];
	bezCurves->push_back(newBezier);
}

/*
*  FitCubic :
*  	Fit a Bezier curve to a (sub)set of points
*/
void FitCubic(vector<cv::Point2d> const &d, vector<BezierCurve> *bezCurves,
	int first, int last, cv::Vec2d tHat1, cv::Vec2d tHat2, double error)
{
	// Control points of fitted Bezier curve;
	vector<cv::Vec2d> bezCurve(4);
	// Parameter values for point
	vector<double> *u = new vector<double>(last - first + 1);
	// Improved parameter values
	vector<double> *uPrime = new vector<double>(last - first + 1);

	double maxError;     // Maximum fitting error
	int splitPoint;      // Point to split point set at
	int nPts;            // Number of points in subset 
	double iterationError; // Error below which you try iterating
	int maxIterations = 20; // Max times to try iterating
	cv::Point2d tHatCenter; // Unit tangent vector at splitPoint
	int i;

	iterationError = error * error;
	nPts = last - first + 1;
	if (nPts == 1)
	{
		cout << "Only have 1 point, so no fitting" << endl;
		return;
	}

	//  Use heuristic if region only has two points in it
	if (nPts == 2)
	{
		double dist = getDistance(d[last], d[first]) / 3.0;
		bezCurve[0] = d[first];
		bezCurve[3] = d[last];
		bezCurve[1] = bezCurve[0] + scaleVec(tHat1, dist);
		bezCurve[2] = bezCurve[3] + scaleVec(tHat2, dist);
		addBezierCurve(bezCurve, bezCurves);
		cout << "Fit 2 Points, use heuristic" << endl;
		return;
	}

	// Parameterize points, and attempt to fit curve
	chordLengthParameterize(d, first, last, u);
	generateBezier(d, &bezCurve, first, last, *u, tHat1, tHat2);

	//  Find max deviation of points to fitted curve    
	maxError = computeMaxError(d, first, last, &bezCurve, *u, &splitPoint);
	//cout << maxError << endl;
	//cout << splitPoint << endl;
	if (maxError < error) {
		addBezierCurve(bezCurve, bezCurves);
		return;
	}

	//  If error not too large, try some reparameterization
	//  and iteration 
	if (maxError < iterationError) {
		for (i = 0; i < maxIterations; ++i) {
			uPrime = reparameterize(d, first, last, *u, &bezCurve);
			generateBezier(d, &bezCurve, first, last, *uPrime, tHat1, tHat2);
			maxError = computeMaxError(d, first, last, &bezCurve, *uPrime, &splitPoint);
			if (maxError < error) {
				addBezierCurve(bezCurve, bezCurves);
				return;
			}
			u = uPrime;
		}
	}

	// Fitting failed -- split at max error point and fit recursively
	tHatCenter = computeCenterTangent(d, splitPoint);
	FitCubic(d, bezCurves, first, splitPoint, tHat1, tHatCenter, error);
	FitCubic(d, bezCurves, splitPoint, last, -tHatCenter, tHat2, error);
}


void generateBezier(vector<cv::Point2d> const &d, vector<cv::Vec2d> *bezCurve,
	int first, int last, vector<double> const &uPrime, cv::Vec2d tHat1, cv::Vec2d tHat2)
{
	cv::Vec2d A[MAXPOINTS][2];  // Precomputed rhs for eqn
	int nPts;               // Number of pts in sub-curve
	double C[2][2];         // Matrix C
	double X[2];            // Matrix X
	double det_C0_C1, det_C0_X, det_X_C1; // Determinants of matrices
	double alpha_l, alpha_r; // Alpha values, left and right
	cv::Point2d tmp;        // Utility variable

	nPts = last - first + 1;

	// Compute the A
	for (int i = 0; i < nPts; ++i)
	{
		cv::Vec2d v1, v2;
		v1 = scaleVec(tHat1, B1(uPrime[i]));
		v2 = scaleVec(tHat2, B2(uPrime[i]));
		A[i][0] = v1;
		A[i][1] = v2;
	}

	// Create the C and X matrices
	C[0][0] = 0.0;
	C[0][1] = 0.0;
	C[1][0] = 0.0;
	C[1][1] = 0.0;
	X[0] = 0.0;
	X[1] = 0.0;

	for (int i = 0; i < nPts; i++) {
		C[0][0] += A[i][0].dot(A[i][0]);
		C[0][1] += A[i][0].dot(A[i][1]);

		C[1][0] = C[0][1];

		C[1][1] += A[i][1].dot(A[i][1]);

		tmp = (d[first + i]) - 
				((d[first] * B0(uPrime[i])) +
					((d[first] * B1(uPrime[i])) +
						((d[last] * B2(uPrime[i])) +
							(d[last] * B3(uPrime[i])))));

		X[0] += A[i][0].dot(tmp);
		X[1] += A[i][1].dot(tmp);
	}

	// Compute the determinants of C and X
	det_C0_C1 = C[0][0] * C[1][1] - C[1][0] * C[0][1];
	det_C0_X = C[0][0] * X[1] - C[1][0] * X[0];
	det_X_C1 = X[0] * C[1][1] - X[1] * C[0][1];

	// Finally, derive alpha values
	alpha_l = (det_C0_C1 == 0) ? 0.0 : det_X_C1 / det_C0_C1;
	alpha_r = (det_C0_C1 == 0) ? 0.0 : det_C0_X / det_C0_C1;

	// Checks for "dangerous" points, meaning that the alpha_l or alpha_r are abnormally large
	// from here http://newsgroups.derkeiler.com/Archive/Comp/comp.graphics.algorithms/2005-08/msg00419.html
	// This is a common problem with this algoithm.

	double dif1 = getDistance(d[first], d[last]);
	bool danger = false;
	if ((alpha_l > dif1 * 2) || (alpha_r > dif1 * 2)) {
		first += 0;
		danger = true;
	}

	//  If alpha negative, use the Wu/Barsky heuristic (see text)
	//  (if alpha is 0, you get coincident control points that lead to
	//  divide by zero in any subsequent NewtonRaphsonRootFind() call.
	double segLength = getDistance(d[first], d[last]);
	double epsilon = 1.0e-6 * segLength;
	if (alpha_l < epsilon || alpha_r < epsilon || danger)
	{
		// fall back on standard (probably inaccurate) formula, and subdivide further if needed. 
		double dist = segLength / 3.0;
		bezCurve->at(0) = d[first];
		bezCurve->at(3) = d[last];
		bezCurve->at(1) = bezCurve->at(0) + scaleVec(tHat1, dist);
		bezCurve->at(2) = bezCurve->at(3) + scaleVec(tHat2, dist);
		return; //bezCurve;
	}

	//  First and last control points of the Bezier curve are
	//  positioned exactly at the first and last data points 
	//  Control points 1 and 2 are positioned an alpha distance out
	//  on the tangent vectors, left and right, respectively
	bezCurve->at(0) = d[first];
	bezCurve->at(3) = d[last];
	bezCurve->at(1) = bezCurve->at(0) + scaleVec(tHat1, alpha_l);
	bezCurve->at(2) = bezCurve->at(3) + scaleVec(tHat2, alpha_r);
}

/*
*  Reparameterize:
*	Given set of points and their parameterization, try to find
*   a better parameterization.
*/

vector<double>* reparameterize(vector<cv::Point2d> const &d, int first, int last, vector<double> const &u, vector<cv::Vec2d> *bezCurve)
{
	int nPts = last - first + 1;
	vector<double> *uPrime = new vector<double>(nPts); //  New parameter values

	for (int i = first; i <= last; i++) {
		uPrime->at(i - first) = newtonRaphsonRootFind(*bezCurve, d[i], u.at(i - first));
	}
	return uPrime;
}

/*
*  NewtonRaphsonRootFind :
*	Use Newton-Raphson iteration to find better root.
*/

double newtonRaphsonRootFind(vector<cv::Vec2d> const &Q, cv::Point2d P, double u)
{
	double numerator, denominator;
	vector<cv::Vec2d> Q1(3), Q2(2); //  Q' and Q''
	cv::Vec2d Q_u, Q1_u, Q2_u; // u evaluated at Q, Q', & Q''
	double uPrime; // Improved u

	// Compute Q(u)
	Q_u = bezierII(3, Q, u);

	// Generate control vertices for Q'
	for (int i = 0; i <= 2; i++) {
		Q1[i][0] = (Q[i + 1][0] - Q[i][0]) * 3.0;
		Q1[i][1] = (Q[i + 1][1] - Q[i][1]) * 3.0;
	}

	// Generate control vertices for Q'' 
	for (int i = 0; i <= 1; i++) {
		Q2[i][0] = (Q1[i + 1][0] - Q1[i][0]) * 2.0;
		Q2[i][1] = (Q1[i + 1][1] - Q1[i][1]) * 2.0;
	}

	// Compute Q'(u) and Q''(u)	
	Q1_u = bezierII(2, Q1, u);
	Q2_u = bezierII(1, Q2, u);

	// Compute f(u)/f'(u) 
	numerator = (Q_u[0] - P.x) * (Q1_u[0]) + (Q_u[1] - P.y) * (Q1_u[1]);
	denominator = (Q1_u[0]) * (Q1_u[0]) + (Q1_u[1]) * (Q1_u[1]) +
		(Q_u[0] - P.x) * (Q2_u[0]) + (Q_u[1] - P.y) * (Q2_u[1]);
	if (denominator == 0.0) return u;

	// u = u - f(u)/f'(u) 
	uPrime = u - (numerator / denominator);
	return (uPrime);
}

/*
*  Bezier :
*  	Evaluate a Bezier curve at a particular parameter value
*/

cv::Point2d bezierII(int degree, vector<cv::Vec2d> const &V, double t)
{
	cv::Point2d Q;  // Point on curve at parameter t	
	cv::Vec2d *Vtemp; // Local copy of control points

	// copy array
	Vtemp = new cv::Vec2d[degree + 1];
	for (int i = 0; i <= degree; ++i)
	{
		Vtemp[i] = V[i];
	}
	// Triangle computation	
	for (int i = 1; i <= degree; i++) {
		for (int j = 0; j <= degree - i; j++) {
			Vtemp[j][0] = (1.0 - t) * Vtemp[j][0] + t * Vtemp[j + 1][0];
			Vtemp[j][1] = (1.0 - t) * Vtemp[j][1] + t * Vtemp[j + 1][1];
		}
	}
	Q = Vtemp[0];
	return Q;
}

/*
*  B0, B1, B2, B3 :
*	Bezier multipliers
*/

double B0(double u)
{
	double tmp = 1.0 - u;
	return (tmp * tmp * tmp);
}
double B1(double u)
{
	double tmp = 1.0 - u;
	return (3 * u * (tmp * tmp));
}

double B2(double u)
{
	double tmp = 1.0 - u;
	return (3 * u * u * tmp);
}

double B3(double u)
{
	return (u * u * u);
}

/*
* ComputeLeftTangent, ComputeRightTangent, ComputeCenterTangent :
* Approximate unit tangents at endpoints and "center" of the curve.
*/
cv::Vec2d computeLeftTangent(vector<cv::Point2d> const &d, int end)
{
	cv::Vec2d tHat1;

	if (end == 0)
	{
		tHat1 = d[end + 1] - d[end];
	}else {
		tHat1 = d[end + 1] - d[end - 1];
	}
	cv::normalize(tHat1, tHat1);
	return tHat1;
}

cv::Vec2d computeRightTangent(vector<cv::Point2d> const &d, int end)
{
	cv::Vec2d tHat2;
	if (end == d.size() - 1)
	{
		tHat2 = d[end - 1] - d[end];
	}else {
		tHat2 = d[end - 1] - d[end + 1];
	}
	cv::normalize(tHat2, tHat2);
	return tHat2;
}

cv::Vec2d computeCenterTangent(vector<cv::Point2d> const &d, int center)
{
	cv::Vec2d V1, V2, tHatCenter;
	V1 = d[center - 1] - d[center];
	V2 = d[center] - d[center + 1];
	tHatCenter[0] = (V1[0] + V2[0]) / 2.0;
	tHatCenter[1] = (V1[1] + V2[1]) / 2.0;
	cv::normalize(tHatCenter, tHatCenter);
	return tHatCenter;
}

/*
*  ChordLengthParameterize :
*	Assign parameter values to points
*	using relative distances between points.
*/
void chordLengthParameterize(vector<cv::Point2d> const &d, int first, int last, vector<double> *u)
{
	int i;
	u->at(0) = 0.0;
	for (i = first + 1; i <= last; ++i)
	{
		u->at(i - first) = u->at(i - first - 1) + getDistance(d[i], d[i - 1]);
	}

	for (i = first + 1; i <= last; ++i)
	{
		u->at(i - first) = u->at(i - first) / u->at(last - first);
	}
}

/*
*  ComputeMaxError :
*	Find the maximum squared distance of digitized points
*	to fitted curve.
*/

double computeMaxError(vector<cv::Point2d> const &d, int first, int last,
	vector<cv::Vec2d> *bezCurve, vector<double> const &u, int *splitPoint)
{
	double maxDist; // Maximum error
	double dist; // Current error
	cv::Point2d P; // Point on curve
	cv::Vec2d v; // Vector from point to curve

	*splitPoint = (last - first + 1) / 2;
	maxDist = 0.0;
	for (int i = first + 1; i < last; ++i)
	{
		P = bezierII(3, *bezCurve, u[i - first]);
		v = P - d[i];
		dist = v[0] * v[0] + v[1] * v[1];
		if (dist >= maxDist)
		{
			maxDist = dist;
			*splitPoint = i;
		}
	}
	return (maxDist);
}

double getDistance(cv::Point2d p0, cv::Point p1)
{
	double dx = p0.x - p1.x;
	double dy = p0.y - p1.y;
	return sqrt(dx*dx + dy * dy);
}

cv::Vec2d scaleVec(cv::Vec2d v, double newlen)
{
	double len = sqrt(v[0] * v[0] + v[1] * v[1]);
	if (len != 0.0)
	{
		v[0] *= newlen / len;
		v[1] *= newlen / len;
	}
	return v;
}