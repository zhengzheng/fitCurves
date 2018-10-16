## fitCurvesCPP

C++ implementation of
"An Algorithm for Automatically Fitting Digitized Curves"
by Philip J. Schneider
from "Graphics Gems", Academic Press, 1990

# Contents

- [类](#类)
- [fitCurves 曲线拟合](#fitcurves-曲线拟合)
  - [FitCurve](#fitcurve)
  - [addBezierCurve](#addbeziercurve)
  - [FitCubic](#fitcubic)
  - [generateBezier](#generatebezier)
  - [reparameterize](#reparameterize)
  - [newtonRaphsonRootFind](#newtonraphsonrootfind)
  - [BezierII](#bezierii)
  - [B0, B1, B2, B3](b0-b1-b2-b3)
  - [ComputeLeftTangent, ComputeRightTangent, ComputeCenterTangent](#computelefttangent-computerighttangent-computecentertangent)
  - [ChordLengthParameterize](##chordlengthparameterize)
  - [computeMaxError](#computemaxerror)
  - [getDistance](#getdistance)
  - [scaleVec](#scalevec)
- [utils 杂项](#utils-杂项)



## 类

```c++
class BezierCurve
{
	public:
		cv::Point2d pt1;
		cv::Point2d pt2;
		cv::Point2d c1;
		cv::Point2d c2;
};
```

贝塞尔曲线控制点类

## fitCurves 曲线拟合

### FitCurve

```c++
/*
*  FitCurve :
*  	Fit a Bezier curve to a set of points.
*/
vector<BezierCurve> FitCurve(vector<cv::Point2d> const &d, // 输入的2D Points vector
						  int startIndex,                // 2D Points拟合曲线的起始索引
						  int endIndex, 			    // 2D Points拟合曲线的终止索引
						  double error)                  // 拟合曲线和2D Points之间的误差阈值
```

`FitCurve`为 fitCurvesCPP 的入口，它会调用 `computeLeftTangent`和`computeRightTangent`计算两个端点的单位正切向量（Unit tangent vectors），然后调用`FitCubic`拟合曲线，最终以`vector<BezierCurve>`返回拟合曲线的结果。

### addBezierCurve

```c++
// add curve to an array of curves which will then be returned on FitCurve.
void addBezierCurve(vector<cv::Vec2d> const &bezCurve,     // 生成的贝塞尔曲线
                    vector<BezierCurve> *bezCurves)        // 贝塞尔曲线的集合
```

将生成的贝塞尔曲线参数从`vector<cv::Vec2d>`转为`BezierCurve`并放入至`bezCurves`中

### FitCubic

```c++
/*
*  FitCubic :
*  	Fit a Bezier curve to a (sub)set of points
*/
void FitCubic(vector<cv::Point2d> const &d,      // 输入的2D Points vector
              vector<BezierCurve> *bezCurves,    // 返回 拟合贝塞尔曲线的集合
              int first,                         // 2D Points中拟合曲线的第一个点
              int last,                          // 2D Points中拟合曲线的最后一个点
              cv::Vec2d tHat1,                   // 2D Points中拟合曲线的第一个点的单位正切向量
              cv::Vec2d tHat2,                   // 2D Points中拟合曲线的最后一个点的单位正切向量
              double error)                      // 拟合曲线和2D Points之间的误差阈值
```

实际的拟合曲线函数，使用`chordLengthParameterize`方法参数化点，并尝试拟合曲线，找到点集中到拟合曲线的最大偏差`maxError`的点，判断`maxError`与`error`：

1. 若`maxError < error`: 将生成的贝塞尔曲线添加至`bezCurves`，结束
2. 若`maxError > error`，但`maxError < error*error`，即误差不是很大，尝试重新参数化并迭代计算，直至到达1或者达到最大迭代次数
3. 若`maxError > error*error`则拟合失败，在最大误差点分割点集，并递归拟合点集的两部分，直至到达1或者达到最大迭代次数

### generateBezier

```c++
/*
 *  GenerateBezier :
 *  Use least-squares method to find Bezier control points for region.
 */
void generateBezier(vector<cv::Point2d> const &d,   // 输入的2D Points vector
                    vector<cv::Vec2d> *bezCurve,    // 返回 拟合贝塞尔曲线
                    int first,                      // 2D Points中拟合曲线的第一个点
                    int last,                       // 2D Points中拟合曲线的最后一个点
                    vector<double> const &uPrime,   // 区域的参数值
                    cv::Vec2d tHat1,                // 2D Points中拟合曲线的第一个点的单位正切向量
                    cv::Vec2d tHat2)                // 2D Points中拟合曲线的最后一个点的单位正切向量
```

`generateBezier`根据所给参数，利用最小二乘法拟合2D Points

### reparameterize

```c++
/*
 *  Reparameterize:
 *	Given set of points and their parameterization, try to find
 *   a better parameterization.
 */
vector<double>* reparameterize(vector<cv::Point2d> const &d,  // 输入的2D Points vector
                               int first,                     // 2D Points中拟合曲线的第一个点
                               int last,                      // 2D Points中拟合曲线的最后一个点
                               vector<double> const &u,       // 当前参数值
                               vector<cv::Vec2d> *bezCurve)   // 已拟合贝塞尔曲线
```

`reparameterize`实现FitCubic中步骤2中，重新参数化点，尝试获得更好的参数值

### newtonRaphsonRootFind

```c++
/*
*  NewtonRaphsonRootFind :
*	Use Newton-Raphson iteration to find better root.
*/
double newtonRaphsonRootFind(vector<cv::Vec2d> const &Q,       // 当前拟合曲线
                             cv::Point2d P,                    // 2D Points点集
                             double u)                         // “P”的参数值
```

使用牛顿法加速计算曲线拟合

### BezierII

```c++
/*
*  Bezier :
*  	Evaluate a Bezier curve at a particular parameter value
*/
cv::Point2d bezierII(int degree,                         // 贝塞尔曲线的degree
                     vector<cv::Vec2d> const &V,         // 曲线的控制点集
                     double t)                           // 对应点的参数值
```

用于验证贝塞尔曲线

### B0, B1, B2, B3

```c++
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
```
### ComputeLeftTangent, ComputeRightTangent, ComputeCenterTangent

```c++
/*
* ComputeLeftTangent, ComputeRightTangent, ComputeCenterTangent :
* Approximate unit tangents at endpoints and "center" of the curve.
*/
cv::Vec2d computeLeftTangent(vector<cv::Point2d> const &d, int end)

cv::Vec2d computeRightTangent(vector<cv::Point2d> const &d, int end)

cv::Vec2d computeCenterTangent(vector<cv::Point2d> const &d, int center)
```

用于计算贝塞尔曲线左端点、右端点、以及“中心”点的单位正切向量

### ChordLengthParameterize

```c++
/*
*  ChordLengthParameterize :
*	Assign parameter values to points
*	using relative distances between points.
*/
void chordLengthParameterize(vector<cv::Point2d> const &d,    // 输入的2D Points vector
                             int first,                       // 2D Points中拟合曲线的第一个点
                             int last,                        // 2D Points中拟合曲线的最后一个点
                             vector<double> *u)               // 返回 参数化后的参数
```

实现`chordLengthParameterize`方法来参数化点

### computeMaxError

```c++
/*
*  ComputeMaxError :
*	Find the maximum squared distance of digitized points
*	to fitted curve.
*/
double computeMaxError(vector<cv::Point2d> const &d,         // 输入的2D Points vector
                       int first,                            // 2D Points中拟合曲线的第一个点
                       int last,                             // 2D Points中拟合曲线的最后一个点
                       vector<cv::Vec2d> *bezCurve,          // 已拟合的贝塞尔曲线
                       vector<double> const &u,              // 点集的参数化值
                       int *splitPoint)                      // 最大偏差`maxError`的点的索引
```

找到点集中到拟合曲线的最大偏差`maxError`的点的索引，并返回`maxError`

### getDistance

```c++
double getDistance(cv::Point2d p0, cv::Point p1)
```

计算两点间的欧几里得距离

### scaleVec

```c++
cv::Vec2d scaleVec(cv::Vec2d v, double newlen)
```

根据`newlen`归一化向量`v`的长度

## utils 杂项（一些与曲线拟合无关的函数）

### ReadImgs

```c++
/*
*  ReadImgs:
*  	Read Img1 and Img2, then merge them and split according to boundary
*   (the value lower than threshold will be zero).
*/
void ReadImgs(string imgPath1, string imgPath2, cv::Mat& leftImg, cv::Mat& rightImg, cv::Mat& img, int boundary)
```

读取图1和图2，并融合，根据boundary分成左图和右图，并二值化（只是用于测试...与曲线拟合无关）

### getNonZeroPoints

```c++
vector<cv::Point2d> getNonZeroPoints(cv::Mat& img,    // 输入图像
                                     int offset,      // x方向的偏移，一般为0
                                     int threshold)   // 阈值
```

得到图像中的大于threshold的点

### drawBezier 

```c++
void drawBezier(cv::Mat img,                        //  输入图像
                vector<BezierCurve> bezCurves,      //  贝塞尔曲线参数集合
                int nPts)                           //  每个贝塞尔曲线点的个数
```

将生成的多个贝塞尔曲线画到 img 中，其中每个贝塞尔曲线有 nPts个点

### drawControlPoints

```c++
void drawControlPoints(cv::Mat& img, vector<BezierCurve> bezCurves)
```

在img上画出贝塞尔曲线的控制点

### q

```c++
cv::Point2d q(BezierCurve bezCurve, double t)
```

根据 t 计算对应贝塞尔曲线的点的值

### mulBezier

```c++
void mulBezier(vector<BezierCurve>& bezCurves, double mul)
```

放大贝塞尔曲线，每个贝塞尔曲线的控制点乘以 mul 