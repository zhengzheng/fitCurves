# fitCurves

Using C++ and Python implement of Philip J. Schneider's "An Algorithm for Automatically Fitting Digitized Curves"

Adaptively fit a set of 2D points with one or more cubic Bezier curves.

The C++ implementations uses OpenCV. (In addition to reading the picture and drawing the picture part, the rest of the dependency of OpenCV will be removed later.)

And the C++ and Python implementations refer to [fitCurves](https://github.com/volkerp/fitCurves) and the original C code.

- [fitCurves](https://github.com/volkerp/fitCurves) has a example gui application (demo.py) using Tkinter

- The original C code is available on <http://graphicsgems.org/> as well as in <https://github.com/erich666/GraphicsGems>

The test.cpp in the folder src and  process.py in the folder py are just some test files that can be ignored and will be removed later. 

### Briefly Documentation

[Chinese中文](https://github.com/sikasjc/fitCurves/blob/master/src/DOC_CN.md)

## ToDo


- [ ] Remove the dependencies of OpenCV in the fitting curve section from C++ code.

