/*
 * Display.h
 *
 *  Created on: Apr 24, 2016
 *      Author: gamjatang1
 */

#ifndef DISPLAY_H
#define DISPLAY_H


using namespace cv;
using namespace std;

extern Mat image_orig;
extern clock_t start;
extern Point curr_pt;

#define _DEBUG //if we want to print debug statements to stdout
#ifdef _DEBUG
#define DEBUG(x) x
#else 
#define DEBUG(x)
#endif

#define _EXPORT //if we want to export data to csv
#ifdef _EXPORT
#define EXPORT(x) x
#else 
#define EXPORT(x)
#endif

void exportToCSV(Mat &matrix, string filename);
void plotAxis(Point cntr, double angle);
void redAndMoving(Mat frame);
int blurImg(Mat& src, Mat& dst, int i);
int thresholdImgByHSV (Mat& src, Mat& dst);
int showImgWithHSV(Mat image);
int findAndDrawContours (Mat img_bw, Mat& img_orig);
double getOrientation(const vector<Point> &, Mat&);


#endif /* DISPLAY_H_ */
