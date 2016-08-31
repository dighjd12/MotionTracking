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
extern Point curr_pt;


void exportToCSV(Mat &matrix, string filename);
void plotAxis(Point cntr, double angle);
void redAndMoving(Mat frame);
int blurImg(Mat& src, Mat& dst, int i);
int thresholdImgByHSV (Mat& src, Mat& dst);
int showImgWithHSV(Mat image);
int findAndDrawContours (Mat img_bw, Mat& img_orig);
double getOrientation(const vector<Point> &, Mat&);


#endif /* DISPLAY_H_ */
