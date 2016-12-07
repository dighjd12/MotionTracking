/*
 * DisplayImage.cpp
 *
 *  Created on: Feb 17, 2016
 *      Author: gamjatang1
 */

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <fstream>
#include "Display.h"

using namespace cv;
using namespace std;

#include "astar.h"
using namespace AStar;

const int blur_kernel_length = 5;
const char original_frame_window[20] = "original frame";
const char red_thresholded_window[30] = "thresholded by red (hsv)";

int frameCounter = 0;

double dst_angle = M_PI/2; //TODO make this user-specified

// vectors of length = low_pass_length 
// used for low-pass filter
vector<double> xData = vector<double>();
vector<double> yData = vector<double>();
vector<double> thData = vector<double>();
vector<double> th2Data = vector<double>();
int low_pass_length = 15;
Mat weightsMat;

// mat to record data throughout the session and export to csv
Mat dataMat;

//vector<Point> prev_snake;
Mat image_orig;
clock_t start;
VideoCapture cap;

// indicates that enough points have incremented to run astar
bool pt_flag = false;
// indicates that destination point has been specified
bool dst_flag = false;
Point3f curr_pt;
//double curr_angle = 0;
Point3f dst_pt;

//global size object for resizing all images
int resize_height = 500;
int resize_width = 1000;
Size size(resize_width,resize_height);

// + video cap vs realtime
// connected components? masking the background, ml to optimize the thresholding
// at first, point to where about the snake will be,
//threshold choosing??

//downsampling

//robot watching the robot

//have the webcam watch the robot from far, remove all objects that interfere with thresholding

//axis ---> make it consistent, s.t. green one is the longer axis

// things to consider: (+= bounds for rotation and translation) how to send commands to rotation (watch until it reaches the right angle,,
//etc), adding more actions, blocking out (or clicking on ) obstacles
//mix of actions to consider

int lower_red_lower_bound;
int lower_red_upper_bound;
int upper_red_lower_bound;
int upper_red_upper_bound;

Mat curr_frame;

bool live = false;
int numFrameChunk = 500;

/* applies segmenting on the event that
 threshold values change on gui*/
void on_trackbar( int, void* ){
	redAndMoving(curr_frame);
}

/* sets destination point */
static void setDst( int event, int x, int y, int f, void* ){
	if  ( event == EVENT_LBUTTONDOWN ){
		dst_flag = true;
		dst_pt = Point3f(x,y, dst_angle);
	}

}

int main( int argc, char** argv ) {

	DEBUG(cout << "hello world" << endl;)
	if (argc!=3){ //maybe use flag?
		cout << "invalid args" << endl;
		return -1;
	}else{
		if (atoi(argv[1]) == 1){
			live = true;

		}else{
			live = false;
		}
	}

	if (live){
		cap = VideoCapture(1);
		dataMat = Mat(numFrameChunk, 4,  CV_64FC1);
	}else{
		cap = VideoCapture(argv[2]); // open the video
		double numFrames = cap.get(CV_CAP_PROP_FRAME_COUNT);

		dataMat = Mat(numFrames, 4,  CV_64FC1);
	}
	//cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
	//cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);

	lower_red_lower_bound = 0;
	lower_red_upper_bound = 15;
	upper_red_lower_bound = 165;
	upper_red_upper_bound = 179;

	//weighted moving average
	vector<double> weights = vector<double>();
	//double ratio = 1/(double)low_pass_length;
	for (int i=1; i<=low_pass_length; i++){
		weights.insert(weights.end(), i);
	}
	weightsMat = Mat(weights, true);

	namedWindow(original_frame_window, 1);
	//namedWindow(red_thresholded_window, 1);

	int const max_val = 255;
	//createTrackbar( "lo red, lower bound", red_thresholded_window, &lower_red_lower_bound, max_val, on_trackbar );
	//createTrackbar( "lo red, upper bound", red_thresholded_window, &lower_red_upper_bound, max_val, on_trackbar );
	//createTrackbar( "up red, lower bound", red_thresholded_window, &upper_red_lower_bound, max_val, on_trackbar );
	//createTrackbar( "up red, upper bound", red_thresholded_window, &upper_red_upper_bound, max_val, on_trackbar );

	if(!cap.isOpened())  // check if we succeeded
		return -1;

	setMouseCallback( original_frame_window, setDst, 0 );
	makeActions();
	/* loops through the video and processes the images.
	 * there will be three windows:
	 * 		1) binary frame that shows the motion of the robot.
	 * 		2) thresholded frame by red
	 * 		3) original frame with the contour and pca drawn */
	for(;;)
	{
		Mat frame;
		cap >> frame;

		if (frame.empty()) { //EOF
			 EXPORT(exportToCSV(dataMat, "dataCSV.csv");)
					  break;
		}

		resize(frame,frame,size);
		curr_frame = frame;
		image_orig = frame;

		DEBUG(start = clock();)
		redAndMoving(frame); //threshold + pca analysis
		DEBUG(cout << "duration for vision algs " << (clock() - start) / (double) CLOCKS_PER_SEC << endl;)

		drawGrids(frame, GRID_SIZE);
		if (pt_flag && dst_flag){
			DEBUG(start = clock();)
			planPathOnVideo (frame, curr_pt, dst_pt, frameCounter);
			DEBUG(cout << "duration for planning algs " << (clock() - start) / (double) CLOCKS_PER_SEC << endl;)
		}
		imshow( original_frame_window, frame );

		/* if key pressed is 'f', then will stop and show hsv values of that frame.
		 * press any other key to quit that mode.
		 * press other keys to exit the program. */
		int res = waitKey(10);
		if (res % 256 == 102){
			showImgWithHSV(frame); //TODO when stop, show the picture with the axes on it
		} else if(res >= 0) {
			EXPORT(exportToCSV(dataMat, "dataCSV.csv");)
			break;
		}

		frameCounter++;
	}

	return 0;

}

/* export the matrix into csv file */
void exportToCSV(Mat &matrix, string filename){

	std::ofstream outputFile(filename);
	outputFile << format(matrix, "CSV") << endl;
	outputFile.close();

}

/* 
******************************
********** CV codes **********
******************************
*/

/* processes the frame.
 * */
void redAndMoving(Mat frame){

	/* creates binary image that is a result of
	 * thresholding the original image by red hues. */
	Mat image_blurred;
	Mat image_threshed;
	Mat image_final;
	clock_t temp;

	DEBUG(temp = clock();)
	blurImg(image_orig, image_blurred, blur_kernel_length);
	DEBUG(cout << "duration for blurring " << (clock() - temp) / (double) CLOCKS_PER_SEC << endl;)

	DEBUG(temp = clock();)
	thresholdImgByHSV(image_blurred, image_threshed);
	DEBUG(cout << "duration for thresholding " << (clock() - temp) / (double) CLOCKS_PER_SEC << endl;)

	int dilation_type = MORPH_ELLIPSE;
	int dilation_size = 2;
	Mat element_thresh = getStructuringElement( dilation_type,
									   Size( 2*dilation_size + 1, 2*dilation_size+1 ),
									   Point( dilation_size, dilation_size ) );
	dilate(image_threshed, image_final, element_thresh);
	//imshow(red_thresholded_window, image_final);

	/* find and draw contours on the original image,
	 * given the thresholded image*/
	DEBUG(temp = clock();)
	findAndDrawContours (image_final, frame);
	DEBUG(cout << "duration for segmentation and contouring " << (clock() - temp) / (double) CLOCKS_PER_SEC << endl;)

}


/* does pca analysis using the set of input points,
 * and draws the axis and center on the input img */
double getOrientation(const vector<Point> &pts, Mat &img) {

	/* copy data points to Mat datastructure */
    int sz = static_cast<int>(pts.size());
    Mat data_pts = Mat(sz, 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }

    /* perform pca using the data points */
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);

    /* store the center and eigenvalues and eigenvectors of the result */
    Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                      static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }

    //to make the axis point to some consistent direction, cast the signs explicitly
    //eigen_vecs[0].y = eigen_vecs[0].y > 0 ? -eigen_vecs[0].y : eigen_vecs[0].y; //y neg x pos
    //eigen_vecs[0].x = eigen_vecs[0].x > 0 ? eigen_vecs[0].x : -eigen_vecs[0].x;
    //eigen_vecs[1].y = eigen_vecs[1].y > 0 ? eigen_vecs[1].y : -eigen_vecs[1].y; //y pos, x pos
    //eigen_vecs[1].x = eigen_vecs[1].x > 0 ? eigen_vecs[1].x : -eigen_vecs[1].x;

    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
    double angle2 = atan2(eigen_vecs[1].y, eigen_vecs[1].x); //angle of our second principal axis

    //axis might jump around. add pi to angles that are about opposite from previous angles explicitly
    if (thData.size() != 0){

    	double prev_val =  thData.at(thData.size()-1) - 2*M_PI;
    	double bounds = M_PI/2;

    	//first quad
    	if (prev_val >= 0 && prev_val <= M_PI/2){
    		if (angle <= 0 && angle > prev_val-bounds){
    			//within bounds
    		}else if (angle > 0 && angle < prev_val + bounds){
    			//within bounds
    		}else{
    			angle = angle + M_PI;
    		}
    	}

    	//second quad
    	if (prev_val > M_PI/2 && prev_val <= M_PI){
			if (angle <= 0 && angle < prev_val + bounds - 2*M_PI){
				//within bounds
			}else if (angle > 0 && angle > prev_val - bounds){
				//within bounds
			}else{
				angle = angle + M_PI;
			}
		}

    	//fourth quad
		if (prev_val < 0 && prev_val >= -M_PI/2){
			if (angle <= 0 && angle > prev_val - bounds){
				//within bounds
			}else if (angle > 0 && angle < prev_val + bounds){
				//within bounds
			}else{
				angle = angle + M_PI;
			}
		}

		//third quad
		if (prev_val < -M_PI/2 && prev_val >= -M_PI){
			if (angle >= 0 && angle > prev_val - bounds + 2*M_PI){
				//within bounds
			}else if (angle < 0 && angle < prev_val + bounds){
				//within bounds
			}else{
				angle = angle + M_PI;
			}
		}
    }

    if (th2Data.size() != 0){

      	double prev_val =  th2Data.at(th2Data.size()-1) - 2*M_PI;
      	double bounds = M_PI/2;

      	//first quad
      	if (prev_val >= 0 && prev_val <= M_PI/2){
      		if (angle2 <= 0 && angle2 > prev_val-bounds){
      		}else if (angle2 > 0 && angle2 < prev_val + bounds){
      		}else{
      			angle2 = angle2 + M_PI;
      		}
      	}

      	//second quad
      	if (prev_val > M_PI/2 && prev_val <= M_PI){
  			if (angle2 <= 0 && angle2 < prev_val + bounds - 2*M_PI){
  			}else if (angle2 > 0 && angle2 > prev_val - bounds){
  			}else{
  				angle2 = angle2 + M_PI;
  			}
  		}

      	//fourth quad
  		if (prev_val < 0 && prev_val >= -M_PI/2){
  			if (angle2 <= 0 && angle2 > prev_val - bounds){
  			}else if (angle2 > 0 && angle2 < prev_val + bounds){
  			}else{
  				angle2 = angle2 + M_PI;
  			}
  		}

  		//third quad
  		if (prev_val < -M_PI/2 && prev_val >= -M_PI){
  			if (angle2 >= 0 && angle2 > prev_val - bounds + 2*M_PI){
  				//within bounds
  			}else if (angle2 < 0 && angle2 < prev_val + bounds){
  				//within bounds
  			}else{
  				angle2 = angle2 + M_PI;
  			}
  		}
      }


    /* [-180, 180] normalization for angles */
    angle = fmod(angle + M_PI,2*M_PI);
	if (angle < 0)
		angle += 2*M_PI;
	angle =  angle - M_PI;

	angle2 = fmod(angle2 + M_PI,2*M_PI);
	if (angle2 < 0)
		angle2 += 2*M_PI;
	angle2 =  angle2 - M_PI;

	/* record data */
    EXPORT(
    if (dataMat.rows <= frameCounter){
    	Mat temp;
    	copyMakeBorder(dataMat, temp, 0, numFrameChunk, 0, 0, BORDER_CONSTANT, 0 );
    	dataMat = temp;
    }
    dataMat.at <double> (frameCounter, 0) = cntr.x;
    dataMat.at <double>(frameCounter, 1) = cntr.y;
    dataMat.at <double> (frameCounter, 2) = angle; // radians
    dataMat.at <double> (frameCounter, 3) = angle * 180/M_PI; //degrees
    )

    /* low pass filter */
    if (xData.size() < low_pass_length){
    		xData.insert(xData.end(), cntr.x);
    		yData.insert(yData.end(), cntr.y);
    		thData.insert(thData.end(), angle + 2*M_PI);
    		th2Data.insert(th2Data.end(), angle2 + 2*M_PI);
    } else{

    		pt_flag = true;

    		xData.erase(xData.begin());
    		yData.erase(yData.begin());
    		thData.erase(thData.begin());
    		th2Data.erase(th2Data.begin());
    		xData.insert(xData.end(), cntr.x);
    		yData.insert(yData.end(), cntr.y);
    		thData.insert(thData.end(), angle + 2*M_PI);
    		th2Data.insert(th2Data.end(), angle2 + 2*M_PI);

			vector<double> thPrime = vector<double>();
    		vector<double> th2Prime = vector<double>();
    		thPrime.insert(thPrime.end(), thData[0]);
    		th2Prime.insert(th2Prime.end(), th2Data[0]);
    		for (int i=1; i<low_pass_length; i++){
    			if ((thData[i]-thPrime[i-1]) < -M_PI/2){
    				thPrime.insert(thPrime.end(), thData[i] + 2*M_PI);
    			}
    			else if ((thData[i]- (thPrime[i-1]) > M_PI/2)){
    				thPrime.insert(thPrime.end(), thData[i] - 2*M_PI);
    			}else {
    				thPrime.insert(thPrime.end(), thData[i]);
    			}
    			if ((th2Data[i]-th2Prime[i-1]) < -M_PI){
    				th2Prime.insert(th2Prime.end(), th2Data[i] + 2*M_PI);
    				//th2Prime[i] = th2Data[i] + 2*M_PI;
    			}
    			else if ((th2Data[i]- (th2Prime[i-1]) > M_PI)){
    				th2Prime.insert(th2Prime.end(), th2Data[i] - 2*M_PI);
    				//th2Prime[i] = th2Data[i] - 2*M_PI;
    			}else {
    				th2Prime.insert(th2Prime.end(), th2Data[i]);
    			}
    		}
    	/*	for (int i=1; i<low_pass_length; i++){
    			cout << " " << thData[i] << " vs " << thPrime[i];
    		}
    		cout << endl;
    		for (int i=1; i<low_pass_length; i++){
    			cout << " " << th2Data[i] << " vs " << th2Prime[i];
    		}
    		cout << endl;*/
    		
    		
    		//cout << thPrime.size() << endl;

    		double denom = (low_pass_length*(low_pass_length+1))/2;

    		double avg_x = (weightsMat.dot(Mat(xData,true)))/denom;
    		double avg_y = (weightsMat.dot(Mat(yData,true)))/denom;
    		//double avg_th = (weightsMat.dot(Mat(thData,true)))/denom;
    		//double avg_th2 = (weightsMat.dot(Mat(th2Data,true)))/denom;
    		double avg_th = (weightsMat.dot(Mat(thPrime,true)))/denom;
    		double avg_th2 = (weightsMat.dot(Mat(th2Prime,true)))/denom;

    		cntr = Point(avg_x, avg_y);
    		angle = avg_th;
    		angle = fmod(angle + M_PI,2*M_PI);
			if (angle < 0)
				angle += 2*M_PI;
			angle =  angle - M_PI;
    		//curr_angle = angle;
    		angle2 = avg_th2;
    		angle2 = fmod(angle2 + M_PI,2*M_PI);
			if (angle2 < 0)
				angle2 += 2*M_PI;
			angle2 =  angle2 - M_PI;
    		curr_pt = Point3f(avg_x, avg_y, angle);

    		   
    	/*	if (!inBoundsAngle(angle2, angle, M_PI/2+1.0)){
    			cout << "angles : " << angle << " " << angle2 << endl; 
    			//cout << "angles : " << angle3 << " " << angle4 << endl; 
    			cout << "*****************here!!!!!!!!!!!!!!!!!! "<< endl;
    			for (int i=0; i<low_pass_length; i++)
    				cout << ", " << thData[i];
    			cout << endl;
    			for (int i=0; i<low_pass_length; i++)
    				cout << ", " << thPrime[i];
    			cout << endl;

    			for (int i=0; i<low_pass_length; i++)
    				cout << ", " << th2Data[i];
    			cout << endl;
    			for (int i=0; i<low_pass_length; i++)
    				cout << ", " << th2Prime[i];
    			cout << endl;
    			waitKey(0);
    		}*/

    		 /* draw the principal components */
			circle(img, cntr, 3, Scalar(255, 0, 255), 2);
			Point p1 = cntr + 0.02 * Point(static_cast<int>(cos(angle) * eigen_val[0]), static_cast<int>(sin(angle) * eigen_val[0]));
			line(img, cntr, p1, Scalar(0, 255, 0), 1, CV_AA); //principal axis - green axis
			Point p2 = cntr - 0.12 *Point(static_cast<int>(cos(angle2) * eigen_val[1]), static_cast<int>(sin(angle2) * eigen_val[1]));
			line(img, cntr, p2, Scalar(255, 255, 0), 1, CV_AA); //second principal axis

    }

    return angle;
}

/* finds and draws contour of the snake on the original image given the thresholded image.
 * if the snake is not 'found' yet, then will simply draw out all 'candidate' snakes on the image
 * otherwise, will use the previously defined snake to find the most matching snake. */
int findAndDrawContours (Mat img_bw, Mat& img_orig){

		//double area_lower_bound = 5e3;
		//double area_upper_bound = 1e10;
	/*    if (!prev_snake.empty()){
	    	// if previous snake is defined,
	    	//find a bounding box for the prev_snake and bloat it to account for some movement
	    	//mask the image using the box and find the contours only within that area

	    	area_lower_bound = 1e2;

			int scale = 1;
			int bloat_amount = 10;

			Rect box = boundingRect(prev_snake);
			//x,y is top, left
			int bloat_x = max(box.x - bloat_amount*scale, 0);
			int bloat_y = max(box.y - bloat_amount*scale, 0);
			int bloat_width = min(box.width + 2*bloat_amount*scale, img_bw.cols - bloat_x);
			int bloat_height = min(box.height + 2*bloat_amount*scale, img_bw.rows - bloat_y);

			Rect new_box = Rect(bloat_x, bloat_y, bloat_width, bloat_height);
			Mat mask(img_bw.size(), CV_8U, Scalar::all(0)); //1 is white 0 is black
			mask(new_box).setTo(Scalar::all(1));

			Mat image_masked;
			bitwise_and(mask, img_bw, image_masked);
			img_bw = image_masked;
	    }
	    */

	    // Find all the contours in the result image
		vector<Vec4i> hierarchy;
		vector<vector<Point> > contours;
		findContours(img_bw, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

		int max_idx = -1;
		double max_area = 0;
	    for (size_t i = 0; i < contours.size(); ++i)
	    {
	        // Calculate the area of each contour
	        double area = contourArea(contours[i]);

	        // Ignore contours that are too small or too large
	     //   if (area < area_lower_bound || area_upper_bound < area) continue;

	        if (max_idx == -1){
	        	max_area = area;
	        	max_idx = i;
	        }else if (area > max_area){
	        	max_idx = i;
	        	max_area = area;
	        }

	    }

	    if (max_idx != -1){
	    	drawContours(img_orig, contours, static_cast<int>(max_idx), Scalar(0, 0, 255), 2, 8, hierarchy, 0);
	    	getOrientation(contours[max_idx], img_orig);
	    }

	    return 0;
}

/* event-based onMouse function that will show the hsv values of the pixel
 * that the mouse is hovering on */
static void onMouse( int event, int x, int y, int f, void* ){
	Mat image = image_orig.clone();

	//Mat HSV_1;
	Mat HSV;
	blurImg(image, HSV, blur_kernel_length);

	//cvtColor(image, HSV_1,CV_BGR2HSV);
	//medianBlur ( HSV_1, HSV,  );

    Vec3b hsv=HSV.at<Vec3b>(y,x);
    int H=hsv.val[0];
    int S=hsv.val[1];
    int V=hsv.val[2];

    char name[30];
    sprintf(name,"H=%d",H);
    putText(HSV,name, Point(25,40) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,255,0), 2,8,false );

    sprintf(name,"S=%d",S);
    putText(HSV,name, Point(25,80) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,255,0), 2,8,false );

    sprintf(name,"V=%d",V);
    putText(HSV,name, Point(25,120) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,255,0), 2,8,false );

    sprintf(name,"X=%d",x);
    putText(HSV,name, Point(25,300) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,0,255), 2,8,false );

    sprintf(name,"Y=%d",y);
    putText(HSV,name, Point(25,340) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,0,255), 2,8,false );

	imshow( original_frame_window, HSV );
}


/* shows the hsv image where mouse location will tell the pixel value
 * press a key to exit */
int showImgWithHSV(Mat image){

	Mat image2 = image.clone();
	setMouseCallback( original_frame_window, onMouse, 0 );
	imshow(original_frame_window, image2);

	waitKey(0);

	return 0;
}

/* blurs the source image and stores in dst image. currently using median blur
 * 		i - the kernel size of the operation */
int blurImg(Mat& src, Mat& dst, int kernel_size){

	Mat src_hsv;
	cvtColor(src, src_hsv, CV_BGR2HSV);
	medianBlur ( src_hsv, dst, kernel_size );

	return 0;
}

/* thresholds the src image using hsv hues of red, and stores in dst image */
int thresholdImgByHSV (Mat& src, Mat& dst) {

	//yellow thresholding
	inRange(src, Scalar(20, 30, 50), Scalar(35, 255, 255), dst);

	//red thresholding
	
	/*Mat lower_red;
	Mat upper_red;

	inRange(src, Scalar(lower_red_lower_bound, 50, 50), Scalar(lower_red_upper_bound, 150, 150), lower_red);
	inRange(src, Scalar(upper_red_lower_bound, 30, 30), Scalar(upper_red_upper_bound, 255, 255), upper_red);

	bitwise_or(lower_red, upper_red, dst);
	*/
	return 0;
}
