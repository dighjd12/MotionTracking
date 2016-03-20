/*
 * DisplayImage.cpp
 *
 *  Created on: Feb 17, 2016
 *      Author: gamjatang1
 */

//#include <opencv2/cv.h>
#include <cv.h>
#include <highgui.h>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/core.hpp>
//#include "plot.hpp"
//#include <opencv2/plot.hpp>

using namespace cv;
using namespace std;

int blur_kernel_length = 5;
char window_name[20] = "original frame";

char x_window[] = "frame no vs x";
char y_window[] = "frame no vs y";

Mat x_plot;
Mat y_plot;

int frame_no = 0;
vector<Point> xyData = vector<Point>();
vector<double> thData = vector<double>();

vector<Point> prev_snake;
Mat image_orig;
VideoCapture cap;
//Mat data;
//int dataCounter = 0;

//global size object for resizing all images
int resize_height = 500;
int resize_width = 500;
Size size(resize_height,resize_width);

void plotAxis(Point cntr, double angle);
void redAndMoving(Mat frame, Mat prev_frame);
int blurImg(Mat& src, Mat& dst, int i);
int thresholdImgByHSV (Mat& src, Mat& dst);
int showImgWithHSV(Mat image);
int findAndDrawContours (Mat img_bw);
double getOrientation(const vector<Point> &, Mat&);
//int countOverlaps(vector<Point> candidate, vector<Point>& result);

int main( int argc, char** argv ) {

	cout << "hello world" << endl;

	cap = VideoCapture(argv[6]); // open the video
	double numFrames = cap.get(CV_CAP_PROP_FRAME_COUNT);
	int num_col = resize_width;
	int num_row = resize_height;
	//data.create(numFrames, 1, CV_64F);

	x_plot= Mat::zeros( num_col, numFrames, CV_8UC3 );
	y_plot = Mat::zeros( num_row, numFrames, CV_8UC3 );
	xyData.insert(xyData.end(), Point(0,0));
	thData.insert(thData.end(), 0.0);


	if(!cap.isOpened())  // check if we succeeded
		return -1;

	/* loops through the video and processes the images.
	 * there will be three windows:
	 * 		1) binary frame that shows the motion of the robot.
	 * 		2) thresholded frame by red
	 * 		3) original frame with the contour and pca drawn */
	Mat prev_frame;
	int frameCounter = 0;
	for(;;)
	{
		Mat frame;
		cap >> frame;
		if (frame.empty()) { //EOF
					  break;
		}
		if (frameCounter != 0){
			redAndMoving(frame, prev_frame);
		}

		/* if key pressed is 'f', then will stop and show hsv values of that frame.
		 * press any other key to quit that mode.
		 * press other keys to exit the program. */
		int res = waitKey(20);
		if (res % 256 == 102){
			showImgWithHSV(frame);
		} else if(res >= 0) {
			break;
		}

		frameCounter++;
		frame.copyTo(prev_frame);
	}

	return 0;

}

/* processes the frame. prev_frame is used to find difference in frames to detect motion.
 * */
void redAndMoving(Mat frame, Mat prev_frame){

	resize(frame,frame,size);
	resize(prev_frame,prev_frame,size);

	Mat diffImg;
	Mat diff_thresh;
	Mat result;
	Mat prev_frame_gray;
	Mat frame_gray;
	Mat prev_frame_blurred;
	Mat frame_blurred;
	Mat diff_gray;
	Mat diff_blurred;

	/* create a binary image that detects pixels that have moved between previous frame
	 * and the current frame. */
	GaussianBlur( frame, frame_blurred, Size( 3, 3 ), 0, 0 );
	GaussianBlur( prev_frame, prev_frame_blurred, Size( 3, 3 ), 0, 0 );
	absdiff(prev_frame_blurred, frame_blurred, diffImg);

	cvtColor(diffImg, diff_gray, CV_RGB2GRAY);
	GaussianBlur( diff_gray, diff_blurred, Size( 3, 3 ), 0, 0 );
	threshold( diff_blurred, diff_thresh, 20, 255, THRESH_BINARY );

	/* dilate the motion image to fill 'holes'*/
	int dilation_type = MORPH_RECT;
	int dilation_size = 1;

	Mat element_diff = getStructuringElement( dilation_type,
									   Size( 2*dilation_size + 1, 2*dilation_size+1 ),
									   Point( dilation_size, dilation_size ) );
	dilate(diff_thresh, result, element_diff);
	imshow("motion frames", result);

	/* creates binary image that is a result of
	 * thresholding the original image by red hues. */
	image_orig = frame;
	Mat image_blurred;
	Mat image_threshed;
	Mat image_final;

	blurImg(image_orig, image_blurred, blur_kernel_length);
	thresholdImgByHSV(image_blurred, image_threshed);
	dilation_type = MORPH_ELLIPSE;
	dilation_size = 2;

	Mat element_thresh = getStructuringElement( dilation_type,
									   Size( 2*dilation_size + 1, 2*dilation_size+1 ),
									   Point( dilation_size, dilation_size ) );
	dilate(image_threshed, image_final, element_thresh);
	imshow("thresholded by red frames", image_final);

	/* find and draw contours on the original image,
	 * given the thresholded image*/
	findAndDrawContours (image_final);
	imshow( window_name, image_orig );

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

    /* draw the principal components */
    circle(img, cntr, 3, Scalar(255, 0, 255), 2);
    Point p1 = cntr + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
    line(img, cntr, p1, Scalar(0, 255, 0), 1, CV_AA); //principal axis
    Point p2 = cntr - 0.08 *Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
    line(img, cntr, p2, Scalar(255, 255, 0), 1, CV_AA); //second principal axis

    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians

    plotAxis (cntr, angle);

    return angle;
}

/* plots center and angle of the axes on one window */
void plotAxis(Point cntr, double angle){
	//TODO: do time instead of frame no?? *.*

	Point prevPoint = xyData.back();

	/* inverted y just for easy visualization */
	Point prevX = Point(frame_no, prevPoint.x);
	Point prevY = Point(frame_no, resize_height-prevPoint.y);
	frame_no++;
	Point currX = Point(frame_no, cntr.x);
	Point currY = Point(frame_no, resize_height-cntr.y);

	line(x_plot, prevX, currX, Scalar(255, 255, 255), 1, CV_AA);
	line(y_plot, prevY, currY, Scalar(255, 255, 255), 1, CV_AA);
	//line(img, cntr, p2, Scalar(255, 255, 255), 1, CV_AA);

	xyData.insert(xyData.end(), cntr);

	imshow( x_window, x_plot );
	imshow( y_window, y_plot );
}

//TODO: maybe only do this after doing normal contour finding and num contours not 1?
/* finds and draws contour of the snake on the original image given the thresholded image.
 * if the snake is not 'found' yet, then will simply draw out all 'candidate' snakes on the image
 * otherwise, will use the previously defined snake to find the most matching snake. */
int findAndDrawContours (Mat img_bw){

		double area_lower_bound = 1e3;
		double area_upper_bound = 1e10;
	    if (!prev_snake.empty()){
	    	// if previous snake is defined,
	    	//find a bounding box for the prev_snake and bloat it to account for some movement
	    	//mask the image using the box and find the contours only within that area

	    	area_lower_bound = 1e2;

			int scale = 1;
			int bloat_amount = 10; //TODO: way to calculate good bloat amount using scale, size of orig img??

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

	    // Find all the contours in the result image
		vector<Vec4i> hierarchy;
		vector<vector<Point> > contours;
		findContours(img_bw, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	    vector<int> candidates = vector<int>();
	    for (size_t i = 0; i < contours.size(); ++i)
	    {
	        // Calculate the area of each contour
	        double area = contourArea(contours[i]);

	        // Ignore contours that are too small or too large
	        if (area < area_lower_bound || area_upper_bound < area) continue;

	        candidates.insert(candidates.end(), i);
	    }

	    // if previous snake is not defined, and we have none or multiple candidates,
	    // then we just draw all candidates until we find the snake
	    if (prev_snake.empty() && candidates.size()!=1){
				//TODO: what should i do? more robust needed

			//just draw all the viable candidates out
			for (size_t i = 0; i < candidates.size(); ++i)
			{

				int candidate_idx = candidates[i];
				// Draw each contour only for visualisation purposes
				drawContours(image_orig, contours, static_cast<int>(candidate_idx), Scalar(0, 0, 255), 2, 8, hierarchy, 0);
				// Find the orientation of each shape
				getOrientation(contours[candidate_idx], image_orig);
			}
	    }
	   else{

			vector<Point> snakes = vector<Point>();
			for (size_t i = 0; i < candidates.size(); ++i) { //loop through all the candidates
				int candidate_idx = candidates[i];
				drawContours(image_orig, contours, static_cast<int>(candidate_idx), Scalar(0, 0, 255), 2, 8, hierarchy, 0);
				snakes.insert(snakes.end(), contours[candidate_idx].begin(), contours[candidate_idx].end());
				//}//TODO: what if the snake is merged with another red stuff??/ ++maybe consider prev prev snake
			}
			getOrientation(snakes, image_orig);
			prev_snake = snakes;
	    }

	    return 0;
}

/*
int countOverlaps(vector<Point> candidate, vector<Point>& result){


	Mat mean;
	reduce(candidate, mean, 0, CV_REDUCE_AVG, -1);
	Point mean_pt(mean.at<float>(0,0), mean.at<float>(0,1));

	if (find(prev_snake.begin(), prev_snake.end(), mean_pt) != prev_snake.end()){ //found the mean in prev snake
		return 1;
	}
	return 0;


	int counter = 0;

	for (size_t i = 0; i < candidate.size(); ++i){
		if (find(prev_snake.begin(), prev_snake.end(), candidate[i]) != prev_snake.end()){
			counter++;
		}
		//TODO: also find the intersection of those and then also add candidate pixels that are not "far away" from those
	}

	return counter;

}*/

/* event-based onMouse function that will show the hsv values of the pixel
 * that the mouse is hovering on */
static void onMouse( int event, int x, int y, int f, void* ){
	Mat image = image_orig.clone();

	Mat HSV_1;
	Mat HSV;
	cvtColor(image, HSV_1,CV_BGR2HSV);
	medianBlur ( HSV_1, HSV,  blur_kernel_length);

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

	imshow( window_name, HSV );
}

static void onMouseX( int event, int x, int y, int f, void* ){

	Mat x_plot_copy = x_plot.clone();

    char name[30];
    sprintf(name,"frame number=%d",x);
    putText(x_plot_copy,name, Point(25,40) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,255,0), 2,8,false );

    int coord = -1;
	try{
		coord = xyData.at(x).x;
	} catch (std::out_of_range){
		coord = -1;
	}
    sprintf(name,"X=%d",coord);
    putText(x_plot_copy,name, Point(25,300) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,0,255), 2,8,false );

	imshow( x_window, x_plot_copy );
}

/* event-based onMouse function that will show the hsv values of the pixel
 * that the mouse is hovering on */
static void onMouseY( int event, int x, int y, int f, void* ){

	Mat y_plot_copy = y_plot.clone();

	char name[30];
	sprintf(name,"frame number=%d",x);
	putText(y_plot_copy,name, Point(25,40) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,255,0), 2,8,false );

	int coord = -1;
	try{
		coord = xyData.at(x).y;
	} catch (std::out_of_range){
		coord = -1;
	}
	sprintf(name,"Y=%d", coord);
	putText(y_plot_copy,name, Point(25,300) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,0,255), 2,8,false );

	imshow( y_window, y_plot_copy );
}

/* shows the hsv image where mouse location will tell the pixel value
 * press a key to exit */
int showImgWithHSV(Mat image){

	Mat dst;
	resize(image, dst, size);

	setMouseCallback( window_name, onMouse, 0 );
	setMouseCallback( x_window, onMouseX, 0 );
	setMouseCallback( y_window, onMouseY, 0 );
	imshow(window_name, dst);
	imshow(x_window, x_plot);
	imshow(y_window, y_plot);

	waitKey(0);

	return 0;
}

/* blurs the source image and stores in dst image. currently using median blur
 * 		i - the kernel size of the operation */
int blurImg(Mat& src, Mat& dst, int i){

	Mat src_hsv;
	cvtColor(src, src_hsv, CV_BGR2HSV);
	medianBlur ( src_hsv, dst, i );
	//GaussianBlur( src, dst, Size( i, i ), 0, 0 );
	//   	   blur( src, dst, Size( i, i ), Point(-1,-1) );
	//	 bilateralFilter ( src, dst, i, i*2, i/2 );

	return 0;
}

/* thresholds the src image using hsv hues of red, and stores in dst image */
int thresholdImgByHSV (Mat& src, Mat& dst) {

	Mat lower_red;
	Mat upper_red;

	inRange(src, Scalar(0, 50, 50), Scalar(15, 150, 150), lower_red);
	inRange(src, Scalar(165, 30, 30), Scalar(179, 255, 255), upper_red);

	bitwise_or(lower_red, upper_red, dst);

	return 0;
}
