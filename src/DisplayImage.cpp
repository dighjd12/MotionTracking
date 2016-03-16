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
//using namespace cv::plot;

int MAX_KERNEL_LENGTH = 31;
char window_name[14] = "Display Image";

Mat image_rgb;
Mat image_orig;
VideoCapture cap;
Mat data;
int dataCounter = 0;

//void plotAxis(Point cntr, double angle);
int processImg(Mat img);
int blurImg(Mat src, Mat dst, int i);
int showImg(Mat image);
int thresholdImgByHSV (Mat src, Mat dst);
int showImgWithHSV(Mat image);
int findAndDrawContours (Mat img_bw);
void drawAxis(Mat&, Point, Point, Scalar, const float);
double getOrientation(const vector<Point> &, Mat&);

int main( int argc, char** argv ) {
	//printing stdout?
	cout << "hello world" << endl;


	cap = VideoCapture(argv[5]); // open the video
	double numFrames = cap.get(CV_CAP_PROP_FRAME_COUNT);
	data.create(numFrames, 1, CV_64F);

	if(!cap.isOpened())  // check if we succeeded
		return -1;

	/*Mat frame;
	cap >> frame;
	showImgWithHSV(frame);
	return 0;*/
	    for(;;)
	    {
	        Mat frame;
	        cap >> frame;

	        if (frame.empty()) {
	        	        			 // printf( "end of file \n" );
	        	        			  break;
	        	        }
	  //      showImgWithHSV(frame);
	    //    pause();
	        processImg(frame);
	    //    showImgWithHSV(frame);

	        if(waitKey(30) >= 0) break;
	    }
	    // the camera will be deinitialized automatically in VideoCapture destructor
	    return 0;


    //Mat image_rgb;
	//  image_rgb = imread( argv[1], 1 );
	//Mat image_orig; //includes alpha channel if exists
//	image_orig = imread(argv[1], -1);
//	image_orig = imread(argv[1], 1);


	//processImg();

  return 0;
}

int processImg(Mat img){
	image_orig = img;
	Size size(800,800);//the dst image size,e.g.100x100
		Mat dst3;//dst image
		resize(image_orig,dst3,size);
		image_orig = dst3;

		  Mat dst;
		  Mat dst2;
		  blurImg(image_orig, dst, 31);
	//	  showImg(image_rgb);
		  thresholdImgByHSV(image_rgb, dst2);
//		  showImg(image_rgb);
		  findAndDrawContours (image_rgb);
		  showImg(image_orig);

		  return 0;
}

void drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale/* = 0.2*/) {
    double angle;
    double hypotenuse;
    angle = atan2( (double) p.y - q.y, (double) p.x - q.x ); // angle in radians
    hypotenuse = sqrt( (double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
//    double degrees = angle * 180 / CV_PI; // convert radians to degrees (0-180 range)
//    cout << "Degrees: " << abs(degrees - 180) << endl; // angle in 0-360 degrees range
    // Here we lengthen the arrow by a factor of scale
    q.x = (int) (p.x - scale * hypotenuse * cos(angle));
    q.y = (int) (p.y - scale * hypotenuse * sin(angle));
    line(img, p, q, colour, 1, CV_AA);
    // create the arrow hooks
    p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
    line(img, p, q, colour, 1, CV_AA);
    p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
    line(img, p, q, colour, 1, CV_AA);
}

double getOrientation(const vector<Point> &pts, Mat &img) {
    //Construct a buffer used by the pca analysis
    int sz = static_cast<int>(pts.size());
    Mat data_pts = Mat(sz, 2, CV_64FC1); //empty 1d (x,y) mat
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }

    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
    //Store the center of the object
    Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                      static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
    //Store the eigenvalues and eigenvectors
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }

    //cout << eigen_val[1] << endl;
   // cout << eigen_val[0] << endl;

    // Draw the principal components
    circle(img, cntr, 3, Scalar(255, 0, 255), 2);
    Point p1 = cntr + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
    Point p2 = cntr - 0.02 *Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
    drawAxis(img, cntr, p1, Scalar(0, 255, 0), 0.5);
    //..line(img, cntr, p1, Scalar(0, 255, 0), 1, CV_AA);
    drawAxis(img, cntr, p2, Scalar(255, 255, 0), 1);
    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians

  //  plotAxis (cntr, angle);

    return angle;
}

/*
void plotAxis(Point cntr, double angle){

	data.at(dataCounter++, 0) = cntr.x;
	 Mat plot_result;
	 Mat data2;
	 //Ptr<plot::Plot2d> plot;

	    Ptr<plot::Plot2d> plot2 = plot::createPlot2d( data2 );
	//    plot->setPlotBackgroundColor( Scalar( 50, 50, 50 ) ); // i think it is not implemented yet
	  //  plot->setPlotLineColor( Scalar( 50, 50, 255 ) );
	  //  plot->render( plot_result );

	    imshow( "plot", plot_result );

}*/

int findAndDrawContours (Mat img_bw){
	// Find all the contours in the thresholded image
	    vector<Vec4i> hierarchy;
	    vector<vector<Point> > contours;
	    findContours(img_bw, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	//	cout << "size of contours is" << contours.size() << endl;

	  //  vector<vector<Point> > allPoints;

	    for (size_t i = 0; i < contours.size(); ++i)
	    {
	        // Calculate the area of each contour
	        double area = contourArea(contours[i]);
	    	//cout << "current area is" << area << endl;

	        // Ignore contours that are too small or too large
	        if (area < 1e3 || 1e10 < area) continue;
	        // Draw each contour only for visualisation purposes
	        drawContours(image_orig, contours, static_cast<int>(i), Scalar(0, 0, 255), 2, 8, hierarchy, 0);
	        // Find the orientation of each shape
	        getOrientation(contours[i], image_orig);
	    }

	    return 0;
}

static void onMouse( int event, int x, int y, int f, void* ){
	Mat image;
	Size size(800,800);//the dst image size,e.g.100x100
	 // Mat dst;//dst image
	  resize(image_rgb,image,size);
	//  HSV = dst.clone();

	//Vec3b rgb=image.at<Vec3b>(y,x);
	//int B=rgb.val[0];
	//int G=rgb.val[1];
	//int R=rgb.val[2];

  Mat HSV;
  cvtColor(image, HSV,CV_BGR2HSV);
  //Mat RGB=image(Rect(x,y,1,1));

	//Mat image = sr.clone();
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

	//Mat dst;
	//Size size(800,800);//the dst image size,e.g.100x100
		//Mat dst;//dst image
	//	resize(HSV,dst,size);
 //imwrite("hsv.jpg",image);
 imshow( window_name, HSV );
}

//TODO: could onMouse take the input image??? Or make image_rgb global to a pointer and set it here
int showImgWithHSV(Mat image){
	Size size(800,800);//the dst image size,e.g.100x100
	Mat dst;//dst image
	Mat dst2;
	resize(image,dst,size);
	blurImg(dst, dst2, 31);
	//image_rgb = dst;

	namedWindow( window_name, CV_WINDOW_AUTOSIZE );
	setMouseCallback( window_name, onMouse, 0 );
	imshow(window_name, dst);

	waitKey(0);

	return 0;
}

int showImg(Mat image){

	//Size size(1200,1000);//the dst image size,e.g.100x100
	//Mat dst;//dst image
	//resize(image,dst,size);
	  namedWindow( "Display Image", CV_WINDOW_AUTOSIZE );
	  imshow( "Display Image", image );

	//  waitKey(0);

	  return 0;
}

int blurImg(Mat src, Mat dst, int i){

	//   	   blur( src, dst, Size( i, i ), Point(-1,-1) );
		medianBlur ( src, dst, i );
	//GaussianBlur( src, dst, Size( i, i ), 0, 0 );
		image_rgb = dst;
	  //

    //	 bilateralFilter ( src, dst, i, i*2, i/2 );
     return 0;
}

int thresholdImgByHSV (Mat src, Mat dst) {

	cvtColor(src, dst, CV_BGR2HSV);

	Mat lower_red;
	Mat upper_red;
	inRange(dst, Scalar(0, 70, 50), Scalar(10, 140, 140), lower_red);
	inRange(dst, Scalar(170, 30, 30), Scalar(179, 100, 255), upper_red);

	inRange(dst, Scalar(0, 50, 50), Scalar(10, 255, 255), lower_red);
	inRange(dst, Scalar(165, 30, 30), Scalar(179, 255, 255), upper_red);

	Mat dst2;
	bitwise_or(lower_red, upper_red, dst2);

	/*imshow("lowerred", lower_red);
	waitKey(0);
	imshow("hightred", upper_red);
		waitKey(0);
	imshow("or-ed", dst2);
	waitKey(0);*/

	image_rgb = dst2;

	return 0;
}
