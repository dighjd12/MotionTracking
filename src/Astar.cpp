/*
 * Astar.cpp
 *
 *  Created on: Apr 24, 2016
 *      Author: gamjatang1
 */

#include <cv.h>
#include <highgui.h>
#include "Display.h"
#include "astar.h"

namespace AStar{

	using namespace cv;
	using namespace std;

	int GRID_SIZE = 40;
	double final_angle = M_PI/2; //TODO make this user-specified

	// overlay transparent image and draw on it!!
	// rotation + final dst angle* bugg......
	// TODO log the sequence of actions + astar every 10 frame*****
	//docs + interface/ highlight major parts

	// state , bounds for rotation
	// astar every 10 frame?
	// astar in snake frame
	// obstacles?

	// implement on the snake .. order rotation + cost for motions/actions
	// simulations ? testing
	//c++ 11

	Point3f src;
	Point3f dst;
	double angle;

	vector<node> current_path;
	vector<Point3f> visited_set;

	action roll, rotate, forward;
	action actions[3];
	vector<node*> pq;

	Mat img;
	//Mat overlay_img;

	//install actions and fill in moves
	static void makeActions(){

		action roll;
		action forward;
		action rotate;
		roll.cost = &roll_cost;
		roll.succ = &roll_succ;
		forward.cost = &forward_cost;
		forward.succ = &forward_succ;
		rotate.cost = &rotate_cost;
		rotate.succ = &rotate_succ;

		actions[0] = roll;
		actions[1] = forward;
		actions[2] = rotate;

	}

	void drawGrids(Mat &image, int grid_size){

		img = image;
		int width=image.cols;
		int height=image.rows;

		for(int i=0; i<height; i+=grid_size){
		  line(image, Point(0,i), Point(width,i), Scalar(255,255,255));
		}

		for(int i=0; i<width; i+=grid_size){
		  line(image, Point(i,0), Point(i,height), Scalar(255,255,255));
		}

	}

	void planPathOnVideo (Mat &image, Point src_pt, Point dst_pt, double curr_angle, double dst_angle){

		//overlay_img = Mat(image.size, CV_64FC1);


		final_angle = dst_angle;

		roundToGridPoint(src_pt, GRID_SIZE);
		roundToGridPoint(dst_pt, GRID_SIZE);
		//round curr angle
		double curr_angle2 = 0;//((curr_angle + (M_PI/2-1)) / (M_PI/2)) * (M_PI/2); //rounds to multiple of pi/2

		char name[100];
		sprintf(name,"source is: x=%d, y=%d, th=%.2f. destination is: x=%d, y=%d, th=%.2f.", src_pt.x, src_pt.y, curr_angle, dst_pt.x, dst_pt.y, dst_angle);
		putText(image, name, Point(25,40) , FONT_HERSHEY_SIMPLEX, .7, Scalar(255,255,255), 2,8,false );

		planPath (src_pt, dst_pt, curr_angle2);


	}


	void roundToGridPoint(Point &pt, int grid_size){

		//rounds down
		int x = pt.x - (pt.x % grid_size);
		int y = pt.y - (pt.y % grid_size);

		pt.x = x;
		pt.y = y;

	}

	void roundToGridPoint(Point3f &pt, int grid_size){

		//rounds down
		int x = (int) pt.x - (((int) pt.x) % grid_size);
		int y = (int) pt.y - (((int) pt.y) % grid_size);

		pt.x = x;
		pt.y = y;

	}

	void freePtrs(){

		if (!pq.empty()){
			for (int i=0; i<pq.size(); i++){
				
			}
		}

		if (!current_path.empty()){

		}

	}


	void planPath(Point src_pt, Point dst_pt, double curr_angle){

		src = Point3f(src_pt.x, src_pt.y, curr_angle);
		dst = Point3f(dst_pt.x, dst_pt.y, curr_angle);
		angle = curr_angle; //start angle
		visited_set = vector<Point3f>();
		pq = vector<node *>();

		//do the astar
		makeActions();

		//TODO be able to tell if dst is reachable from src
		node *start_node = new node;
		start_node->priority = 0;
		start_node->dist = 0;
		start_node->pos = Point3f(src_pt.x, src_pt.y, curr_angle);
		start_node->prev = nullptr;
		//move is undefined!
		insertPQ(start_node);

		while (!pq.empty()){

			node *n = delminPQ();

			if (n->pos.x == dst_pt.x && n->pos.y == dst_pt.y && n->pos.z > final_angle-M_PI/6 && n->pos.z < final_angle+M_PI/6){ //TODO bounds check by quadrants!

				cout << "done astar" << endl;

					vector<node> path = vector<node>();
					while (n != nullptr){

						//print the pt/action or store in array
						path.insert(path.begin(), *n);
						node *temp = n->prev;
						//delete n;
						n = temp;
					}

					current_path = path;
					//drawPath(path);
					return;

			}

			if (!visited(n->pos)){

				insertVisited(n->pos);
				//expand
				for (int i=0; i<(sizeof(actions)/sizeof(*actions)); i++){

					vector<Point3f> candidates  = actions[i].succ (n->pos);

					for (int j = 0; j < candidates.size(); j++){

						if (isValid(n->pos, candidates[j]) && !visited(candidates[j])){ //detect obstacle using this method, isValid! TODO

							node *new_node = new node;
							new_node->dist = n->dist + actions[i].cost(n->pos, candidates[j]);
							new_node->priority = new_node->dist + heuristics(candidates[j]);
							new_node->pos = candidates[j];

							new_node->prev = n;
							new_node->move = actions[i];

							insertPQ(new_node);
							//checkPQ();
						}
					}
				}
			}
		}
	}

	void drawPath(){

		if (current_path.empty()) return;
		vector<node> path = current_path;

		for (int j = 0; j < path.size()-1; j++){

			line(image_orig, Point(path[j].pos.x, path[j].pos.y), Point(path[j+1].pos.x, path[j+1].pos.y), Scalar(0, 255, 0), 1, CV_AA);
/*
			if (path[j].pos.x == path[j+1].pos.x && path[j].pos.y == path[j+1].pos.y){
				//rotation
				//cout << "?" << endl; //??????? three rotation and a diagonal ???????????
				circle(image_orig, Point(path[j].pos.x, path[j].pos.y), 3, Scalar(255, 0, 255), 2);
				char name[50];
				sprintf(name,"rotate %.2f", path[j+1].pos.z - path[j].pos.z);
				putText(image_orig, name, Point(path[j].pos.x, path[j].pos.y-5), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,0,255), 2,8,false );
			}else{
				line(image_orig, Point(path[j].pos.x, path[j].pos.y), Point(path[j+1].pos.x, path[j+1].pos.y), Scalar(0, 255, 0), 1, CV_AA);
			}*/
		}

	}


	/* ################# */
	/* graph search methods  */
	/* ################# */

	void insertPQ(node *new_node){

		int insert_idx = pq.size();
		for (int i = 0; i < pq.size(); i++) {
			if ((pq[i])->priority > new_node->priority){
				insert_idx = i;
				break;
			}

		}

		pq.insert(pq.begin() + insert_idx, new_node);
	}

	void checkPQ(){

		for (int i = 0; i < pq.size()-1; i++) {

				if ((pq[i])->priority > (pq[i+1])->priority){
					cout << "PQ WRONG!!!" << endl;
					for (int i = 0; i < pq.size(); i++){
						cout << pq[i]->priority << endl;
					}

					while(true){

					}
				}
			}

	}

	//call only if pq is not empty!
	node *delminPQ(){

		node *min_node = pq[0];
		pq.erase(pq.begin());
		return min_node;

	}

	bool isValid (Point3f start, Point3f end){

		//boundary check
		if (end.x > img.cols || end.x < 0 || end.y > img.rows || end.y < 0){
			return false;
		}

		return true;
	}


	bool visited(Point3f pt){

		for (int j = 0; j < visited_set.size(); j++){

			if (pt.x == visited_set[j].x && pt.y == visited_set[j].y && pt.z == visited_set[j].z){
				return true;
			}
		}
		return false;
	}

	void insertVisited(Point3f pt){
		visited_set.insert(visited_set.end(),pt);
	}

	double euclidean_dist(Point3f start, Point3f end){
		return sqrt((start.x-end.x)*(start.x-end.x) + (start.y-end.y)*(start.y-end.y)); //TODO should return float?
	}

	double heuristics(Point3f curr){
		return 0;//euclidean_dist(curr, dst);
	}

	double rotate_cost (Point3f start, Point3f end){
		return 1;
	}

	double roll_cost (Point3f start, Point3f end){
		return 1;//euclidean_dist(start, end);
	}

	double forward_cost (Point3f start, Point3f end){
		return 1;//euclidean_dist(start, end);
	}

	vector<Point3f> rotate_succ (Point3f curr){ //TODO: currently only 4-way connected, assuming th is aligned with one of the axes.

		//cout << "here is called!" << endl;

		double angle = curr.z + M_PI/2;
		angle = fmod(angle + M_PI,2*M_PI);
		if (angle < 0)
			angle += 2*M_PI;
		angle =  angle - M_PI;
		Point3f p1 = Point3f(curr.x, curr.y, angle);

		angle = curr.z - M_PI/2;
		angle = fmod(angle + M_PI,2*M_PI);
		if (angle < 0)
			angle += 2*M_PI;
		angle =  angle - M_PI;
		Point3f p2 = Point3f(curr.x, curr.y, angle);

		angle = curr.z + M_PI;
		angle = fmod(angle + M_PI,2*M_PI);
		if (angle < 0)
			angle += 2*M_PI;
		angle =  angle - M_PI;
		Point3f p3 = Point3f(curr.x, curr.y, angle);

		angle = curr.z - M_PI;
		angle = fmod(angle + M_PI,2*M_PI);
		if (angle < 0)
			angle += 2*M_PI;
		angle =  angle - M_PI;
		Point3f p4 = Point3f(curr.x, curr.y, angle);

		roundToGridPoint(p1, GRID_SIZE);
		roundToGridPoint(p2, GRID_SIZE);
		roundToGridPoint(p3, GRID_SIZE);
		roundToGridPoint(p4, GRID_SIZE);
		vector<Point3f> v1 = vector<Point3f>();
		v1.insert(v1.begin(), p1);
		v1.insert(v1.begin(), p2);
		v1.insert(v1.begin(), p3);
		v1.insert(v1.begin(), p4);
		return v1;
	}

	vector<Point3f> roll_succ (Point3f curr){ //orientation head of our snake (rad)

		Point3f p1 = Point3f(curr.x + 3/2*GRID_SIZE*cos(curr.z+M_PI/2), curr.y + 3/2*GRID_SIZE*sin(curr.z+M_PI/2), curr.z);
		Point3f p2 = Point3f(curr.x + 3/2*GRID_SIZE*cos(curr.z-M_PI/2), curr.y + 3/2*GRID_SIZE*sin(curr.z-M_PI/2), curr.z);
		roundToGridPoint(p1, GRID_SIZE);
		roundToGridPoint(p2, GRID_SIZE);
		vector<Point3f> v1 = vector<Point3f>();
		v1.insert(v1.begin(), p1);
		v1.insert(v1.begin(), p2);
		return v1;

	}

	vector<Point3f> forward_succ (Point3f curr){

		Point3f p1 = Point3f(curr.x + 3/2*GRID_SIZE*cos(curr.z), curr.y +3/2*GRID_SIZE*sin(curr.z), curr.z);
		Point3f p2 = Point3f(curr.x + 3/2*GRID_SIZE*cos(curr.z+M_PI), curr.y +3/2*GRID_SIZE*sin(curr.z+M_PI), curr.z);
		roundToGridPoint(p1, GRID_SIZE);
		roundToGridPoint(p2, GRID_SIZE);
		vector<Point3f> v1 = vector<Point3f>();
		v1.insert(v1.begin(), p1);
		v1.insert(v1.begin(), p2);
		return v1;
	}


}
