/*
 * Astar.cpp
 *
 *  Created on: Apr 24, 2016
 *      Author: gamjatang1
 */

#include <cv.h>
#include <highgui.h>
#include <string>
#include "Display.h"
#include "astar.h"
#include <queue>
#include <math.h>

#define ACTIONS_SIZE 3

namespace AStar{

	using namespace cv;
	using namespace std;

	int GRID_SIZE = 40;

	// change resolution ++ frame rate

	// reuse path in the astar
	// astar any-time invariant
	// putting pre-computed costs ** 

	// time the rotation
	// DEMO - estimate how far it moved / seqs of movements
	// as we drive, distnace reduce
	// cam looking at robot, move the robot according to the path/actions

	// overlay transparent image and draw on it!!
	// rotation + final dst angle* bugg...... TODO

	// state, bounds for rotation
	// astar in snake frame
	// obstacles?

	// implement on the snake .. order rotation + cost for motions/actions
    // simulations ? testing


	Point3f dst; //used by heuristics 

	vector<node *> current_path;
	const int astarFrameCount = 20;

	action roll, rotate_, forward_;
	action actions[ACTIONS_SIZE];
	void makeActions (void) __attribute__((constructor)); //calls makeActions before main

	vector<node*> pq;
	vector<node*> visited_set;

	//Mat overlay_img;


	/*
	* populates actions - array of all possible action for the snake
	*/
	void makeActions(){

		roll.name = "roll";
		forward_.name = "forward";
		rotate_.name = "rotate";
		roll.cost = &roll_cost;
		roll.succ = &roll_succ;
		forward_.cost = &forward_cost;
		forward_.succ = &forward_succ;
		rotate_.cost = &rotate_cost;
		rotate_.succ = &rotate_succ;

		actions[0] = roll;
		actions[1] = forward_;
		actions[2] = rotate_;

	}

	/*
	* draws n x n grid on the image
	* n = grid_size
	*/
	void drawGrids(Mat &image, int grid_size){

		int width=image.cols;
		int height=image.rows;

		for(int i=0; i<height; i+=grid_size){
		  line(image, Point(0,i), Point(width,i), Scalar(255,255,255));
		}

		for(int i=0; i<width; i+=grid_size){
		  line(image, Point(i,0), Point(i,height), Scalar(255,255,255));
		}

	}

	/*
	* rounds the 2D point down to upper left corner of the grid it's in
	*/
	void roundToGridPoint(Point &pt, int grid_size){

		//rounds down
		int x = pt.x - (pt.x % grid_size);
		int y = pt.y - (pt.y % grid_size);

		pt.x = x;
		pt.y = y;

	}

	/*
	* rounds the 3D point to upper left corner of the grid it's in
	*/
	void roundToGridPoint(Point3f &pt, int grid_size){

		//rounds down
		int x = (int) pt.x - (((int) pt.x) % grid_size);
		int y = (int) pt.y - (((int) pt.y) % grid_size);

		pt.x = x;
		pt.y = y;

	}


	/* 
	* outputs the path planning information 
	* and plans new path every astarFrameCount frame 
	* also draws the path on the image
	*/
	void planPathOnVideo (Mat &image, Point3f src_pt, Point3f dst_pt, int frameCounter){

		//overlay_img = Mat(image.size, CV_64FC1);

		roundToGridPoint(src_pt, GRID_SIZE);
		roundToGridPoint(dst_pt, GRID_SIZE);
		//round curr angle
		src_pt.z = 0; //TODO//((curr_angle + (M_PI/2-1)) / (M_PI/2)) * (M_PI/2); //rounds to multiple of pi/2

		char name[100];
		sprintf(name,"source is: x=%.0f, y=%.0f, th=%.2f. destination is: x=%.0f, y=%.0f, th=%.2f.", src_pt.x, src_pt.y, src_pt.z, dst_pt.x, dst_pt.y, dst_pt.z);
		putText(image, name, Point(25,40), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,255,255), 2,8,false );
		char distText[50];
		double dist = euclidean_dist2D(src_pt, dst_pt);
		sprintf(distText, "distance from current position to goal: %.02f", dist);
		putText(image, distText, Point(25,60), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,255,255), 2,8,false);

		if (frameCounter % astarFrameCount == 0){
			DEBUG(cout << "**************** replanning ****************" << endl;)
			//DEBUG(start = clock();)
			freePath();
			current_path = vector<node *>();
			planPath (src_pt, dst_pt, current_path);
			//DEBUG(cout << "		astar: " << (clock() - start) / (double) CLOCKS_PER_SEC << endl;)
		}
		drawPath(current_path);

	}

	/*
	* runs Astar
	*/
	void planPath(Point3f src_pt, Point3f dst_pt, vector<node *>& path){

		dst = dst_pt;
		visited_set = vector<node *>();
        auto cmp = [](node *left, node *right) {
            return left->priority < right->priority;
        };
        std::priority_queue<node *, std::vector<node *>,
            decltype(cmp)> pq;

		cout << "here" << actions[0].name << endl;

		//TODO be able to tell if dst is reachable from src
		node *start_node = new node;
		start_node->priority = 0;
		start_node->dist = 0;
		start_node->pos = src_pt;
		start_node->prev = nullptr;
		//move is undefined at start
		pq.push(start_node);

		while (!pq.empty()){
			//pq has lowest G cost

			node *n = pq.front();
			pq.pop();


			if (n->pos.x == dst_pt.x && n->pos.y == dst_pt.y && n->pos.z > dst_pt.z-M_PI/6 && n->pos.z < dst_pt.z+M_PI/6){ //TODO bounds check by quadrants!

					DEBUG(cout << "done astar" << endl;)
					while (n != nullptr){
						//copy values
						node *path_node = new node;
						path_node->dist = n->dist;
						path_node->pos = n->pos;
						path_node->move = n->move;
						path.insert(path.begin(), path_node);
						n = n->prev;
					}

					insertVisited(n);
					freeSets();
					return;

			}

			if (!visited(n->pos)){
				//expand
				for (int i=0; i<(sizeof(actions)/sizeof(*actions)); i++) {
					vector<Point3f> candidates  = actions[i].succ (n->pos);

					for (int j = 0; j < candidates.size(); j++){

						if (isValid(n->pos, candidates[j]) && !visited(candidates[j])) {

							node *new_node = new node;
							new_node->dist = n->dist + actions[i].cost(n->pos, candidates[j]);
							new_node->priority = new_node->dist + heuristics(candidates[j]);
							new_node->pos = candidates[j];

							new_node->prev = n;
							new_node->move = actions[i];


							pq.push(new_node);
						}
					}
				}
			}

			insertVisited(n);
		}
	}

	/*
	* draws path on the current frame 
	* if _DEBUG is defined, prints the action sequence to stdout
	*/
	void drawPath(vector<node *> path){

		if (path.empty()) return;

		for (int j = 0; j < path.size()-1; j++){
			if (path[j]->pos.x == path[j+1]->pos.x && path[j]->pos.y == path[j+1]->pos.y){
				//rotation
				circle(image_orig, Point(path[j]->pos.x, path[j]->pos.y), 3, Scalar(255, 0, 255), 2);
				char name[25];
				sprintf(name,"rotate %.2f", path[j+1]->pos.z - path[j]->pos.z);
				putText(image_orig, name, Point(path[j]->pos.x, path[j]->pos.y-5), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,0,255), 2,8,false );
				DEBUG(cout << name << endl;)
			}else{
				line(image_orig, Point(path[j]->pos.x, path[j]->pos.y), Point(path[j+1]->pos.x, path[j+1]->pos.y), Scalar(0, 255, 0), 1, CV_AA);
				DEBUG(cout << ((path[j+1])->move).name << " from (" << path[j]->pos.x << "," << path[j]->pos.y << ") to (" << path[j+1]->pos.x << "," << path[j+1]->pos.y << ")" << endl;)
			}
		}

	}

	/*
	* prints current path (actions to perform to reach dst) out to the stdout
	*/
	void printPath(){
		DEBUG(cout << "printing the action sequence" << endl);
		if (!current_path.empty()){
			for (int i=0; i<current_path.size(); i++){
				DEBUG(cout << ((current_path[i])->move).name << endl;)
			}
		}

	}

	/*
	* frees the data structures used by astar
	*/
	void freeSets(){
		if (!pq.empty()){
			for (int i=0; i<pq.size(); i++){
				delete pq[i];
			}
		}

		if (!visited_set.empty()){
			for (int i=0; i<visited_set.size(); i++){
				delete visited_set[i];
			}
		}
	}

	/*
	* frees current path
	*/
	void freePath(){
		if (!current_path.empty()){
			for (int i=0; i<current_path.size(); i++){
				delete current_path[i];
			}
		}
	}


	/* ################# */
	/* graph search methods  */
	/* ################# */

	/*void insertPQ(node *new_node){

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

	}*/

	bool isValid (Point3f start, Point3f end){

		//boundary check
		if (end.x > image_orig.cols || end.x < 0 || end.y > image_orig.rows || end.y < 0){
			return false;
		}

		return true;
	}


	bool visited(Point3f pt){

		for (int j = 0; j < visited_set.size(); j++){

			if (pt.x == visited_set[j]->pos.x && pt.y == visited_set[j]->pos.y && pt.z == visited_set[j]->pos.z){
				return true;
			}
		}
		return false;
	}

	void insertVisited(node *new_node){
		visited_set.insert(visited_set.end(),new_node);
	}

	double euclidean_dist2D(Point3f start, Point3f end){
		return sqrt((start.x-end.x)*(start.x-end.x) + (start.y-end.y)*(start.y-end.y));
	}

	double euclidean_dist(Point3f start, Point3f end){
        //admissible heurisitcs
		return sqrt((start.x-end.x)*(start.x-end.x) + (start.y-end.y)*(start.y-end.y) + (start.z-end.z)*(start.z-end.z));
	}

    /*double min(double x, double y) {
        x < y ? x : y;
    }*/

	double heuristics(Point3f curr){
		return euclidean_dist(curr, dst);
	}


	double rotate_cost (Point3f start, Point3f end){
		return std::min(std::fabs(end - start), 2 * M_PI - std::fabs(start - end)) * 4.0 / M_PI;
	}

	double roll_cost (Point3f start, Point3f end){
		return euclidean_dist(start, end);
	}

	double forward_cost (Point3f start, Point3f end){
		return 4 * euclidean_dist(start, end); //this is a guess..
	}

    Point3f calc_angle_sum(Point3f curr, double angle) {
        //to optimize, reduce redundant operations (like 2*pi)

        angle = curr.z + angle;
        angle = fmod(angle + M_PI, 2*M_PI);
        if (angle < 0)
            angle += 2*M_PI;
        angle =  angle - M_PI;
        return Point3f(curr.x, curr.y, angle);
    }

   vector<Point3f> rotate_succ (Point3f curr){
        vector<Point3f> v1 = vector<Point3f>();
        for (int i = 0; i < 7; i++) {
            double angle = (i + 1) * M_PI / 4.0;
            Point3f p = calc_angle_sum(curr, angle);
            roundToGridPoint(p, GRID_SIZE);
            v1.insert(v1.begin(), p);
        }

        return v1;
    }

	/*vector<Point3f> rotate_succ (Point3f curr){ //TODO: currently only 4-way connected, assuming th is aligned with one of the axes.

		// [-180, 180] normalization for angles
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
	}*/

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
