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

namespace AStar{

	using namespace cv;
	using namespace std;

	int GRID_SIZE = 40;
	float ANGLE_SIZE = M_PI/4;

	// change resolution ++ frame rate

	// reuse path in the astar
	// astar any-time invariant
	// putting pre-computed costs ** 

	// time the rotation
	// DEMO - estimate how far it moved / seqs of movements
	// as we drive, distance reduce
	// cam looking at robot, move the robot according to the path/actions

	// overlay transparent image and draw on it!!
	// informations..

	// state, bounds for rotation
	// astar in snake frame
	// obstacles?

	// implement on the snake .. order rotation + cost for motions/actions

	Point3f dst; //used by heuristics 

	vector<node *> current_path;
	const int astarFrameCount = 10;

	action roll, rotate, forward;
	action actions[3];
	//void makeActions (void) __attribute__((constructor)); //calls makeActions before main

	vector<node*> pq;
	vector<node*> visited_set;

	//Mat overlay_img;

	/*
	* populates actions - array of all possible action for the snake
	*/
	void makeActions(){

		roll.name = "roll";
		forward.name = "forward";
		rotate.name = "rotate";
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
		roundAngle(src_pt, ANGLE_SIZE/2);

		char name[100];
		sprintf(name,"source is: x=%.0f, y=%.0f, th=%.3f. destination is: x=%.0f, y=%.0f, th=%.2f.", src_pt.x, src_pt.y, degree(src_pt.z), dst_pt.x, dst_pt.y, degree(dst_pt.z));
		putText(image, name, Point(25,40), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,255,255), 2,8,false);
		char distText[50];
		double dist = euclidean_dist2D(src_pt, dst_pt);
		sprintf(distText, "distance from current position to goal: %.02f", dist);
		putText(image, distText, Point(25,60), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,255,255), 2,8,false);

		//if on the path and dst is still same, no need to replan
		if (inPath(src_pt) && dst.x == dst_pt.x && dst.y == dst_pt.y && inBoundsAngle(dst_pt.z, dst.z, M_PI/4)){
			cout << "&&&&&&&&&&&&&& inpath &&&&&&&&&&&&&&&&" << endl;
			putText(image, "INPATH", Point(25,80), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,255,255), 2,8,false );
			drawPath(current_path);
			return;
		}

		if (frameCounter % astarFrameCount == 0){
			cout << "**************** replanning ****************" << endl;
			putText(image, "REPLANNED", Point(25,80), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,255,255), 2,8,false );
			freePath();
			dst = dst_pt;
			current_path = vector<node *>();
			planPath (src_pt, dst_pt, current_path);
		}
		drawPath(current_path);

	}

	bool inBoundsXY(Point3f path_pt, Point3f src_pt, int error_size){

		float error = error_size*GRID_SIZE;
		//cout << " error " << error << " ";
		//cout << " path " << path_pt.x << " " << path_pt.y << " src_pt " << src_pt.x << " " << src_pt.y << endl;
		return (path_pt.x + error >= src_pt.x && path_pt.x - error <= src_pt.x
			&& path_pt.y + error >= src_pt.y && path_pt.y - error <= src_pt.y);

	}

	bool inPath(Point3f src_pt){
		if (current_path.empty()) return false;
		//float x = src_pt.x;
		//float y = src_pt.y;
		//bool flag = false;
		int curr = -1;

		//TODO ANGLE

		for (int i=0; i<current_path.size(); i++){
			
			if (inBoundsXY(current_path[i]->pos, src_pt, 1)){
				//cout << "in bounds!!" << endl;
				//flag = true;
				if (current_path[i]->move.name == "rotate"){
					//check if angle is working
					float curr_diff = (dst.z - src_pt.z);
					float orig_diff = (dst.z - current_path[i]->pos.z);
					//TEST THIS TODO
					if (inBoundsAngle(curr_diff, 0, M_PI/4)){
						curr = i+1;
						break;
					}

					if (abs(curr_diff) <= abs(orig_diff)){
						curr = i;
						break;
					}
					//if angle diff is not decreasing, decide out of path..
					return false;

				} else {
					curr = i;
					break;
				}	
			}
		}

		if (curr >= 0){
			while (curr > 0){ //delete curr-1 elems from front
				delete current_path[0];
				current_path.erase (current_path.begin());
				curr--;
			}
			return true;
		}

		return false;

	}

	/* round the angle to nearest */
	void roundAngle(Point3f &pt, float roundTo){

		float halfRound = roundTo/2.0;
		float temp = pt.z >= 0 ? (pt.z + halfRound) : (pt.z - halfRound);

		float new_angle = ((int)(temp / roundTo)) * roundTo;
		//cout << " angle " << pt.z << " new " << new_angle << endl;
		pt.z = new_angle;
		//cout << " ??????? "<< pt.z << endl;
	}

	/*
	* runs Astar
	*/
	void planPath(Point3f src_pt, Point3f dst_pt, vector<node *>& path){

		
		visited_set = vector<node *>();
		pq = vector<node *>();

		//TODO be able to tell if dst is reachable from src
		node *start_node = new node;
		start_node->priority = 0;
		start_node->dist = 0;
		start_node->pos = src_pt;
		start_node->prev = nullptr;
		//move is undefined at start
		insertPQ(start_node);

		while (!pq.empty()){

			node *n = delminPQ();
			//cout << "node: " << n->pos.x << " " << n->pos.y << " " << n->pos.z << endl;
			//cout << " dst " << dst_pt.x << " " << dst_pt.y << " " << endl;

			if (inBoundsXY(n->pos, dst_pt, 1) && inBoundsAngle(n->pos.z, dst_pt.z, M_PI/4)){

					DEBUG(cout << "done astar" << endl;)
					while (n != nullptr){
						//copy values
						node *path_node = new node;
						path_node->dist = n->dist;
						path_node->pos = n->pos;
						path_node->move = n->move;
						//cout << "????? " << (n->move).name << endl;
						path.insert(path.begin(), path_node);
						n = n->prev;
					}

					insertVisited(n);
					freeSets();
					return;

			}

			if (!visited(n->pos)){
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

			insertVisited(n);
		}
	}

	bool inBoundsAngle(float curr_angle, float dst_angle, float bound){
		return (curr_angle >= dst_angle-bound && curr_angle <= dst_angle+bound);
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
				sprintf(name,"rotate %.2f", degree(path[j+1]->pos.z - path[j]->pos.z));
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

	float degree(float rad){
		return rad*180.0/M_PI;
	}

	//call only if pq is not empty!
	node *delminPQ(){

		node *min_node = pq[0];
		pq.erase(pq.begin());
		return min_node;

	}

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
		return sqrt((start.x-end.x)*(start.x-end.x) + (start.y-end.y)*(start.y-end.y) + (start.z-end.z)*(start.z-end.z));
	}

	double heuristics(Point3f curr){
		return euclidean_dist(curr, dst);
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

		/* [-180, 180] normalization for angles */
		vector<Point3f> v1 = vector<Point3f>();
		

		float angle = 0;
		while (angle <= M_PI){

			Point3f p1 = Point3f(curr.x, curr.y, angle);
			//roundToGridPoint(p1, GRID_SIZE);
			//cout << " " << p1.z << endl;
			v1.insert(v1.begin(), p1);

			angle += ANGLE_SIZE;
		}

		angle = 0;
		while (angle >= M_PI){

			Point3f p1 = Point3f(curr.x, curr.y, angle);
			//cout << " " << p1.z << endl;
			//roundToGridPoint(p1, GRID_SIZE);
			v1.insert(v1.begin(), p1);

			angle -= ANGLE_SIZE;
		}

		return v1;
		/*
		double angle = curr.z + ANGLE_SIZE;
		angle = fmod(angle + M_PI,2*M_PI);
		if (angle < 0)
			angle += 2*M_PI;
		angle =  angle - M_PI;
		Point3f p1 = Point3f(curr.x, curr.y, ANGLE_SIZE);
		angle = curr.z - ANGLE_SIZE;
		angle = fmod(angle + M_PI,2*M_PI);
		if (angle < 0)
			angle += 2*M_PI;
		angle =  angle - M_PI;
		Point3f p2 = Point3f(curr.x, curr.y, -ANGLE_SIZE);
		angle = curr.z + ANGLE_SIZE;
		angle = fmod(angle + M_PI,2*M_PI);
		if (angle < 0)
			angle += 2*M_PI;
		angle =  angle - M_PI;
		Point3f p3 = Point3f(curr.x, curr.y, 2*ANGLE_SIZE);
		angle = curr.z - ANGLE_SIZE;
		angle = fmod(angle + M_PI,2*M_PI);
		if (angle < 0)
			angle += 2*M_PI;
		angle =  angle - M_PI;
		Point3f p4 = Point3f(curr.x, curr.y, -2*ANGLE_SIZE);
		roundToGridPoint(p1, GRID_SIZE);
		roundToGridPoint(p2, GRID_SIZE);
		roundToGridPoint(p3, GRID_SIZE);
		roundToGridPoint(p4, GRID_SIZE);
		cout << " " << p1.z << " " <<p2.z << " " <<p3.z << " " <<p4.z << endl;
		vector<Point3f> v1 = vector<Point3f>();
		v1.insert(v1.begin(), p1);
		v1.insert(v1.begin(), p2);
		v1.insert(v1.begin(), p3);
		v1.insert(v1.begin(), p4);*/
		
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