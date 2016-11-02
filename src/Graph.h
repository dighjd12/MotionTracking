
#ifndef GRAPH_H
#define GRAPH_H

using namespace cv;
using namespace std;
using namespace AStar;

class Action {
	public:
		virtual double cost(Point3f, Point3f){
			return 0;
		}
		virtual vector<Point3f> succ(Point3f){
			return 
		}
};

class Roll: public Action {
   public:
      	double cost (Point3f start, Point3f end){
			return 2;//euclidean_dist(start, end);
		}
		vector<Point3f> succ (Point3f curr){ //orientation head of our snake (rad)

			Point3f p1 = Point3f(curr.x + 3/2*GRID_SIZE*cos(curr.z+M_PI/2), curr.y + 3/2*GRID_SIZE*sin(curr.z+M_PI/2), curr.z);
			Point3f p2 = Point3f(curr.x + 3/2*GRID_SIZE*cos(curr.z-M_PI/2), curr.y + 3/2*GRID_SIZE*sin(curr.z-M_PI/2), curr.z);
			roundToGridPoint(p1, GRID_SIZE);
			roundToGridPoint(p2, GRID_SIZE);
			vector<Point3f> v1 = vector<Point3f>();
			v1.insert(v1.begin(), p1);
			v1.insert(v1.begin(), p2);
			return v1;
		}
};

class Forward: public Action {
   public:
      	double forward_cost (Point3f start, Point3f end){
			return 1;//euclidean_dist(start, end);
		}
		vector<Point3f> succ (Point3f curr){

			Point3f p1 = Point3f(curr.x + 3/2*GRID_SIZE*cos(curr.z), curr.y +3/2*GRID_SIZE*sin(curr.z), curr.z);
			Point3f p2 = Point3f(curr.x + 3/2*GRID_SIZE*cos(curr.z+M_PI), curr.y +3/2*GRID_SIZE*sin(curr.z+M_PI), curr.z);
			roundToGridPoint(p1, GRID_SIZE);
			roundToGridPoint(p2, GRID_SIZE);
			vector<Point3f> v1 = vector<Point3f>();
			v1.insert(v1.begin(), p1);
			v1.insert(v1.begin(), p2);
			return v1;
		}
};

class Rotate: public Action {
   public:
      	double cost (Point3f start, Point3f end){
			return 1;
		}
		vector<Point3f> succ (Point3f curr){ //TODO: currently only 4-way connected, assuming th is aligned with one of the axes.

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
};


class Node;

class Node {
    public:
    	double priority;   // priority
		double dist;  // distance so far
		Point3f pos;
		int visited;
		Node prev;
		Action move;
		Node(double, double, Point3f, int, Node, Action);
};




/*
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
*/

/*struct action{
	double (*cost) (Point3f, Point3f); //returns the cost given the start and the end
	vector<Point3f> (*succ) (Point3f); //returns the possible successors as vector
} ;*/

#endif /* GRAPH_H_ */