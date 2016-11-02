/*
 * astar.h
 *
 *  Created on: Apr 24, 2016
 *      Author: gamjatang1
 */

#ifndef ASTAR_H
#define ASTAR_H

namespace AStar{

using namespace cv;
using namespace std;

extern int GRID_SIZE;
/*struct action{
	double (*cost) (Point3f, Point3f); //returns the cost given the start and the end
	vector<Point3f> (*succ) (Point3f); //returns the possible successors as vector
} ; //one more function pointer that basically 'commands' the move given start and end

struct node{
	double priority;
	double dist;
	Point3f pos;
	int visited;
	node *prev;
	action move;
};*/
class Action {
	public:
		virtual double cost(Point3f, Point3f){
			return 0;
		}
		virtual vector<Point3f> succ(Point3f){
			return vector<Point3f>();
		}
};

class Node {
    public:
    	double priority;   // priority
		double dist;  // distance so far
		Point3f pos;
		int visited;
		Node *prev;
		Action move;
		Node (double p, double d, Point3f po,
      			int v, Node *pr, Action mo) {
			priority = p;
			dist = d;
			pos = po;
			visited = v;
			prev = pr;
			move = mo;
		}
};

    //void setDst( int event, int x, int y, int f, void* );
	void planPath(Point src, Point dst, double angle);
	void planPathOnVideo (Mat &image, Point src_pt, Point dst_pt, double curr_angle, double dst_angle);
	void drawGrids(Mat &image, int grid_size);
	void roundToGridPoint(Point &pt, int grid_size);
	vector<Point3f> forward_succ (Point3f curr);
	vector<Point3f> roll_succ (Point3f curr);
	vector<Point3f> rotate_succ (Point3f curr);
	double forward_cost (Point3f start, Point3f end);
	double roll_cost (Point3f start, Point3f end);
	double rotate_cost (Point3f start, Point3f end);
	void insertPQ(Node new_node);
	void checkPQ();
	Node delminPQ();
	double heuristics(Point3f curr);
	double euclidean_dist(Point3f start, Point3f end);
	void drawPath();
	void insertVisited(Point3f pt);
	bool visited(Point3f pt);
	bool isValid (Point3f start, Point3f end);
	void roundToGridPoint(Point3f &pt, int grid_size);

}
#endif /* ASTAR_H_ */
