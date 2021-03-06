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
struct action{
	string name;
	
	double (*cost) (Point3f, Point3f); //returns the cost given the start and the end
	vector<Point3f> (*succ) (Point3f); //returns the possible successors as vector
} ; //one more function pointer that basically 'commands' the move given start and end

struct node{
	double priority;
	double dist;
	Point3f pos;
	node *prev;
	action move;
};

    //void setDst( int event, int x, int y, int f, void* );
	bool inPath(Point3f src_pt);
	bool inBoundsXY(Point3f path_pt, Point3f src_pt, int error_size);
	void makeActions();
	void planPath(Point3f src, Point3f dst, vector<node *>& path);
	void planPathOnVideo (Mat &image, Point3f src_pt, Point3f dst_pt, int frameCounter);
	void drawGrids(Mat &image, int grid_size);
	void roundToGridPoint(Point &pt, int grid_size);
	void printPath();
	void freePath();
	void freeSets();
	node *delminPQ();
	void insertPQ(node *new_node);
	void checkPQ();
	vector<Point3f> forward_succ (Point3f curr);
	vector<Point3f> roll_succ (Point3f curr);
	vector<Point3f> rotate_succ (Point3f curr);
	double forward_cost (Point3f start, Point3f end);
	double roll_cost (Point3f start, Point3f end);
	double rotate_cost (Point3f start, Point3f end);
	double heuristics(Point3f curr);
	double euclidean_dist2D(Point3f start, Point3f end);
	double euclidean_dist(Point3f start, Point3f end);
	void drawPath(vector<node *> path);
	void insertVisited(node *new_node);
	bool visited(Point3f pt);
	bool isValid (Point3f start, Point3f end);
	void roundToGridPoint(Point3f &pt, int grid_size);
	void roundAngle(Point3f &pt, float roundTo);
	bool inBoundsAngle(float curr_angle, float dst_angle, float bound);
	float degree(float rad);

}
#endif /* ASTAR_H_ */
