/*
 * tracker.cxx
 * 
 * Copyright 2015 Joseph Lindgren <joseph.lindgren@uky.edu>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

/* 
 * This program uses the C++ standard library list container to develop
 * an algorithm for navigating from one point to another around 
 * polygonal obstacles (note that the start point of each obstacle must 
 * be copied to the end of the list to form a closed figure). Vector 
 * methods are implemented in the class "CPair" and employed throughout 
 * non-member functions below to perform this task. There is also a 
 * function written to optimize the avoidance path by cutting corners 
 * in the route wherever possible.
 * 
 * The "DEBUGGING" flag can be turned on to see more details of the 
 * process. Some (almost correct) WxMaxima code is generated at the end
 * of the program output to help the programmer visualize what the 
 * program has accomplished.
 */


#include <iostream>
#include <stdio.h>
#include <string>
#include <cmath>
#include <list>

#define RIGHT	true
#define LEFT	false
#define DEBUGGING  false

using namespace std;

class CPair
{
	private:
		float x,y;
		bool on_obstacle, clockwise;
		bool side;

	public:
		CPair();
		CPair(float x, float y);
		CPair(float x, float y, bool on_obstacle, bool clockwise);
		
		void SetClockwise(bool cw);
		bool GetClockwise(void);
		void SetOnObstacle(bool on_obs);
		bool GetOnObstacle(void);
		void SetSide(bool side);
		bool GetSide(void);
		
		CPair & operator=( const CPair & other );
		bool operator==( const CPair & other );
		CPair & operator+=( const CPair & other );
		CPair & operator-=( const CPair & other );
		const CPair operator+( const CPair & rhs );
		const CPair operator-( const CPair & rhs );
		float operator*( const CPair & rhs );
		const CPair operator*( float scalar );

		string SPrint(void);
		float GetX(void);
		float GetY(void);
		
		float Angle( CPair other );
		CPair Normal(void);
};

CPair::CPair()
{
}

CPair::CPair(float x, float y)
{
	this->x = x;
	this->y = y;

	this->on_obstacle = false;
	this->clockwise = true;
	this->side = RIGHT;//added to fix valgrind complaint of uninit'd value
}

CPair::CPair(float x, float y, bool on_obstacle, bool clockwise)
{
	this->x = x;
	this->y = y;

	this->on_obstacle = on_obstacle;
	this->clockwise = clockwise;
}

void CPair::SetClockwise(bool cw)
{
	this->clockwise = cw;
}

bool CPair::GetClockwise(void)
{
	return this->clockwise;
}

void CPair::SetOnObstacle(bool on_obs)
{
	this->on_obstacle = on_obs;
}

bool CPair::GetOnObstacle(void)
{
	return this->on_obstacle;
}

void CPair::SetSide(bool side)
{
	this->side = side;
}

bool CPair::GetSide(void)
{
	return this->side;
}

CPair & CPair::operator=( const CPair & other )
{
	this->x = other.x;
	this->y = other.y;
	this->clockwise = other.clockwise;
	this->on_obstacle = other.on_obstacle;
	this->side = other.side;
	return *this;
}

bool CPair::operator==( const CPair & other )
{
	if ( (this->x == other.x) and (this->y == other.y) )
		return true;
	else
		return false;
}

CPair & CPair::operator+=( const CPair & other )
{
	this->x += other.x;
	this->y += other.y;
	return *this;
}

CPair & CPair::operator-=( const CPair & other )
{
	this->x -= other.x;
	this->y -= other.y;
	return *this;
}

const CPair CPair::operator+( const CPair & rhs )
{
	CPair result(this->x, this->y);
	result += rhs;
	return result;
}

const CPair CPair::operator-( const CPair & rhs )
{
	CPair result(this->x, this->y);
	result -= rhs;
	return result;
}

float CPair::operator*( const CPair & rhs )
// Returns the dot product of 'this' pair with 'rhs'
{
	return (this->x * rhs.x + this->y * rhs.y);
}

const CPair CPair::operator*( float scalar )
// Returns a scalar multiple of 'this' pair
{
	return CPair( scalar * this->x, scalar * this->y );
}

///

string CPair::SPrint(void)
// Returns a string describing the pair
{
	char * pair;
	pair = new char[50];
	if ( (int)(this->x) == this->x )
		snprintf( pair, 50, "(%d,%d)", (int)(this->x), (int)(this->y) );
	else
		snprintf( pair, 50, "(%f,%f)", this->x, this->y);
	
	string str;
	str.assign(pair);
	delete [] pair;
	return str;
}

float CPair::GetX(void)
{
	return this->x;
}

float CPair::GetY(void)
{
	return this->y;
}
		
float CPair::Angle( CPair other )
// Returns the angle from 'this' to 'other' (vector) pair
{
	CPair v = *this, w = other;
	float radians = 0, cosine = 0, sign = 0;
	float denom = sqrt(( v * v ) * ( w * w ));

	if ( denom == 0 )
		return 0;

	cosine = v * w / denom;
	radians = acos(cosine);
		
	sign = v.Normal() * w;

	if ( sign < 0 )
		radians = - radians;

	return radians * 180 / 3.141592654;
}

CPair CPair::Normal(void)
{
	CPair v;
	v.x = - this->y;
	v.y = 	this->x;
	return v;
}

void PrintPath( list<CPair> * path )
{
	cout << "[(start)";
	for ( CPair & pt : *path )
	{
		cout << ( pt.GetSide() ? "-R" : "-L" )
			 << pt.SPrint();
	}
	cout << "(end)]" << endl;
}

void PrintWXMaxGraph( list<CPair> * path, string blurb )
{
	cout << "(Approx.) wxMaxima code to visualize the " << blurb << endl
		 << " (delete trailing commas in coord lists):" << endl;
	cout << "wxplot2d([discrete," << endl << '[';
	for ( CPair & pt : *path )
		cout << pt.GetX() << ',';
	cout << "]," << endl << '[';
	for ( CPair & pt : *path )
		cout << pt.GetY() << ',';
	cout << "]]," << endl;
	cout << "[x,0,35],[y,0,35])" << endl;
}

list<CPair> ConcatPaths( list<CPair> * path1, list<CPair> * path2 )
{
	list<CPair> result;
	for ( CPair & pt : *path1 )
		result.push_back(pt);
	for ( CPair & pt : *path2 )
		result.push_back(pt);
	return result;
}

list<CPair> ConcatPaths( list<list<CPair> *> paths )
{
	list<CPair> result;
	for ( list<CPair> * path : paths )
	{
		for ( CPair & pt : *path )
			result.push_back(pt);
	}
	return result;
}

struct SSeg
{
	CPair start, end;
};

void PrintSeg( SSeg seg )
{
	cout << '[' << seg.start.SPrint() << ", " << seg.end.SPrint() << ']' << endl; 
}

class CObstacle
{
	private:
		list<CPair> pts;

	public:
		CObstacle();
		CObstacle( list<CPair> * pts );
		CObstacle & operator=( const CObstacle & rhs );
		CObstacle & operator+( const CObstacle & rhs );

		void push_back( CPair pt );
		unsigned int size( void );
		list<CPair> GetPts(void);
		void Print(void);
		
		int Index(CPair pt); // returns the offset of the first match to 'pt'
		int Split( CPair split_pt, list<CPair> * part1, list<CPair> * part2 ); // splits a list at 'split_pt' and sets each parts to include 'split_pt'. Returns the zero-based index of 'split_pt'
		CObstacle Subset( int start, int end );
		bool Segment( int segpos, SSeg * segment );
		void Reverse(void);
};

CObstacle::CObstacle()
{
}

CObstacle::CObstacle( list<CPair> * points )
{
	for ( CPair & pt : * points )
	{
		pt.SetOnObstacle(true);
		pt.SetClockwise(false);
		//~ pt.SetSide(RIGHT);
		
		this->pts.push_back(pt);
	}
}

CObstacle & CObstacle::operator=( const CObstacle & rhs )
{
	if (this == &rhs)
		return *this;

	this->pts.clear();
	for ( CPair pt : rhs.pts )
		this->pts.push_back(pt);
	return *this;
}

CObstacle & CObstacle::operator+( const CObstacle & rhs )
{
	for ( CPair pt : rhs.pts )
		this->pts.push_back(pt);
	return *this;
}

void CObstacle::push_back( CPair pt )
{
	this->pts.push_back(pt);
}

unsigned int CObstacle::size(void)
{
	return this->pts.size();
}

list<CPair> CObstacle::GetPts(void)
{
	return this->pts;
}

void CObstacle::Print(void)
{
	PrintPath(&this->pts);
}

int CObstacle::Split( CPair split_pt, list<CPair> * part1, list<CPair> * part2 )
{
	bool before = true;
	int n = 0;
	part1->clear(), part2->clear();
	for ( CPair & pt : this->pts )
	{
		if ( before )
		{
			part1->push_back(pt);
			n ++;
		}

		if ( pt == split_pt )
		{
			before = false;
			n --;
		}

		if ( !before )
			part2->push_back(pt);
	}
	return n;
}


void CObstacle::Reverse(void)
{
	this->pts.reverse();
	for ( CPair & pt : this->pts )
	{
		pt.SetClockwise( not pt.GetClockwise() ); // toggle cwise and ctrcwise
		pt.SetSide( not pt.GetSide() ); // toggle LEFT to RIGHT and vice versa
	}
}

//Non-member functions ///////////////////////////////////////////////////////////////////

bool Solve2by2System( float A, float B, float a, float b, float c, float d, CPair * soln )
/* Solves
 * A = ax + by
 * B = cx + dy
 * and returns the pair (x,y) */
{
	float denom = b * c - a * d;
	
	if ( denom == 0 )
		return 0;
	else
	{
		*soln = CPair( (b*B-d*A)/denom, (c*A-a*B)/denom );
		return 1;
	}
}

bool SegIntersect( CPair seg1start, CPair seg1end, CPair seg2start, CPair seg2end, float * kappa, CPair * isect )
/* If an intersection DOES exist:
 * 		we return success,
 * 		set 'isect' as the point of intersection,
 * 		and set 'kappa' to indicate how far along the first segment the intersection lies.
 * 		( we have 0 <= k <= 1 if an intersection exists )
 * If an intersection does NOT exist:
 * 		return failure,
 * 		do not set 'kappa'. */
{
	CPair	a = seg1start,
			b = seg1end,
			c = seg2start,
			d = seg2end;
	float	A = a.GetX() - c.GetX(),
			B = a.GetY() - c.GetY();
	CPair	alpha = a - b,
			beta  = d - c;
	CPair	soln;

	//If no intersection (no soln or !(0 <= k <= 1) ), return failure
	if (	!Solve2by2System( A, B, alpha.GetX(), beta.GetX(), alpha.GetY(), beta.GetY(), &soln )
		 or (soln.GetX() > 1) or (soln.GetX() < 0) or (soln.GetY() > 1) or (soln.GetY() < 0) )
		return 0;

	//Otherwise, set 'kappa' and 'isect' and return success
	else
	{
		*kappa = soln.GetX();
		*isect = a - alpha * (*kappa);
		return 1;
	}
}

bool SegIntersect( SSeg seg1, SSeg seg2, float * kappa, CPair * isect )
/* If an intersection DOES exist:
 * 		we return success,
 * 		set 'isect' as the point of intersection,
 * 		and set 'kappa' to indicate how far along the first segment the intersection lies.
 * 		( we have 0 <= k <= 1 if an intersection exists )
 * If an intersection does NOT exist:
 * 		return failure,
 * 		do not set 'kappa. */
{
	CPair	a = seg1.start,
			b = seg1.end,
			c = seg2.start,
			d = seg2.end;
	float	A = a.GetX() - c.GetX(),
			B = a.GetY() - c.GetY();
	CPair	alpha = a - b,
			beta  = d - c;
	CPair	soln;

	//If no intersection (no soln or !(0 <= k <= 1) ), return failure
	if (	!Solve2by2System( A, B, alpha.GetX(), beta.GetX(), alpha.GetY(), beta.GetY(), &soln )
		 or (soln.GetX() > 1) or (soln.GetX() < 0) or (soln.GetY() > 1) or (soln.GetY() < 0) )
		return 0;

	//Otherwise, set 'kappa' and 'isect' and return success
	else
	{
		*kappa = soln.GetX();
		*isect = a - alpha * (*kappa);
		return 1;
	}
}

struct SIsectData
{
	float k;
	CPair pair;
	SSeg  o_seg;
};

bool compare_k( const SIsectData & first, const SIsectData & second )
{
	if ( first.k < second.k )
		return true;
	else
		return false;
}

int ObstacleIntersection( SSeg seg, CObstacle * obstacle, list<SIsectData> * isects, float tolerance )
/* Sets a vector of parameters, CPairs, and segments (of the obstacle) intersected,
 * sorted in order of intersection by the parameter list of 'k's. */
{
	int num = 0;
	{// Get the intersection info
		SSeg o_seg;
		float k;
		CPair isect;
		bool first = true;
		CPair oldpt = obstacle->GetPts().front();
		for ( CPair & pt : obstacle->GetPts() )
		{
			if ( first )
			{
				first = false;
				continue;
			}

			{ //The work is done here
				o_seg = { oldpt, pt };
				
				if ( SegIntersect( seg, o_seg, &k, &isect ) and (k >= tolerance and k <= 1 - tolerance) )
					isects->push_back( { k, isect, o_seg} ), num ++;
			}
			oldpt = pt;
		}
	}
	// If there aren't multiple intersections, no sorting is needed!
	if ( isects->size() < 2 )	
		return 0;
	//Arrange info by k-value
	isects->sort(compare_k);

	return num;
}

list<CPair> Circumvent( SSeg seg, CObstacle * obstacle, bool clockwise )
/* Given a segment 'seg' to describe entry and exit points on 'obstacle',
 * and given a direction of travel by 'clockwise',
 * this function returns a path along 'obstacle'. */
{
	// make the obstacle start and end with 'seg.start'
	list<CPair> first, second;
	obstacle->Split(seg.start, &first, &second);
	CObstacle 	obs = CObstacle(&second) + CObstacle(&first);
#if DEBUGGING
	obs.Print();
#endif
	// set the direction to go by inverting the list if necessary
	if ( clockwise == false )
		obs.Reverse();

	// split at the exit point 'seg.end'
	obs.Split(seg.end, &first, &second);

	// return the route from entry to exit
	return first;
}

float SegLen( CPair start, CPair end )
{
	CPair v = end - start;
	float len = sqrt(v * v);
	return len;
}

float PathLen( list<CPair> * ptlist )
{
	float length = 0;
	bool first = true;
	CPair oldpt = ptlist->front();
	for ( CPair & pt : *ptlist )
	{
		if ( first )
		{
			first = false;
			continue;
		}

		length += SegLen( oldpt, pt );

		oldpt = pt;
	}
	return length;
}

list<CPair> minCircumvent( SSeg seg, CObstacle * obstacle )
/* Just like 'Circumvent', only this function computes clockwise and cclockwise paths,
 * returning the shorter of the two. */
{
	list<CPair> 	path1 = Circumvent( seg, obstacle, true ),
					path2 = Circumvent( seg, obstacle, false );

	float 	len1 = PathLen(&path1),
			len2 = PathLen(&path2);
	cout << len1 << ", " << len2 << endl;

	if (len1 < len2)
		return path1;
	else
		return path2;
}

list<CPair> FindPath( SSeg seg, list<CObstacle> * obstacles, float tolerance )
{
	list<CObstacle> o_list = *obstacles;

	// Stop recurring if there are no more obstacles
	if ( o_list.size() < 1 )
	{
#if DEBUGGING
		cout << "no more obstacles" << endl;							//DEBUG LINE
#endif
		return {seg.start, seg.end};
	}
#if DEBUGGING
	else {cout << o_list.size() << " obstacle(s) remain(s)" << endl;}												//DEBUG LINE
#endif
	// Deal with the next obstacle
	CObstacle obstacle = o_list.front();
	list<SIsectData> isects;// replaces the previous three data lists
#if DEBUGGING
	cout << "active obstacle is ";										//DEBUG LINE
	obstacle.Print();													//DEBUG LINE
#endif

	if ( !ObstacleIntersection( seg, &obstacle, &isects, tolerance ) )
	// if obstacle is not in the way
	{
		o_list.erase(o_list.begin());
#if DEBUGGING
		cout << "No intersection with obstacle " << &obstacle << endl;	//DEBUG LINE
#endif
		return FindPath( seg, &o_list, tolerance );
	}

	// else, we have a naive path to be broken down (for now, we ignore any intermediate intersections as irrelevant):
	//	seg.start, isect[0], ... , isect[n], seg.end
	// we assume the last intersection is the exit point. Bad things happen/get ignored if the destination is inside an obstacle.
	CPair 	i_pt_in  = isects.front().pair,
			i_pt_out = isects.back().pair; 
	SSeg 	o_seg_in  = isects.front().o_seg,
			o_seg_out = isects.back().o_seg;
	bool orientation;

	i_pt_out.SetOnObstacle(true);

	// Now, navigate around the obstacle
	list<CPair> path1 = Circumvent( {o_seg_in.end, o_seg_out.start}, &obstacle, RIGHT );
	orientation = path1.begin()->GetSide();
	i_pt_in.SetSide(orientation);
	i_pt_out.SetSide(orientation);
#if DEBUGGING
	cout << "intersections with obstacle are " << i_pt_in.SPrint() << "and" << i_pt_out.SPrint() << endl;
	cout << "path1 orientation has been set to " << ( path1.begin()->GetSide() ? 'L' : 'R' ) << endl;
#endif
	path1.push_front(i_pt_in);
	path1.push_back(i_pt_out);

	list<CPair> path2 = Circumvent( {o_seg_in.start, o_seg_out.end}, &obstacle, LEFT );
	orientation = path2.begin()->GetSide();
	i_pt_in.SetSide(orientation);
	i_pt_out.SetSide(orientation);
#if DEBUGGING
	cout << "path2 orientation has been set to " << ( path2.begin()->GetSide() ? 'L' : 'R' ) << endl;
#endif
	path2.push_front(i_pt_in);
	path2.push_back(i_pt_out);

	//Choose the path with minimal length
	float 	len1 = PathLen(&path1),
			len2 = PathLen(&path2);
	list<CPair> path;
	path = { (len1 < len2) ? path1 : path2 };
#if DEBUGGING
	cout << len1 << ", " << len2 << endl;								//DEBUG LINE
	if (len1 < len2)
		cout << "chose path1 for its local efficiency:";				//DEBUG LINE
	else
		cout << "chose path2 for its local efficiency:";				//DEBUG LINE
	cout << endl;														//DEBUG LINE
	PrintPath(&path);
#endif
	// Figure out the rest of the path (looking for ways around the other obstacles) ...
	o_list.erase(o_list.begin());
#if DEBUGGING
	cout << o_list.size() << endl;										//DEBUG LINE
	cout << "Moving to next of " << o_list.size() << " obstacles" << endl;//DEBUG LINE
	cout << "in-vector is ";
	PrintSeg({seg.start, i_pt_in});										//DEBUG LINE
	cout << "out-vector is ";
	PrintSeg({i_pt_out,  seg.end});										//DEBUG LINE
#endif
	//.. by splitting the path and recurring to remaining obstacles
	list<CPair> prepath  = FindPath( {seg.start, i_pt_in}, &o_list, tolerance );
	list<CPair> postpath = FindPath( {i_pt_out,  seg.end}, &o_list, tolerance );

	prepath.pop_back();  //important: unique gets rid of the wrong pts
	postpath.pop_front();//important: unique gets rid of the wrong pts
	list<CPair> route = ConcatPaths( {&prepath, &path, &postpath} );
	route.unique();
	return route;
}

list<CPair> FindPath( SSeg seg, list<CObstacle> * obstacles)
{
	return FindPath( seg, obstacles, 0 );
}

bool OptimizePath( list<CPair> * sequence, list<CObstacle> * obstacles, float tolerance )
{
	bool path_not_shortened;
	list<CPair>::iterator pt = sequence->begin(), candidate, anchor;
	advance( pt, 2 );
#if DEBUGGING
	cout << "Optimizing---------------" << endl;						//DEBUG LINE
#endif
	while ( pt != sequence->end() )
	{
		// set a flag to say when to move on (default is 'move on')
		path_not_shortened = true;
		
		// set ref to candidate, the point to be possibly deleted
		candidate = --pt; 
		++pt;
		if ( candidate == sequence->begin() )
		{
			++pt;
			continue;
		}

		// Set ref to anchor, the point from which the vectors are measured
		anchor = --candidate;
		++candidate;
		// Locally, the path now looks like < ..., *anchor, *candidate, *pt, ... >

		// Determine angle between vectors to 'candidate' and 'pt' from 'anchor'.
		// 		Vectors to be used in this computation
		CPair 	v = 	*candidate - *anchor,
				w = 	*pt		   - *anchor,
				vperp = v.Normal() * ( candidate->GetSide() == LEFT ? 1 : -1 );

		if ( vperp * w > 0 )
		{
			list<CPair> path = {*anchor, *candidate, *pt};
			list<CPair> altpath = FindPath( {*anchor, *pt}, obstacles, tolerance );
#if DEBUGGING
			cout << "\t old path of length " << PathLen(&path) << " is " << endl;	//DEBUG LINE
			PrintPath(&path);														//DEBUG LINE
			cout << "\t alternate path of length " << PathLen(&altpath) << " is " << endl;//DEBUG LINE
			PrintPath(&altpath);													//DEBUG LINE
#endif
			if ( PathLen(&altpath) < PathLen(&path) )
			{
				altpath.pop_front();
				altpath.pop_back();
#if DEBUGGING
				cout << candidate->SPrint() << " replaced";				//DEBUG LINE
#endif				
				if ( altpath.size() > 0 )
				{
#if DEBUGGING
					cout << " by ";										//DEBUG LINE
					PrintPath(&altpath);								//DEBUG LINE
#endif
					sequence->insert( candidate, altpath.begin(), altpath.end() );
				}
#if DEBUGGING
				else													//DEBUG LINE
					cout << endl;										//DEBUG LINE
#endif
				sequence->erase(candidate);
#if DEBUGGING
				cout << "new path is ";									//DEBUG LINE
				PrintPath(sequence);									//DEBUG LINE
#endif
				path_not_shortened = false;
			}
		}
#if DEBUGGING
		else
		{																//DEBUG BLOCK
			cout << "\t failed to remove waypoint " << candidate->SPrint() << endl;
			cout << "\t\t <*anchor, *candidate, *pt> is " << anchor->SPrint() << ", " << candidate->SPrint() << ", " << pt->SPrint() << endl;
			cout << "\t\tdot prod is " << vperp * w << endl;
			cout << "\t\tvperp is " << vperp.SPrint() << ". v is " << v.SPrint() << ". w is " << w.SPrint() << endl;

		}
#endif		
		if ( path_not_shortened )
			++ pt;
	}
#if DEBUGGING
	cout << endl << "Optimization complete; no more waypoints could be removed/altered" << endl;					//DEBUG LINE
#endif
	return false;
}



/// MAIN ///////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
	//Test Data
	list<CPair> 	Obstacle1 =
	{ CPair(24,6), CPair(28,6), CPair(28,12), CPair(26,17), CPair(25,19),
	  CPair(23,25), CPair(21,30), CPair(19,34), CPair(15,34), CPair(14,31),
	  CPair(14,26), CPair(15,22), CPair(19,12), CPair(20,5) },
					Obstacle2 = { CPair(10,10), CPair(15,2), CPair(12,15) };

	CObstacle 	obstacle1(&Obstacle1),
				obstacle2(&Obstacle2);

	//Print out the obstacle data
	cout << "Obstacles:" << endl;
	obstacle1.Print();
	obstacle2.Print();

	{//We do a quick test of the functions declared above.
		CPair start = CPair(6,7), end = CPair(32,23);
		cout << endl << "FindPath test:" << endl;
		list<CObstacle> obs_list;
		list<CPair> route2,
					obs1_pts = obstacle1.GetPts(),
					obs2_pts = obstacle2.GetPts();
		
		//Package obstacles in a list for processing convenience.
		obs_list.push_back(obstacle1);
		obs_list.push_back(obstacle2);

		//Find an avoidance path from start to end.
		cout << "Find a path around these obstacles from " 
			 << start.SPrint() << " to " << end.SPrint() << " ..."
			 << endl;
		route2 = FindPath( {start, end}, &obs_list );
		cout << "Path finding complete!" << endl;

		//Print out the result.
		cout << "---------------------------" << endl
			 << "Original route:" << endl;
		PrintPath(&route2);
		cout << "of length " << PathLen(&route2) << endl;

		//Optimize the route by cutting empty corners.
		OptimizePath( &route2, &obs_list, 0.001 );

		//Print the optimized route.
		cout << "---------------------------" << endl
			 << "Optimized route is" << endl;
		PrintPath(&route2);
		cout << "of length " << PathLen(&route2) << endl;
		cout << "---------------------------" << endl;

		//Print out some WxMaxima code so that you can visualize what
		// the program has done.
		PrintWXMaxGraph(&route2, "path");
		cout << endl;
		PrintWXMaxGraph(&obs1_pts, "first obstacle");
		cout << endl;
		PrintWXMaxGraph(&obs2_pts, "second obstacle");
	}

	return 0;
}
