#pragma once

#ifndef Helpers_H
#define Helpers_H

#include <ros/ros.h>
#include <custom_robot_msgs/Positions.h>
#include <geometry_msgs/Point.h>

/// Class that represents a segment
class Segment {
private:
	geometry_msgs::Point p1; ///< the first point of the segment
	geometry_msgs::Point p2; ///< the second point of the segment
	double length;           ///< the length of the segment
public:
	/// A segment basic constructor
	Segment() {};

	/// A segment constructor
	/**
	 * @param p1 the first segment's point
	 * @param p2 the second segment's point
	 */
	Segment(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2);

	/// A segment destructor
	virtual ~Segment() {};

	/// Returns the first point of the segment
    /**
     * @return the segment's first point
     */
	geometry_msgs::Point getP1() const;

	/// Returns the second point of the segment
	/**
	 * @return the segment's second point
	 */
	geometry_msgs::Point getP2() const;

	/// Returns the length of the segment
	/**
	 * @return the segment's length
	 */
	double getLength() const;

        void invert();
	
	/// Returns the point of the segment that is at distance t*length of the first segment's point (p = p1 + t*(p2-p1))
	/**
	 * @param t the point's normed distance to the first segment's point
	 * @return the associated point
	*/
	geometry_msgs::Point getPoint(double t) const;

	/// Prints the segment: [(x,y) -> (x,y)]
	void print() const;
};

class CurtainSegment {
private:
	Segment segment; ///< the curtain's segment
	double a;        ///< the absciss of the vector director of the segment's raw
	double b;        ///< the ordinate of the vector director of the segment's raw 
public:
	/// A basic CurtainSegment constructor
	CurtainSegment(Segment &segment);

	/// A basic CurtainSegment constructor
	CurtainSegment(geometry_msgs::Point &p1, geometry_msgs::Point &p2);

	/// A CurtainSegment destructor
	virtual ~CurtainSegment() {};

	/// Modifies the segment and computes a and b
	void setSegment(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2);

	/// Returns the absciss of the vector director of the segment's raw
	double getA() const;

	/// Returns the ordinate of the vector director of the segment's raw
	double getB() const;

	/// Returns the curtain's segment
	Segment getSegment() const;

	/// Returns the absciss of the vector director of the segment starting from p1 if is_first and p2 if !is_first
	/**
	 * @param is_first true iif the origin of the vector director is the first segment's point
	 * @return the absciss of the vector director
	 */
	double getA(bool is_first) const;

	/// Returns the ordinate of the vector director of the segment starting from p1 if is_first and p2 if !is_first
	/**
	 * @param is_first true iif the origin of the vector director is the first segment's point
	 * @return the ordinate of the vector director
	 */
	double getB(bool is_first) const;

	/// Returns the first segment's point iif is_first
	geometry_msgs::Point getPoint(bool is_first) const;

	/// Returns the segment's points
	custom_robot_msgs::Positions getPoints() const;
};

/// Class that represents a reference point in the laser scanner and curtain RA
class ReferencePoint {
private:
	geometry_msgs::Point point; ///< the reference point
	bool on_circle;             ///< true iif the point is on the circle part of the half circle
	bool on_segment;            ///< true iif the point is on the segment part of the half circle
	double radius;              ///< the distance between the point and the curtain
	/**
	 * 1 -> the point is on the first curtain's point raw 
	 * 2 -> the point is on the second curtain's point raw
	 * 0 -> the point is one of the two points of the segment's half circle
	 * -1-> the point is on the circle's half circle and closer to the first curtain's point
	 * -2-> the point is on the circle's half circle and closer to the second curtain's point
	 */
	int ref_is_first;           
public:
	/// A basic ReferencePoint constructor
	ReferencePoint(geometry_msgs::Point &point, double radius, int ref_is_first, bool on_circle, bool on_segment);
	
	/// A ReferencePoint destructor
	virtual ~ReferencePoint() {};

	/// Returns the radius of the point
	double getRadius() const;

	/// Returns ref_is_first
	int getRefIsFirst() const;

	/// returns true iif the point is on the circle
	bool onCircle() const;

	/// returns true iif the point is on the segment
	bool onSegment() const;

	/// Sets the radius of the point
	void setRadius(double radius);

	/// Returns the point
	geometry_msgs::Point getPoint() const;
};

class HalfCircle {
private:
	geometry_msgs::Point center; ///< the center of the circle part of the half circle
	Segment segment; ///< the segment part of the half circle
	double radius; ///< the radius of the half circle
	double angle; ///< the angle betwenn the half circle's segment and the absciss' axis
public:

        /// An empty HalfCircle constructor
        HalfCircle(){};

	/// A HalfCircle constructor
	HalfCircle(const geometry_msgs::Point &center, double radius, double angle);

	/// A HalfCircle destructor
	virtual ~HalfCircle() {};

	/// Returns the angle of the half circle
	double getAngle() const;

	/// Returns the center of the half circle
	geometry_msgs::Point getCenter() const;

	/// Returns the segment of the half circle
	Segment getSegment() const;

	/// Returns true iif the point is on the half circle's circle
	/**
	 * @param p the point to check
	 * @return true iif p is on the half circle's circle
	 */
	bool isOn(const geometry_msgs::Point &p) const;

	/// Returns true iif the point is in the half circle
	/**
	 * @param p the point to check
	 * @return true iif p is in the half circle
	 */
	bool isIn(const geometry_msgs::Point &p) const;

	/// Computes the intersections between the segment of the half circle and a half raw of origin p, director vector (delta_x, delta_y)
	/**
	 * @param p the origin of the half raw
	 * @param delta_x the absciss of the director vector of the half raw
	 * @param delta_y the ordinate of the director vector of the half raw
	 * @param[out] answer the point where the result is stored
	 * @return the distance between intersections point and the first point of the segemnt of the half circle (-1 if there is no intersection)
	 */
	double intersection(const geometry_msgs::Point &p, double delta_x, double delta_y, geometry_msgs::Point &answer) const;
	
	/// Computes the intersections point between the half circle and a segment of origin p, director vector (delta_x, delta_y) and length length.
	/// The result is stored in the cond vector
	/**
	 * @param p the origin of the segment
	 * @param delta_x the absciss of the director vector of the segment
	 * @param delta_y the ordinate of the director vector of the segment
	 * @param length the length of the segment
	 * @param is_first enable the ReferencePoint construction
	 * @param[out] cond the vector where the result is stored
	 * @return the number of intersections point
	 */
	int intersections(const geometry_msgs::Point &p, double delta_x, double delta_y, int is_first, double length, std::vector<ReferencePoint> &cond) const;
	
	/// Half circle printer
	void print() const;
};

class ReferencePolygon {
private:
	ReferencePoint ref;   ///< the first point of the polygon
	ReferencePoint other; ///< the last point of the polygon
	int points_begin;     ///< the index (in the reference points' vector) of the first reference point of the polygon
	int points_end;       ///< the index (in the reference points' vector) of the last reference point of the polygon
	bool ref_is_first;    ///< true iif the first point of the polygon if the first point of the curtain's segment
	int index_ref;        ///< the index (starting from points_begin) of the first reference point that is on one of the two curtain's raw
	bool end_expend;      ///< true iif the polygon should expend when the last reference point is reached
	double other_radius;  ///< the distance bewteen the last point of the polygon and the curtain
public:
	/// A basic ReferencePolygon constructor
	ReferencePolygon(int points_begin, int points_end, const ReferencePoint &ref, const ReferencePoint &other, bool ref_is_first, int index, bool end_expend, double other_radius);
	
	/// A ReferencePolygon destructor
	virtual ~ReferencePolygon() {};

	/// Returns ref_is_first
	bool refIsFirst() const;

	/// Returns end expend
	bool endExpend() const;

	/// Returns the index of the first reference point
	int getPointsBegin() const;

	/// Returns the index of the last reference point
	int getPointsEnd() const;

	/// Returns the index of the first intersected reference point
	int getIntersectionIndex() const;

	/// Returns the radius of the last polygon's point
	double getOtherRadius() const;

	/// Returns the reference point
	ReferencePoint getRef() const;

	/// Returns the last point
	ReferencePoint getOther() const;
};

class Situation {
private:
	CurtainSegment curt;                          ///< the curtain
	HalfCircle hc;                                ///< the half circle
	std::vector<ReferencePolygon> polygons;       ///< the polygons created by the curtain and the half circle
	std::vector<ReferencePoint> reference_points; ///< the reference points
	bool circle_low;                              ///< true iif the half circle is turned toward the curtain
	double limit_radius;                          ///< parameter that enables to emphasizes some particular situations

	/// Initialize the polygons and reference_points
	void setPolygons();

	/// Computes the reachability analysis of the nb-th polygon
	/**
	 * @param nb the index of the polygon
	 * @param radius the width of the curtain
	 * @param[out] the vector where the result is set
	 */
	void computesRA(int nb, double radius, std::vector<custom_robot_msgs::Positions> &points) const;
public:
	/// A basic Situation constructor
	Situation(const CurtainSegment &curt, const HalfCircle &hc);

	/// A Situation destructor
	virtual ~Situation() {};

	/// Given a curtain's width, computes the reachability analysis
	/**
	 * @param radius the width of the curtain
	 * @return the points that forms the different polygons
	 */
	std::vector<custom_robot_msgs::Positions> getRA(double radius) const;
};

struct hcComparator {
	explicit hcComparator(double angle) : angle(angle) {}

	inline bool operator() (const ReferencePoint& c1, const ReferencePoint& c2) {
		return (c1.getPoint().y * sin(angle) + c1.getPoint().x * cos(angle)) < (c2.getPoint().y * sin(angle) + c2.getPoint().x * cos(angle));
	}
	double angle;
};

struct hcInvComparator {
	explicit hcInvComparator(double angle) : angle(angle) {}

	inline bool operator() (const ReferencePoint& c1, const ReferencePoint& c2) {
		return (c1.getPoint().y * sin(angle) + c1.getPoint().x * cos(angle)) > (c2.getPoint().y * sin(angle) + c2.getPoint().x * cos(angle));
	}
	double angle;
};


#endif
