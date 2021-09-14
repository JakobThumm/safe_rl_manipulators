#include <Helpers.hpp>

#include <global_library/global_library.h>

struct radiusComparator 
{
  inline bool operator() (const ReferencePoint &c1, const ReferencePoint &c2) {
    return (c1.getRadius() < c2.getRadius());
  }
};

///////////////////////////////////Segment//////////////////////////////////////////

Segment::Segment(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) 
{
  this->p1 = p1;
  this->p2 = p2;
  double length2 = pow(p1.x - p2.x,2) + pow(p1.y - p2.y,2);
  this->length = sqrt(length2);
}

////////////////////////////////////////////////////////////////////////////////////

geometry_msgs::Point Segment::getP1() const 
{
  return p1;
}

////////////////////////////////////////////////////////////////////////////////////

geometry_msgs::Point Segment::getP2() const 
{
  return p2;
}

////////////////////////////////////////////////////////////////////////////////////

double Segment::getLength() const 
{
  return length;
}

////////////////////////////////////////////////////////////////////////////////////

void Segment::invert()
{
  geometry_msgs::Point temp = p2;
  p2 = p1;
  p1 = temp;
}

////////////////////////////////////////////////////////////////////////////////////

geometry_msgs::Point Segment::getPoint(double t) const 
{
  geometry_msgs::Point p;
  p.x = p1.x + t * (p2.x - p1.x);
  p.y = p1.y + t * (p2.y - p1.y);
  return p;
}

////////////////////////////////////////////////////////////////////////////////////

void Segment::print() const 
{
  geometry_helpers::print(p1);
  std::cout << " -> ";
  geometry_helpers::print(p2);
}

/////////////////////////////////CurtainSegment/////////////////////////////////////

CurtainSegment::CurtainSegment(Segment &segment): 
  segment(segment)
{
  b = (segment.getP2().x - segment.getP1().x);
  a = -(segment.getP2().y - segment.getP1().y);
  if (2 * segment.getP1().x * 0.1 * a + 0.1 * a * 0.1 * a + 2 * segment.getP1().y * 0.1 * b + 0.1 * b * 0.1 * b > 0) {
    b = -b;
    a = -a;
    this->segment.invert();
  }
  a = a / segment.getLength();
  b = b / segment.getLength();
}

////////////////////////////////////////////////////////////////////////////////////

CurtainSegment::CurtainSegment(geometry_msgs::Point &p1, geometry_msgs::Point &p2):
  segment(Segment(p1,p2))
{
  b = (segment.getP2().x - segment.getP1().x);
  a = -(segment.getP2().y - segment.getP1().y);
  if (2 * segment.getP1().x * 0.1 * a + 0.1 * a * 0.1 * a + 2 * segment.getP1().y * 0.1 * b + 0.1 * b * 0.1 * b > 0) {
    b = -b;
    a = -a;
    this->segment.invert();
  }
  a = a / segment.getLength();
  b = b / segment.getLength();
}
	
////////////////////////////////////////////////////////////////////////////////////

void CurtainSegment::setSegment(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) 
{
  this->segment = Segment(p1,p2);
  b = (segment.getP2().x - segment.getP1().x);
  a = -(segment.getP2().y - segment.getP1().y);
  if (2 * segment.getP1().x * 0.1 * a + 0.1 * a * 0.1 * a + 2 * segment.getP1().y * 0.1 * b + 0.1 * b * 0.1 * b > 0) {
    b = -b;
    a = -a;
    this->segment.invert();
  }
  a = a / segment.getLength();
  b = b / segment.getLength();
}

////////////////////////////////////////////////////////////////////////////////////

double CurtainSegment::getA() const 
{
  return a;
}

////////////////////////////////////////////////////////////////////////////////////

double CurtainSegment::getB() const 
{
  return b;
}

////////////////////////////////////////////////////////////////////////////////////

Segment CurtainSegment::getSegment() const 
{
  return segment;
}

////////////////////////////////////////////////////////////////////////////////////

double CurtainSegment::getA(bool is_first) const 
{
  if (is_first) {
    return b;
  }
  return -b;
}

////////////////////////////////////////////////////////////////////////////////////

double CurtainSegment::getB(bool is_first) const 
{
  if (is_first) {
    return -a;
  }
  return a;
}

////////////////////////////////////////////////////////////////////////////////////

geometry_msgs::Point CurtainSegment::getPoint(bool is_first) const 
{
  if (is_first) {
    return segment.getP1();
  }
  return segment.getP2();
}

////////////////////////////////////////////////////////////////////////////////////

custom_robot_msgs::Positions CurtainSegment::getPoints() const 
{
  custom_robot_msgs::Positions points;
  points.data.push_back(segment.getP1());
  points.data.push_back(segment.getP2());
  return points;
}

//////////////////////////////ReferencePoint////////////////////////////////////////

ReferencePoint::ReferencePoint(geometry_msgs::Point& point, double radius, int ref_is_first, bool on_circle, bool on_segment) :
  point(point),
  radius(radius),
  ref_is_first(ref_is_first),
  on_circle(on_circle),
  on_segment(on_segment)
{}

////////////////////////////////////////////////////////////////////////////////////

double ReferencePoint::getRadius() const 
{
  return radius;
}

////////////////////////////////////////////////////////////////////////////////////

bool ReferencePoint::onCircle() const 
{
  return on_circle;
}

////////////////////////////////////////////////////////////////////////////////////

bool ReferencePoint::onSegment() const 
{
  return on_segment;
}

////////////////////////////////////////////////////////////////////////////////////

int ReferencePoint::getRefIsFirst() const 
{
  return ref_is_first;
}

////////////////////////////////////////////////////////////////////////////////////

void ReferencePoint::setRadius(double radius) 
{
  this->radius = radius;
}

////////////////////////////////////////////////////////////////////////////////////

geometry_msgs::Point ReferencePoint::getPoint() const 
{
  return point;
}

//////////////////////////////HelfCircle////////////////////////////////////////////

HalfCircle::HalfCircle(const geometry_msgs::Point& center, double radius, double angle) :
  center(center),
  radius(radius),
  angle(angle)
{
  geometry_msgs::Point p1;
  p1.x = center.x + radius * cos(angle);
  p1.y = center.y + radius * sin(angle);
  geometry_msgs::Point p2;
  p2.x = center.x - radius * cos(angle);
  p2.y = center.y - radius * sin(angle);
  segment = Segment(p1, p2);
}

////////////////////////////////////////////////////////////////////////////////////

double HalfCircle::getAngle() const 
{
  return angle;
}

////////////////////////////////////////////////////////////////////////////////////

geometry_msgs::Point HalfCircle::getCenter() const 
{
  return center;
}

////////////////////////////////////////////////////////////////////////////////////

Segment HalfCircle::getSegment() const 
{
  return segment;
}

////////////////////////////////////////////////////////////////////////////////////

bool HalfCircle::isOn(const geometry_msgs::Point &p) const 
{
  if (std::abs(pow(p.x - center.x, 2) + pow(p.y - center.y, 2) - radius * radius) < 1e-6){
    return (p.y - center.y) * cos(angle) - (p.x - center.x) * sin(angle) > 0;
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////////

bool HalfCircle::isIn(const geometry_msgs::Point &p) const 
{
  if (pow(p.x - center.x, 2) + pow(p.y - center.y, 2) > radius * radius) {
    return false;
  }
  return (p.y - center.y) * cos(angle) - (p.x - center.x) * sin(angle) > 0;
}

////////////////////////////////////////////////////////////////////////////////////

double HalfCircle::intersection(const geometry_msgs::Point &p, double delta_x, double delta_y, geometry_msgs::Point &answer) const 
{
  double a1 = delta_x;
  double a2 = delta_y;
  double b1 = -(segment.getP2().x - segment.getP1().x);
  double b2 = -(segment.getP2().y - segment.getP1().y);
  double c1 = segment.getP1().x - p.x;
  double c2 = segment.getP1().y - p.y;

  double det = a1 * b2 - a2 * b1;
  if (det != 0) {
    double t = (c1 * b2 - c2 * b1) / det;
    double t1 = (a1 * c2 - a2 * c1) / det;
    if (t > 0 && t1 < 1 && t1 >= 0) {
      answer.x = p.x + t * delta_x;
      answer.y = p.y + t * delta_y;
      return t;
    }
  }
  return -1;
}

////////////////////////////////////////////////////////////////////////////////////

int HalfCircle::intersections(const geometry_msgs::Point &p, double delta_x, double delta_y, int is_first, double length, std::vector<ReferencePoint> &cond) const 
{
  double t = (delta_x * (center.x - p.x) + delta_y * (center.y - p.y));
  geometry_msgs::Point middle;
  middle.x = t * delta_x + p.x;
  middle.y = t* delta_y + p.y;

  double distance_center = geometry_helpers::distance(middle, center);
  int i = 0;

  if (distance_center < radius) {
    double distance_inter = sqrt(pow(radius, 2) - pow(distance_center,2));
    double d_1 = t - distance_inter;
    if (d_1 > 0 && d_1 < length) {
      geometry_msgs::Point first;
      first.x = p.x + d_1 * delta_x;
      first.y = p.y + d_1 * delta_y;
      if (isOn(first)) {
        cond.push_back(ReferencePoint(first, d_1, is_first, true, false));
        i += 1;
      }
    }
    geometry_msgs::Point seg;
    d_1 = intersection(p, delta_x, delta_y, seg);
    if (d_1 > 0 && d_1 < length) {
      cond.push_back(ReferencePoint(seg, d_1, is_first, false, true));
      i += 1;
    }
    d_1 = t + distance_inter;
    if (d_1 > 0 && d_1 < length) {
      geometry_msgs::Point second;
      second.x = p.x + d_1 * delta_x;
      second.y = p.y + d_1 * delta_y;
      if (isOn(second)) {
        cond.push_back(ReferencePoint(second, d_1, is_first, true, false));
        i += 1;
      }
    }
    return i;
  }
  i = -1;
  return i;
}

////////////////////////////////ReferencePolygon////////////////////////////////////

ReferencePolygon::ReferencePolygon(int points_begin, int points_end, const ReferencePoint& ref, const ReferencePoint& other, bool ref_is_first, int index, bool end_expend, double other_radius):
  points_begin(points_begin),
  points_end(points_end),
  ref(ref),
  other(other),
  ref_is_first(ref_is_first),
  index_ref(index),
  end_expend(end_expend),
  other_radius(other_radius)
{}

////////////////////////////////////////////////////////////////////////////////////

bool ReferencePolygon::refIsFirst() const
{
  return ref_is_first;
}

////////////////////////////////////////////////////////////////////////////////////

bool ReferencePolygon::endExpend() const
{
  return end_expend;
}

////////////////////////////////////////////////////////////////////////////////////

int ReferencePolygon::getPointsBegin() const
{
  return points_begin;
}

////////////////////////////////////////////////////////////////////////////////////

int ReferencePolygon::getPointsEnd() const
{
  return points_end;
}

////////////////////////////////////////////////////////////////////////////////////

double ReferencePolygon::getOtherRadius() const
{
  return other_radius;
}

////////////////////////////////////////////////////////////////////////////////////

ReferencePoint ReferencePolygon::getRef() const 
{
  return ref;
}

////////////////////////////////////////////////////////////////////////////////////

ReferencePoint ReferencePolygon::getOther() const
{
  return other;
}

////////////////////////////////////////////////////////////////////////////////////

int ReferencePolygon::getIntersectionIndex() const
{
  return index_ref;
}

////////////////////////////////Situation///////////////////////////////////////////

Situation::Situation(const CurtainSegment &curt, const HalfCircle &hc) :
  curt(curt), 
  hc(hc) 
{
  double alpha = acos((curt.getSegment().getP2().x - curt.getSegment().getP1().x) / curt.getSegment().getLength());
  if (curt.getSegment().getP2().y < curt.getSegment().getP1().y) {
    alpha = alpha + M_PI;
  }
  double diff = hc.getAngle() - alpha;
  if (diff < 0) {
    diff += 2 * M_PI;
  }
  circle_low = (diff < M_PI / 2 || diff > 3 * M_PI / 2);
  setPolygons();
}

////////////////////////////////////////////////////////////////////////////////////

void Situation::setPolygons() 
{
  // check if the circle cuts the curtain
  std::vector<ReferencePoint> init;
  int nb_points = hc.intersections(curt.getSegment().getP1(), curt.getB(), -curt.getA(), true, curt.getSegment().getLength(), init);
  
  for (auto c = init.begin(); c != init.end(); c++) {
    c->setRadius(0);
  }
  
  // check if the circle will cut the curtain
  std::vector<ReferencePoint> reference_points;
  int i1 = hc.intersections(curt.getSegment().getP1(), curt.getA(), curt.getB(), 1, 1e16, reference_points);
  int i2 = hc.intersections(curt.getSegment().getP2(), curt.getA(), curt.getB(), 2, 1e16, reference_points);
  
  // if necessary, get more points in the circle to add the precision
  // 1) the "opposite point" of the circle
  if (nb_points < 2 || (i1 <= 0 && i2 <= 0)) {
    geometry_msgs::Point middle;
    middle.x = hc.getCenter().x + (hc.getSegment().getP2().y - hc.getSegment().getP1().y) / 2;
    middle.y = hc.getCenter().y + (hc.getSegment().getP1().x - hc.getSegment().getP2().x) / 2;
    double d = curt.getA() * (middle.x - curt.getSegment().getP1().x) + curt.getB() * (middle.y - curt.getSegment().getP1().y);
    if (d > 0 && nb_points == 0) {
      nb_points = -1;
    }
    double t = (curt.getB() * (middle.x - curt.getSegment().getP1().x) - curt.getA() * (middle.y - curt.getSegment().getP1().y));
    if (d > 0 && t > 0 && t < curt.getSegment().getLength()) {
      if (t >= curt.getSegment().getLength() / 2) {
        reference_points.push_back(ReferencePoint(middle, d, -2, true, false));
      }
      else {
        reference_points.push_back(ReferencePoint(middle, d, -1, true, false));
      }
    }
  }
  // 2) the circle's segment points
  if (i1 <= 0 || i2 <= 0) {
    double r = curt.getA() * (hc.getSegment().getP1().x - curt.getSegment().getP1().x) + curt.getB() * (hc.getSegment().getP1().y - curt.getSegment().getP1().y);
    double t = (curt.getB() * (hc.getSegment().getP1().x - curt.getSegment().getP1().x) - curt.getA() * (hc.getSegment().getP1().y - curt.getSegment().getP1().y));
    if (r > 0 && t > 0 && t < curt.getSegment().getLength()) {
      geometry_msgs::Point p = hc.getSegment().getP1();
      reference_points.push_back(ReferencePoint(p, r, 0, true, true));
    }
    r = curt.getA() * (hc.getSegment().getP2().x - curt.getSegment().getP1().x) + curt.getB() * (hc.getSegment().getP2().y - curt.getSegment().getP1().y);
    t = (curt.getB() * (hc.getSegment().getP2().x - curt.getSegment().getP1().x) - curt.getA() * (hc.getSegment().getP2().y - curt.getSegment().getP1().y));
    if (r > 0 && t > 0 && t < curt.getSegment().getLength()) {
      geometry_msgs::Point p = hc.getSegment().getP2();
      reference_points.push_back(ReferencePoint(p, r, 0, true, true));
    }
  }

  // set the polygon according the situation
  geometry_msgs::Point p_1 = curt.getSegment().getP1();
  ReferencePoint p1 = ReferencePoint(p_1, 0, 0, false, false);
  geometry_msgs::Point p_2 = curt.getSegment().getP2();
  ReferencePoint p2 = ReferencePoint(p_2, 0, 0, false, false);

  if (nb_points == 2) {
    if (i1 == 2 && i2 == 2) {
      iter_swap(reference_points.begin() + 1, reference_points.begin() + 2);
      polygons.push_back(ReferencePolygon(0, 1, p1, init[0], true, 0, false, reference_points.begin()->getRadius()));
      polygons.push_back(ReferencePolygon(1, 2, p2, init[1], false, 0, false, (reference_points.begin() + 1)->getRadius()));
      std::sort(reference_points.begin() + 2, reference_points.end(), radiusComparator());
      auto c = reference_points.begin() + 2;
      int index = 0;
      while (c != reference_points.end() && c->getRefIsFirst() <= 0) {
        c++;
        index++;
      }
      bool ref_is_first = -1 * (c->getRefIsFirst() - 2);
      polygons.push_back(ReferencePolygon(4, reference_points.size(), reference_points[2], reference_points[3], ref_is_first, index - 1, true, 0));
      this->limit_radius = -1;
    }
    else if (i1 == 2) {
      polygons.push_back(ReferencePolygon(0, 1, p1, init[0], true, 0, false, reference_points.begin()->getRadius()));
      std::sort(reference_points.begin() + 1, reference_points.end(), radiusComparator());
      auto c = reference_points.begin() + 1;
      int index = 0;
      while (c != reference_points.end() && c->getRefIsFirst() <= 0) {
        c++;
        index++;
      }
      polygons.push_back(ReferencePolygon(1, reference_points.size(), p2, init[1], false, index, true, reference_points.begin()->getRadius()));
      this->limit_radius = -1;
    }
    else if (i2 == 2) {
      polygons.push_back(ReferencePolygon(0, 1, p2, init[1], false, 0, false, reference_points.begin()->getRadius()));
      std::sort(reference_points.begin() + 1, reference_points.end(), radiusComparator());
      auto c = reference_points.begin() + 1;
      int index = 0;
      while (c != reference_points.end() && c->getRefIsFirst() <= 0) {
        c++;
        index++;
      }
      polygons.push_back(ReferencePolygon(1, reference_points.size(), p1, init[0], true, index, true, reference_points.begin()->getRadius()));
      this->limit_radius = -1;
    }
    else {
      bool swap = false;
      if (reference_points.begin()->getRadius() >= (reference_points.begin() + 1)->getRadius()) {
        swap = true;
        iter_swap(reference_points.begin(), reference_points.begin() + 1);
      }
      double rad = (reference_points.end() - 1)->getRadius();
      this->limit_radius = rad;
      if (reference_points.begin()->getRefIsFirst() == -1 || 
         (reference_points.begin() + 1)->getRefIsFirst() == -2) {
        polygons.push_back(ReferencePolygon(1, 2, p1, init[0], true, 1, false, reference_points.begin()->getRadius()));
        polygons.push_back(ReferencePolygon(2, 3, p2, init[1], false, 1, false, (reference_points.begin() + 1)->getRadius()));
        reference_points.insert(reference_points.begin(), init[0]);
        reference_points.push_back(init[1]);
        polygons.push_back(ReferencePolygon(0, reference_points.size(), p1, p2, true, 0, true, rad));
      }
      else if (reference_points.begin()->getRefIsFirst() == -2 || 
              (reference_points.begin() + 1)->getRefIsFirst() == -1 || swap) {
        polygons.push_back(ReferencePolygon(2, 3, p1, init[0], true, 1, false, (reference_points.begin() + 1)->getRadius()));
        polygons.push_back(ReferencePolygon(1, 2, p2, init[1], false, 1, false, reference_points.begin()->getRadius()));
        reference_points.insert(reference_points.begin(), init[1]);
        reference_points.push_back(init[0]);
        polygons.push_back(ReferencePolygon(0, reference_points.size(), p2, p1, false, 0, true, rad));
      }
      else {
        polygons.push_back(ReferencePolygon(1, 2, p1, init[0], true, 1, false, reference_points.begin()->getRadius()));
        polygons.push_back(ReferencePolygon(2, 3, p2, init[1], false, 1, false, (reference_points.begin() + 1)->getRadius()));
        reference_points.insert(reference_points.begin(), init[0]);
        reference_points.push_back(init[1]);
        polygons.push_back(ReferencePolygon(0, reference_points.size(), p1, p2, true, 0, true, rad));
      }
    }
  }
  else if (nb_points == 1) {
    if (i1 == 2) {
      polygons.push_back(ReferencePolygon(0, 1, p1, init[0], true, 0, false, reference_points.begin()->getRadius()));
      std::sort(reference_points.begin() + 1, reference_points.end(), radiusComparator());
      auto c = reference_points.begin() + 1;
      int index = 0;
      while (c != reference_points.end() && c->getRefIsFirst() <= 0) {
        c++;
        index++;
      }
      bool ref_is_first = -1 * (c->getRefIsFirst() - 2);
      polygons.push_back(ReferencePolygon(3, reference_points.size(), reference_points[1], reference_points[2], ref_is_first, index - 1, true, 0));
      this->limit_radius = -1;
    }
    else if (i2 == 2) {
      iter_swap(reference_points.begin(), reference_points.begin() + i1);
      polygons.push_back(ReferencePolygon(0, 1, p2, init[0], false, 0, false, reference_points.begin()->getRadius()));
      std::sort(reference_points.begin() + 1, reference_points.end(), radiusComparator());
      auto c = reference_points.begin() + 1;
      int index = 0;
      while (c != reference_points.end() && c->getRefIsFirst() <= 0) {
        c++;
        index++;
      }
      bool ref_is_first = -1 * (c->getRefIsFirst() - 2);
      polygons.push_back(ReferencePolygon(3, reference_points.size(), reference_points[1], reference_points[2], ref_is_first, index - 1, true, 0));
      this->limit_radius = -1;
    }
    else {
      if (i1 <= 0) {
        std::sort(reference_points.begin(), reference_points.end(), radiusComparator());
        auto c = reference_points.begin();
        int index = 0;
        while (c != reference_points.end() && c->getRefIsFirst() <= 0) {
          c++;
          index++;
        }
        polygons.push_back(ReferencePolygon(0, reference_points.size(), p1, init[0], true, index, true, 0));
        this->limit_radius = -1;
      }
      else {
        std::sort(reference_points.begin(), reference_points.end(), radiusComparator());
        auto c = reference_points.begin();
        int index = 0;
        while (c != reference_points.end() && c->getRefIsFirst() <= 0) {
          c++;
          index++;
        }
        polygons.push_back(ReferencePolygon(0, reference_points.size(), p2, init[0], false, index, true, 0));
        this->limit_radius = -1;
      }
    }
  }
  else if (i1 == 2 && i2 == 2) {
    iter_swap(reference_points.begin() + 1, reference_points.begin() + 2);
    if (this->circle_low) {
      std::sort(reference_points.begin(), reference_points.begin() + 2, radiusComparator());
      auto c = reference_points.begin();
      int index = 0;
      while (c != reference_points.begin() + 2 && c->getRefIsFirst() <= 0) {
        c++;
        index++;
      }
      bool ref_is_first = -1 * (c->getRefIsFirst() - 2);
      if (ref_is_first) {
        polygons.push_back(ReferencePolygon(0, 2, p1, p2, ref_is_first, index, false, 0));
      }
      else {
        polygons.push_back(ReferencePolygon(0, 2, p2, p1, ref_is_first, index, false, 0));
      }
      std::sort(reference_points.begin() + 2, reference_points.end(), radiusComparator());
      ref_is_first = -1 * (reference_points[2].getRefIsFirst() - 2);
      polygons.push_back(ReferencePolygon(4, reference_points.size(), reference_points[2], reference_points[3], ref_is_first, 1, true, reference_points[2].getRadius()));
    }
    else {
      int first = reference_points.size() - i1 - i2;
      if (first) {
        iter_swap(reference_points.begin() + 2, reference_points.end() - 1);
      }
      std::sort(reference_points.begin(), reference_points.begin() + 2 + first, radiusComparator());
      auto c = reference_points.begin();
      int index = 0;
      while (c != reference_points.begin() + 2 + first && c->getRefIsFirst() <= 0) {
        c++;
        index++;
      }
      bool ref_is_first = -1 * (c->getRefIsFirst() - 2);
      if (ref_is_first) {
        polygons.push_back(ReferencePolygon(0, 2 + first, p1, p2, ref_is_first, index, false, 0));
      }
      else {
        polygons.push_back(ReferencePolygon(0, 2 + first, p2, p1, ref_is_first, index, false, 0));
      }

      std::sort(reference_points.begin() + 2 + first, reference_points.end(), radiusComparator());
      ref_is_first = -1 * (reference_points[2 + first].getRefIsFirst() - 2);
      polygons.push_back(ReferencePolygon(4 + first, reference_points.size(), reference_points[2 + first], reference_points[3 + first], ref_is_first, 1, true, reference_points[2 + first].getRadius()));
    }
    this->limit_radius = -1;
  }
  else if (i1 <= 0 && i2 <= 0 && nb_points <= 0) {
    std::sort(reference_points.begin(), reference_points.end(), radiusComparator());
    auto c = reference_points.end() - 1;
    int index = reference_points.size() - 1;
    while (c != reference_points.begin() && c->getRefIsFirst() < 0) {
      c--;
      index--;
    }
    bool ref_is_first = (curt.getB() * (c->getPoint().x - curt.getSegment().getP1().x) - curt.getA() * (c->getPoint().y - curt.getSegment().getP1().y) <= curt.getSegment().getLength() / 2);
    if (ref_is_first) {
      polygons.push_back(ReferencePolygon(0, reference_points.size(), p1, p2, ref_is_first, reference_points.size(), true, 0));
    }
    else {
      polygons.push_back(ReferencePolygon(0, reference_points.size(), p2, p1, ref_is_first, reference_points.size(), true, 0));
    }
    this->limit_radius = -1 - (reference_points.end() - 1)->getRadius();
  }
  else if (nb_points <= 0 && i1 == 1 && i2 == 1) {
    std::sort(reference_points.begin(), reference_points.end(), radiusComparator());
    bool ref_is_first = -1 * (reference_points[0].getRefIsFirst() - 2);
    polygons.push_back(ReferencePolygon(2, reference_points.size(), reference_points[0], reference_points[1], ref_is_first, 2, true, reference_points[0].getRadius()));
  }
  else {
    std::sort(reference_points.begin(), reference_points.end(), radiusComparator());
    auto c = reference_points.begin();
    int index = 0;
    while (c != reference_points.end() && c->getRefIsFirst() <= 0) {
      c++;
      index++;
    }
    bool ref_is_first = -1 * (c->getRefIsFirst() - 2);
    if (ref_is_first) {
      polygons.push_back(ReferencePolygon(0, reference_points.size(), p1, p2, ref_is_first, index, true, 0));
    }
    else {
      polygons.push_back(ReferencePolygon(0, reference_points.size(), p2, p1, ref_is_first, index, true, 0));
    }
    this->limit_radius = -1;
  }
  this->reference_points = reference_points;
}

////////////////////////////////////////////////////////////////////////////////////

std::vector<custom_robot_msgs::Positions> Situation::getRA(double radius) const 
{
  std::vector<custom_robot_msgs::Positions> polygon_vector;
  int i = 0;
  if (this->limit_radius >= 0) {
    if (limit_radius < radius) {
      ReferencePolygon poly = polygons[2];
      custom_robot_msgs::Positions new_polygon;
      new_polygon.data.push_back(poly.getOther().getPoint());
      geometry_msgs::Point inter_half; 
      inter_half.x = poly.getRef().getPoint().x + (radius - poly.getRef().getRadius()) * curt.getA();
      inter_half.y = poly.getRef().getPoint().y + (radius - poly.getRef().getRadius()) * curt.getB();

      geometry_msgs::Point opposite;
      opposite.x = inter_half.x + curt.getA(poly.refIsFirst()) * curt.getSegment().getLength();
      opposite.y = inter_half.y + curt.getB(poly.refIsFirst()) * curt.getSegment().getLength();
      new_polygon.data.push_back(opposite);
      new_polygon.data.push_back(inter_half);
      new_polygon.data.push_back(poly.getRef().getPoint());
      for (auto pt = reference_points.begin(); pt != reference_points.end(); pt++) {
        new_polygon.data.push_back(pt->getPoint());
      }
      new_polygon.data.push_back(poly.getOther().getPoint());
      polygon_vector.push_back(new_polygon);
    }
    else {
      computesRA(0, radius, polygon_vector);
      computesRA(1, radius, polygon_vector);
    }
  }
  else if (this->limit_radius < -1 && -(limit_radius + 1) < radius) { // the circle is totally inside the curtain's RA
    custom_robot_msgs::Positions new_polygon;
    new_polygon.data.push_back(curt.getSegment().getP1());
    geometry_msgs::Point inter_half;
    inter_half.x = curt.getSegment().getP1().x + radius * curt.getA();
    inter_half.y = curt.getSegment().getP1().y + radius * curt.getB();
    new_polygon.data.push_back(inter_half);
    geometry_msgs::Point opposite;
    opposite.x = curt.getSegment().getP2().x + radius * curt.getA();
    opposite.y = curt.getSegment().getP2().y + radius * curt.getB();
    new_polygon.data.push_back(opposite);
    new_polygon.data.push_back(curt.getSegment().getP2());
    new_polygon.data.push_back(curt.getSegment().getP1());

    // the circle
    geometry_msgs::Point p;
    p.x = -100;
    p.y = 100;
    new_polygon.data.push_back(p);
    for (auto pt = reference_points.begin(); pt != reference_points.end(); pt++) {
      new_polygon.data.push_back(pt->getPoint());
    }
    new_polygon.data.push_back(reference_points.begin()->getPoint());
    polygon_vector.push_back(new_polygon);
  }
  else {
    for (auto poly = polygons.begin(); poly != polygons.end(); poly++, i++) {
      computesRA(i, radius, polygon_vector);
    }
  }
  return polygon_vector;
}

////////////////////////////////////////////////////////////////////////////////////

void Situation::computesRA(int nb, double radius, std::vector<custom_robot_msgs::Positions> &polygon_vector) const 
{
  ReferencePolygon poly = polygons[nb];
  
  //if the width is too small, there is no polygon (see case ?)
  if (radius < poly.getRef().getRadius()) {
    return;
  }
  
  std::vector<ReferencePoint> circle_points; // all the references points that are on the circle part of the half circle
  std::vector<ReferencePoint> segment_points; // all the references points that are on the segment part of the half circle

  // get all the references points and put them on the right vector
  auto pt = reference_points.begin() + poly.getPointsBegin();
  int i = -1;
  while (pt != reference_points.begin() + poly.getPointsEnd() && 
         pt->getRadius() < radius) {
    if (pt->onSegment()) {
      segment_points.push_back(*pt);
    }
    if (pt->onCircle()) {
      circle_points.push_back(*pt);
    }
    pt++;
    i++;
  }
  if (poly.getOther().onCircle() && this->circle_low) {
    circle_points.push_back(poly.getOther());
  }

  // get the intersections point between the "width line" of the curtain and the half circle's part that is in the reference polygon
  double delta_X = curt.getA(poly.refIsFirst());
  double delta_Y = curt.getB(poly.refIsFirst());
  double length = (radius > poly.getOtherRadius()) ? curt.getSegment().getLength() : sqrt(pow(poly.getOther().getPoint().x - poly.getRef().getPoint().x, 2) + pow(poly.getOther().getPoint().y - poly.getRef().getPoint().y, 2));
  length = (length > curt.getSegment().getLength()) ? curt.getSegment().getLength() : length;
  
  // the intersection point between the reference point's ray and the width line of the curtain
  geometry_msgs::Point inter_half; 
  inter_half.x = poly.getRef().getPoint().x + (radius - poly.getRef().getRadius()) * curt.getA();
  inter_half.y = poly.getRef().getPoint().y + (radius - poly.getRef().getRadius()) * curt.getB();
 
  std::vector<ReferencePoint> conds;
  int nb_intersect = 0;
  if (this->limit_radius > 0 && nb <= 1) { // in case (?) only the first intersection point have to be considered
    nb_intersect = hc.intersections(inter_half, delta_X, delta_Y, -3, curt.getSegment().getLength(), conds);
    auto pt = conds.begin();
    if (pt->onCircle()) {
      circle_points.push_back(*pt);
    }
    if (pt->onSegment()) {
      segment_points.push_back(*pt);
    }
    nb_intersect = 1;
  }
  else {
    if (poly.endExpend() || 
        i + 1 < poly.getPointsEnd() - poly.getPointsBegin()) { //if the polygon is not an extended one, when the width of the curtain is too high (ie there are too many references points), the associated polygon is a fixed one and thus the intersections points are not interesting (cf case 1)
      nb_intersect = hc.intersections(inter_half, delta_X, delta_Y, 0, length, conds);
    }
    for (auto c = conds.begin(); c != conds.end(); c++) {
      if (c->onCircle()) {
        circle_points.push_back(*c);
      }
      if (c->onSegment()) {
        segment_points.push_back(*c);
      }
    }
  }

  // sort the circle and segment vector to be able to created a closed poygon
  if (!poly.refIsFirst()) {
    std::sort(circle_points.begin(), circle_points.end(), hcComparator(hc.getAngle()));
    std::sort(segment_points.begin(), segment_points.end(), hcInvComparator(hc.getAngle()));
  }
  else {
    std::sort(circle_points.begin(), circle_points.end(), hcInvComparator(hc.getAngle()));
    std::sort(segment_points.begin(), segment_points.end(), hcComparator(hc.getAngle()));
  }
  
  custom_robot_msgs::Positions new_polygon;
  // put the first point in the polygon
  new_polygon.data.push_back(poly.getRef().getPoint());

  auto first = circle_points.begin();
  if (i < poly.getIntersectionIndex()) { // the polygon is firstly the segment's point and then the circle's point or inversely
    new_polygon.data.push_back(inter_half);
    if (nb_intersect > 0 && 
        conds.begin()->onSegment()) { // if the first intersection point is on the segment, the enumeration should begin by the segment's point
      first = circle_points.end();
    }
    geometry_msgs::Point opposite;
    opposite.x = inter_half.x + delta_X * curt.getSegment().getLength();
    opposite.y = inter_half.y + delta_Y * curt.getSegment().getLength();
    if (nb_intersect <= 0) { // no intersection => the polygon is a quadrilateral
      new_polygon.data.push_back(opposite);
      first = circle_points.end();
    }
    for (auto pt = first; pt != circle_points.end(); pt++) { // add all the circle's points that are before the segment's one
      new_polygon.data.push_back(pt->getPoint());
    }
  }
  else {
    geometry_msgs::Point first_intersect = (reference_points.begin() + poly.getPointsBegin() + poly.getIntersectionIndex())->getPoint(); // first point of the enumeration which correspond to an intersection between the curtain and the half-circle

    while (first != circle_points.end() && !geometry_helpers::equal(first->getPoint(),first_intersect)) { // get the circle's point that correpond to the first intersect point (end if this point is on the segment
      first++;
    } 
    if (first != circle_points.end()) {
      for (auto pt = first + 1; pt != circle_points.end(); pt++) {
        if (pt->getRefIsFirst() == first->getRefIsFirst()) {
          first = pt;
        } 
      }
    }
    if (this->limit_radius < -1) {
      first--;
    }
    if (poly.endExpend()) {
      int index = poly.getPointsBegin() + poly.getIntersectionIndex();
      int b = -1 * (reference_points[index].getRefIsFirst() - 2); // get the reference point of the first intersection point (ie the closest point to the reference point of the enumeration that is on its raw 
      if (!hc.isIn(inter_half) && !(b == poly.refIsFirst())) { // the intersection point is print iif it is not in the circle and there is no other point closer (and on one of the raw) to the curtain's line in the reference points list
        new_polygon.data.push_back(inter_half);
        geometry_msgs::Point opposite;
        opposite.x = inter_half.x + delta_X * curt.getSegment().getLength();
        opposite.y = inter_half.y + delta_Y * curt.getSegment().getLength();
        if (nb_intersect <= 0) { // ie the radius is higher than all the reference's points one
          new_polygon.data.push_back(opposite);
        }
      }
    }
    if (nb_intersect == 2 && (!geometry_helpers::equal((segment_points.end() - 1)->getPoint(), circle_points.begin()->getPoint()) || 
        !geometry_helpers::equal((circle_points.end() - 1)->getPoint(), segment_points.begin()->getPoint()))) { // there is a small polygon created by the curtain's width
      if (reference_points[poly.getPointsBegin() + poly.getIntersectionIndex()].onCircle()) { // this polygon is created by the circle
        if (!(conds.begin()->onSegment())) { // the first points of the enumeration are the segment's one
          for (auto pt = first; pt != circle_points.end(); pt++) {
            new_polygon.data.push_back(pt->getPoint());
          }
        }
        for (auto pt = segment_points.begin(); pt != segment_points.end(); pt++) { // add all the segment's point of the enumeration that are not on the circle (to avoid repetitions)
          if (!pt->onCircle()) {
            new_polygon.data.push_back(pt->getPoint());
          }
        }
        auto c = circle_points.begin();
        if (conds.begin()->onSegment()) { // there are points on the circle after the segment ones
          while (c != first) {
            new_polygon.data.push_back(c->getPoint());
            c++;
          }
        }
        geometry_msgs::Point opposite;
        opposite.x = inter_half.x + delta_X * curt.getSegment().getLength();
        opposite.y = inter_half.y + delta_Y * curt.getSegment().getLength();
        if (i > poly.getIntersectionIndex()) { // => there are two intersections on the reference raw and thus the reference raw is the raw which creates the new polygon
          new_polygon.data.push_back(opposite);
        }
        else {
          inter_half = opposite;
        }
        new_polygon.data.push_back(poly.getOther().getPoint());
        new_polygon.data.push_back(poly.getRef().getPoint());
        polygon_vector.push_back(new_polygon);
 
        //second
        custom_robot_msgs::Positions second_polygon;
        second_polygon.data.push_back(inter_half);
        if (conds.begin()->onSegment()) {
          for (auto pt = first; pt != circle_points.end(); pt++) {
            second_polygon.data.push_back(pt->getPoint());
          }
        }
        else {
          for (auto pt = circle_points.begin(); pt != first; pt++) {
            second_polygon.data.push_back(pt->getPoint());
          }
        }
        second_polygon.data.push_back(inter_half);
        polygon_vector.push_back(second_polygon);
        return;
      }
      geometry_msgs::Point opposite;
      opposite.x = inter_half.x + delta_X * curt.getSegment().getLength();
      opposite.y = inter_half.y + delta_Y * curt.getSegment().getLength();
      inter_half = opposite;
      for (auto pt = circle_points.begin(); pt != first; pt++) { // add the circle points (according to the previous lines, first == end)
        new_polygon.data.push_back(pt->getPoint());
      }
      new_polygon.data.push_back(poly.getOther().getPoint());
      new_polygon.data.push_back(poly.getRef().getPoint());
      polygon_vector.push_back(new_polygon);
      
      //second
      custom_robot_msgs::Positions second_polygon;
      second_polygon.data.push_back(inter_half);
      for (auto pt = segment_points.begin(); pt != segment_points.end(); pt++) {
        if (!pt->onCircle()) {
          second_polygon.data.push_back(pt->getPoint());
        }
      }
      second_polygon.data.push_back(inter_half);
      polygon_vector.push_back(second_polygon);
      return;
    }
    else {
      for (auto pt = first; pt != circle_points.end(); pt++) {
        new_polygon.data.push_back(pt->getPoint());
      }
    }
  }
  
  for (auto pt = segment_points.begin(); pt != segment_points.end(); pt++) { // add the segment's point that are not on the circle (to avoid redondency)
    if (!pt->onCircle()) {
      new_polygon.data.push_back(pt->getPoint());
    }
  }
  for (auto pt = circle_points.begin(); pt != first; pt++) { // add the circle's points that are after the segment
    new_polygon.data.push_back(pt->getPoint());
  }

  if (poly.endExpend() && i > poly.getIntersectionIndex() + poly.getPointsBegin()) { // ie the radius is higher than all the reference points' one
    int index = poly.getPointsBegin() + poly.getIntersectionIndex();
    int b = -1 * (reference_points[index].getRefIsFirst() - 2);
    if (!hc.isIn(inter_half) && !poly.getOther().onCircle() && 
        !poly.getOther().onSegment()) {
      new_polygon.data.push_back(inter_half);
      geometry_msgs::Point opposite;
      opposite.x = inter_half.x + delta_X * curt.getSegment().getLength();
      opposite.y = inter_half.y + delta_Y * curt.getSegment().getLength();
      if (!hc.isIn(opposite)) {
        new_polygon.data.push_back(opposite);
      }
    }
  }
  if (nb_intersect > 0 && limit_radius <= -1 && length >= curt.getSegment().getLength()) {
    geometry_msgs::Point opposite;
    opposite.x = inter_half.x + delta_X * curt.getSegment().getLength();
    opposite.y = inter_half.y + delta_Y * curt.getSegment().getLength();
    if (!hc.isIn(opposite)) {
      new_polygon.data.push_back(opposite);
    }
  }
  if (radius > poly.getOther().getRadius() && !(poly.getOther().onCircle() && this->circle_low)) {
    new_polygon.data.push_back(poly.getOther().getPoint());
  }
  new_polygon.data.push_back(poly.getRef().getPoint());
  polygon_vector.push_back(new_polygon);
}
