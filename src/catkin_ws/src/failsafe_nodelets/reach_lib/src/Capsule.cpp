#include "reach_lib/Capsule.hpp"



Capsule::Capsule(){
    this->p1 = Point();
    this->p2 = Point();
    this->r = 0.0;
}

Capsule::Capsule(Point p1, Point p2, double r){
    this->p1 = p1;
    this->p2 = p2;
    this->r = r;
}

Capsule::~Capsule(){};


Capsule Capsule::minkowski(Capsule c1, Capsule c2){
    Point p1 = Point::add(c1.p1,c2.p1);
    Point p2 = Point::add(c1.p1,c2.p1);
    return Capsule(p1,p2,c1.r + c2.r);
}

std::tuple<double,double> Capsule::alphaBeta(double ri, double rj, Point x){
    double n = Point::norm(x);
    return std::make_tuple(std::max(ri - rj, n), std::min(ri - rj, n));
}

Point Capsule::pk(Point pj, Point x, double b){
    double norm = Point::norm(x);

    if(norm == 0.0){
        return pj;
    }
    Point pk = Point();
    pk.x = pj.x + (x.x/norm)*b;
    pk.y = pj.y + (x.y/norm)*b;
    pk.z = pj.z + (x.z/norm)*b;
    
    return pk;
}

Capsule Capsule::capsuleEnclosure(Capsule c1, Capsule c2){
    double ri = 0.0;
    double rj = 0.0;
    Point pi = Point();
    Point pj = Point();

    if(c1.r >= c2.r){
        ri = c1.r;
        rj = c2.r;
        pi = c1.p1;
        pj = c2.p1;
    }else{
        ri = c2.r;
        rj = c1.r;
        pi = c2.p1;
        pj = c1.p1;
    }

    Point x = Point::diff(pi,pj);
    std::tuple<double,double> ab = Capsule::alphaBeta(ri, rj, x);
    Point pk = Capsule::pk(pj, x, std::get<1>(ab));
    double dis = Point::norm(pi, pk);

    return Capsule(c1.p1, c2.p1, ri);
}

Capsule Capsule::ballEnclosure(Capsule c1, Capsule c2){
    double ri = 0.0;
    double rj = 0.0;
    Point pi;
    Point pj;

    if(c1.r >= c2.r){
        ri = c1.r;
        rj = c2.r;
        pi = c1.p1;
        pj = c2.p1;
    }else{
        ri = c2.r;
        rj = c1.r;
        pi = c2.p1;
        pj = c1.p1;
    }

    Point x = Point::diff(pi, pj);
    std::tuple<double,double> ab = Capsule::alphaBeta(ri, rj, x);
    Point pk = Capsule::pk(pj, x, std::get<1>(ab));
    pi.x = (pi.x + pk.x)/2.0;
    pi.y = (pi.y + pk.y)/2.0;
    pi.z = (pi.z + pk.z)/2.0;

    return Capsule(pi, pi, (ri + rj + std::get<0>(ab))/2.0);
}

bool Capsule::point_in_capsule(Capsule c, Point p, double r){
    Point d = Point::diff(c.p2, c.p1);
    Point p1d = Point::diff(p, c.p1);
    Point p2d = Point::diff(p, c.p2);
    double h = Point::norm(d);
    double hsq = pow(h,2);
    double dotprod = Point::inner_dot(p1d, d);

    
    if((dotprod < 0.0) || (dotprod > hsq)){
        /*
        if(dotprod < 0.0 && Point::norm(p1d) > c.r + r){
            return false;
        }else{
            return true;
        }
        if(dotprod > hsq && Point::norm(p2d) > c.r + r){
            return false;
        }else{
            return true;
        }*/
        if(Point::norm(p1d) > c.r + r && Point::norm(p2d) > c.r + r){
            return false;
        }else{
            return true;
        }
    }else{
        double rsq = pow(c.r + r, 2);
        double dsq = Point::inner_dot(p1d, p1d) - ((dotprod*dotprod)/hsq);
        if(dsq > rsq){
            return false;
        }else{
            return true;
        }
    }
}

bool Capsule::point_in_ball(Capsule c, Point p, double r){
    if(Point::norm(c.p1,p) <= c.r + r){
        return true;
    }else{
        return false;
    }
}

bool Capsule::point_in_reach(Point p, double r){
    if(this->p1 == this->p2){
        return Capsule::point_in_ball(*this,p,r);
    }else{
        return Capsule::point_in_capsule(*this,p,r);
    }
}

bool Capsule::capsule_capsule_intersection(Capsule c1, Capsule c2, double s){
    if(Capsule::point_in_capsule(c1, c2.p1, c2.r) == true || Capsule::point_in_capsule(c1, c2.p2, c2.r) == true){
        return true;
    }else{
        Point e1 = Point::diff(c1.p2, c1.p1);
        Point e2 = Point::diff(c2.p2, c2.p1);
        Point n = Point::cross(e1,e2);

        Point diff;
        if(c1.p1.z <= c2.p1.z){
            diff = Point::diff(c1.p1,c2.p1);
        }else{
            diff = Point::diff(c2.p1,c1.p1);
        }

        double d = Point::inner_dot(n,diff)/Point::norm(n);
        if(std::abs(d) <= c1.r + c2.r){
            return true;
        }else{
            return false;
        }
    }
}

bool Capsule::intersection(Capsule c1, Capsule c2){
    std::vector<int> b;
    if(c1.p1 == c1.p2){
        b.push_back(1);
    }else{
        b.push_back(0);
    }
    if(c2.p1 == c2.p2){
        b.push_back(1);
    }else{
        b.push_back(0);
    }

    if(b[0] == 0 && b[1] == 0){
        return Capsule::capsule_capsule_intersection(c1, c2, 0.0);
    }
    if(b[0] == 1 && b[1] == 0){
        return Capsule::point_in_capsule(c2, c1.p1, c1.r);
    }
    if(b[0] == 0 && b[1] == 1){
        return Capsule::point_in_capsule(c1, c2.p1, c2.r);
    }
    if(b[0] == 1 && b[1] == 1){
        return Capsule::point_in_ball(c1, c2.p1, c2.r);
    }
    return false;
}

std::string Capsule::to_string(){
    return "\np1: " + this->p1.to_string() + "\np2: " + this->p2.to_string() + "\nr: " + std::to_string(this->r);
}

custom_robot_msgs::Capsule Capsule::toCapsuleMsg(){
    custom_robot_msgs::Capsule cap;
    cap.segment.p = this->p1.toPointMsg();
    cap.segment.q = this->p2.toPointMsg();
    cap.radius = this->r;
    return cap;
}