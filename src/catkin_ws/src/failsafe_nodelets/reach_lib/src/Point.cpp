#include <reach_lib/Point.hpp>



Point::Point(){
    this->x = 0.0;
    this->y = 0.0;
    this->z = 0.0;
}

Point::Point(double x, double y, double z){
    this->x = x;
    this->y = y;
    this->z = z;
}

Point::~Point(){};

Point Point::diff(Point p1, Point p2){
    Point p = Point();
    p.x = p1.x - p2.x;
    p.y = p1.y - p2.y;
    p.z = p1.z - p2.z;
    return p;
}

Point Point::add(Point p1, Point p2){
    Point p = Point();
    p.x = p1.x + p2.x;
    p.y = p1.y + p2.y;
    p.z = p1.z + p2.z;
    return p;
}

Point Point::origin(Point p1, Point p2){
    Point p = Point::diff(p1,p2);
    p.x = p2.x + p.x/2.0;
    p.y = p2.y + p.y/2.0;
    p.z = p2.z + p.z/2.0;
    return p;
}

double Point::norm(Point p){
    return sqrt(pow(p.x,2) + pow(p.y,2) + pow(p.z,2));
}

double Point::norm(Point p1, Point p2){
    return sqrt(pow(p1.x - p2.x,2) + pow(p1.y - p2.y,2) + pow(p1.z - p2.z,2));
}


double Point::inner_dot(Point p1, Point p2){
    return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
}

Point Point::cross(Point p1, Point p2){
    Point c = Point();
    c.x = p1.y*p2.z - p1.z*p2.y;
    c.y = p1.z*p2.x - p1.x*p2.z;
    c.z = p1.x*p2.y - p1.y*p2.x;
    return c;
}


//obsolete
std::tuple<double,double,double,double> Point::orientation(Point p1, Point p2){
    
    if(p2.z < p1.z){
        Point temp = p1;
        p1 = p2;
        p2 = temp;
    }

    double dis = norm(p1,p2);
    double f = 0.0174533;
    double r = 0.0;
    double p = 0.0;
    double y = 0.0;

    if(dis == 0){
        p = 0.0;
    }else{
        p = -asin((p2.y - p1.y) / dis);
    }

    if(cos(p) == 0.0 ||dis == 0.0){
        y = 0.0;
    }else{
        y = asin((p2.x - p1.x)/(cos(p) * dis));
    }

    // capsules are roll invariant

    double cy = cos(y * 0.5);
    double sy = sin(y * 0.5);
    double cp = cos(p * 0.5);
    double sp = sin(p * 0.5);
    double cr = cos(r * 0.5);
    double sr = sin(r * 0.5);

    // q := (x,y,z,w)
    //q.w = cr * cp * cy + sr * sp * sy;
    //q.x = sr * cp * cy - cr * sp * sy;
    //q.y = cr * sp * cy + sr * cp * sy;
    //q.z = cr * cp * sy - sr * sp * cy;

    return std::make_tuple(sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy);
}

std::string Point::to_string(){
    return std::to_string(this->x) + " " + std::to_string(this->y) + " " + std::to_string(this->z);
}

geometry_msgs::Point Point::toPointMsg(){
    geometry_msgs::Point p;
    p.x = this->x;
    p.y = this->y;
    p.z = this->z;
    return p;
}