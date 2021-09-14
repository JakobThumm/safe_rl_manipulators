#include "reach_lib/Articulated_V.hpp"


Articulated_V::Articulated_V(double measurement_error_pos, double measurement_error_vel, double delay, 
                std::map<std::string, human_reach::jointPair>& joint_pair_map, std::map<std::string, double>& thickness, 
                std::vector<double>& max_v):
    Articulated(measurement_error_pos, measurement_error_vel, delay, joint_pair_map),
    max_v(max_v)
{
    for(const auto& it : joint_pair_map){
        this->bodies.push_back(BodyPart_V(it.first, thickness.at(it.first), max_v.at(it.second.first), max_v.at(it.second.second)));
    }
}


std::vector<Capsule> Articulated_V::update(std::vector<Point> p, double t_break, std::vector<Point> v) {
    std::vector<Capsule> reachable_sets;
    for(auto& body : bodies){
        int p_joint_id = joint_pair_map.at(body.Name()).first;
        int d_joint_id = joint_pair_map.at(body.Name()).second;
        reachable_sets.push_back(body.CalculateReach(p[p_joint_id], p[d_joint_id], v[p_joint_id], v[d_joint_id], t_break,
                               measurement_error_pos, measurement_error_vel, delay));
    }
    return reachable_sets;
}

//// OLD EXAMPLE --- Move this to test case.
/*
Articulated::Articulated(std::string mode){
    this->mode = mode;
    if(mode == "ACCEL" || mode == "VEL"){
        this->sensor = System();

        // Right arm init
        this->index_list.push_back(std::make_tuple("RightUpperArm",0,1));
        this->index_list.push_back(std::make_tuple("RightForearm",1,2));
        this->index_list.push_back(std::make_tuple("RightUpperArm",2,2));

        // Left arm init
        this->index_list.push_back(std::make_tuple("LeftUpperArm",3,4));
        this->index_list.push_back(std::make_tuple("LeftForearm",4,5));
        this->index_list.push_back(std::make_tuple("LeftUpperArm",5,5));

        // Torso init
        this->index_list.push_back(std::make_tuple("Torso",6,7));

        // Right leg init
        this->index_list.push_back(std::make_tuple("RightThigh",8,9));
        this->index_list.push_back(std::make_tuple("RightShin",9,10));
        this->index_list.push_back(std::make_tuple("RightFoot",10,10));

        // Left leg init
        this->index_list.push_back(std::make_tuple("LefttThigh",11,12));
        this->index_list.push_back(std::make_tuple("LefttShin",12,13));
        this->index_list.push_back(std::make_tuple("LefttFoot",13,13));

        // Head init
        this->index_list.push_back(std::make_tuple("Head",14,14));

        // Default max velocity for all body-parts: 14.0 m/s
        std::vector<double> max_v;
        for(int i = 0; i < 15; i++){
            max_v.push_back(14.0);
        }

        // Default max acceleration for all body-part: order as defined above
        std::vector<double> max_a = {50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 20.0, 20.0, 30.0, 30.0, 50.0, 30.0, 30.0, 50.0, 25.0};

        // Default thickness for all body-parts: order as defined above
        // Torso estimated at 40 cm diameter
        std::vector<double> thickness = {0.1, 0.1, 0.205, 0.01, 0.01, 0.205, 0.2, 0.2, 0.4, 0.2, 0.2, 0.4, 0.4, 0.3};

        int entry = 0;
        for(auto const& it: this->index_list){
            this->body.push_back(BodyPart(std::get<0>(it), Point(), Point(), max_a[std::get<1>(it)], max_v[std::get<1>(it)], max_a[std::get<2>(it)], max_v[std::get<2>(it)], thickness[entry]));
            entry++;
        }

        this->t_start = 0.0;
        this->t_end = 0.02;
    }else{
        this->sensor = System();

        // Assignement of points to the four balls
        this->index_list.push_back(std::make_tuple("RightArm",0,0));
        this->index_list.push_back(std::make_tuple("LeftArm",1,1));
        this->index_list.push_back(std::make_tuple("RightLeg",2,2));
        this->index_list.push_back(std::make_tuple("LeftLeg",3,3));

        // Default max_v for all balls
        std::vector<double> max_v;
        for(int i = 0; i < 4; i++){
            max_v.push_back(14.0);
        }

        // Default reach of hands and feet
        std::vector<double> thickness;
        thickness.push_back(0.205);
        thickness.push_back(0.205);
        thickness.push_back(0.4);
        thickness.push_back(0.4);

        int entry = 0;
        for(auto const& it: this->index_list){
            this->extremities.push_back(Extremity(std::get<0>(it), Point(), max_v[std::get<1>(it)], thickness[entry]));
            entry++;
        }

        this->t_start = 0.0;
        this->t_end = 0.02;
    }
}
*/