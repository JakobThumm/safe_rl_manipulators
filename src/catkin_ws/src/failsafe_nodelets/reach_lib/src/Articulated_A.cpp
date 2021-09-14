#include "reach_lib/Articulated_A.hpp"


Articulated_A::Articulated_A(double measurement_error_pos, double measurement_error_vel, double delay, 
                std::map<std::string, human_reach::jointPair>& joint_pair_map, std::map<std::string, double>& thickness, 
                std::vector<double>& max_v, std::vector<double>& max_a):
    Articulated(measurement_error_pos, measurement_error_vel, delay, joint_pair_map),
    max_v(max_v),
    max_a(max_a)
{
    for(const auto& it : joint_pair_map){
        this->bodies.push_back(BodyPart_A(it.first, thickness.at(it.first), max_v.at(it.second.first), max_v.at(it.second.second), 
                                        max_a.at(it.second.first), max_a.at(it.second.second)));
    }
}

std::vector<Capsule> Articulated_A::update(std::vector<Point> p, double t_break, std::vector<Point> v) {
    std::vector<Capsule> reachable_sets;
    for(auto& body : bodies){
        int p_joint_id = joint_pair_map.at(body.Name()).first;
        int d_joint_id = joint_pair_map.at(body.Name()).second;
        reachable_sets.push_back(body.CalculateReach(p[p_joint_id], p[d_joint_id], v[p_joint_id], v[d_joint_id], t_break,
                               measurement_error_pos, measurement_error_vel, delay));
    }
    return reachable_sets;
}