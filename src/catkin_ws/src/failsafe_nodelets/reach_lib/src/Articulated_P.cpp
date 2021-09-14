#include "reach_lib/Articulated_P.hpp"


Articulated_P::Articulated_P(double measurement_error_pos, double measurement_error_vel, double delay, 
                std::map<std::string, human_reach::jointPair>& joint_pair_map, std::map<std::string, double>& thickness, 
                std::vector<double>& max_v, std::vector<double>& shoulder_ids, std::vector<double>& elbow_ids, std::vector<std::string>& wrist_names):
    Articulated(measurement_error_pos, measurement_error_vel, delay, joint_pair_map),
    shoulder_ids(shoulder_ids),
    elbow_ids(elbow_ids)
{
    assert(shoulder_ids.size() == wrist_names.size());
    assert(elbow_ids.size() == wrist_names.size());
    for(int i = 0; i < wrist_names.size(); i++){
        // Name, radius hand, max_v shoulder
        this->extremities.push_back(Extremity(wrist_names.at(i), thickness.at(wrist_names.at(i)), max_v.at(shoulder_ids.at(i))));
    }
}


std::vector<Capsule> Articulated_P::update(std::vector<Point> p, double t_break, std::vector<Point> v) {
    std::vector<Capsule> reachable_sets;
    for(int i = 0; i < extremities.size(); i++){
        // The joint id of the wrist is taken from the wrist (or hand) body. We take the first joint of the pair.
        int wrist_id = joint_pair_map.at(extremities[i].Name()).first;
        int elbow_id = elbow_ids[i];
        int shoulder_id = shoulder_ids[i];
        reachable_sets.push_back(extremities[i].CalculateReach(p[shoulder_id], p[elbow_id], p[wrist_id], t_break,
                                                               measurement_error_pos, delay));
    }
    return reachable_sets;
}