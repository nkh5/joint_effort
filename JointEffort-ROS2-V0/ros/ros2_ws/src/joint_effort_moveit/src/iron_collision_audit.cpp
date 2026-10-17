// Offline MoveIt/FCL collision validation of the generated iron CAD model.
#include <rclcpp/rclcpp.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <moveit/planning_scene/planning_scene.hpp>
#include <fstream>
#include <sstream>
#include <random>
#include <iostream>
#include <iomanip>
std::string read_file(const char *path) {std::ifstream f(path);std::ostringstream s;s<<f.rdbuf();return s.str();}
int main(int argc,char **argv) {
 if(argc!=4)return 2;
 rclcpp::init(argc,argv);
 auto u=urdf::parseURDF(read_file(argv[1]));auto s=std::make_shared<srdf::Model>();if(!u||!s->initString(*u,read_file(argv[2])))return 2;
 auto model=std::make_shared<moveit::core::RobotModel>(u,s);planning_scene::PlanningScene scene(model);auto &state=scene.getCurrentStateNonConst();auto *group=model->getJointModelGroup("iron_arm");
 std::mt19937 rng(3215);std::uniform_real_distribution<double> angle(-M_PI/6,M_PI/6);std::ofstream out(argv[3]);out<<std::setprecision(12)<<"{\"samples\":[";
 collision_detection::CollisionRequest request;request.contacts=true;request.max_contacts=50;request.max_contacts_per_pair=1;
 for(int i=0;i<201;++i) {
  std::vector<double> q(5,0.);if(i>0)for(auto &v:q)v=angle(rng);
  state.setJointGroupPositions(group,q);state.update();collision_detection::CollisionResult result;scene.checkSelfCollision(request,result,state);
  if(i)out<<",";out<<"{\"q_rad\":[";for(int k=0;k<5;++k){if(k)out<<",";out<<q[k];}out<<"],\"collision\":"<<(result.collision?"true":"false")<<",\"contacts\":[";
  bool first=true;for(auto &pair:result.contacts){if(!first)out<<",";first=false;out<<"[\""<<pair.first.first<<"\",\""<<pair.first.second<<"\"]";}out<<"]}";
 }
 state.setToDefaultValues(group,"park");state.update();collision_detection::DistanceRequest dr;dr.acm=&scene.getAllowedCollisionMatrix();dr.type=collision_detection::DistanceRequestTypes::SINGLE;dr.distance_threshold=.05;collision_detection::DistanceResult d;scene.getCollisionEnv()->distanceSelf(dr,d,state);out<<"],\"startup_near_pairs\":[";
 bool first=true;for(auto &pair:d.distances)for(auto &item:pair.second){if(!first)out<<",";first=false;out<<"{\"links\":[\""<<pair.first.first<<"\",\""<<pair.first.second<<"\"],\"distance_m\":"<<item.distance<<"}";}
 out<<"],\"scope\":\"201 deterministic endpoints using MoveIt FCL, convex CAD hulls and provisional fixtures; not complete workspace or physical validation.\"}\n";out.close();rclcpp::shutdown();return 0;
}
