#include "arena_multi_hunav_core/core.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
using namespace arena_multi_hunav_core;
using Agent=hunav_msgs::msg::Agent;
int main(){
 std::cout<<std::hexfloat;
 for(int seed=0;seed<10;++seed)for(int count:{1,6}){
  Compute::Request q;q.dt=.025;q.epoch=11;q.current_agents.header.frame_id="map";q.robots.header.frame_id="map";
  for(int i=0;i<count;++i){
   Agent a;a.id=i+1;a.name="person_"+std::to_string(i+1);a.type=Agent::PERSON;a.group_id=-1;a.behavior.type=1;
   a.radius=.4;a.desired_velocity=1;a.goal_radius=.3;a.cyclic_goals=true;
   a.behavior.goal_force_factor=2;a.behavior.obstacle_force_factor=10;a.behavior.social_force_factor=5;
   a.position.position.x=8+1.4*i;a.position.position.y=i%2?9.:4.5;
   a.yaw=i%2?-1.5707963267948966:1.5707963267948966;a.position.orientation.z=std::sin(a.yaw/2);a.position.orientation.w=std::cos(a.yaw/2);
   for(double y:{i%2?4.5:9.,i%2?9.:4.5}){geometry_msgs::msg::Pose g;g.position.x=a.position.position.x;g.position.y=y;g.orientation.w=1;a.goals.push_back(g);}
   q.current_agents.agents.push_back(a);arena_multi_hunav_msgs::msg::BehaviorModifiers m;m.agent_id=a.id;q.modifiers.push_back(m);
  }
  for(int i=0;i<2;++i){Agent r;r.id=-i-1;r.name="robot_"+std::to_string(i+1);r.type=Agent::ROBOT;r.group_id=-1;r.radius=.35;r.position.orientation.w=1;q.robots.agents.push_back(r);}
  for(int k=0;k<1200;++k){
   q.step=k+1;q.robots.header.stamp=q.current_agents.header.stamp;
   for(int i=0;i<2;++i){auto &r=q.robots.agents[i];r.position.position.x=3.+.2*std::sin(k*.025*.3+seed);r.position.position.y=3.+4*i;r.velocity.linear.x=.06*std::cos(k*.025*.3+seed);}
   auto out=integrate(q,{});if(!out.success){std::cerr<<out.error;return 2;}
   std::cout<<seed<<' '<<count<<' '<<k;
   for(auto &a:out.updated_agents.agents){std::cout<<' '<<a.id<<' '<<a.position.position.x<<' '<<a.position.position.y<<' '<<a.yaw<<' '<<a.velocity.linear.x<<' '<<a.velocity.linear.y<<' '<<a.velocity.angular.z<<' '<<a.goals.size();for(auto &g:a.goals)std::cout<<' '<<g.position.x<<' '<<g.position.y;}
   for(auto &f:out.influences)std::cout<<' '<<f.agent_id<<' '<<f.robot_name<<' '<<f.force.x<<' '<<f.force.y<<' '<<f.clearance;
   std::cout<<'\n';q.current_agents=out.updated_agents;
  }
 }
}
