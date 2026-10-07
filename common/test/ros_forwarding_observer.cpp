// Private forwarding benchmark: one ROS node with two callback workers observes
// every canonical output. It adds no production process, API or dependency.
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <fstream>
#include <mutex>
#include <map>
#include <vector>
static volatile std::sig_atomic_t stopped = 0;
static void stop(int) { stopped=1; }
int main(int argc,char **argv) {
 if(argc!=5 && argc!=6 && argc!=7)return 2;
 std::string kind=argv[1],output=argv[3],ready=argv[4];const int count=std::stoi(argv[2]);
 const bool all=argc>=6 && std::string(argv[5])=="all";
 const bool direct=argc>=6 && std::string(argv[5])=="direct";
 const int first_robot=argc==7?std::stoi(argv[6]):0;
 ros::init(argc,argv,"private_forwarding_observer",ros::init_options::AnonymousName|ros::init_options::NoSigintHandler);
 std::signal(SIGTERM,stop);std::signal(SIGINT,stop);
 ros::NodeHandle node;std::mutex mutex;
 std::map<std::string,std::vector<unsigned>> received;
 received["pose"].resize(count);
 if(direct)received["twist"].resize(count);
 if(all){received["twist"].resize(count);received["accel"].resize(count);if(kind=="px4")received["vision"].resize(count);}
 std::vector<double> delays;delays.reserve(count*100*received.size());
 std::uint64_t first=0;unsigned invalid=0;
 auto record=[&](const std::string &channel,int robot,const std_msgs::Header &header,bool valid){
   const auto now=ros::WallTime::now().toNSec();std::lock_guard<std::mutex> lock(mutex);
   ++received.at(channel)[robot];if(!first)first=now;
   delays.push_back((double(now)-header.stamp.toNSec())/1e6);
   if(!valid || header.frame_id!="source-world")++invalid;
 };
 std::vector<ros::Subscriber> subscribers;subscribers.reserve(count*received.size());
 for(int i=0;i<count;++i){
  const auto root="/"+kind+std::to_string(first_robot+i);
  for(const auto &name:{"pose","vision"}){
   if(!received.count(name))continue;
   const std::string channel=name;
   subscribers.push_back(node.subscribe<geometry_msgs::PoseStamped>(root+(channel=="pose"?"/pose":"/mavros/vision_pose/pose"),100,
    [&,i,channel](const geometry_msgs::PoseStamped::ConstPtr &message){
     record(channel,i,message->header,std::abs(message->pose.position.x-(direct?3:4.25))<1e-8 && std::abs(message->pose.position.y-(direct?4:2))<1e-8 && std::abs(message->pose.position.z-(direct?5:5.1))<1e-8);
    },ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay()));
  }
  if(all || direct){
   subscribers.push_back(node.subscribe<geometry_msgs::TwistStamped>(root+"/twist",100,
    [&,i](const geometry_msgs::TwistStamped::ConstPtr &message){record("twist",i,message->header,std::abs(message->twist.linear.x-1)<1e-8 && std::abs(message->twist.angular.z-.5)<1e-8);},ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay()));
  }
  if(all){
   subscribers.push_back(node.subscribe<geometry_msgs::AccelStamped>(root+"/accel",100,
    [&,i](const geometry_msgs::AccelStamped::ConstPtr &message){record("accel",i,message->header,std::abs(message->accel.linear.x-2)<1e-8 && std::abs(message->accel.angular.z-.25)<1e-8);},ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay()));
  }
 }
 ros::AsyncSpinner callbacks(2);callbacks.start();
 const auto until=ros::WallTime::now()+ros::WallDuration(20);
 for(;;){bool connected=true;for(auto &subscriber:subscribers)connected=connected&&subscriber.getNumPublishers()>0;
  if(connected){std::ofstream(ready)<<"ready\n";break;}
  if(stopped || ros::WallTime::now()>until)break;ros::WallDuration(.01).sleep();
 }
 while(!stopped && ros::ok())ros::WallDuration(.02).sleep();
 callbacks.stop();std::sort(delays.begin(),delays.end());
 auto percentile=[&](double p){return delays.empty()?0:delays[std::min(delays.size()-1,std::size_t(p*delays.size()))];};
 std::ofstream report(output);report<<"{\"all_robot_samples\":"<<delays.size()<<",\"first_sample_unix_ns\":"<<first<<",\"invalid_samples\":"<<invalid<<",\"latency_ms\":{\"p50\":"<<percentile(.5)<<",\"p95\":"<<percentile(.95)<<",\"p99\":"<<percentile(.99)<<"},\"per_robot_samples\":[";
 for(int i=0;i<count;++i){if(i)report<<',';report<<received.at("pose")[i];}report<<"],\"channel_samples\":{";
 bool separator=false;for(const auto &channel:received){if(separator)report<<',';separator=true;report<<'"'<<channel.first<<"\":[";for(int i=0;i<count;++i){if(i)report<<',';report<<channel.second[i];}report<<']';}report<<"}}\n";
 ros::shutdown();return invalid?1:0;
}
