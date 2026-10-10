#include <ros/ros.h>
#include <xgc2/xrpc/http.hpp>
#include <xgc2/xrpc/runtime_policy.hpp>
#include <xgc2/xrpc/unix.hpp>
#include "xgc_ros1_tools_adapter/tools.hpp"
#include "xgc_ros1_tools_adapter/error.hpp"
#include <atomic>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

extern char** environ;
namespace {
using namespace xgc2::xrpc;
using namespace xgc_ros1_tools_adapter;
std::atomic<bool> stopping{false};
void signalStop(int) { stopping.store(true); }
std::string json(const Json::Value& value) {
  Json::StreamWriterBuilder writer; writer["indentation"] = "";
  return Json::writeString(writer, value);
}
struct Work { HttpRequest request; HttpReply reply; };
class Workers {
 public:
  explicit Workers(Tools& tools) : tools_(tools) {
    try {
      for (int i=0;i<16;++i) threads_.emplace_back([this]{ run(); });
    } catch (...) {
      stop();
      throw;
    }
  }
  ~Workers() { stop(); }
  void push(HttpRequest request, HttpReply reply) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_ || queue_.size() >= 16) {
      reply.complete(http_error(503, "resource_exhausted", "ROS work capacity exhausted"));
      return;
    }
    queue_.push_back({std::move(request), std::move(reply)}); changed_.notify_one();
  }
  void stop() {
    { std::lock_guard<std::mutex> lock(mutex_); closed_=true; }
    stopping.store(true); changed_.notify_all();
    for (auto& thread:threads_) if(thread.joinable()) thread.join();
  }
 private:
  void run() {
    for (;;) {
      Work work;
      { std::unique_lock<std::mutex> lock(mutex_);
        changed_.wait(lock,[this]{return closed_ || !queue_.empty();});
        if (queue_.empty()) return;
        work=std::move(queue_.front());queue_.pop_front();
      }
      if (work.reply.cancelled() || stopping.load()) continue;
      try {
        const auto remaining=work.request.deadline-Clock::now();
        const auto deadline=std::chrono::system_clock::now().time_since_epoch()+remaining;
        CallContext call{std::chrono::duration_cast<std::chrono::nanoseconds>(deadline).count(),
          [&work]{return stopping.load() || work.reply.cancelled();}};
        auto result=work.request.target=="/api/v1/publish"
          ? tools_.Publish(work.request.body,call):tools_.CallService(work.request.body,call);
        work.reply.complete({200,{{"Content-Type","application/json"}},json(result)});
      } catch(const Ros1ToolsError& error) {
        Json::Value body(Json::objectValue);
        body["error"]["class"]=error.errorClass();body["error"]["code"]=error.code();body["error"]["message"]=error.what();
        work.reply.complete({200,{{"Content-Type","application/json"}},json(body)});
      } catch(const std::exception&) {
        // A transport/client must never replay a request whose native outcome is unknown.
        work.reply.complete(http_error(500,"outcome_unknown","ROS operation did not produce a known terminal result"));
      }
    }
  }
  Tools& tools_;std::mutex mutex_;std::condition_variable changed_;
  bool closed_=false;std::deque<Work> queue_;std::vector<std::thread> threads_;
};
}
int main(int argc,char** argv) {
  try {
    if(argc==2 && std::string(argv[1])=="--help") {
      std::cout<<"Usage: xgc_ros1_tools_adapter_node --socket-path PATH --target-id ID --ros-master-uri URI --ros-ip IP --ros-hostname HOST\n";return 0;
    }
    std::map<std::string,std::string> args;
    for(int i=1;i<argc;i+=2) {
      if(i+1>=argc || !args.emplace(argv[i],argv[i+1]).second) throw std::invalid_argument("invalid or duplicate argument");
    }
    for(const auto& [name,value]:args) {
      (void)value;
      if(name!="--socket-path" && name!="--target-id" && name!="--ros-master-uri" && name!="--ros-ip" && name!="--ros-hostname") throw std::invalid_argument("unknown argument");
    }
    if(args.size()!=5 || args.at("--target-id").empty()) throw std::invalid_argument("five explicit arguments required");
    Json::Value config(Json::objectValue);
    config["rosMasterUri"]=args.at("--ros-master-uri");config["rosIp"]=args.at("--ros-ip");config["rosHostname"]=args.at("--ros-hostname");
    auto context=NativeContext::FromJson(json(config));context.ApplyEnvironment();
    const auto target=args.at("--target-id");
    if(target.size()>128 || target.find_first_of("\r\n\t ")==0 || target.find_first_of("\r\n\t ")!=std::string::npos) throw std::invalid_argument("invalid target identity");
    RuntimePolicyOptions options;
    options.capabilities={"host","http","rpc","transport"};
    options.defaults={{"CALL_TIMEOUT_MS","300000"},{"HOST_MAX_IN_FLIGHT","16"}};
    options.default_source="ros1.tools";
    options.ceilings={{"HOST_MAX_IN_FLIGHT",16},{"HOST_MAX_CONNECTIONS",32},{"MAX_REQUEST_BYTES",1<<20},{"MAX_RESPONSE_BYTES",1<<20},{"CALL_TIMEOUT_MS",300000}};
    for(char** p=environ;*p;++p) {
      const std::string entry(*p);const auto split=entry.find('=');
      if(entry.rfind("XGC2_XRPC_",0)==0) options.environment.emplace_back(entry.substr(0,split),entry.substr(split+1));
    }
    const auto policy=resolve_runtime_policy(options);auto limits=http_limits(policy);
    ros::init(argc,argv,"xgc_ros1_tools",ros::init_options::AnonymousName|ros::init_options::NoSigintHandler|ros::init_options::NoRosout);
    ros::NodeHandle node;ros::AsyncSpinner spinner(4);spinner.start();Tools tools(node,context);
    const auto instance=new_instance_id();
    Json::Value described(Json::objectValue);
    auto& ref=described["service_ref"];
    ref["target_id"]=target;ref["service"]="ros1.tools";ref["api_version"]="1";ref["instance_id"]=instance;ref["profile"]="http.v1";
    ref["endpoint"]["kind"]="unix";ref["endpoint"]["address"]=args.at("--socket-path");
    described["native_context"]=config;
    Workers workers(tools);
    HttpServer server(UnixOptions{args.at("--socket-path")},[&](HttpRequest request,HttpReply reply){
      if(request.method=="GET" && request.target=="/api/v1/describe") {
        reply.complete({200,{{"Content-Type","application/json"}},json(described)});return;
      }
      if(request.method!="POST" || (request.target!="/api/v1/publish" && request.target!="/api/v1/call")) {
        reply.complete(http_error(404,"not_found","unknown ROS1 Tools operation"));return;
      }
      workers.push(std::move(request),std::move(reply));
    },limits,HttpIdentity{instance,{"/api/v1/describe"}});
    std::signal(SIGINT,signalStop);std::signal(SIGTERM,signalStop);
    std::jthread master([&](std::stop_token token){
      int unavailable=0;
      while(!token.stop_requested() && !stopping.load() && ros::ok()) {
        const auto state=tools.ProbeMasterBinding();
        if(state==MasterBindingState::Changed || (state==MasterBindingState::Unavailable && ++unavailable>=10)) {stopping.store(true);break;}
        if(state!=MasterBindingState::Unavailable) unavailable=0;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    });
    while(!stopping.load() && ros::ok()) server.poll(std::chrono::milliseconds(50));
    server.request_stop();stopping.store(true);master.join();workers.stop();spinner.stop();ros::shutdown();return 0;
  } catch(const std::exception& error) {
    std::cerr<<"ros1.tools: "<<error.what()<<'\n';if(ros::isInitialized()) ros::shutdown();return 1;
  }
}
