#pragma once
#include "lease_gate.hpp"
#include <std_srvs/Trigger.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <atomic>
#include <random>
#include <ros/master.h>

std::mutex gate_mutex;
LeaseGate gate;
std::atomic<double> last_base_state{-1};
std::atomic<bool> gate_worker_stop{false};
ros::Publisher gate_status;
std::string boot_id;
int last_stop_result = -1;
bool was_moving = false;
double steady_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::string nonce() {
  std::random_device r;std::ostringstream out;
  for(int i=0;i<4;i++) out<<std::hex<<std::setw(8)<<std::setfill('0')<<r();
  return out.str();
}
std::string encode_tree(const boost::property_tree::ptree& data) {
  std::ostringstream out;boost::property_tree::write_json(out,data,false);return out.str();
}
bool guarded_claim(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
  std::lock_guard<std::mutex> lock(gate_mutex);
  const auto now=steady_seconds();
  if (now-last_base_state.load()>.25) {response.success=false;response.message="base_feedback_stale";return true;}
  response.success=gate.claim(nonce(),now);
  boost::property_tree::ptree result;
  result.put("boot_id",boot_id);result.put("token",response.success?gate.token:"");
  result.put("reason",response.success?"claimed_stationary":"disabled_or_recovery_required");
  response.message=encode_tree(result);return true;
}
bool guarded_abort(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
  std::lock_guard<std::mutex> lock(gate_mutex);
  gate.revoke("operator_abort");response.success=true;response.message="requested; check status and measured velocity; not physical emergency stop";return true;
}
void guarded_command(const std_msgs::String::ConstPtr& message) {
  std::lock_guard<std::mutex> lock(gate_mutex);
  try {
    if(message->data.size()>4096) throw std::runtime_error("oversized_request");
    boost::property_tree::ptree data;std::istringstream input(message->data);boost::property_tree::read_json(input,data);
    const auto now=steady_seconds();
    if(data.get<std::string>("boot_id")!=boot_id){gate.revoke("boot_mismatch");return;}
    std::array<double,3> v{{data.get<double>("vx"),data.get<double>("vy"),data.get<double>("wz")}};
    gate.accept(data.get<std::string>("token"),data.get<std::string>("task_id"),data.get<uint64_t>("sequence"),
                data.get<std::string>("stage"),v,now,ros::Time::now().toSec()-data.get<double>("stamp"),now-last_base_state.load()<=.25);
  } catch(const std::exception& e) {gate.revoke(std::string("invalid_command: ")+e.what());}
}
void guarded_loop() {
  while(!gate_worker_stop.load() && ros::ok()) {
    boost::property_tree::ptree status;
    {
      std::lock_guard<std::mutex> lock(gate_mutex);
      const auto now=steady_seconds();
      if(gate.phase!=LeaseGate::Phase::IDLE && now-last_base_state.load()>.25) gate.revoke("base_feedback_stale");
      const bool navigation=gate.tick(now);
      if(navigation) {
        auto v=gate.velocity;
        int result=callSport([&](){return sport_client->Move(v[0],v[1],v[2]);});
        was_moving=true;
        if(result!=0)gate.revoke("SDK_Move_unconfirmed");
        last_stop_result=-1;
      }
      // Only stop a session that this driver has actually commanded to navigate.
      // Keep retrying an unconfirmed software stop, never send a new move.
      if(!gate.tick(steady_seconds()) && was_moving) {
        last_stop_result=callSport([](){return sport_client->StopMove();});
        if(last_stop_result==0)was_moving=false;
      }
      status.put("boot_id",boot_id);status.put("task_id",gate.task);status.put("sequence",gate.sequence);
      status.put("stage",gate.name());status.put("reason",gate.reason);
      status.put("legacy_inputs_disabled",true);status.put("enabled",gate.enabled);
      status.put("lease_valid",gate.phase!=LeaseGate::Phase::IDLE && gate.phase!=LeaseGate::Phase::RECOVERY);
      status.put("sdk_stop_confirmed",last_stop_result==0);status.put("base_feedback_fresh",steady_seconds()-last_base_state.load()<=.25);
      status.put("stamp",ros::Time::now().toSec());
    }
    std_msgs::String message;message.data=encode_tree(status);gate_status.publish(message);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::lock_guard<std::mutex> lock(gate_mutex);
  if(was_moving)callSport([](){return sport_client->StopMove();});
}
