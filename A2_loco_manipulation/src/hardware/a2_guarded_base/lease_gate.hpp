#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

// Pure state machine: no ROS, no SDK. Caller serializes access and uses steady time.
struct LeaseGate {
  enum class Phase { IDLE, STATIONARY, NAVIGATION, GRASP, RECOVERY };
  Phase phase = Phase::IDLE;
  bool enabled = false, ever_navigation = false;
  std::string token, task, reason;
  uint64_t sequence = 0;
  double deadline = -1;
  std::array<double,3> velocity{{0,0,0}};
  static constexpr double ttl = .4;
  bool claim(const std::string& nonce, double now) {
    if (!enabled || phase != Phase::IDLE || nonce.empty() || !std::isfinite(now)) return false;
    token=nonce; phase=Phase::STATIONARY; deadline=now+ttl; return true;
  }
  void revoke(const std::string& why) {
    if (phase == Phase::IDLE) return;
    phase=Phase::RECOVERY; velocity={{0,0,0}}; reason=why;
  }
  bool tick(double now) {
    if (phase!=Phase::IDLE && phase!=Phase::RECOVERY && (!std::isfinite(now) || now>deadline)) revoke("lease_expired");
    return phase==Phase::NAVIGATION;
  }
  bool accept(const std::string& nonce, const std::string& id, uint64_t seq,
              const std::string& stage, const std::array<double,3>& v,
              double now, double message_age, bool feedback_fresh) {
    tick(now);
    if (phase==Phase::IDLE || phase==Phase::RECOVERY) return false;
    auto fail=[&](const std::string& s){revoke(s);return false;};
    if (nonce!=token || id.empty() || (!task.empty() && task!=id)) return fail("owner_mismatch");
    if (seq<=sequence) return fail("sequence_replay");
    if (!std::isfinite(message_age) || message_age<0 || message_age>.2 || !feedback_fresh) return fail("stale_input_or_feedback");
    const std::array<double,3> limit{{.10,.05,.15}};
    for (int i=0;i<3;i++) if (!std::isfinite(v[i]) || std::abs(v[i])>limit[i]) return fail("invalid_or_excess_velocity");
    Phase next;
    if (stage=="stationary") next=Phase::STATIONARY;
    else if (stage=="navigation") next=Phase::NAVIGATION;
    else if (stage=="grasp") next=Phase::GRASP;
    else return fail("invalid_stage");
    // Navigation may be entered only once, and never after entering grasp.
    if (next==Phase::NAVIGATION && (phase==Phase::GRASP || (phase!=Phase::NAVIGATION && ever_navigation))) return fail("navigation_reentry");
    if (next!=Phase::NAVIGATION && v!=std::array<double,3>{{0,0,0}}) return fail("motion_outside_navigation");
    if (phase==Phase::GRASP && next!=Phase::GRASP) return fail("grasp_latched");
    task=id; sequence=seq; phase=next; velocity=v; deadline=now+ttl;
    if (phase==Phase::NAVIGATION) ever_navigation=true;
    return true;
  }
  const char* name() const {
    switch(phase){case Phase::IDLE:return "idle";case Phase::STATIONARY:return "stationary";case Phase::NAVIGATION:return "navigation";case Phase::GRASP:return "grasp";default:return "recovery";}
  }
};
