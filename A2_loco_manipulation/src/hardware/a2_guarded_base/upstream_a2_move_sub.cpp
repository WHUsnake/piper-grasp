#include <unitree/robot/a2/sport/sport_client.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/MainBoardState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#ifdef HAVE_PINOCCHIO
#include <Eigen/Dense>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#endif

using namespace unitree::robot;

std::shared_ptr<a2::SportClient> sport_client;
std::mutex sport_client_mutex;

ros::Publisher imu_pub;
ros::Publisher status_pub;
ros::Publisher odom_pub;
ros::Publisher odom_lf_pub;
ros::Publisher raw_lowstate_pub;
ros::Publisher raw_sportstate_pub;
ros::Publisher raw_sportstate_lf_pub;
ros::Publisher raw_mainboardstate_pub;
ros::Publisher raw_bmsstate_pub;
ros::Publisher raw_secondary_bmsstate_pub;

std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>> lowstate_subscriber;
std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> sportstate_subscriber;
std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> sportstate_lf_subscriber;
std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::MainBoardState_>> mainboardstate_subscriber;
std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::BmsState_>> bmsstate_subscriber;
std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::BmsState_>> secondary_bmsstate_subscriber;

std::string imu_parent_frame = "base_link";
std::string imu_child_frame = "base_imu";
double imu_xyz[3] = {0.0, 0.0, 0.0};
// base_imu uses RFU (right-forward-up); base_link stays FLU (forward-left-up).
// /a2/imu and /a2/odom publish body-frame data in base_imu (RFU).
constexpr double kFluToRfuYaw = -M_PI / 2.0;
double imu_rpy[3] = {0.0, 0.0, kFluToRfuYaw};
std::string odom_frame_id = "odom";
std::string odom_child_frame_id = "base_imu";
double foot_force_hip_offset = 0.12779;
double foot_force_thigh_length = 0.275;
double foot_force_calf_length = 0.275;
double foot_force_foot_radius = 0.032;
bool foot_force_use_pinocchio_gravity = true;
std::string foot_force_urdf_path = "/home/unitree/a2_nav/unitree_sdk2/example/a2_description/urdf/a2.urdf";
double foot_force_scale = 1.0;
double foot_force_offset = 0.0;
double foot_force_max = 2000.0;
double foot_force_damping = 0.05;
double foot_force_vertical_sign = 1.0;
bool foot_force_use_abs_z = false;
double foot_force_joint_q_sign[3] = {1.0, 1.0, 1.0};
double foot_force_joint_tau_sign[3] = {1.0, 1.0, 1.0};
double foot_force_joint_q_offset[3] = {0.0, 0.0, 0.0};
double foot_force_joint_tau_offset[3] = {0.0, 0.0, 0.0};
double foot_force_joint_viscous[3] = {0.2, 0.2, 1.0};
bool publish_raw_dds_topics = true;

const char *motor_names[12] = {
    "FR_0", "FR_1", "FR_2",
    "FL_0", "FL_1", "FL_2",
    "RR_0", "RR_1", "RR_2",
    "RL_0", "RL_1", "RL_2"};

const char *motor_joint_names[12] = {
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

struct LowStateData
{
  bool received = false;
  uint8_t mode_pr = 0;
  uint8_t mode_machine = 0;
  uint32_t tick = 0;

  float imu_quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float imu_gyroscope[3] = {0.0f, 0.0f, 0.0f};
  float imu_accelerometer[3] = {0.0f, 0.0f, 0.0f};
  float imu_rpy[3] = {0.0f, 0.0f, 0.0f};
  int16_t imu_temperature = 0;

  struct MotorData
  {
    uint8_t mode = 0;
    float q = 0.0f;
    float dq = 0.0f;
    float ddq = 0.0f;
    float tau_est = 0.0f;
    int16_t temperature[2] = {0, 0};
    float voltage = 0.0f;
    uint32_t motorstate = 0;
  } motors[12];
};

struct MotionStateData
{
  bool received = false;
  uint8_t mode = 0;
  uint8_t gait_type = 0;
  float position[3] = {0.0f, 0.0f, 0.0f};
  float velocity[3] = {0.0f, 0.0f, 0.0f};
  float yaw_speed = 0.0f;
  int16_t foot_force[4] = {0, 0, 0, 0};
};

struct MainBoardStateData
{
  bool received = false;
  uint16_t fan_state[6] = {0};
  int16_t temperature[6] = {0};
  float value[6] = {0.0f};
  uint32_t state[6] = {0};
};

struct BmsStateData
{
  bool received = false;
  uint8_t version_high = 0;
  uint8_t version_low = 0;
  uint8_t fn = 0;
  uint16_t cell_vol[40] = {0};
  uint32_t bmsvoltage[3] = {0};
  int32_t current = 0;
  uint8_t soc = 0;
  uint8_t soh = 0;
  int16_t temperature[12] = {0};
  uint16_t cycle = 0;
  uint16_t manufacturer_date = 0;
  uint32_t bmsstate[5] = {0};
  uint32_t reserve[3] = {0};
};

struct A2ServiceState
{
  bool received = false;
  std::map<std::string, std::string> values;
} service_state_data;

LowStateData low_state_data;
MotionStateData motion_state_data;
MainBoardStateData mainboard_state_data;
BmsStateData bms_state_data;
BmsStateData secondary_bms_state_data;
std::mutex state_mutex;

std::string trimLower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c)
                 { return std::tolower(c); });
  const std::string whitespace = " \t\n\r";
  const auto begin = value.find_first_not_of(whitespace);
  if (begin == std::string::npos)
  {
    return "";
  }
  const auto end = value.find_last_not_of(whitespace);
  return value.substr(begin, end - begin + 1);
}

int32_t callSport(const std::function<int32_t()> &fn)
{
  if (!sport_client)
  {
    ROS_WARN("sport_client not ready");
    return -1;
  }
  std::lock_guard<std::mutex> lock(sport_client_mutex);
  return fn();
}

std::string getA2FsmString(uint32_t fsm_id)
{
  static const std::vector<std::string> names = {
      "PASSIVE", "STAND_DOWN", "STAND_UP", "DEFAULT_MODE", "RUNNING_MODE",
      "CLIMB_MODE", "LEFT_SIDE_GAIT", "RIGHT_SIDE_GAIT", "HANDSTAND",
      "BIPED_STAND", "FRONT_FLIP", "BACK_FLIP", "RECOVERY", "BASE_HEIGHT_CTRL",
      "POSE", "EULER"};
  if (fsm_id < names.size())
  {
    return names[fsm_id];
  }
  return "Unknown";
}

std::string getModeString(uint32_t mode)
{
  return getA2FsmString(mode);
}

std::string getGaitString(uint32_t gait)
{
  switch (gait)
  {
  case 0:
    return "Idle";
  case 1:
    return "Trot";
  case 2:
    return "Walk";
  case 3:
    return "Run";
  case 4:
    return "Climb";
  default:
    return "Unknown";
  }
}

std::string mapValue(const std::map<std::string, std::string> &values, const std::string &key, const std::string &fallback = "N/A")
{
  auto it = values.find(key);
  if (it == values.end() || it->second.empty())
  {
    return fallback;
  }
  return it->second;
}

std::string getAutoChargeString(const BmsStateData &bms, const BmsStateData &secondary_bms)
{
  if (!bms.received && !secondary_bms.received)
  {
    return "N/A";
  }

  constexpr int32_t kChargingCurrentThresholdMa = 100;
  if ((bms.received && bms.current > kChargingCurrentThresholdMa) ||
      (secondary_bms.received && secondary_bms.current > kChargingCurrentThresholdMa))
  {
    return "自动充电中";
  }

  return "未充电";
}

std::string formatFootForces(const int16_t foot_force[4])
{
  static const char *kNames[4] = {"FR", "FL", "RR", "RL"};
  std::ostringstream ss;
  for (int i = 0; i < 4; ++i)
  {
    ss << kNames[i] << ':' << foot_force[i] << "N";
    if (i != 3)
    {
      ss << " | ";
    }
  }
  return ss.str();
}

template <typename T>
void appendCArray(std::ostringstream &ss, const T *values, int count)
{
  ss << '[';
  for (int i = 0; i < count; ++i)
  {
    ss << values[i];
    if (i + 1 < count)
    {
      ss << ',';
    }
  }
  ss << ']';
}

std::string formatLowStateRaw(const LowStateData &data)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6);
  ss << "{\"mode_pr\":" << static_cast<int>(data.mode_pr)
     << ",\"mode_machine\":" << static_cast<int>(data.mode_machine)
     << ",\"tick\":" << data.tick
     << ",\"imu_state\":{\"quaternion\":";
  appendCArray(ss, data.imu_quaternion, 4);
  ss << ",\"gyroscope\":";
  appendCArray(ss, data.imu_gyroscope, 3);
  ss << ",\"accelerometer\":";
  appendCArray(ss, data.imu_accelerometer, 3);
  ss << ",\"rpy\":";
  appendCArray(ss, data.imu_rpy, 3);
  ss << ",\"temperature\":" << data.imu_temperature << "},\"motor_state\":[";
  for (int i = 0; i < 12; ++i)
  {
    const auto &motor = data.motors[i];
    ss << "{\"index\":" << i
       << ",\"name\":\"" << motor_names[i] << "\""
       << ",\"mode\":" << static_cast<int>(motor.mode)
       << ",\"q\":" << motor.q
       << ",\"dq\":" << motor.dq
       << ",\"ddq\":" << motor.ddq
       << ",\"tau_est\":" << motor.tau_est
       << ",\"temperature\":";
    appendCArray(ss, motor.temperature, 2);
    ss << ",\"vol\":" << motor.voltage
       << ",\"motorstate\":" << motor.motorstate << '}';
    if (i + 1 < 12)
    {
      ss << ',';
    }
  }
  ss << "]}";
  return ss.str();
}

std::string formatMainBoardRaw(const MainBoardStateData &data)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6);
  ss << "{\"fan_state\":";
  appendCArray(ss, data.fan_state, 6);
  ss << ",\"temperature\":";
  appendCArray(ss, data.temperature, 6);
  ss << ",\"value\":";
  appendCArray(ss, data.value, 6);
  ss << ",\"state\":";
  appendCArray(ss, data.state, 6);
  ss << '}';
  return ss.str();
}

std::string formatMotionStateRaw(const MotionStateData &data)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6);
  ss << "{\"mode\":" << static_cast<int>(data.mode)
     << ",\"gait_type\":" << static_cast<int>(data.gait_type)
     << ",\"position\":";
  appendCArray(ss, data.position, 3);
  ss << ",\"velocity\":";
  appendCArray(ss, data.velocity, 3);
  ss << ",\"yaw_speed\":" << data.yaw_speed
     << ",\"foot_force\":";
  appendCArray(ss, data.foot_force, 4);
  ss << '}';
  return ss.str();
}

std::string formatBmsRaw(const BmsStateData &data)
{
  std::ostringstream ss;
  ss << "{\"version_high\":" << static_cast<int>(data.version_high)
     << ",\"version_low\":" << static_cast<int>(data.version_low)
     << ",\"fn\":" << static_cast<int>(data.fn)
     << ",\"cell_vol\":";
  appendCArray(ss, data.cell_vol, 40);
  ss << ",\"bmsvoltage\":";
  appendCArray(ss, data.bmsvoltage, 3);
  ss << ",\"current\":" << data.current
     << ",\"soc\":" << static_cast<int>(data.soc)
     << ",\"soh\":" << static_cast<int>(data.soh)
     << ",\"temperature\":";
  appendCArray(ss, data.temperature, 12);
  ss << ",\"cycle\":" << data.cycle
     << ",\"manufacturer_date\":" << data.manufacturer_date
     << ",\"bmsstate\":";
  appendCArray(ss, data.bmsstate, 5);
  ss << ",\"reserve\":";
  appendCArray(ss, data.reserve, 3);
  ss << '}';
  return ss.str();
}

void publishRawString(const ros::Publisher &publisher, const std::string &payload)
{
  if (!publish_raw_dds_topics || !publisher)
  {
    return;
  }
  std_msgs::String msg;
  msg.data = payload;
  publisher.publish(msg);
}

struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

Vec3 operator+(const Vec3 &a, const Vec3 &b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(const Vec3 &a, const Vec3 &b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 rotateX(const Vec3 &v, double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return {v.x, c * v.y - s * v.z, s * v.y + c * v.z};
}

Vec3 rotateY(const Vec3 &v, double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return {c * v.x + s * v.z, v.y, -s * v.x + c * v.z};
}

Vec3 cross(const Vec3 &a, const Vec3 &b)
{
  return {a.y * b.z - a.z * b.y,
          a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}

bool solve3x3(double a[3][3], const double b[3], Vec3 &x)
{
  double det =
      a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
      a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
      a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

  if (std::abs(det) < 1e-6)
  {
    return false;
  }

  double ax[3][3] = {
      {b[0], a[0][1], a[0][2]},
      {b[1], a[1][1], a[1][2]},
      {b[2], a[2][1], a[2][2]}};
  double ay[3][3] = {
      {a[0][0], b[0], a[0][2]},
      {a[1][0], b[1], a[1][2]},
      {a[2][0], b[2], a[2][2]}};
  double az[3][3] = {
      {a[0][0], a[0][1], b[0]},
      {a[1][0], a[1][1], b[1]},
      {a[2][0], a[2][1], b[2]}};

  auto determinant = [](double m[3][3])
  {
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
           m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  };

  x.x = determinant(ax) / det;
  x.y = determinant(ay) / det;
  x.z = determinant(az) / det;
  return std::isfinite(x.x) && std::isfinite(x.y) && std::isfinite(x.z);
}

#ifdef HAVE_PINOCCHIO
struct PinocchioGravityContext
{
  bool ready = false;
  pinocchio::Model model;
  std::unique_ptr<pinocchio::Data> data;
  Eigen::VectorXd q;
  std::array<int, 12> q_index{};
  std::array<int, 12> v_index{};
};

PinocchioGravityContext pinocchio_gravity;

bool initPinocchioGravity(const std::string &urdf_path)
{
  pinocchio_gravity.ready = false;
  pinocchio_gravity.q_index.fill(-1);
  pinocchio_gravity.v_index.fill(-1);

  try
  {
    pinocchio::urdf::buildModel(urdf_path, pinocchio_gravity.model);
    pinocchio_gravity.data = std::make_unique<pinocchio::Data>(pinocchio_gravity.model);
    pinocchio_gravity.q = pinocchio::neutral(pinocchio_gravity.model);

    for (int i = 0; i < 12; ++i)
    {
      if (!pinocchio_gravity.model.existJointName(motor_joint_names[i]))
      {
        ROS_WARN("Pinocchio gravity disabled: joint %s not found in %s", motor_joint_names[i], urdf_path.c_str());
        return false;
      }
      const pinocchio::JointIndex joint_id = pinocchio_gravity.model.getJointId(motor_joint_names[i]);
      const auto &joint = pinocchio_gravity.model.joints[joint_id];
      if (joint.nq() != 1 || joint.nv() != 1)
      {
        ROS_WARN("Pinocchio gravity disabled: joint %s has nq=%d nv=%d", motor_joint_names[i], joint.nq(), joint.nv());
        return false;
      }
      pinocchio_gravity.q_index[i] = joint.idx_q();
      pinocchio_gravity.v_index[i] = joint.idx_v();
    }

    pinocchio_gravity.ready = true;
    ROS_INFO("Pinocchio gravity ready: %s nq=%d nv=%d", urdf_path.c_str(), pinocchio_gravity.model.nq, pinocchio_gravity.model.nv);
    return true;
  }
  catch (const std::exception &e)
  {
    ROS_WARN("Pinocchio gravity disabled: failed to load %s: %s", urdf_path.c_str(), e.what());
  }
  return false;
}

bool computePinocchioGravityTorques(const LowStateData &data, double gravity_tau[12])
{
  std::fill(gravity_tau, gravity_tau + 12, 0.0);
  if (!foot_force_use_pinocchio_gravity || !pinocchio_gravity.ready || !pinocchio_gravity.data)
  {
    return false;
  }

  for (int i = 0; i < 12; ++i)
  {
    const int q_index = pinocchio_gravity.q_index[i];
    if (q_index < 0 || q_index >= pinocchio_gravity.q.size())
    {
      return false;
    }
    const int joint_slot = i % 3;
    pinocchio_gravity.q[q_index] =
        foot_force_joint_q_sign[joint_slot] * (static_cast<double>(data.motors[i].q) + foot_force_joint_q_offset[joint_slot]);
  }

  const Eigen::VectorXd &g = pinocchio::computeGeneralizedGravity(pinocchio_gravity.model, *pinocchio_gravity.data, pinocchio_gravity.q);
  for (int i = 0; i < 12; ++i)
  {
    const int v_index = pinocchio_gravity.v_index[i];
    if (v_index < 0 || v_index >= g.size())
    {
      return false;
    }
    const int joint_slot = i % 3;
    gravity_tau[i] = foot_force_joint_tau_sign[joint_slot] * g[v_index];
  }
  return true;
}
#else
bool initPinocchioGravity(const std::string &)
{
  ROS_WARN("Pinocchio gravity disabled: binary was built without HAVE_PINOCCHIO");
  return false;
}

bool computePinocchioGravityTorques(const LowStateData &, double gravity_tau[12])
{
  std::fill(gravity_tau, gravity_tau + 12, 0.0);
  return false;
}
#endif

void buildFootForceFromMotorTorques(const LowStateData &data, int16_t foot_force[4])
{
  static const double kLegSideSign[4] = {-1.0, 1.0, -1.0, 1.0};
  const double hip_offset = std::max(0.0, foot_force_hip_offset);
  const double l1 = std::max(0.01, foot_force_thigh_length);
  const double l2 = std::max(0.01, foot_force_calf_length);
  const double foot_radius = std::max(0.0, foot_force_foot_radius);
  const double damping = std::max(0.0, foot_force_damping);
  double gravity_tau[12] = {0.0};
  computePinocchioGravityTorques(data, gravity_tau);

  for (int leg = 0; leg < 4; ++leg)
  {
    const int base_index = leg * 3;
    const double q0 = foot_force_joint_q_sign[0] * (static_cast<double>(data.motors[base_index].q) + foot_force_joint_q_offset[0]);
    const double q1 = foot_force_joint_q_sign[1] * (static_cast<double>(data.motors[base_index + 1].q) + foot_force_joint_q_offset[1]);
    const double q2 = foot_force_joint_q_sign[2] * (static_cast<double>(data.motors[base_index + 2].q) + foot_force_joint_q_offset[2]);
    const double hip_tau = foot_force_joint_tau_sign[0] *
                           (static_cast<double>(data.motors[base_index].tau_est) - foot_force_joint_tau_offset[0] -
                            foot_force_joint_viscous[0] * static_cast<double>(data.motors[base_index].dq));
    const double thigh_tau = foot_force_joint_tau_sign[1] *
                             (static_cast<double>(data.motors[base_index + 1].tau_est) - foot_force_joint_tau_offset[1] -
                              foot_force_joint_viscous[1] * static_cast<double>(data.motors[base_index + 1].dq));
    const double calf_tau = foot_force_joint_tau_sign[2] *
                            (static_cast<double>(data.motors[base_index + 2].tau_est) - foot_force_joint_tau_offset[2] -
                             foot_force_joint_viscous[2] * static_cast<double>(data.motors[base_index + 2].dq));

    const Vec3 hip_joint{0.0, 0.0, 0.0};
    const Vec3 hip_axis{1.0, 0.0, 0.0};
    const Vec3 thigh_offset{0.0, kLegSideSign[leg] * hip_offset, 0.0};
    const Vec3 upper_leg{0.0, 0.0, -l1};
    const Vec3 lower_leg{0.0, 0.0, -l2};
    const Vec3 foot_contact_offset{0.0, 0.0, -foot_radius};

    const Vec3 thigh_joint = rotateX(thigh_offset, q0);
    const Vec3 thigh_axis = rotateX({0.0, 1.0, 0.0}, q0);
    const Vec3 calf_joint = thigh_joint + rotateX(rotateY(upper_leg, q1), q0);
    const Vec3 calf_axis = rotateX(rotateY({0.0, 1.0, 0.0}, q1), q0);
    const Vec3 foot_center = calf_joint + rotateX(rotateY(rotateY(lower_leg, q2), q1), q0);
    const Vec3 foot = foot_center + foot_contact_offset;

    const Vec3 j0 = cross(hip_axis, foot - hip_joint);
    const Vec3 j1 = cross(thigh_axis, foot - thigh_joint);
    const Vec3 j2 = cross(calf_axis, foot - calf_joint);

    double jacobian_t[3][3] = {
        {j0.x, j0.y, j0.z},
        {j1.x, j1.y, j1.z},
        {j2.x, j2.y, j2.z}};
    // Quasi-static contact equilibrium:
    //   g(q) = tau_motor + J(q)^T * F_ground
    // so J(q)^T * F_ground = g(q) - tau_motor.
    const double contact_torque[3] = {
        gravity_tau[base_index] - hip_tau,
        gravity_tau[base_index + 1] - thigh_tau,
        gravity_tau[base_index + 2] - calf_tau};

    Vec3 contact_force{};
    bool solved = solve3x3(jacobian_t, contact_torque, contact_force);
    if (!solved)
    {
      double normal[3][3] = {
          {damping * damping, 0.0, 0.0},
          {0.0, damping * damping, 0.0},
          {0.0, 0.0, damping * damping}};
      double rhs[3] = {0.0, 0.0, 0.0};
      for (int row = 0; row < 3; ++row)
      {
        for (int col = 0; col < 3; ++col)
        {
          rhs[col] += jacobian_t[row][col] * contact_torque[row];
          for (int k = 0; k < 3; ++k)
          {
            normal[col][k] += jacobian_t[row][col] * jacobian_t[row][k];
          }
        }
      }
      solved = solve3x3(normal, rhs, contact_force);
    }

    double vertical_force = foot_force_vertical_sign * contact_force.z;
    if (foot_force_use_abs_z)
    {
      vertical_force = std::abs(contact_force.z);
    }
    double force = solved ? vertical_force * foot_force_scale + foot_force_offset : 0.0;
    if (!std::isfinite(force))
    {
      force = 0.0;
    }
    force = std::max(0.0, std::min(std::min(foot_force_max, 32767.0), force));
    foot_force[leg] = static_cast<int16_t>(std::lround(force));
  }
}

std::string formatMainBoardFlags(uint32_t flags)
{
  struct FlagInfo
  {
    uint32_t mask;
    const char *text;
  };

  static const FlagInfo flag_infos[] = {
      {1u << 0, "上层控制命令超时"},
      {1u << 1, "下层反馈数据超时"},
      {1u << 2, "IMU反馈数据超时"},
      {1u << 3, "电机反馈数据超时"},
      {1u << 5, "遥控器反馈数据超时"},
      {1u << 6, "电池型号错误"},
      {1u << 7, "软启动错误"},
      {1u << 8, "电机状态错误"},
      {1u << 9, "电机过流低限位保护"},
      {1u << 10, "电机欠压高限位保护"},
      {1u << 11, "电机过流高限位保护"},
      {1u << 12, "软急停开关被按下"},
      {1u << 13, "SN错误"},
      {1u << 14, "上层机型错误"},
      {1u << 15, "下层机型错误"},
      {1u << 16, "USB设备错误"},
      {1u << 17, "关节超限位错误"},
      {1u << 18, "Double IMU反馈数据超时"},
      {1u << 19, "主板判断电池UVP错误"},
      {1u << 20, "主板判断电机UVP错误"},
      {1u << 21, "下层机器大类错误"},
      {1u << 22, "特殊关节开机检测失败"}};

  std::vector<std::string> active_flags;
  for (const auto &info : flag_infos)
  {
    if (flags & info.mask)
    {
      active_flags.emplace_back(info.text);
    }
  }
  if (flags & (1u << 4))
  {
    active_flags.emplace_back("缺少第1块电池");
  }
  if (flags & (1u << 23))
  {
    active_flags.emplace_back("缺少第2块电池");
  }

  if (active_flags.empty())
  {
    return "所有子系统正常";
  }

  std::ostringstream ss;
  for (std::size_t i = 0; i < active_flags.size(); ++i)
  {
    ss << active_flags[i];
    if (i + 1 < active_flags.size())
    {
      ss << ", ";
    }
  }
  return ss.str();
}

std::string parseMainBoardStatus(uint32_t flags)
{
  if (flags & ((1u << 8) | (1u << 9) | (1u << 10) | (1u << 11) | (1u << 17) | (1u << 20)))
  {
    return "电机错误";
  }
  if (flags & ((1u << 6) | (1u << 19)))
  {
    return "电池错误";
  }
  if (flags != 0)
  {
    return "系统警告";
  }
  return "正常";
}

std::string formatBmsAlarmDetails(uint32_t alarms)
{
  if (alarms == 0)
  {
    return "正常";
  }

  static const char *alarm_names[] = {
      "单节电芯过压",
      "单节电芯欠压",
      "单节电芯死电池",
      "整体过压",
      "整体欠压",
      "充电过流1",
      "充电过流2",
      "放电过流1",
      "放电过流2",
      "充电短路",
      "放电短路",
      "MOS过温",
      "电池过温",
      "电池低温",
      "平衡过温",
      "电芯开路",
      "电芯过度不平衡",
      "MOS驱动器故障",
      "芯片系统故障",
      "模拟部分偏差过大",
      "并联失败"};

  std::vector<std::string> active_alarms;
  for (std::size_t i = 0; i < sizeof(alarm_names) / sizeof(alarm_names[0]); ++i)
  {
    if (alarms & (1u << i))
    {
      active_alarms.emplace_back(alarm_names[i]);
    }
  }

  std::ostringstream ss;
  for (std::size_t i = 0; i < active_alarms.size(); ++i)
  {
    ss << active_alarms[i];
    if (i + 1 < active_alarms.size())
    {
      ss << ", ";
    }
  }
  if (alarms & 0xFFE00000u)
  {
    if (!active_alarms.empty())
    {
      ss << ", ";
    }
    ss << "保留报警位=0x" << std::hex << std::uppercase << (alarms & 0xFFE00000u);
  }
  return ss.str();
}

std::string formatMotorSummary(const LowStateData &data)
{
  if (!data.received)
  {
    return "尚无电机数据";
  }

  int hottest_index = 0;
  int coolest_index = 0;
  int16_t hottest_temp = data.motors[0].temperature[0];
  int16_t coolest_temp = data.motors[0].temperature[0];
  int error_count = 0;

  for (int i = 0; i < 12; ++i)
  {
    const int16_t temp = data.motors[i].temperature[0];
    if (temp > hottest_temp)
    {
      hottest_temp = temp;
      hottest_index = i;
    }
    if (temp < coolest_temp)
    {
      coolest_temp = temp;
      coolest_index = i;
    }
    if (data.motors[i].motorstate != 0)
    {
      ++error_count;
    }
  }

  std::ostringstream ss;
  ss << "Hottest " << motor_names[hottest_index] << "=" << hottest_temp << "°C"
     << ", Coolest " << motor_names[coolest_index] << "=" << coolest_temp << "°C"
     << ", Error Motors=" << error_count;
  return ss.str();
}

void TransformFluVectorToRfu(double x_flu, double y_flu, double z_flu,
                             double &x_rfu, double &y_rfu, double &z_rfu)
{
  x_rfu = -y_flu;
  y_rfu = x_flu;
  z_rfu = z_flu;
}

tf::Quaternion TransformFluQuaternionToRfu(double w, double x, double y, double z)
{
  tf::Quaternion q_flu(x, y, z, w);
  tf::Quaternion q_flu_rfu;
  q_flu_rfu.setRPY(0.0, 0.0, kFluToRfuYaw);
  tf::Quaternion q_rfu = q_flu * q_flu_rfu;
  q_rfu.normalize();
  return q_rfu;
}

sensor_msgs::Imu BuildImuMessage(const LowStateData &data)
{
  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.frame_id = imu_child_frame;

  const tf::Quaternion q_rfu = TransformFluQuaternionToRfu(
      data.imu_quaternion[0], data.imu_quaternion[1],
      data.imu_quaternion[2], data.imu_quaternion[3]);
  imu_msg.orientation.x = q_rfu.x();
  imu_msg.orientation.y = q_rfu.y();
  imu_msg.orientation.z = q_rfu.z();
  imu_msg.orientation.w = q_rfu.w();

  TransformFluVectorToRfu(
      data.imu_gyroscope[0], data.imu_gyroscope[1], data.imu_gyroscope[2],
      imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);
  TransformFluVectorToRfu(
      data.imu_accelerometer[0], data.imu_accelerometer[1], data.imu_accelerometer[2],
      imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);

  for (int i = 0; i < 9; ++i)
  {
    imu_msg.orientation_covariance[i] = 0.0;
    imu_msg.angular_velocity_covariance[i] = 0.0;
    imu_msg.linear_acceleration_covariance[i] = 0.0;
  }
  return imu_msg;
}

void BroadcastImuTransform(const ros::Time &stamp)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(imu_xyz[0], imu_xyz[1], imu_xyz[2]));
  transform.setRotation(tf::createQuaternionFromRPY(imu_rpy[0], imu_rpy[1], imu_rpy[2]));
  br.sendTransform(tf::StampedTransform(transform, stamp, imu_parent_frame, imu_child_frame));
}

void PublishImuTopic(const LowStateData &data)
{
  sensor_msgs::Imu imu_msg = BuildImuMessage(data);
  imu_pub.publish(imu_msg);
  BroadcastImuTransform(imu_msg.header.stamp);
}

std::string BuildStatusString(const LowStateData &data,
                              const MotionStateData &motion,
                              const MainBoardStateData &mainboard,
                              const BmsStateData &bms,
                              const BmsStateData &secondary_bms,
                              const A2ServiceState &service)
{
  constexpr double kRad2Deg = 180.0 / M_PI;
  const double roll_deg = data.imu_rpy[0] * kRad2Deg;
  const double pitch_deg = data.imu_rpy[1] * kRad2Deg;
  const double yaw_deg = data.imu_rpy[2] * kRad2Deg;

  std::ostringstream status_stream;
  status_stream << std::fixed << std::setprecision(2);
  status_stream << "\n========== ROBOT STATUS ==========";

  status_stream << "\n[Battery]\n";
  if (bms.received || secondary_bms.received)
  {
    const BmsStateData &display_bms = bms.received ? bms : secondary_bms;
    const int received_battery_count = (bms.received ? 1 : 0) + (secondary_bms.received ? 1 : 0);
    const int combined_soc = received_battery_count > 0
                                 ? ((bms.received ? static_cast<int>(bms.soc) : 0) +
                                    (secondary_bms.received ? static_cast<int>(secondary_bms.soc) : 0)) /
                                       received_battery_count
                                 : 0;
    const uint32_t combined_alarms = (bms.received ? bms.bmsstate[2] : 0) |
                                     (secondary_bms.received ? secondary_bms.bmsstate[2] : 0);
    const uint16_t combined_cycle = std::max(bms.received ? bms.cycle : 0,
                                             secondary_bms.received ? secondary_bms.cycle : 0);
    const double battery_current_a = (static_cast<double>(bms.received ? bms.current : 0) +
                                      static_cast<double>(secondary_bms.received ? secondary_bms.current : 0)) /
                                     1000.0;
    const double power_board_v = mainboard.received ? mainboard.value[0] : static_cast<double>(display_bms.bmsvoltage[0]) / 1000.0;
    const double power_board_a = mainboard.received ? mainboard.value[2] : battery_current_a;
    status_stream << "  Mode: " << formatBmsAlarmDetails(combined_alarms)
                  << " | SOC: " << combined_soc << "%"
                  << " | Cycle: " << combined_cycle << "\n";
    status_stream << "  Temp(BAT1/BAT2): "
                  << (bms.received ? std::to_string(bms.temperature[0]) + "°C" : "N/A") << "/"
                  << (secondary_bms.received ? std::to_string(secondary_bms.temperature[0]) + "°C" : "N/A") << "\n";
    status_stream << "  Current: " << battery_current_a << "A"
                  << " | Power Board: " << power_board_v << "V, " << power_board_a << "A\n";
  }
  else
  {
    status_stream << "  Mode: N/A | SOC: N/A | Cycle: N/A\n";
    status_stream << "  Temp(BAT1/BAT2): N/A/N/A\n";
    status_stream << "  Current: N/A | Power Board: N/A\n";
  }

  status_stream << "[Motion]\n";
  if (motion.received)
  {
    status_stream << "  Mode: " << getModeString(motion.mode)
                  << " | Gait: " << getGaitString(motion.gait_type)
                  << " | Yaw Speed: " << motion.yaw_speed << "rad/s\n";
    status_stream << "  Pos (m): [" << motion.position[0] << ", " << motion.position[1] << ", " << motion.position[2] << "]\n";
    status_stream << "  Vel (m/s): [" << motion.velocity[0] << ", " << motion.velocity[1] << ", " << motion.velocity[2] << "]\n";
  }
  else
  {
    status_stream << "  Waiting for rt/sportmodestate\n";
  }
  status_stream << "  RPY (deg): [" << roll_deg << ", " << pitch_deg << ", " << yaw_deg << "]"
                << " | IMU Temp: " << data.imu_temperature << "°C\n";

  status_stream << "[Foot Force]\n";
  int16_t display_foot_force[4] = {0, 0, 0, 0};
  if (data.received)
  {
    buildFootForceFromMotorTorques(data, display_foot_force);
  }
  status_stream << "  " << formatFootForces(display_foot_force) << "\n";

  status_stream << "[Motors]\n";
  status_stream << "  " << formatMotorSummary(data) << "\n";

  status_stream << "[System]\n";
  const uint32_t main_flags = mainboard.received ? mainboard.state[0] : 0;
  status_stream << "  Flags: " << parseMainBoardStatus(main_flags)
                << " -> " << (mainboard.received ? formatMainBoardFlags(main_flags) : "Waiting for rt/lf/mainboardstate")
                << "\n";
  status_stream << "  Auto Charge: " << getAutoChargeString(bms, secondary_bms) << "\n";
  if (mainboard.received)
  {
    status_stream << "  Main Board Temp: " << mainboard.temperature[0] << "°C"
                  << " | Charging Temp: " << mainboard.temperature[1] << "°C\n";
  }
  else
  {
    status_stream << "  Main Board Temp: N/A | Charging Temp: N/A\n";
  }
  status_stream << "\n===================================\n";

  return status_stream.str();
}

void refreshServiceState()
{
  if (!sport_client)
  {
    return;
  }

  std::map<std::string, std::string> state_map;
  int32_t ret = -1;
  {
    std::lock_guard<std::mutex> lock(sport_client_mutex);
    ret = sport_client->GetState(state_map);
  }

  if (ret >= 0)
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    service_state_data.received = true;
    service_state_data.values = state_map;
  }
}

void PublishStatusTopic(const LowStateData &data,
                        const MotionStateData &motion,
                        const MainBoardStateData &mainboard,
                        const BmsStateData &bms,
                        const BmsStateData &secondary_bms,
                        const A2ServiceState &service)
{
  static ros::Time last_publish_time;
  const ros::Time now = ros::Time::now();
  if (!last_publish_time.isZero() && (now - last_publish_time).toSec() < 1.0)
  {
    return;
  }
  last_publish_time = now;

  std_msgs::String status_msg;
  status_msg.data = BuildStatusString(data, motion, mainboard, bms, secondary_bms, service);
  status_pub.publish(status_msg);
}

void PublishOdomMessage(const LowStateData &data, const MotionStateData &motion, ros::Publisher &publisher)
{
  if (!publisher)
  {
    return;
  }

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = odom_frame_id;
  odom_msg.child_frame_id = odom_child_frame_id;
  if (motion.received)
  {
    // Position is expressed in the odom/world frame and is unchanged.
    odom_msg.pose.pose.position.x = motion.position[0];
    odom_msg.pose.pose.position.y = motion.position[1];
    odom_msg.pose.pose.position.z = motion.position[2];
    TransformFluVectorToRfu(
        motion.velocity[0], motion.velocity[1], motion.velocity[2],
        odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z);
  }

  const tf::Quaternion q_rfu = TransformFluQuaternionToRfu(
      data.imu_quaternion[0], data.imu_quaternion[1],
      data.imu_quaternion[2], data.imu_quaternion[3]);
  odom_msg.pose.pose.orientation.x = q_rfu.x();
  odom_msg.pose.pose.orientation.y = q_rfu.y();
  odom_msg.pose.pose.orientation.z = q_rfu.z();
  odom_msg.pose.pose.orientation.w = q_rfu.w();

  double angular_x_rfu = 0.0;
  double angular_y_rfu = 0.0;
  double angular_z_rfu = 0.0;
  TransformFluVectorToRfu(
      data.imu_gyroscope[0], data.imu_gyroscope[1], data.imu_gyroscope[2],
      angular_x_rfu, angular_y_rfu, angular_z_rfu);
  odom_msg.twist.twist.angular.x = angular_x_rfu;
  odom_msg.twist.twist.angular.y = angular_y_rfu;
  // Body yaw rate is about the vertical axis, which is +Z in both FLU and RFU.
  odom_msg.twist.twist.angular.z = motion.received ? motion.yaw_speed : angular_z_rfu;

  publisher.publish(odom_msg);
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  float vx = std::max(-1.0f, std::min(1.0f, static_cast<float>(msg->linear.x)));
  float vy = std::max(-1.0f, std::min(1.0f, static_cast<float>(msg->linear.y)));
  float vyaw = std::max(-4.0f, std::min(4.0f, static_cast<float>(msg->angular.z)));
  int32_t result = callSport([&]()
                             { return sport_client->Move(vx, vy, vyaw); });
  if (result == 0)
  {
    ROS_INFO("Move command sent: vx=%.2f, vy=%.2f, vyaw=%.2f", vx, vy, vyaw);
  }
  else
  {
    ROS_WARN("Move command failed with error code: %d", result);
  }
}

void upDownCallback(const std_msgs::String::ConstPtr &msg)
{
  const std::string cmd = trimLower(msg->data);
  int32_t result = -1;
  if (cmd == "up")
  {
    result = callSport([]()
                       { return sport_client->StandUp(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    result = callSport([]()
                       { return sport_client->BalanceStand(); });
    ROS_INFO("upDown: StandUp + BalanceStand sent, result=%d", result);
  }
  else if (cmd == "down")
  {
    result = callSport([]()
                       { return sport_client->StandUp(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    result = callSport([]()
                       { return sport_client->StandDown(); });
    ROS_INFO("upDown: StandDown sent, result=%d", result);
  }
  else
  {
    ROS_WARN("upDown: unknown command '%s' (expected 'up' or 'down')", cmd.c_str());
  }
}

void temStatusCallback(const std_msgs::String::ConstPtr &msg)
{
  const std::string cmd = trimLower(msg->data);
  if (cmd == "start")
  {
    int32_t result = callSport([]()
                               { return sport_client->StandUp(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    result = callSport([]()
                       { return sport_client->BalanceStand(); });
    ROS_INFO("temStatus: Start sent, result=%d", result);
  }
  else if (cmd == "stop")
  {
    int32_t result = callSport([]()
                               { return sport_client->StandUp(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    result = callSport([]()
                       { return sport_client->StandDown(); });
    ROS_INFO("temStatus: Stop sent, result=%d", result);
  }
}

void modeSwitchCallback(const std_msgs::String::ConstPtr &msg)
{
  const std::string mode = trimLower(msg->data);
  int32_t result = -1;
  if (mode == "lingdong")
  {
    result = callSport([]()
                       { return sport_client->SpeedLevel(1); });
    result = callSport([]()
                       { return sport_client->SwitchGait(0); });
    ROS_INFO("modeSwitch: A2 agile/default gait sent, result=%d", result);
  }
  else if (mode == "classic")
  {
    result = callSport([]()
                       { return sport_client->SpeedLevel(0); });
    result = callSport([]()
                       { return sport_client->SwitchGait(0); });
    ROS_INFO("modeSwitch: A2 classic/default gait sent, result=%d", result);
  }
  else
  {
    ROS_WARN("modeSwitch: unknown mode '%s' (expected 'lingdong' or 'classic')", mode.c_str());
  }
}

void sportCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  const std::string cmd = trimLower(msg->data);
  int32_t result = -1;

  if (cmd == "damp")
    result = callSport([]()
                       { return sport_client->Damp(); });
  else if (cmd == "balance_stand")
    result = callSport([]()
                       { return sport_client->BalanceStand(); });
  else if (cmd == "stop_move")
    result = callSport([]()
                       { return sport_client->StopMove(); });
  else if (cmd == "recovery_stand")
    result = callSport([]()
                       { return sport_client->RecoveryStand(); });
  else if (cmd == "stand_up")
    result = callSport([]()
                       { return sport_client->StandUp(); });
  else if (cmd == "stand_down")
    result = callSport([]()
                       { return sport_client->StandDown(); });
  else if (cmd == "left_side_gait_on")
    result = callSport([]()
                       { return sport_client->LeftSideGait(1); });
  else if (cmd == "left_side_gait_off")
    result = callSport([]()
                       { return sport_client->LeftSideGait(0); });
  else if (cmd == "right_side_gait_on")
    result = callSport([]()
                       { return sport_client->RightSideGait(1); });
  else if (cmd == "right_side_gait_off")
    result = callSport([]()
                       { return sport_client->RightSideGait(0); });
  else if (cmd == "handstand_on")
    result = callSport([]()
                       { return sport_client->HandStand(1); });
  else if (cmd == "handstand_off")
    result = callSport([]()
                       { return sport_client->HandStand(0); });
  else if (cmd == "biped_on")
    result = callSport([]()
                       { return sport_client->BipedStand(1); });
  else if (cmd == "biped_off")
    result = callSport([]()
                       { return sport_client->BipedStand(0); });
  else if (cmd == "front_flip")
    result = callSport([]()
                       { return sport_client->FrontFlip(); });
  else if (cmd == "back_flip")
    result = callSport([]()
                       { return sport_client->BackFlip(); });
  else if (cmd == "auto_recovery_on")
    result = callSport([]()
                       { return sport_client->SetAutoRecovery(1); });
  else if (cmd == "auto_recovery_off")
    result = callSport([]()
                       { return sport_client->SetAutoRecovery(0); });
  else
  {
    ROS_WARN("a2/sport_cmd: unknown command '%s'", cmd.c_str());
    return;
  }

  ROS_INFO("a2/sport_cmd: %s result=%d", cmd.c_str(), result);
}

void bodyHeightCallback(const std_msgs::Float32::ConstPtr &msg)
{
  const float height = std::max(-0.3f, std::min(0.3f, msg->data));
  int32_t result = callSport([&]()
                             { return sport_client->BodyHeight(height); });
  ROS_INFO("a2/body_height: %.2f result=%d", height, result);
}

void speedLevelCallback(const std_msgs::Int32::ConstPtr &msg)
{
  const int level = std::max(0, std::min(2, msg->data));
  int32_t result = callSport([&]()
                             { return sport_client->SpeedLevel(level); });
  ROS_INFO("a2/speed_level: %d result=%d", level, result);
}

void eulerCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  int32_t result = callSport([&]()
                             { return sport_client->Euler(static_cast<float>(msg->angular.x),
                                                          static_cast<float>(msg->angular.y),
                                                          static_cast<float>(msg->angular.z)); });
  ROS_INFO("a2/euler result=%d", result);
}

void bodyPoseCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  int32_t result = callSport([&]()
                             { return sport_client->BodyPosition(static_cast<float>(msg->linear.x),
                                                                 static_cast<float>(msg->linear.y),
                                                                 static_cast<float>(msg->linear.z),
                                                                 static_cast<float>(msg->angular.z)); });
  ROS_INFO("a2/body_pose result=%d", result);
}

void lowStateCallback(const void *message)
{
  const auto &low_state = *static_cast<const unitree_hg::msg::dds_::LowState_ *>(message);
  LowStateData updated_data;
  updated_data.received = true;
  updated_data.mode_pr = low_state.mode_pr();
  updated_data.mode_machine = low_state.mode_machine();
  updated_data.tick = low_state.tick();

  for (int i = 0; i < 4; ++i)
  {
    updated_data.imu_quaternion[i] = low_state.imu_state().quaternion()[i];
  }
  for (int i = 0; i < 3; ++i)
  {
    updated_data.imu_gyroscope[i] = low_state.imu_state().gyroscope()[i];
    updated_data.imu_accelerometer[i] = low_state.imu_state().accelerometer()[i];
    updated_data.imu_rpy[i] = low_state.imu_state().rpy()[i];
  }
  updated_data.imu_temperature = low_state.imu_state().temperature();

  for (int i = 0; i < 12; ++i)
  {
    updated_data.motors[i].mode = low_state.motor_state()[i].mode();
    updated_data.motors[i].q = low_state.motor_state()[i].q();
    updated_data.motors[i].dq = low_state.motor_state()[i].dq();
    updated_data.motors[i].ddq = low_state.motor_state()[i].ddq();
    updated_data.motors[i].tau_est = low_state.motor_state()[i].tau_est();
    updated_data.motors[i].temperature[0] = low_state.motor_state()[i].temperature()[0];
    updated_data.motors[i].temperature[1] = low_state.motor_state()[i].temperature()[1];
    updated_data.motors[i].voltage = low_state.motor_state()[i].vol();
    updated_data.motors[i].motorstate = low_state.motor_state()[i].motorstate();
  }

  MotionStateData motion_snapshot;
  MainBoardStateData mainboard_snapshot;
  BmsStateData bms_snapshot;
  BmsStateData secondary_bms_snapshot;
  A2ServiceState service_snapshot;
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    low_state_data = updated_data;
    motion_snapshot = motion_state_data;
    mainboard_snapshot = mainboard_state_data;
    bms_snapshot = bms_state_data;
    secondary_bms_snapshot = secondary_bms_state_data;
    service_snapshot = service_state_data;
  }

  PublishImuTopic(updated_data);
  PublishOdomMessage(updated_data, motion_snapshot, odom_pub);
  PublishStatusTopic(updated_data, motion_snapshot, mainboard_snapshot, bms_snapshot, secondary_bms_snapshot, service_snapshot);
  publishRawString(raw_lowstate_pub, formatLowStateRaw(updated_data));
}

void mainBoardStateCallback(const void *message)
{
  const auto &state = *static_cast<const unitree_hg::msg::dds_::MainBoardState_ *>(message);
  MainBoardStateData updated_data;
  updated_data.received = true;

  for (int i = 0; i < 6; ++i)
  {
    updated_data.fan_state[i] = state.fan_state()[i];
    updated_data.temperature[i] = state.temperature()[i];
    updated_data.value[i] = state.value()[i];
    updated_data.state[i] = state.state()[i];
  }

  std::lock_guard<std::mutex> lock(state_mutex);
  mainboard_state_data = updated_data;
  publishRawString(raw_mainboardstate_pub, formatMainBoardRaw(updated_data));
}

void updateBmsSnapshot(const unitree_hg::msg::dds_::BmsState_ &state, BmsStateData &target)
{
  target.received = true;
  target.version_high = state.version_high();
  target.version_low = state.version_low();
  target.fn = state.fn();
  target.current = state.current();
  target.soc = state.soc();
  target.soh = state.soh();
  target.cycle = state.cycle();
  target.manufacturer_date = state.manufacturer_date();

  for (int i = 0; i < 40; ++i)
  {
    target.cell_vol[i] = state.cell_vol()[i];
  }
  for (int i = 0; i < 3; ++i)
  {
    target.bmsvoltage[i] = state.bmsvoltage()[i];
  }
  for (int i = 0; i < 12; ++i)
  {
    target.temperature[i] = state.temperature()[i];
  }
  for (int i = 0; i < 5; ++i)
  {
    target.bmsstate[i] = state.bmsstate()[i];
  }
  for (int i = 0; i < 3; ++i)
  {
    target.reserve[i] = state.reserve()[i];
  }
}

void bmsStateCallback(const void *message)
{
  const auto &state = *static_cast<const unitree_hg::msg::dds_::BmsState_ *>(message);
  BmsStateData updated_data;
  std::lock_guard<std::mutex> lock(state_mutex);
  updateBmsSnapshot(state, bms_state_data);
  updated_data = bms_state_data;
  publishRawString(raw_bmsstate_pub, formatBmsRaw(updated_data));
}

void secondaryBmsStateCallback(const void *message)
{
  const auto &state = *static_cast<const unitree_hg::msg::dds_::BmsState_ *>(message);
  BmsStateData updated_data;
  std::lock_guard<std::mutex> lock(state_mutex);
  updateBmsSnapshot(state, secondary_bms_state_data);
  updated_data = secondary_bms_state_data;
  publishRawString(raw_secondary_bmsstate_pub, formatBmsRaw(updated_data));
}

MotionStateData buildMotionStateSnapshot(const unitree_go::msg::dds_::SportModeState_ &state)
{
  MotionStateData updated_motion;
  updated_motion.received = true;
  updated_motion.mode = state.mode();
  updated_motion.gait_type = state.gait_type();
  updated_motion.yaw_speed = state.yaw_speed();
  for (int i = 0; i < 3; ++i)
  {
    updated_motion.position[i] = state.position()[i];
    updated_motion.velocity[i] = state.velocity()[i];
  }
  for (int i = 0; i < 4; ++i)
  {
    updated_motion.foot_force[i] = state.foot_force()[i];
  }
  return updated_motion;
}

void sportStateCallback(const void *message)
{
  const auto &state = *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(message);
  const MotionStateData updated_motion = buildMotionStateSnapshot(state);
  LowStateData data_snapshot;

  {
    std::lock_guard<std::mutex> lock(state_mutex);
    motion_state_data = updated_motion;
    data_snapshot = low_state_data;
  }
  PublishOdomMessage(data_snapshot, updated_motion, odom_pub);
  publishRawString(raw_sportstate_pub, formatMotionStateRaw(updated_motion));
}

void sportStateLowFreqCallback(const void *message)
{
  const auto &state = *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(message);
  const MotionStateData updated_motion = buildMotionStateSnapshot(state);
  LowStateData data_snapshot;
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    motion_state_data = updated_motion;
    data_snapshot = low_state_data;
  }
  publishRawString(raw_sportstate_lf_pub, formatMotionStateRaw(updated_motion));
  PublishOdomMessage(data_snapshot, updated_motion, odom_lf_pub);
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    return -1;
  }

  ros::init(argc, argv, "unitree_a2_ros_control");
  ros::NodeHandle nh;

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  sport_client = std::make_shared<a2::SportClient>();
  sport_client->SetTimeout(10.0f);
  sport_client->Init();

  ros::Subscriber cmdVelSub = nh.subscribe("cmd_vel", 10, cmdVelCallback);
  ros::Subscriber upDownSub = nh.subscribe("upDown", 10, upDownCallback);
  ros::Subscriber temStatusSub = nh.subscribe("/temStatus", 10, temStatusCallback);
  ros::Subscriber modeSwitchSub = nh.subscribe("/mode_switch", 10, modeSwitchCallback);
  ros::Subscriber sportCmdSub = nh.subscribe("/a2/sport_cmd", 10, sportCommandCallback);
  ros::Subscriber bodyHeightSub = nh.subscribe("/a2/body_height", 10, bodyHeightCallback);
  ros::Subscriber speedLevelSub = nh.subscribe("/a2/speed_level", 10, speedLevelCallback);
  ros::Subscriber eulerSub = nh.subscribe("/a2/euler", 10, eulerCallback);
  ros::Subscriber bodyPoseSub = nh.subscribe("/a2/body_pose", 10, bodyPoseCallback);

  imu_pub = nh.advertise<sensor_msgs::Imu>("/a2/imu", 10);
  status_pub = nh.advertise<std_msgs::String>("/a2/status", 10);
  odom_pub = nh.advertise<nav_msgs::Odometry>("/a2/odom", 50);
  odom_lf_pub = nh.advertise<nav_msgs::Odometry>("/a2/odom_lf", 10);
  raw_lowstate_pub = nh.advertise<std_msgs::String>("/a2/lowstate", 10);
  raw_sportstate_pub = nh.advertise<std_msgs::String>("/a2/sportmodestate", 10);
  raw_sportstate_lf_pub = nh.advertise<std_msgs::String>("/a2/sportmodestate_lowfreq", 10);
  raw_mainboardstate_pub = nh.advertise<std_msgs::String>("/a2/mainboardstate", 10);
  raw_bmsstate_pub = nh.advertise<std_msgs::String>("/a2/bmsstate", 10);
  raw_secondary_bmsstate_pub = nh.advertise<std_msgs::String>("/a2/secondary_bmsstate", 10);

  nh.param<std::string>("imu_parent_frame", imu_parent_frame, std::string("base_link"));
  nh.param<std::string>("imu_child_frame", imu_child_frame, std::string("base_imu"));
  nh.param<double>("imu_x", imu_xyz[0], 0.0);
  nh.param<double>("imu_y", imu_xyz[1], 0.0);
  nh.param<double>("imu_z", imu_xyz[2], 0.0);
  nh.param<double>("imu_roll", imu_rpy[0], 0.0);
  nh.param<double>("imu_pitch", imu_rpy[1], 0.0);
  nh.param<double>("imu_yaw", imu_rpy[2], kFluToRfuYaw);
  nh.param<std::string>("odom_frame_id", odom_frame_id, std::string("odom"));
  nh.param<std::string>("base_frame_id", odom_child_frame_id, std::string("base_imu"));
  nh.param<double>("foot_force_hip_offset", foot_force_hip_offset, 0.12779);
  nh.param<double>("foot_force_thigh_length", foot_force_thigh_length, 0.275);
  nh.param<double>("foot_force_calf_length", foot_force_calf_length, 0.275);
  nh.param<double>("foot_force_foot_radius", foot_force_foot_radius, 0.032);
  nh.param<bool>("foot_force_use_pinocchio_gravity", foot_force_use_pinocchio_gravity, true);
  nh.param<std::string>("foot_force_urdf_path", foot_force_urdf_path,
                        std::string("/home/unitree/a2_nav/unitree_sdk2/example/a2_description/urdf/a2.urdf"));
  nh.param<double>("foot_force_scale", foot_force_scale, 1.0);
  nh.param<double>("foot_force_offset", foot_force_offset, 0.0);
  nh.param<double>("foot_force_max", foot_force_max, 2000.0);
  nh.param<double>("foot_force_damping", foot_force_damping, 0.05);
  nh.param<double>("foot_force_vertical_sign", foot_force_vertical_sign, 1.0);
  nh.param<bool>("foot_force_use_abs_z", foot_force_use_abs_z, false);
  nh.param<double>("foot_force_hip_q_sign", foot_force_joint_q_sign[0], 1.0);
  nh.param<double>("foot_force_thigh_q_sign", foot_force_joint_q_sign[1], 1.0);
  nh.param<double>("foot_force_calf_q_sign", foot_force_joint_q_sign[2], 1.0);
  nh.param<double>("foot_force_hip_tau_sign", foot_force_joint_tau_sign[0], 1.0);
  nh.param<double>("foot_force_thigh_tau_sign", foot_force_joint_tau_sign[1], 1.0);
  nh.param<double>("foot_force_calf_tau_sign", foot_force_joint_tau_sign[2], 1.0);
  nh.param<double>("foot_force_hip_q_offset", foot_force_joint_q_offset[0], 0.0);
  nh.param<double>("foot_force_thigh_q_offset", foot_force_joint_q_offset[1], 0.0);
  nh.param<double>("foot_force_calf_q_offset", foot_force_joint_q_offset[2], 0.0);
  nh.param<double>("foot_force_hip_tau_offset", foot_force_joint_tau_offset[0], 0.0);
  nh.param<double>("foot_force_thigh_tau_offset", foot_force_joint_tau_offset[1], 0.0);
  nh.param<double>("foot_force_calf_tau_offset", foot_force_joint_tau_offset[2], 0.0);
  nh.param<double>("foot_force_hip_viscous", foot_force_joint_viscous[0], 0.2);
  nh.param<double>("foot_force_thigh_viscous", foot_force_joint_viscous[1], 0.2);
  nh.param<double>("foot_force_calf_viscous", foot_force_joint_viscous[2], 1.0);
  nh.param<bool>("publish_raw_dds_topics", publish_raw_dds_topics, true);

  if (foot_force_use_pinocchio_gravity)
  {
    initPinocchioGravity(foot_force_urdf_path);
  }

  std::string dds_lowstate_topic = "rt/lowstate";
  std::string dds_sportstate_topic = "rt/sportmodestate";
  std::string dds_sportstate_lf_topic = "rt/lf/sportmodestate";
  std::string dds_mainboardstate_topic = "rt/lf/mainboardstate";
  std::string dds_bmsstate_topic = "rt/lf/bmsstate";
  std::string dds_secondary_bmsstate_topic = "rt/lf/secondary_bmsstate";
  nh.param<std::string>("dds_lowstate_topic", dds_lowstate_topic, dds_lowstate_topic);
  nh.param<std::string>("dds_sportstate_topic", dds_sportstate_topic, dds_sportstate_topic);
  nh.param<std::string>("dds_sportstate_lf_topic", dds_sportstate_lf_topic, dds_sportstate_lf_topic);
  nh.param<std::string>("dds_mainboardstate_topic", dds_mainboardstate_topic, dds_mainboardstate_topic);
  nh.param<std::string>("dds_bmsstate_topic", dds_bmsstate_topic, dds_bmsstate_topic);
  nh.param<std::string>("dds_secondary_bmsstate_topic", dds_secondary_bmsstate_topic, dds_secondary_bmsstate_topic);

  lowstate_subscriber = std::make_unique<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(dds_lowstate_topic);
  lowstate_subscriber->InitChannel(std::bind(&lowStateCallback, std::placeholders::_1), 1);

  sportstate_subscriber = std::make_unique<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(dds_sportstate_topic);
  sportstate_subscriber->InitChannel(std::bind(&sportStateCallback, std::placeholders::_1), 1);

  sportstate_lf_subscriber = std::make_unique<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(dds_sportstate_lf_topic);
  sportstate_lf_subscriber->InitChannel(std::bind(&sportStateLowFreqCallback, std::placeholders::_1), 1);

  mainboardstate_subscriber = std::make_unique<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::MainBoardState_>>(dds_mainboardstate_topic);
  mainboardstate_subscriber->InitChannel(std::bind(&mainBoardStateCallback, std::placeholders::_1), 1);

  bmsstate_subscriber = std::make_unique<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::BmsState_>>(dds_bmsstate_topic);
  bmsstate_subscriber->InitChannel(std::bind(&bmsStateCallback, std::placeholders::_1), 1);

  secondary_bmsstate_subscriber = std::make_unique<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::BmsState_>>(dds_secondary_bmsstate_topic);
  secondary_bmsstate_subscriber->InitChannel(std::bind(&secondaryBmsStateCallback, std::placeholders::_1), 1);

  ROS_INFO("Unitree A2 ROS Control Node Started with Go2-compatible status format");
  ROS_INFO("Publishing IMU data on: /a2/imu from rt/lowstate");
  ROS_INFO("Publishing status data on: /a2/status");
  ROS_INFO("Publishing Odometry on: /a2/odom and /a2/odom_lf");
  ROS_INFO("Raw DDS ROS topics %s: /a2/lowstate, /a2/sportmodestate, /a2/sportmodestate_lowfreq, /a2/mainboardstate, /a2/bmsstate, /a2/secondary_bmsstate",
           publish_raw_dds_topics ? "enabled" : "disabled");
  ROS_INFO("Subscribing DDS status topics: %s, %s, %s, %s", dds_lowstate_topic.c_str(), dds_mainboardstate_topic.c_str(), dds_bmsstate_topic.c_str(), dds_secondary_bmsstate_topic.c_str());
  ROS_INFO("Subscribing to: cmd_vel, upDown, /temStatus, /mode_switch, /a2/sport_cmd");

  ros::spin();

  return 0;
}
