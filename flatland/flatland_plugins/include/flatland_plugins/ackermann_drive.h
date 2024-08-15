#include <Box2D/Box2D.h>
#include <flatland_plugins/update_timer.h>
#include <flatland_plugins/dynamics_limits.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <random>

#ifndef FLATLAND_PLUGINS_ACKERMANN_DRIVE_H
#define FLATLAND_PLUGINS_ACKERMANN_DRIVE_H

using namespace flatland_server;
using namespace std;

namespace flatland_plugins {

class ackermannDrive : public flatland_server::ModelPlugin {
 public:
  Body* body_;
  // Joint* front_wj_;       ///<  front wheel joint
  Joint* front_left_wj_;
  Joint* front_right_wj_;
  Joint* rear_left_wj_;   ///< rear left wheel joint
  Joint* rear_right_wj_;  ///< rear right wheel joint
  double axel_track_;     ///< normal distrance between the rear two wheels
  double wheelbase_;      ///< distance between the front and rear wheel
  b2Vec2 rear_center_;    ///< middle point between the two rear wheels
  b2Vec2 front_center_;
  bool invert_steering_angle_lf_;     ///< whether to invert steering angle
  bool invert_steering_angle_rf_;     ///< whether to invert steering angle

  double max_steer_angle_;         ///< max abs. steering allowed [rad]
  DynamicsLimits angular_dynamics_; ///< Angular dynamics constraints
  DynamicsLimits linear_dynamics_;  ///< Linear dynamics constraints
  double delta_command_;  ///< The current target (commanded) wheel angle
  double theta_f_;        ///< The current angular offset of the front wheel
  double d_delta_;        ///< The current angular speed of the front wheel
  double v_f_ = 0.0;      ///< The current velocity at the front wheel

  double R = 0.0;         ///< The current turning radius

  // 用于角度进行分解的变量
  double delta_command_lf_;
  double delta_command_rf_;
  double delta_command_lr_;
  double delta_command_rr_;

  double d_delta_lf_;
  double d_delta_rf_;
  double d_delta_lr_;
  double d_delta_rr_;

  double theta_lf_;
  double theta_rf_; 
  double theta_lr_;
  double theta_rr_;

  geometry_msgs::Twist twist_msg_;
  nav_msgs::Odometry odom_msg_;
  nav_msgs::Odometry ground_truth_msg_;
  ros::Subscriber twist_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher ground_truth_pub_;

  UpdateTimer update_timer_;

  default_random_engine rng_;
  array<normal_distribution<double>, 6> noise_gen_;

  /**
   * @name                OnInitialize
   * @brief               initialize the bicycle plugin
   * @param world_file    The path to the world.yaml file
   */
  void OnInitialize(const YAML::Node& config) override;

  /**
   * @brief This is a helper function that is used to valid and extract
   * parameters from  joints
   */
  void ComputeJoints();

  /**
   * @brief     Updates the vehicle state given the twist command;
   *            overrides the BeforePhysicsStep method
   * @details   Uses a 2nd-order approximation of the steering & drive systems.
   *            Does not account for dynamics such as:
   *            - Motor winding current/rpm/torque behaviour
   *            - Other motor PID controllers in the loop
   *            - Communication delays
   *            - Any mechanical lag in the drive mechanism (chains, inertia)
   *            - Measurement dynamics other than Gaussian noise
   *            A separate plugin which subscribes to different messages
   *            could be used for more accurate modelling.
   *
   *            References:
   *            Some notation and ideas borrowed from
   *            http://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf
   */
  void BeforePhysicsStep(const Timekeeper& timekeeper) override;

  /**
  * @name          TwistCallback
  * @brief         callback to apply twist (velocity and omega)
  * @param[in]     timestep how much the physics time will increment
  */
  void TwistCallback(const geometry_msgs::Twist& msg);

  /**
   * @brief     Saturates the input between the lower and upper limits
   * @param[in] in: value to saturate
   * @param[in] lower: lower limit of saturation bound
   * @param[in] upper: upper limit of saturation bound
   * @return    input value capped between lower and upper
   */
  double Saturate(double in, double lower, double upper);
};
}

#endif
