#include <Box2D/Box2D.h>
#include <flatland_plugins/dackermann_drive.h>
#include <flatland_server/debug_visualization.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/tf.h>

namespace flatland_plugins {

// 初始化
void DackermannDrive::OnInitialize(const YAML::Node& config) {
  YamlReader r(config);

  // 加载所有参数
  string body_name = r.Get<string>("body");
  string front_left_wj_name = r.Get<string>("front_left_wheel_joint");
  string front_right_wj_name = r.Get<string>("front_right_wheel_joint");
  string rear_left_wj_name = r.Get<string>("rear_left_wheel_joint");
  string rear_right_wj_name = r.Get<string>("rear_right_wheel_joint");
  string odom_frame_id = r.Get<string>("odom_frame_id", "odom");

  string twist_topic = r.Get<string>("twist_sub", "cmd_vel");
  string odom_topic = r.Get<string>("odom_pub", "odometry/filtered");
  string ground_truth_topic = r.Get<string>("ground_truth_pub", "odometry/ground_truth");
  string ground_truth_frame_id = r.Get<string>("ground_truth_frame_id", "map");

  vector<double> odom_twist_noise = r.GetList<double>("odom_twist_noise", {0, 0, 0}, 3, 3);
  vector<double> odom_pose_noise = r.GetList<double>("odom_pose_noise", {0, 0, 0}, 3, 3);

  double pub_rate = r.Get<double>("pub_rate", numeric_limits<double>::infinity());
  update_timer_.SetRate(pub_rate);

  array<double, 36> odom_pose_covar_default = {0};
  odom_pose_covar_default[0] = odom_pose_noise[0];
  odom_pose_covar_default[7] = odom_pose_noise[1];
  odom_pose_covar_default[35] = odom_pose_noise[2];

  array<double, 36> odom_twist_covar_default = {0};
  odom_twist_covar_default[0] = odom_twist_noise[0];
  odom_twist_covar_default[7] = odom_twist_noise[1];
  odom_twist_covar_default[35] = odom_twist_noise[2];

  auto odom_twist_covar = r.GetArray<double, 36>("odom_twist_covariance", odom_twist_covar_default);
  auto odom_pose_covar = r.GetArray<double, 36>("odom_pose_covariance", odom_pose_covar_default);

  max_steer_angle_ = r.Get<double>("max_steer_angle", 0.0);

  angular_dynamics_.Configure(r.SubnodeOpt("angular_dynamics", YamlReader::MAP).Node());

  if (angular_dynamics_.velocity_limit_ == 0.0) angular_dynamics_.velocity_limit_ = r.Get<double>("max_angular_velocity", 0.0);
  if (angular_dynamics_.acceleration_limit_ == 0.0) {
    angular_dynamics_.acceleration_limit_ = r.Get<double>("max_steer_acceleration", 0.0);
    angular_dynamics_.deceleration_limit_ = angular_dynamics_.acceleration_limit_;
  }

  linear_dynamics_.Configure(r.SubnodeOpt("linear_dynamics", YamlReader::MAP).Node());

  delta_command_ = 0.0;
  theta_f_ = 0.0;
  theta_lf_ = 0.0;
  theta_rf_ = 0.0;
  theta_lr_ = 0.0;
  theta_rr_ = 0.0;
  d_delta_ = 0.0;
  d_delta_lf_ = 0.0;
  d_delta_rf_ = 0.0;
  d_delta_lr_ = 0.0;
  d_delta_rr_ = 0.0;

  r.EnsureAccessedAllKeys();

  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw YAMLException("Body with name " + Q(body_name) + " does not exist");
  }

  front_left_wj_ = GetModel()->GetJoint(front_left_wj_name);
  if (front_left_wj_ == nullptr) {
    throw YAMLException("Joint with name " + Q(front_left_wj_name) + " does not exist");
  }

  front_right_wj_ = GetModel()->GetJoint(front_right_wj_name);
  if (front_right_wj_ == nullptr) {
    throw YAMLException("Joint with name " + Q(front_right_wj_name) + " does not exist");
  }

  rear_left_wj_ = GetModel()->GetJoint(rear_left_wj_name);
  if (rear_left_wj_ == nullptr) {
    throw YAMLException("Joint with name " + Q(rear_left_wj_name) + " does not exist");
  }

  rear_right_wj_ = GetModel()->GetJoint(rear_right_wj_name);
  if (rear_right_wj_ == nullptr) {
    throw YAMLException("Joint with name " + Q(rear_right_wj_name) + " does not exist");
  }

  ComputeJoints();

  twist_sub_ = nh_.subscribe(twist_topic, 1, &DackermannDrive::TwistCallback, this);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic, 1);
  ground_truth_pub_ = nh_.advertise<nav_msgs::Odometry>(ground_truth_topic, 1);

  ground_truth_msg_.header.frame_id = ground_truth_frame_id;
  ground_truth_msg_.child_frame_id = tf::resolve("", GetModel()->NameSpaceTF(body_->name_));

  ground_truth_msg_.twist.covariance.fill(0);
  ground_truth_msg_.pose.covariance.fill(0);
  odom_msg_ = ground_truth_msg_;

  for (unsigned int i = 0; i < 36; i++) {
    odom_msg_.twist.covariance[i] = odom_twist_covar[i];
    odom_msg_.pose.covariance[i] = odom_pose_covar[i];
  }

  random_device rd;
  rng_ = default_random_engine(rd());
  for (unsigned int i = 0; i < 3; i++) {
    noise_gen_[i] = normal_distribution<double>(0.0, sqrt(odom_pose_noise[i]));
  }

  for (unsigned int i = 0; i < 3; i++) {
    noise_gen_[i + 3] = normal_distribution<double>(0.0, sqrt(odom_twist_noise[i]));
  }

  ROS_DEBUG_NAMED("DackermannDrive", "Initialized with params body(%p %s) front_left_wj(%p %s) "
                                   "front_right_wj(%p %s) rear_left_wj(%p %s) "
                                   "rear_right_wj(%p %s) odom_frame_id(%s) twist_sub(%s) "
                                   "odom_pub(%s) ground_truth_pub(%s) odom_pose_noise({%f,%f,%f}) "
                                   "odom_twist_noise({%f,%f,%f}) pub_rate(%f)\n",
                  body_, body_->GetName().c_str(), front_left_wj_, front_left_wj_->GetName().c_str(),
                  front_right_wj_, front_right_wj_->GetName().c_str(), rear_left_wj_,
                  rear_left_wj_->GetName().c_str(), rear_right_wj_, rear_right_wj_->GetName().c_str(),
                  odom_frame_id.c_str(), twist_topic.c_str(), odom_topic.c_str(), ground_truth_topic.c_str(),
                  odom_pose_noise[0], odom_pose_noise[1], odom_pose_noise[2],
                  odom_twist_noise[0], odom_twist_noise[1], odom_twist_noise[2], pub_rate);
}
// 计算joint的位置和类型
void DackermannDrive::ComputeJoints() {
  auto get_anchor = [&](Joint* joint, bool* is_inverted = nullptr) {
    b2Vec2 wheel_anchor;
    b2Vec2 body_anchor;
    bool inv = false;

    if (joint->physics_joint_->GetBodyA()->GetUserData() == body_) {
      wheel_anchor = joint->physics_joint_->GetAnchorB();
      body_anchor = joint->physics_joint_->GetAnchorA();
    } else if (joint->physics_joint_->GetBodyB()->GetUserData() == body_) {
      wheel_anchor = joint->physics_joint_->GetAnchorA();
      body_anchor = joint->physics_joint_->GetAnchorB();
      inv = true;
    } else {
      throw YAMLException("Joint " + Q(joint->GetName()) + " does not anchor on body " + Q(body_->GetName()));
    }

    wheel_anchor = body_->physics_body_->GetLocalPoint(wheel_anchor);
    body_anchor = body_->physics_body_->GetLocalPoint(body_anchor);

    if (fabs(wheel_anchor.x) > 1e-5 || fabs(wheel_anchor.y) > 1e-5) {
      throw YAMLException("Joint " + Q(joint->GetName()) + " must be anchored at (0, 0) on the wheel");
    }

    if (is_inverted) {
      *is_inverted = inv;
    }

    return body_anchor;
  };

  // 检查四个关节是否都是转向关节
  if (front_left_wj_->physics_joint_->GetType() != e_revoluteJoint) {
    throw YAMLException("Front left wheel joint must be a revolute joint");
  }

  if (front_right_wj_->physics_joint_->GetType() != e_revoluteJoint) {
    throw YAMLException("Front right wheel joint must be a revolute joint");
  }

  if (rear_left_wj_->physics_joint_->GetType() != e_revoluteJoint) {
    throw YAMLException("Rear left wheel joint must be a revolute joint");
  }

  if (rear_right_wj_->physics_joint_->GetType() != e_revoluteJoint) {
    throw YAMLException("Rear right wheel joint must be a revolute joint");
  }

  // 设置转向关节的限制
  b2RevoluteJoint* j_left = dynamic_cast<b2RevoluteJoint*>(front_left_wj_->physics_joint_);
  j_left->EnableLimit(true);

  b2RevoluteJoint* j_right = dynamic_cast<b2RevoluteJoint*>(front_right_wj_->physics_joint_);
  j_right->EnableLimit(true);

  b2RevoluteJoint* j_rear_left = dynamic_cast<b2RevoluteJoint*>(rear_left_wj_->physics_joint_);
  j_rear_left->EnableLimit(true);

  b2RevoluteJoint* j_rear_right = dynamic_cast<b2RevoluteJoint*>(rear_right_wj_->physics_joint_);
  j_rear_right->EnableLimit(true);

  b2Vec2 front_left_anchor = get_anchor(front_left_wj_, &invert_steering_angle_lf_);
  b2Vec2 front_right_anchor = get_anchor(front_right_wj_, &invert_steering_angle_rf_);
  b2Vec2 rear_left_anchor = get_anchor(rear_left_wj_, &invert_steering_angle_lr_);
  b2Vec2 rear_right_anchor = get_anchor(rear_right_wj_, &invert_steering_angle_rr_);

  //计算前后轮的中心
  front_center_ = 0.5 * (front_left_anchor + front_right_anchor);
  rear_center_ = 0.5 * (rear_left_anchor + rear_right_anchor);

  // 验证车辆的几何约束
  double x1 = rear_left_anchor.x, y1 = rear_left_anchor.y,
         x2 = rear_right_anchor.x, y2 = rear_right_anchor.y,
         x3 = front_center_.x, y3 = front_center_.y;

  double k = ((y2 - y1) * (x3 - x1) - (x2 - x1) * (y3 - y1)) /
             ((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
  double x4 = x3 - k * (y2 - y1);
  double y4 = y3 + k * (x2 - x1);

  if (fabs(x4 - rear_center_.x) > 1e-5 || fabs(y4 - rear_center_.y) > 1e-5) {
    throw YAMLException(
        "The mid point between the rear wheel anchors on the body must equal "
        "the perpendicular intersection between the rear axle (line segment "
        "between rear anchors) and the front wheel anchor");
  }

  // 计算轴距和轮距
  axel_track_ = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  wheelbase_ = sqrt((x4 - x3) * (x4 - x3) + (y4 - y3) * (y4 - y3));
}

// 调用模型进行位置预测
void DackermannDrive::BeforePhysicsStep(const Timekeeper& timekeeper) {
  // 检查是否需要发布数据
  bool publish = update_timer_.CheckUpdate(timekeeper);

  // 获取物理世界中的车辆状态，包括位置和角度，这些数据由物理引擎提供
  b2Body* b2body = body_->physics_body_;
  b2Vec2 position = b2body->GetPosition();
  float angle = b2body->GetAngle();

  // 如果需要发布数据
  if (publish) {
    // 获取车辆的线速度和角速度
    b2Vec2 linear_vel_local = 
        b2body->GetLinearVelocityFromLocalPoint(b2Vec2(0, 0));
    float angular_vel = b2body->GetAngularVelocity();

    ground_truth_msg_.header.stamp = timekeeper.GetSimTime();
    ground_truth_msg_.pose.pose.position.x = position.x;
    ground_truth_msg_.pose.pose.position.y = position.y;
    ground_truth_msg_.pose.pose.position.z = 0;
    ground_truth_msg_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
    ground_truth_msg_.twist.twist.linear.x = linear_vel_local.x;
    ground_truth_msg_.twist.twist.linear.y = linear_vel_local.y;
    ground_truth_msg_.twist.twist.linear.z = 0;
    ground_truth_msg_.twist.twist.angular.x = 0;
    ground_truth_msg_.twist.twist.angular.y = 0;
    ground_truth_msg_.twist.twist.angular.z = angular_vel;

    // 复制地面真值到里程计消息odom_msg_中，并添加噪声。噪声是通过正态分布生成器产生，旨在模拟实际传感器数据的不精确性
    odom_msg_.header.stamp = timekeeper.GetSimTime();
    odom_msg_.pose.pose = ground_truth_msg_.pose.pose;
    odom_msg_.twist.twist = ground_truth_msg_.twist.twist;
    odom_msg_.pose.pose.position.x += noise_gen_[0](rng_);
    odom_msg_.pose.pose.position.y += noise_gen_[1](rng_);
    odom_msg_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle + noise_gen_[2](rng_));
    odom_msg_.twist.twist.linear.x += noise_gen_[3](rng_);
    odom_msg_.twist.twist.linear.y += noise_gen_[4](rng_);
    odom_msg_.twist.twist.angular.z += noise_gen_[5](rng_);

    // 发布里程计和地面真值消息
    ground_truth_pub_.publish(ground_truth_msg_);
    odom_pub_.publish(odom_msg_);

    // 发布tf变换
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = timekeeper.GetSimTime();
    // 因为在flantand中，所有的模型都是在map坐标系中的，所以这里的frame_id是map
    // 当使用建图算法时，这个frame_id是odom
    odom_tf.header.frame_id = "map";
    odom_tf.child_frame_id = "base";

    odom_tf.transform.translation.x = position.x;
    odom_tf.transform.translation.y = position.y;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = tf::createQuaternionMsgFromYaw(angle);

    tf_broadcaster_.sendTransform(odom_tf);
  }



  // 2. Update the tricycle physics based on the twist command
  //    This is a highly simplified kinematic approximation of the response
  //    of the steering and drive mechanisms.

  // Equations of motion for steering (kinematics only)
  // Let δ (delta)  = measured steering angle
        //  测量得到的转向角度
  //     δ_c        = commanded steering angle 
        //  目标转向角度

  // Then the kinematic discrete-time equations are (1st-order approximation):
  //   (1)  δ[t+1] = δ[t] + dδ[t+1] * dt          new steering angle
        // 新的转向角度是上一个时间步的转向角度加上新的转向速度乘以时间步长
  
  //   (2)  dδ[t+1] = dδ[t] + d2δ[t+1] * dt       new steering velocity
        // 新的转向速度是上一个时间步的转向速度加上新的转向加速度乘以时间步长

  //   (3)  d2δ[t+1] = (dδ_c[t+1] - dδ[t]) / dt   new steering acceleration
        // 新的转向加速度是下一时间步的目标转向速度与当前速度的差值除以时间步长

  //   (4)  dδ_c[t+1] = (δ_c[t+1] - δ[t]) / dt    new steering velocity command
        // 新的转向速度命令是下一个时间步的目标转向角度与当前角度的差值除以时间步长
  //          when it requires > 1 dt to reach δ[t+1]
  //        0.0
  //          otherwise
        //  如果可以在一个时间步内到达目标转向角度，那么新的转向速度命令为0
  // subject to:
  //   |δ[t]| <= max_steer_angle_
      //  受到最大转向角度的限制
  //   |dδ[t]| <= angular_dynamics_.velocity_limit_
        // 受到最大转向速度的限制
  //   |d2δ[t]| <= angular_dynamics_.acceleration_limit_ 
        // 受到最大转向加速度的限制


  // twist message contains the speed and angle of the front wheel
  // 从twist消息中获取目标转向角度
  delta_command_ = twist_msg_.angular.z;

  // 通过double-akeman运动学公式计算每个轮子的转向角度
  R = wheelbase_ / (2*tan(delta_command_));

  delta_command_lf_ = atan(wheelbase_ / (R + axel_track_ / 2));
  delta_command_rf_ = atan(wheelbase_ / (R - axel_track_ / 2));
  delta_command_lr_ = -atan(wheelbase_ / (R + axel_track_ / 2));
  delta_command_rr_ = -atan(wheelbase_ / (R - axel_track_ / 2));

  // 从之前的状态或传感器中读取当前的机器人朝向
  double theta = angle;                  // 当前机器人在地图坐标系中的角度
  double dt = timekeeper.GetStepSize();  // 时间步长

  // In the simulation, the equations of motion have to be computed backwards
  // (4) Update the new commanded steering velocity
      // 计算新的命令转向速度
  //     Note: Set target steer velocity = 0 rad/s to avoid overshooting, when
  //           it is possible to reach the commanded steering angle in 1 step

  // 首先设置目标转向速度为0，以避免过冲
  double d_delta_command = 0.0;
  double d_delta_command_lf = 0.0;
  double d_delta_command_rf = 0.0;
  double d_delta_command_lr = 0.0;
  double d_delta_command_rr = 0.0;

  // 根据最大角速度限制与目前的角速度，计算一步的最大转向角度
  double delta_max_one_step = d_delta_ * d_delta_ / 2 / angular_dynamics_.acceleration_limit_;
  double delta_max_one_step_lf = d_delta_lf_ * d_delta_lf_ / 2 / angular_dynamics_.acceleration_limit_;
  double delta_max_one_step_rf = d_delta_rf_ * d_delta_rf_ / 2 / angular_dynamics_.acceleration_limit_;
  double delta_max_one_step_lr = d_delta_lr_ * d_delta_lr_ / 2 / angular_dynamics_.acceleration_limit_;
  double delta_max_one_step_rr = d_delta_rr_ * d_delta_rr_ / 2 / angular_dynamics_.acceleration_limit_;

  // 如果转向加速度限制为0，直接设置最大转向该变量为目标与当前转向角度的差值，这种情况下即为忽略了加速度因素
  if (angular_dynamics_.acceleration_limit_ == 0.0) {
    delta_max_one_step = fabs(delta_command_ - theta_f_);
  }
  if (angular_dynamics_.acceleration_limit_ == 0.0) {
    delta_max_one_step_lf = fabs(delta_command_lf_ - theta_lf_);
  }
  if (angular_dynamics_.acceleration_limit_ == 0.0) {
    delta_max_one_step_rf = fabs(delta_command_rf_ - theta_rf_);
  }
  if (angular_dynamics_.acceleration_limit_ == 0.0) {
    delta_max_one_step_lr = fabs(delta_command_lr_ - theta_lr_);
  }
  if (angular_dynamics_.acceleration_limit_ == 0.0) {
    delta_max_one_step_rr = fabs(delta_command_rr_ - theta_rr_);
  }
  if (angular_dynamics_.acceleration_limit_ == 0.0) {
    delta_max_one_step_lf = fabs(delta_command_lf_ - theta_lf_);
  }
  if (angular_dynamics_.acceleration_limit_ == 0.0) {
    delta_max_one_step_rf = fabs(delta_command_rf_ - theta_rf_);
  }

  // 如果目标转向角与当前转向角的差值大于一步之内可以改变的最大量，则根据时间步长计算新的命令转向速度
  // 这里计算的只是目标转向速度
  if (fabs(delta_command_ - theta_f_) >= delta_max_one_step) {
    d_delta_command = (delta_command_ - theta_f_) / dt;
  }
  if (fabs(delta_command_lf_ - theta_lf_) >= delta_max_one_step_lf) {
    d_delta_command_lf = (delta_command_lf_ - theta_lf_) / dt;
  }
  if (fabs(delta_command_rf_ - theta_rf_) >= delta_max_one_step_rf) {
    d_delta_command_rf = (delta_command_rf_ - theta_rf_) / dt;
  }
  if (fabs(delta_command_lr_ - theta_lr_) >= delta_max_one_step_lr) {
    d_delta_command_lr = (delta_command_lr_ - theta_lr_) / dt;
  }
  if (fabs(delta_command_rr_ - theta_rr_) >= delta_max_one_step_rr) {
    d_delta_command_rr = (delta_command_rr_ - theta_rr_) / dt;
  }

  // Apply angular dynamics constraints
  // 将计算出的命令转向速度通过动力学限制调整，保证不超过速度和加速度限制
  d_delta_ = angular_dynamics_.Limit(d_delta_, d_delta_command, dt);
  d_delta_lf_ = angular_dynamics_.Limit(d_delta_lf_, d_delta_command_lf, dt);
  d_delta_rf_ = angular_dynamics_.Limit(d_delta_rf_, d_delta_command_rf, dt);
  d_delta_lr_ = angular_dynamics_.Limit(d_delta_lr_, d_delta_command_lr, dt);
  d_delta_rr_ = angular_dynamics_.Limit(d_delta_rr_, d_delta_command_rr, dt);

  // (1) Update the new steering angle
  // 使用更新后的转向速度更新前轮的转向角度
  theta_f_ += d_delta_ * dt;
  theta_lf_ += d_delta_lf_ * dt;
  theta_rf_ += d_delta_rf_ * dt;
  theta_lr_ += d_delta_lr_ * dt;
  theta_rr_ += d_delta_rr_ * dt;

  // 如果存在最大的转向角度限制，确保新的转向角度不超过这个限制
  if (max_steer_angle_ != 0.0) {
    theta_f_ = DynamicsLimits::Saturate(theta_f_, -max_steer_angle_, max_steer_angle_);
  }
  if (max_steer_angle_ != 0.0) {
    theta_lf_ = DynamicsLimits::Saturate(theta_lf_, -max_steer_angle_, max_steer_angle_);
  }
  if (max_steer_angle_ != 0.0) {
    theta_rf_ = DynamicsLimits::Saturate(theta_rf_, -max_steer_angle_, max_steer_angle_);
  }
  if (max_steer_angle_ != 0.0) {
    theta_lr_ = DynamicsLimits::Saturate(theta_lr_, -max_steer_angle_, max_steer_angle_);
  }
  if (max_steer_angle_ != 0.0) {
    theta_rr_ = DynamicsLimits::Saturate(theta_rr_, -max_steer_angle_, max_steer_angle_);
  }

  // 获取四个轮子的转向关节
  // 这个部分主要是用于可视化，将转向角度应用到物理关节上
  b2RevoluteJoint* j_left = dynamic_cast<b2RevoluteJoint*>(front_left_wj_->physics_joint_);
  j_left->EnableLimit(true);
  if (invert_steering_angle_lf_) {
    j_left->SetLimits(-theta_lf_, -theta_lf_);
  } else {
    j_left->SetLimits(theta_lf_, theta_lf_);
  }

  b2RevoluteJoint* j_right = dynamic_cast<b2RevoluteJoint*>(front_right_wj_->physics_joint_);
  j_right->EnableLimit(true);
  if (invert_steering_angle_rf_) {
    j_right->SetLimits(-theta_rf_, -theta_rf_);
  } else {
    j_right->SetLimits(theta_rf_, theta_rf_);
  }

  b2RevoluteJoint* j_rear_left = dynamic_cast<b2RevoluteJoint*>(rear_left_wj_->physics_joint_);
  j_rear_left->EnableLimit(true);
  if (invert_steering_angle_lr_) {
    j_rear_left->SetLimits(-theta_lr_, -theta_lr_);
  } else {
    j_rear_left->SetLimits(theta_lr_, theta_lr_);
  }

  b2RevoluteJoint* j_rear_right = dynamic_cast<b2RevoluteJoint*>(rear_right_wj_->physics_joint_);
  j_rear_right->EnableLimit(true);
  if (invert_steering_angle_rr_) {
    j_rear_right->SetLimits(-theta_rr_, -theta_rr_);
  } else {
    j_rear_right->SetLimits(theta_rr_, theta_rr_);
  }

  double theta_r_ = theta_f_;

  // 计算前后轮的线速度
  double v_f_ = linear_dynamics_.Limit(v_f_, twist_msg_.linear.x, dt);
  double v_r_ = v_f_; // 前后轮线速度相等

  // 计算前轮的线速度分量
  double v_f_x = v_f_ * cos(theta_f_) * cos(theta); // 前轮x方向速度
  double v_f_y = v_f_ * cos(theta_f_) * sin(theta); // 前轮y方向速度

  // 计算后轮的线速度分量
  double v_r_x = v_r_ * cos(theta_r_) * cos(theta); // 后轮x方向速度
  double v_r_y = v_r_ * cos(theta_r_) * sin(theta); // 后轮y方向速度

  // 计算前轮的角速度
  double w_f = v_f_ * sin(theta_f_) / wheelbase_;

  // 计算后轮的角速度
  double w_r = v_r_ * sin(theta_r_) / wheelbase_;

  // 合成前后轮的线速度
  double v_x = (v_f_x + v_r_x) / 2; // 取平均值
  double v_y = (v_f_y + v_r_y) / 2; // 取平均值

  // 合成前后轮的角速度
  double w = (w_f + w_r) / 2; // 取平均值

  // 计算轴距中心
  b2Vec2 front_center(0.8, 0); // 前轮中心的坐标
  b2Vec2 rear_center(-0.8, 0); // 后轮中心的坐标
  b2Vec2 axle_center((front_center.x + rear_center.x) * 0.5f, (front_center.y + rear_center.y) * 0.5f); // 轴距中心的坐标

  // 计算质心速度
  b2Vec2 linear_vel(v_x, v_y);

  // r 是从轴距中心到质心的向量（世界坐标系）
  // 这里选择轴距中心是因为目前的模型是双轮驱动(前后都可转向)，所以轴距中心是一个合适的参考点
  b2Vec2 r = b2body->GetWorldCenter() - b2body->GetWorldPoint(axle_center);
  b2Vec2 linear_vel_cm = linear_vel + w * b2Vec2(-r.y, r.x);

  b2body->SetLinearVelocity(linear_vel_cm);
  b2body->SetAngularVelocity(w);
}


void DackermannDrive::TwistCallback(const geometry_msgs::Twist& msg) {
  twist_msg_ = msg;
}

}

PLUGINLIB_EXPORT_CLASS(flatland_plugins::DackermannDrive,
                       flatland_server::ModelPlugin)