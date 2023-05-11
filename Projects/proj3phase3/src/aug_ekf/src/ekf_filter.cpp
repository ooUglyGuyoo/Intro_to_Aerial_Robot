#include <ekf_filter.h>

namespace ekf_imu_vision {
EKFImuVision::EKFImuVision(/* args */) {}

EKFImuVision::~EKFImuVision() {}

void EKFImuVision::init(ros::NodeHandle &nh) {
  node_ = nh;

  /* ---------- parameter ---------- */
  Qt_.setZero();
  Rt1_.setZero();
  Rt2_.setZero();
  M_r_.setZero();
  M_a_.setZero();

  // addition and removel of augmented state
  
  // DONE: 
  // set M_a_ and M_r_
  M_r_.block<15, 15>(0, 0) = Mat15x15::Identity();
  M_a_.block<15, 15>(0, 0) = Mat15x15::Identity();
  M_a_.block<6, 6>(15, 0) = Mat6x6::Identity();


  for (int i = 0; i < 3; i++) {
    /* process noise */
    node_.param("aug_ekf/ng", Qt_(i, i), -1.0);
    node_.param("aug_ekf/na", Qt_(i + 3, i + 3), -1.0);
    node_.param("aug_ekf/nbg", Qt_(i + 6, i + 6), -1.0);
    node_.param("aug_ekf/nba", Qt_(i + 9, i + 9), -1.0);
    node_.param("aug_ekf/pnp_p", Rt1_(i, i), -1.0);
    node_.param("aug_ekf/pnp_q", Rt1_(i + 3, i + 3), -1.0);
    node_.param("aug_ekf/vo_pos", Rt2_(i, i), -1.0);
    node_.param("aug_ekf/vo_rot", Rt2_(i + 3, i + 3), -1.0);
  }

  init_        = false;

  for(int i = 0; i < 4; i++)
    latest_idx[i] = 0;

  /* ---------- subscribe and publish ---------- */
  imu_sub_ =
      node_.subscribe<sensor_msgs::Imu>("/djiros/imu", 100, &EKFImuVision::imuCallback, this);
  pnp_sub_     = node_.subscribe<nav_msgs::Odometry>("tag_odom", 10, &EKFImuVision::PnPCallback, this);
  // opti_tf_sub_ = node_.subscribe<geometry_msgs::PointStamped>("opti_tf_odom", 10,
                                                              // &EKFImuVision::opticalCallback, this);
  stereo_sub_  = node_.subscribe<stereo_vo::relative_pose>("/vo/Relative_pose", 10,
                                                          &EKFImuVision::stereoVOCallback, this);
  fuse_odom_pub_ = node_.advertise<nav_msgs::Odometry>("ekf_fused_odom", 10);
  path_pub_         = node_.advertise<nav_msgs::Path>("/aug_ekf/Path", 100);

  ros::Duration(0.5).sleep();

  ROS_INFO("Start ekf.");
}

// utility funtion copied from previous year students
// copy start
static inline double normalize_angle(double angle) {
  const double result = fmod(angle + M_PI, 2.0 * M_PI);
  if (result <= 0.0)
    return result + M_PI;
  return result - M_PI;
}

static inline void normalize_state(Vec21 &state) {
  state(3) = normalize_angle(state(3));
  state(4) = normalize_angle(state(4));
  state(5) = normalize_angle(state(5));

  state(18) = normalize_angle(state(18));
  state(19) = normalize_angle(state(19));
  state(20) = normalize_angle(state(20));
}
// copy end

void EKFImuVision::PnPCallback(const nav_msgs::OdometryConstPtr &msg) {

  // DONE:
  // construct a new state using the absolute measurement from marker PnP and process the new state

  // ROS_ERROR("----------------PnPCallback--------------------");

  bool pnp_lost = fabs(msg->pose.pose.position.x) < 1e-4 && fabs(msg->pose.pose.position.y) < 1e-4 &&
      fabs(msg->pose.pose.position.z) < 1e-4;
  if (pnp_lost) return;

  Mat3x3 R_w_b = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                     .toRotationMatrix();
  Vec3 t_w_b;
  t_w_b[0] = msg->pose.pose.position.x;
  t_w_b[1] = msg->pose.pose.position.y;
  t_w_b[2] = msg->pose.pose.position.z;

  AugState new_state;

  new_state.mean = Vec21::Zero();
  new_state.covariance = Mat21x21::Zero();
  new_state.time_stamp = msg->header.stamp;
  new_state.type = pnp;
  new_state.ut.head(3)  =     t_w_b;
  new_state.ut.segment(3, 3) = rotation2Euler(R_w_b);

  if (!processNewState(new_state, false)) {
    return;
  }

}

void EKFImuVision::stereoVOCallback(const stereo_vo::relative_poseConstPtr &msg) {

  // DONE:
  // label the previous keyframe
  // construct a new state using the relative measurement from VO and process the new state

  ROS_WARN_STREAM("----------------stereoVOCallback--------------------");

  // Rotation matrix from body to world
  Mat3x3 w_R_b;
  w_R_b = Eigen::Quaterniond( msg->relative_pose.orientation.w,
                              msg->relative_pose.orientation.x,
                              msg->relative_pose.orientation.y,
                              msg->relative_pose.orientation.z).toRotationMatrix();
  
  // Translation vector from body to world
  Vec3 w_t_b;
  w_t_b[0] = msg->relative_pose.position.x;
  w_t_b[1] = msg->relative_pose.position.y;
  w_t_b[2] = msg->relative_pose.position.z;

  AugState new_state;
  new_state.type = vo;
  new_state.time_stamp = msg->header.stamp;
  new_state.key_frame_time_stamp = msg->key_stamp;
  new_state.mean = Vec21::Zero();
  new_state.covariance = Mat21x21::Zero();
  new_state.ut.head(3) = w_t_b;
  new_state.ut.segment(3, 3) = rotation2Euler(w_R_b);
  if (!processNewState(new_state,latest_idx[keyframe] >= 0 
                                  && latest_idx[keyframe] < aug_state_hist_.size() 
                                  && new_state.key_frame_time_stamp != aug_state_hist_[latest_idx[keyframe]].time_stamp)) {
    return;
  }

}

void EKFImuVision::imuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {

  // DONE:
  // construct a new state using the IMU input and process the new state

  // ROS_ERROR("----------------IMU callback--------------------");

  // Rotation matrix from body to world
  Vec3 w_R_b;
  w_R_b[0] = imu_msg->angular_velocity.x;
  w_R_b[1] = imu_msg->angular_velocity.y;
  w_R_b[2] = imu_msg->angular_velocity.z;

  // Translation from body to world
  Vec3 w_t_b;
  w_t_b[0] = imu_msg->linear_acceleration.x;
  w_t_b[1] = imu_msg->linear_acceleration.y;
  w_t_b[2] = imu_msg->linear_acceleration.z;

  AugState new_state;
  new_state.mean = Vec21::Zero();
  new_state.covariance = Mat21x21::Zero();
  new_state.time_stamp = imu_msg->header.stamp;
  new_state.type = imu;
  new_state.ut.head(3) = w_t_b;
  new_state.ut.segment(3, 3) = w_R_b;
  if (!processNewState(new_state, false)) {
    return;
  }

}

void EKFImuVision::predictIMU(AugState &cur_state, AugState &prev_state, Vec6 ut) {

  // DONE:
  // predict by IMU inputs
  // mdoelF:    return the model xdot = f(x,u,n)
  // jacobiFx:  return the derivative wrt original state df/dx
  // jacobiFn:  return the derivative wrt noise df/dn

  double dt = (cur_state.time_stamp - prev_state.time_stamp).toSec();
  if (dt > 0.05)
  {
    ROS_ERROR_STREAM("dt is " << dt);
    cur_state.mean = prev_state.mean;
    cur_state.covariance = prev_state.covariance;
    return;
  }

  Vec15 state = prev_state.mean.head(15);
  Mat15x15 F = Mat15x15::Identity() + dt * jacobiFx(state, ut, Vec12::Zero());
  Mat15x12 V = dt * jacobiFn(state, ut, Vec12::Zero());

  cur_state.mean.head(15) = state + dt * modelF(state, ut, Vec12::Zero());
  cur_state.mean.tail(6) = prev_state.mean.tail(6);

  Mat15x15 covariance = M_r_ * prev_state.covariance * M_r_.transpose();
  cur_state.covariance.topLeftCorner(15, 15) = F * covariance * F.transpose() + V * Qt_ * V.transpose();
  cur_state.covariance.topRightCorner(15, 6) = F * prev_state.covariance.topRightCorner(15, 6);
  cur_state.covariance.bottomLeftCorner(6, 15) = prev_state.covariance.bottomLeftCorner(6,15) * F.transpose();
  cur_state.covariance.bottomRightCorner(6, 6) = prev_state.covariance.bottomRightCorner(6, 6);

  normalize_state(cur_state.mean);
}

void EKFImuVision::updatePnP(AugState &cur_state, AugState &prev_state) {

  // DONE: 
  // update by marker PnP measurements
  // modelG1:   return the model g(x,v), where x = x_origin
  // jacobiG1x: return the derivative wrt original state dz/dx, where x = x_origin
  // jacobiG1v: return the derivative wrt noise dz/dv
  
  Mat6x21 C;
  C << jacobiG1x(prev_state.mean.head(15), Vec6::Zero()), Mat6x6::Zero();
  Mat6x6 W = jacobiG1v(prev_state.mean.head(15), Vec6::Zero());

  Mat21x6 K = prev_state.covariance * C.transpose() * (C * prev_state.covariance * C.transpose() + W * Rt1_ * W.transpose()).inverse();
  Vec6 innovation = cur_state.ut - modelG1(prev_state.mean.head(15), Vec6::Zero());

  innovation(3) = normalize_angle(innovation(3));
  innovation(4) = normalize_angle(innovation(4));
  innovation(5) = normalize_angle(innovation(5));

  cur_state.mean = prev_state.mean + K * innovation;
  cur_state.covariance = prev_state.covariance - K * C * prev_state.covariance;

  normalize_state(cur_state.mean);
}

void EKFImuVision::updateVO(AugState &cur_state, AugState &prev_state) {

  // DONE:
  // update by relative pose measurements
  // modelG2:   return the model g(x,v), where x = (x_origin, x_augmented)
  // jacobiG2x: return the derivative wrt original state dz/dx, where x = (x_origin, x_augmented)
  // jacobiG2v: return the derivative wrt noise dz/dv

  Mat6x21 C = jacobiG2x(prev_state.mean, Vec6::Zero());
  Mat6x6 W = jacobiG2v(prev_state.mean, Vec6::Zero());

  Mat21x6 K = prev_state.covariance * C.transpose() * (C * prev_state.covariance * C.transpose() + W * Rt2_ * W.transpose()).inverse();
  Vec6 innovation = cur_state.ut - modelG2(prev_state.mean, Vec6::Zero());

  innovation(3) = normalize_angle(innovation(3));
  innovation(4) = normalize_angle(innovation(4));
  innovation(5) = normalize_angle(innovation(5));

  cur_state.mean = prev_state.mean + K * innovation;
  cur_state.covariance = prev_state.covariance - K * C * prev_state.covariance;

  normalize_state(cur_state.mean);

}

void EKFImuVision::changeAugmentedState(AugState &state) {
  ROS_WARN_STREAM("----------------change keyframe------------------------");

  // DONE: 
  // change augmented state
  // state.type = keyframe;
  state.mean = M_a_ * M_r_ * state.mean;
  state.covariance = M_a_ * M_r_ * state.covariance * M_r_.transpose() * M_a_.transpose();

}

bool EKFImuVision::processNewState(AugState &new_state, bool change_keyframe) {

  // DONE: 
  // process the new state
  deque<AugState>::iterator new_input_it = insertNewState(new_state);
  
  // step 1: insert the new state into the queue and get the iterator to start to propagate (be careful about the change of key frame).
  // check whether need to change keyframe
  if (change_keyframe) {
    while (new_input_it != aug_state_hist_.begin() && (new_input_it - 1)->time_stamp >= new_state.key_frame_time_stamp) {
      new_input_it--;
    }
    if (new_input_it -> type == vo)
    {
      latest_idx[keyframe] = new_input_it - aug_state_hist_.begin();
      changeAugmentedState(*new_input_it);
      new_input_it++;
    }
  } 

  // step 2: try to initialize the filter if it is not initialized.  
  // step 3: repropagate from the iterator you extracted.
  // step 4: remove the old states.
  // step 5: publish the latest fused odom

  if (!init_){
    if (initFilter())
    {
      ROS_INFO_STREAM("Initialized");
      repropagate(new_input_it, init_);
      init_ = true;
      removeOldState();
    }
    else
    {
      ROS_WARN_STREAM("Not initialized");
    }
  }
  else
  {
    repropagate(new_input_it, init_);
    removeOldState();
    publishFusedOdom();
  }
  return true;

}

deque<AugState>::iterator EKFImuVision::insertNewState(AugState &new_state){
  
  ros::Time time = new_state.time_stamp;
  deque<AugState>::iterator state_it;

  // DONE: 
  // insert the new state to the queue
  // update the latest_idx of the type of the new state
  // return the iterator point to the new state in the queue 

  state_it = aug_state_hist_.end();
  while (state_it != aug_state_hist_.begin() && (state_it - 1)->time_stamp > time)
  {
    state_it--;
  }
  state_it = aug_state_hist_.insert(state_it, new_state);
  if (new_state.type == imu)
  {
    latest_idx[imu] = state_it - aug_state_hist_.begin();
  }
  else if (new_state.type == pnp)
  {
    latest_idx[pnp] = state_it - aug_state_hist_.begin();
  }
  else if (new_state.type == vo)
  {
    latest_idx[vo] = state_it - aug_state_hist_.begin();
  }

  return state_it;

}

void EKFImuVision::repropagate(deque<AugState>::iterator &new_input_it, bool &init) {


  // DONE: 
  // repropagate along the queue from the new input according to the type of the inputs / measurements
  // remember to consider the initialization case  

  deque<AugState>::iterator state_it = new_input_it;
  while (state_it != aug_state_hist_.end()){
    if (state_it == aug_state_hist_.begin())
    {
      state_it++; continue;
    }
    if (state_it->type == imu)
    {
      predictIMU(*state_it, *(state_it-1), state_it->ut);
    }
    else if (state_it->type == pnp)
    {
      updatePnP(*state_it, *(state_it-1));
    }
    else if (state_it->type == vo)
    {
      updateVO(*state_it, *(state_it-1));
    }
    else if (state_it->type == keyframe)
    {
      updateVO(*state_it, *(state_it-1));
    }
    state_it++;
  }
}

void EKFImuVision::removeOldState() {

  // DONE: 
  // remove the unnecessary old states to prevent the queue from becoming too long

  unsigned int remove_idx = min( min(latest_idx[imu], latest_idx[pnp]), latest_idx[keyframe]);

  aug_state_hist_.erase(aug_state_hist_.begin(), aug_state_hist_.begin() + remove_idx);

  for(int i = 0; i < 4; i++){
    latest_idx[i] -= remove_idx;
  }
}

void EKFImuVision::publishFusedOdom() {

  AugState last_state = aug_state_hist_.back();

  // cout << "last state: " << aug_state_hist_.size() << endl;

  double phi, theta, psi;
  phi   = last_state.mean(3);
  theta = last_state.mean(4);
  psi   = last_state.mean(5);

  if (last_state.mean.head(3).norm() > 20) {
    ROS_ERROR_STREAM("error state: " << last_state.mean.head(3).transpose());
    return;
  }

  // using the zxy euler angle
  Eigen::Quaterniond q =  Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY());
  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.header.stamp    = last_state.time_stamp;

  odom.pose.pose.position.x = last_state.mean(0);
  odom.pose.pose.position.y = last_state.mean(1);
  odom.pose.pose.position.z = last_state.mean(2);

  odom.pose.pose.orientation.w = q.w();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();

  odom.twist.twist.linear.x = last_state.mean(6);
  odom.twist.twist.linear.y = last_state.mean(7);
  odom.twist.twist.linear.z = last_state.mean(8);


  fuse_odom_pub_.publish(odom);

  geometry_msgs::PoseStamped path_pose;
  path_pose.header.frame_id = path_.header.frame_id = "world";
  path_pose.pose.position.x = last_state.mean(0);
  path_pose.pose.position.y = last_state.mean(1);
  path_pose.pose.position.z = last_state.mean(2);
  path_.poses.push_back(path_pose);
  path_pub_.publish(path_);

  // ROS_WARN_STREAM("=========== Odometry Published ========");
}

bool EKFImuVision::initFilter() {

  // DONE: 
  // Initial the filter when a keyframe after marker PnP measurements is available

  ROS_WARN_STREAM(latest_idx[imu] << " " << latest_idx[pnp] << " " << latest_idx[vo] << " " << latest_idx[keyframe] << " " << aug_state_hist_.size() << " " << aug_state_hist_[latest_idx[pnp]].type << " " << pnp << " " << aug_state_hist_[latest_idx[keyframe]].type << " " << vo);

  if ((!aug_state_hist_.empty())
      && (latest_idx[keyframe] >= 0
          && latest_idx[keyframe] < aug_state_hist_.size() 
          && aug_state_hist_[latest_idx[keyframe]].type == vo)
      // && (latest_idx[pnp] >= 0
      //     && latest_idx[pnp] < aug_state_hist_.size()
      //     && aug_state_hist_[latest_idx[pnp]].type == pnp)
      )
  {
    deque<AugState>::iterator keyframe_it = aug_state_hist_.begin() + latest_idx[keyframe];
    if (initUsingPnP(keyframe_it)) 
    {
      changeAugmentedState(*keyframe_it);
      
      return true;
    }
  }
  return false;
}

bool EKFImuVision::initUsingPnP(deque<AugState>::iterator start_it) {

  // DONE: 
  // Initialize the absolute pose of the state in the queue using marker PnP measurement.
  // This is only step 1 of the initialization.
  
  start_it->mean = Vec21::Zero();
    start_it->covariance = Mat21x21::Identity();
    ROS_INFO_STREAM("init PnP state: " << start_it->mean.transpose());
    return true;

}

Vec3 EKFImuVision::rotation2Euler(const Mat3x3& R) {
  double phi   = asin(R(2, 1));
  double theta = atan2(-R(2, 0), R(2, 2));
  double psi   = atan2(-R(0, 1), R(1, 1));
  return Vec3(phi, theta, psi);
}

}  // namespace ekf_imu_vision