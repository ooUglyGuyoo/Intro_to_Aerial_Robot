#include "estimator.h"


void drawImage(const cv::Mat& img, const vector<cv::Point2f>& pts, string name) {
  auto draw = img.clone();
  for (unsigned int i = 0; i < pts.size(); i++) {
    cv::circle(draw, pts[i], 2, cv::Scalar(0, 255, 0), -1, 8);
  }
  cv::imshow(name, draw);
  cv::waitKey(1);
}

// From VINS-mono: https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/feature_tracker/src/feature_tracker.cpp
vector<uchar> rejectWithF(vector<cv::Point2f> &ud1_pts, vector<cv::Point2f> &ud2_pts){
  if(ud2_pts.size()>8){
    vector<uchar> status;
    cv::findFundamentalMat(ud1_pts, ud2_pts, cv::FM_RANSAC, 0.005, 0.99, status);
    return status;
  }
  return vector<uchar>();
}

Estimator::Estimator() {
  ROS_INFO("Estimator init begins.");
  prev_frame.frame_time = ros::Time(0.0);
  prev_frame.w_t_c = Eigen::Vector3d(0, 0, 0);
  prev_frame.w_R_c = Eigen::Matrix3d::Identity();
  fail_cnt = 0;
  init_finish = false;
}

void Estimator::reset() {
  ROS_ERROR("Lost, reset!");
  key_frame = prev_frame;
  fail_cnt = 0;
  init_finish = false;
}

void Estimator::setParameter() {
  for (int i = 0; i < 2; i++) {
    tic[i] = TIC[i];
    ric[i] = RIC[i];
    cout << " exitrinsic cam " << i << endl << ric[i] << endl << tic[i].transpose() << endl;
  }

  prev_frame.frame_time = ros::Time(0.0);
  prev_frame.w_t_c = tic[0];
  prev_frame.w_R_c = ric[0];
  key_frame = prev_frame;

  readIntrinsicParameter(CAM_NAMES);

  // transform between left and right camera
  Matrix4d Tl, Tr;
  Tl.setIdentity();
  Tl.block(0, 0, 3, 3) = ric[0];
  Tl.block(0, 3, 3, 1) = tic[0];
  Tr.setIdentity();
  Tr.block(0, 0, 3, 3) = ric[1];
  Tr.block(0, 3, 3, 1) = tic[1];
  Tlr = Tl.inverse() * Tr;
}


void Estimator::readIntrinsicParameter(const vector<string>& calib_file) {
  for (size_t i = 0; i < calib_file.size(); i++) {
    ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
    camodocal::CameraPtr camera =
        camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
    m_camera.push_back(camera);
  }
}

bool Estimator::inputImage(ros::Time time_stamp, const cv::Mat& _img, const cv::Mat& _img1) {

  if(fail_cnt > 20){
    reset();
  }
  std::cout << "     " << std::endl;
  std::cout << "receive new image===========================" << std::endl;

  Estimator::frame cur_frame;
  cur_frame.frame_time = time_stamp; 
  cur_frame.img = _img;

  // cv::imshow("img", _img);
  // cv::waitKey(1);

  vector<cv::Point2f> left_pts_2d, right_pts_2d;
   vector<cv::Point2f> undistort_left_pts_2d, undistort_right_pts_2d;
  vector<cv::Point3f> key_pts_3d; 
  vector<cv::Point2f> cur_pts_3d; 
  vector<cv::Point2f> cur_pts_2d; // trackFeatureBetweenFrames returns the 2d points of the current frame
  
  c_R_k.setIdentity();
  c_t_k.setZero();

  if (init_finish) {
    // FIXME: match features between the key frame and the current left image
    Estimator::trackFeatureBetweenFrames(key_frame, _img, key_pts_3d, cur_pts_2d);

    // FIXME: undistort the points of the left image and compute relative motion with the key frame. 
    Estimator::estimateTBetweenFrames(key_pts_3d, cur_pts_2d, c_R_k, c_t_k);
  }

  // FIXME: extract new features for the current frame.
  Estimator::extractNewFeatures(_img, left_pts_2d);
  
  // FIXME: compute the camera pose of the current frame.
  cur_frame.w_R_c = key_frame.w_R_c * c_R_k.transpose(); 
  cur_frame.w_t_c = key_frame.w_R_c * (-c_R_k.transpose() * c_t_k) + key_frame.w_t_c;

  // FIXME: undistort the 2d points of the current frame and generate the corresponding 3d points. 
  Estimator::trackFeatureLeftRight(_img, _img1, left_pts_2d, right_pts_2d);
  cur_frame.uv = left_pts_2d;
  undistort_left_pts_2d = Estimator::undistortedPts(left_pts_2d, m_camera[0]);
  undistort_right_pts_2d = Estimator::undistortedPts(right_pts_2d, m_camera[1]);
  cur_frame.xyz.clear(); // clear the 3d points of the current frame in order to store the new 3d points
  Estimator::generate3dPoints(undistort_left_pts_2d, undistort_right_pts_2d, cur_frame.xyz , cur_frame.uv);

  // cout << "xyz: " << key_frame.xyz << endl;

  // Change key frame
  if(c_t_k.norm() > TRANSLATION_THRESHOLD || acos(Quaterniond(c_R_k).w()) * 2.0 > ROTATION_THRESHOLD || key_pts_3d.size() < FEATURE_THRESHOLD || !init_finish){
    if (c_t_k.norm() > 0.1)
    {
      ROS_ERROR("Translation too large");
    }
    if (acos(Quaterniond(c_R_k).w()) * 2.0 > 0.1)
    {
      ROS_ERROR("Rotation too large");
    }
    if (key_pts_3d.size() < FEATURE_THRESHOLD)
    {
      ROS_ERROR("Feature too few");
    }

    key_frame = cur_frame;
    ROS_WARN("Change key frame to current frame.");
    cout << "c_t_k.norm(): " << c_t_k.norm() << endl;
    cout << "cal c_R_k: " << acos(Quaterniond(c_R_k).w()) * 2.0 << endl;
    cout << "key_pts_3d: "<< key_pts_3d.size() << endl;
  }
  prev_frame = cur_frame;
  
  updateLatestStates(cur_frame); // the states of the current frame is required to be updated in the updateLatest-States function
  
  init_finish = true;

  return true;
}

void Estimator::extractNewFeatures(const cv::Mat& img, vector<cv::Point2f>& uv) {
  
  // FIXME: extract the new 2d features of img and store them in uv. You may simply use the goodFeaturesToTrack function in OpenCV.

  // cv:goodFeaturesToTrack parameters
  int MaxCorners = 400;
  double QualityLevel = 0.03;
  double MinDistance = 12.0;
  int BlockSize = 15;
  bool UseHarrisDetector = false;
  double k = 0.04;
  // double Mask = ; double GradientSize = ;

  cv::goodFeaturesToTrack(img,uv,MaxCorners,QualityLevel,MinDistance,cv::Mat(),BlockSize,UseHarrisDetector,k);

  drawImage(img,uv,"new_features");

}

bool Estimator::trackFeatureLeftRight(const cv::Mat& _img, const cv::Mat& _img1,
                                         vector<cv::Point2f>& left_pts, vector<cv::Point2f>& right_pts) {

  // FIXME: track features left to right frame and obtain corresponding 2D points. You can track the features using the LK optical flow.
  // Refer to: https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html
  // Use calcOpticalFlowPyrLK() function in OpenCV.
  vector<uchar> status;
  vector<float> err;
  cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
  cv::calcOpticalFlowPyrLK(_img, _img1, left_pts, right_pts, status, err, cv::Size(40,40), 4, criteria);
  
  // delete the points that has status 0
  reduceVector(left_pts, status);
  reduceVector(right_pts, status);
  // drawImage(_img1, right_pts, "LK-trackFeatureLeftRight");

  return true;
}

bool Estimator::trackFeatureBetweenFrames(  const Estimator::frame& keyframe, const cv::Mat& cur_img,
                                            vector<cv::Point3f>& key_pts_3d,
                                            vector<cv::Point2f>& cur_pts_2d) {

  // FIXME: track features between the key frame and the current frame to obtain corresponding 2D, 3D points. You can track the features using the LK optical flow.
  // Refer to: https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html
  // Use calcOpticalFlowPyrLK() function in OpenCV.

  vector<uchar> status;
  vector<float> err;
  cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
  cv::calcOpticalFlowPyrLK(keyframe.img, cur_img, keyframe.uv, cur_pts_2d, status, err, cv::Size(17,17), 3, criteria);
  
  vector<cv::Point2f> key_pts_2d = key_frame.uv;  
  key_pts_3d = key_frame.xyz;

  reduceVector(key_pts_2d, status);
  reduceVector(key_pts_3d, status);
  reduceVector(cur_pts_2d, status);
  vector<cv::Point2f> undistort_key_pts_2d, undistort_cur_pts_2d;
  undistort_key_pts_2d = Estimator::undistortedPts(key_pts_2d, m_camera[0]);
  undistort_cur_pts_2d = Estimator::undistortedPts(cur_pts_2d, m_camera[0]);
  vector<uchar> ransacMask = rejectWithF(undistort_key_pts_2d, undistort_cur_pts_2d);
  reduceVector(key_pts_3d, ransacMask);
  reduceVector(cur_pts_2d, ransacMask);

  // drawImage(cur_img, cur_pts_2d, "LK-trackFeatureBetweenFrames");

  return true;
}

bool Estimator::estimateTBetweenFrames( vector<cv::Point3f>& key_pts_3d, vector<cv::Point2f>& cur_pts_2d, 
                                        Matrix3d& R, 
                                        Vector3d& t) {

  // FIXME: calculate relative pose between the key frame and the current frame using the matched 2d-3d points. You can implement it using the solvePnP or solvePnPRansac in OpenCV
  cout << "cur_pts_2d.size = " << cur_pts_2d.size() << endl;
  vector<cv::Point2f> undistort_cur_pts_2d;
  undistort_cur_pts_2d = Estimator::undistortedPts(cur_pts_2d, m_camera[0]);

  cv::Mat R_Matrix, R_vector, t_Matrix, distCoeffs;
  cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

  cv::solvePnPRansac(key_pts_3d, undistort_cur_pts_2d, cameraMatrix, distCoeffs, R_vector, t_Matrix, false, 10000, 1.0, 0.99, cv::noArray());

  cv::Rodrigues(R_vector, R_Matrix);

  cv::cv2eigen(R_Matrix, R);
  cv::cv2eigen(t_Matrix, t);

  // If can get camera model use the following code to get the cameraMatrix and distCoeffs
  // cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << m_camera[0].fx, 0, m_camera[0].cx, 0, m_camera[0].fy, m_camera[0].cy, 0, 0, 1);
  // cv::Mat distCoeffs = (cv::Mat_<double>(5,1) << m_camera[0].k1, m_camera[0].k2, m_camera[0].p1, m_camera[0].p2, m_camera[0].k3);
  // cv::solvePnPRansac(key_pts_3d, cur_pts_2d, cameraMatrix, distCoeffs, R, t, false, 100, 8.0, 0.99, cv::noArray(), cv::SOLVEPNP_ITERATIVE);

  // cout << "camera to keyframe Rotation matrix:\n" << R_Matrix << endl;
  // cout << "camera to keyframe Translation vector:\n" << t_Matrix << endl;

  cout << "Camera to keyframe Rotation matrix:\n" << R << endl;
  cout << "Camera to keyframe Translation vector:\n" << t << endl;

  return true;
}

void Estimator::updateLatestStates(frame &latest_frame) {
  
  // FIXME: update the latest_time, latest_pointcloud, latest_P, latest_Q, latest_rel_P and latest_rel_Q.
  // latest_P and latest_Q should be the pose of the body (IMU) in the world frame.
  // latest_rel_P and latest_rel_Q should be the relative pose of the current body frame relative to the body frame of the key frame.
  // latest_pointcloud should be in the current camera frame.

  latest_time = latest_frame.frame_time;
  latest_pointcloud = latest_frame.xyz;
  latest_P = latest_frame.w_R_c *(-ric[0].transpose()*tic[0]) + latest_frame.w_t_c;
  latest_Q = Eigen::Quaterniond(latest_frame.w_R_c * ric[0].transpose()).normalized();
  // latest_rel_P = latest_frame.w_t_c - key_frame.w_t_c;
  // latest_rel_Q = Eigen::Quaterniond(latest_frame.w_R_c * ric[0].transpose() * key_frame.w_R_c.transpose()).normalized();
}

void Estimator::generate3dPoints(const vector<cv::Point2f>& left_pts,
                                 const vector<cv::Point2f>& right_pts, 
                                 vector<cv::Point3f>& cur_pts_3d,
                                 vector<cv::Point2f>& cur_pts_2d) {

  Eigen::Matrix<double, 3, 4> P1, P2;

  P1 << 1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 1, 0;
  P2.block(0,0,3,3) = (Tlr.block(0,0,3,3).transpose());
  P2.block(0,3,3,1) = -P2.block(0,0,3,3) * Tlr.block(0,3,3,1);

  vector<uchar> status;

  for (unsigned int i = 0; i < left_pts.size(); ++i) {
    Vector2d pl(left_pts[i].x, left_pts[i].y);
    Vector2d pr(right_pts[i].x, right_pts[i].y);
    Vector3d pt3;
    triangulatePoint(P1, P2, pl, pr, pt3);

    if (pt3[2] > 0) {
      cur_pts_3d.push_back(cv::Point3f(pt3[0], pt3[1], pt3[2]));
      status.push_back(1);
    } else {
      status.push_back(0);
    }
  }

  reduceVector<cv::Point2f>(cur_pts_2d, status);
}


bool Estimator::inBorder(const cv::Point2f& pt, const int& row, const int& col) {
  const int BORDER_SIZE = 1;
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y &&
      img_y < row - BORDER_SIZE;
}


double Estimator::distance(cv::Point2f pt1, cv::Point2f pt2) {
  double dx = pt1.x - pt2.x;
  double dy = pt1.y - pt2.y;
  return sqrt(dx * dx + dy * dy);
}


template <typename Derived>
void Estimator::reduceVector(vector<Derived>& v, vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i]) v[j++] = v[i];
  v.resize(j);
}

void Estimator::triangulatePoint(Eigen::Matrix<double, 3, 4>& Pose0, Eigen::Matrix<double, 3, 4>& Pose1,
                                 Eigen::Vector2d& point0, Eigen::Vector2d& point1,
                                 Eigen::Vector3d& point_3d) {
  Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
  design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
  design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
  design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
  design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
  Eigen::Vector4d triangulated_point;
  triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
  point_3d(0) = triangulated_point(0) / triangulated_point(3);
  point_3d(1) = triangulated_point(1) / triangulated_point(3);
  point_3d(2) = triangulated_point(2) / triangulated_point(3);
}


double Estimator::reprojectionError(Matrix3d &R, Vector3d &t, cv::Point3f &key_pts_3d, cv::Point2f &cur_pts_2d){
    Vector3d pt1(key_pts_3d.x, key_pts_3d.y, key_pts_3d.z);
    Vector3d pt2 = R * pt1 + t;
    pt2 = pt2 / pt2[2];
    return sqrt(pow(pt2[0] - cur_pts_2d.x, 2) + pow(pt2[1] - cur_pts_2d.y, 2));
}


vector<cv::Point2f> Estimator::undistortedPts(vector<cv::Point2f>& pts, camodocal::CameraPtr cam) {
  vector<cv::Point2f> un_pts;
  for (unsigned int i = 0; i < pts.size(); i++) {
    Eigen::Vector2d a(pts[i].x, pts[i].y);
    Eigen::Vector3d b;
    cam->liftProjective(a, b);
    un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
  }
  return un_pts;
}