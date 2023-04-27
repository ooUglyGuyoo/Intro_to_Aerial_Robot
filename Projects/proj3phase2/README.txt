Use the launch file "aug_ekf/launch/augekf.launch" or "aug_ekf/launch/augekf_simple.launch" as the main entry point.

aug_ekf package: you should finish your augmented EKF here.

camera_models: models of stereo camera, the same as proj2phase2.

stereo_vo_estimator: VO estimator you implemented in proj2phase2. We only provide configuration files here. You need to copy paste your VO estimator here.

tag_detector: the aruco tag detector (PnP) you implemented in proj2phase1. We only provide configuration files here. You need to copy paste your tag detector here.

-----------------------------------------------------------------------------------------------------------
Tips:
1. The measurements from PnP and VO are handy in imu_pnp_vo.bag. You can have the correct measurements to finish this phase even if you did not complete the previous PnP or VO.

2. You should copy all required files into corresponding locations, whatever you use which bag.

Contact TAs with any questions you may have.

hwangeh@connect.ust.hk
pliuan@connect.ust.hk
