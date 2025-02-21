/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"

LIVMapper::LIVMapper(rclcpp::Node::SharedPtr nh)
    : extT(0, 0, 0),
      extR(M3D::Identity()),
      nh_(nh),tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(nh))
{
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);

  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess(nh));

  readParameters();
  VoxelMapConfig voxel_config;
  loadVoxelConfig(nh, voxel_config);

  visual_sub_map.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  // vio_manager.reset(new VIOManager());
  root_dir = ROOT_DIR;
  initializeFiles();
  initializeComponents();
  path.header.stamp = nh->now();
  path.header.frame_id = "camera_init";

  pubLaserCloudFullRes = nh->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_topic", 10);
  
}

LIVMapper::~LIVMapper() {}

void LIVMapper::readParameters()
{
  nh_->declare_parameter<string>("common/lid_topic", "/livox/lidar");
  nh_->get_parameter("common/lid_topic", lid_topic);
  nh_->declare_parameter<string>("common/imu_topic", "/livox/imu");
  nh_->get_parameter("common/imu_topic", imu_topic);
  nh_->declare_parameter<bool>("common/ros_driver_bug_fix", false);
  nh_->get_parameter("common/ros_driver_bug_fix", ros_driver_fix_en);
  // nh_->declare_parameter<int>("common/img_en", 1);
  // nh_->get_parameter("common/img_en", img_en);
  nh_->declare_parameter<int>("common/lidar_en", 1);
  nh_->get_parameter("common/lidar_en", lidar_en);

  nh_->declare_parameter<bool>("vio/exposure_estimate_en", true);
  nh_->get_parameter("vio/exposure_estimate_en", exposure_estimate_en);
  nh_->declare_parameter<double>("vio/inv_expo_cov", 0.2);
  nh_->get_parameter("vio/inv_expo_cov", inv_expo_cov);

  nh_->declare_parameter<double>("time_offset/exposure_time_init", 0.0);
  nh_->get_parameter("time_offset/exposure_time_init", exposure_time_init);
  nh_->declare_parameter<double>("time_offset/img_time_offset", 0.0);
  nh_->get_parameter("time_offset/img_time_offset", img_time_offset);
  nh_->declare_parameter<double>("time_offset/imu_time_offset", 0.0);
  nh_->get_parameter("time_offset/imu_time_offset", imu_time_offset);
  nh_->declare_parameter<bool>("uav/imu_rate_odom", false);
  nh_->get_parameter("uav/imu_rate_odom", imu_prop_enable);
  nh_->declare_parameter<bool>("uav/gravity_align_en", false);
  nh_->get_parameter("uav/gravity_align_en", gravity_align_en);

  nh_->declare_parameter<string>("evo/seq_name", "01");
  nh_->get_parameter("evo/seq_name", seq_name);
  nh_->declare_parameter<bool>("evo/pose_output_en", false);
  nh_->get_parameter("evo/pose_output_en", pose_output_en);
  nh_->declare_parameter<double>("imu/gyr_cov", 1.0);
  nh_->get_parameter("imu/gyr_cov", gyr_cov);
  nh_->declare_parameter<double>("imu/acc_cov", 1.0);
  nh_->get_parameter("imu/acc_cov", acc_cov);
  nh_->declare_parameter<int>("imu/imu_int_frame", 3);
  nh_->get_parameter("imu/imu_int_frame", imu_int_frame);
  nh_->declare_parameter<bool>("imu/imu_en", false);
  nh_->get_parameter("imu/imu_en", imu_en);
  nh_->declare_parameter<bool>("imu/gravity_est_en", true);
  nh_->get_parameter("imu/gravity_est_en", gravity_est_en);
  nh_->declare_parameter<bool>("imu/ba_bg_est_en", true);
  nh_->get_parameter("imu/ba_bg_est_en", ba_bg_est_en);

  nh_->declare_parameter<double>("preprocess/blind", 0.01);
  nh_->get_parameter("preprocess/blind", p_pre->blind);
  nh_->declare_parameter<double>("preprocess/filter_size_surf", 0.5);
  nh_->get_parameter("preprocess/filter_size_surf", filter_size_surf_min);
  nh_->declare_parameter<int>("preprocess/lidar_type", AVIA);
  nh_->get_parameter("preprocess/lidar_type", p_pre->lidar_type);
  nh_->declare_parameter<int>("preprocess/scan_line", 6);
  nh_->get_parameter("preprocess/scan_line", p_pre->N_SCANS);
  nh_->declare_parameter<int>("preprocess/point_filter_num", 3);
  nh_->get_parameter("preprocess/point_filter_num", p_pre->point_filter_num);
  nh_->declare_parameter<bool>("preprocess/feature_extract_enabled", false);
  nh_->get_parameter("preprocess/feature_extract_enabled", p_pre->feature_enabled);

  nh_->declare_parameter<int>("pcd_save/interval", -1);
  nh_->get_parameter("pcd_save/interval", pcd_save_interval);
  nh_->declare_parameter<bool>("pcd_save/pcd_save_en", false);
  nh_->get_parameter("pcd_save/pcd_save_en", pcd_save_en);
  nh_->declare_parameter<bool>("pcd_save/colmap_output_en", false);
  nh_->get_parameter("pcd_save/colmap_output_en", colmap_output_en);
  nh_->declare_parameter<double>("pcd_save/filter_size_pcd", 0.5);
  nh_->get_parameter("pcd_save/filter_size_pcd", filter_size_pcd);
  // nh.param<vector<double>>("extrin_calib/extrinsic_T", extrinT, vector<double>());
  // nh.param<vector<double>>("extrin_calib/extrinsic_R", extrinR, vector<double>());
  extrinT = vector<double>{0.04165, 0.02326, -0.0284};
  extrinR = vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1};
  nh_->declare_parameter<double>("debug/plot_time", -10);
  nh_->get_parameter("debug/plot_time", plot_time);
  nh_->declare_parameter<int>("debug/frame_cnt", 6);
  nh_->get_parameter("debug/frame_cnt", frame_cnt);

  nh_->declare_parameter<double>("publish/blind_rgb_points", 0.01);
  nh_->get_parameter("publish/blind_rgb_points", blind_rgb_points);
  nh_->declare_parameter<int>("publish/pub_scan_num", 1);
  nh_->get_parameter("publish/pub_scan_num", pub_scan_num);
  nh_->declare_parameter<bool>("publish/pub_effect_point_en", false);
  nh_->get_parameter("publish/pub_effect_point_en", pub_effect_point_en);
  nh_->declare_parameter<bool>("publish/dense_map_en", false);
  nh_->get_parameter("publish/dense_map_en", dense_map_en);

  p_pre->blind_sqr = p_pre->blind * p_pre->blind;
}

void LIVMapper::initializeComponents() 
{
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_imu_init_frame_num(imu_int_frame);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

void LIVMapper::initializeFiles() 
{
  if (pcd_save_en && colmap_output_en)
  {
      const std::string folderPath = std::string(ROOT_DIR) + "/scripts/colmap_output.sh";
      
      std::string chmodCommand = "chmod +x " + folderPath;
      
      int chmodRet = system(chmodCommand.c_str());  
      if (chmodRet != 0) {
          std::cerr << "Failed to set execute permissions for the script." << std::endl;
          return;
      }

      int executionRet = system(folderPath.c_str());
      if (executionRet != 0) {
          std::cerr << "Failed to execute the script." << std::endl;
          return;
      }
  }
  if(colmap_output_en) fout_points.open(std::string(ROOT_DIR) + "Log/Colmap/sparse/0/points3D.txt", std::ios::out);
  if(pcd_save_interval > 0) fout_pcd_pos.open(std::string(ROOT_DIR) + "Log/PCD/scans_pos.json", std::ios::out);
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
}

void LIVMapper::initializeSubscribersAndPublishers(rclcpp::Node::SharedPtr nh) 
{
  // sub_pcl = p_pre->lidar_type == AVIA ? 
  //           nh.subscribe(lid_topic, 200000, &LIVMapper::livox_pcl_cbk, this): 
  //           nh.subscribe(lid_topic, 200000, &LIVMapper::standard_pcl_cbk, this);
  // sub_imu = nh.subscribe(imu_topic, 200000, &LIVMapper::imu_cbk, this);

  sub_pcl = nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic,200000,
            std::bind(&LIVMapper::livox_pcl_cbk, this, std::placeholders::_1));
  sub_imu = nh->create_subscription< sensor_msgs::msg::Imu>(imu_topic,10,
            std::bind(&LIVMapper::imu_cbk, this, std::placeholders::_1));
  
  pubLaserCloudFullRes = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100);
  pubNormal = nh->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 100);
  pubSubVisualMap = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_visual_sub_map_before", 100);
  pubLaserCloudEffect = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100);
  pubLaserCloudMap = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100);
  pubOdomAftMapped = nh->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 10);
  pubPath = nh->create_publisher<nav_msgs::msg::Path>("/path", 10);
  plane_pub = nh->create_publisher<visualization_msgs::msg::Marker>("/planner_normal", 1);
  voxel_pub = nh->create_publisher<visualization_msgs::msg::MarkerArray>("/voxels", 1);
  pubLaserCloudDyn = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj", 100);
  pubLaserCloudDynRmed = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_removed", 100);
  pubLaserCloudDynDbg = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_dbg_hist", 100);
  mavros_pose_publisher = nh->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);

  pubImuPropOdom = nh->create_publisher<nav_msgs::msg::Odometry>("/LIVO2/imu_propagate", 10000);
  // imu_prop_timer = nh.createTimer(ros::Duration(0.004), &LIVMapper::imu_prop_callback, this);
  imu_prop_timer_ = nh->create_wall_timer(
            std::chrono::milliseconds(4),
            std::bind(&LIVMapper::imu_prop_callback, this));
  voxelmap_manager->voxel_map_pub_= nh->create_publisher<visualization_msgs::msg::MarkerArray>("/planes", 10000);
}

void LIVMapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment() 
{
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu() 
{
  // double t0 = omp_get_wtime();

  p_imu->Process2(LidarMeasures, _state, feats_undistort);

  if (gravity_align_en) gravityAlignment();

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping() 
{
  handleLIO();
}

void convertEulerToQuaternionMsg(const Eigen::Vector3d &euler_cur, geometry_msgs::msg::Quaternion &geoQuat_)
{
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(euler_cur(0), euler_cur(1), euler_cur(2)); // roll, pitch, yaw
    geoQuat_ = tf2::toMsg(quat_tf);
}

void LIVMapper::handleLIO() 
{    
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;
           
  if (feats_undistort->empty() || (feats_undistort == nullptr)) 
  {
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }

  double t0 = omp_get_wtime();

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);
  
  double t_down = omp_get_wtime();

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;
  
  if (!lidar_map_inited) 
  {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  double t1 = omp_get_wtime();

  voxelmap_manager->StateEstimation(state_propagat);
  _state = voxelmap_manager->state_;
  _pv_list = voxelmap_manager->pv_list_;

  double t2 = omp_get_wtime();

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  if (pose_output_en) 
  {
    static bool pos_opend = false;
    static int ocount = 0;
    std::ofstream outFile, evoFile;
    if (!pos_opend) 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::out);
      pos_opend = true;
      if (!evoFile.is_open()) RCLCPP_ERROR(nh_->get_logger(),"open fail\n");
    } 
    else 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::app);
      if (!evoFile.is_open()) RCLCPP_ERROR(nh_->get_logger(),"open fail\n");
    }
    Eigen::Matrix4d outT;
    Eigen::Quaterniond q(_state.rot_end);
    evoFile << std::fixed;
    evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  
  euler_cur = RotMtoEuler(_state.rot_end);
  convertEulerToQuaternionMsg(euler_cur,geoQuat);
  // geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
  publish_odometry();

  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) 
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  std::cout << "[ LIO ] Update Voxel Map" << std::endl;
  _pv_list = voxelmap_manager->pv_list_;
  
  double t4 = omp_get_wtime();

  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }
  
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  publish_frame_world();
  if (pub_effect_point_en) publish_effect_world(voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap(nh_);
  publish_path();
  publish_mavros();

  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
  //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m|                         LIO Mapping Time                    |\033[0m\n");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::savePCD() 
{
  if (pcd_save_en && pcl_wait_save->points.size() > 0 && pcd_save_interval < 0) 
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(pcl_wait_save);
    voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
    voxel_filter.filter(*downsampled_cloud);

    std::string raw_points_dir = std::string(ROOT_DIR) + "Log/PCD/all_raw_points.pcd";
    std::string downsampled_points_dir = std::string(ROOT_DIR) + "Log/PCD/all_downsampled_points.pcd";

    pcl::PCDWriter pcd_writer;

    // Save the raw point cloud data
    pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save);
    std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
              << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;

    // Save the downsampled point cloud data
    pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud);
    std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir 
          << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

    if(colmap_output_en)
    {
      fout_points << "# 3D point list with one line of data per point\n";
      fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
      for (size_t i = 0; i < downsampled_cloud->size(); ++i) 
      {
          const auto& point = downsampled_cloud->points[i];
          fout_points << i << " "
                      << std::fixed << std::setprecision(6)
                      << point.x << " " << point.y << " " << point.z << " "
                      << static_cast<int>(point.r) << " "
                      << static_cast<int>(point.g) << " "
                      << static_cast<int>(point.b) << " "
                      << 0 << std::endl;
      }
    }
  }
}

void LIVMapper::run() 
{
  rclcpp::Rate rate(5000);
  RCLCPP_INFO(nh_->get_logger(),"run run run");
  while (rclcpp::ok()) 
  {
    // RCLCPP_INFO(nh_->get_logger(),"lidar loop");
    rclcpp::spin_some(nh_);
    // RCLCPP_INFO(nh_->get_logger(),"lidar loop 1");
    if (!sync_packages(LidarMeasures)) 
    {
      rate.sleep();
      continue;
    }   
    handleFirstFrame();

    processImu();

    // if (!p_imu->imu_time_init) continue;

    stateEstimationAndMapping();

  }
  savePCD();
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback()
{
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once) { return; }
  mtx_buffer_imu_prop.lock();
  new_imu = false; // 控制propagate频率和IMU频率一致
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg)
    {
      imu_propagate = latest_ekf_state;
      // drop all useless imu pkg
      while ((!prop_imu_buffer.empty() && rclcpp::Time(prop_imu_buffer.front().header.stamp).seconds() < latest_ekf_time))
      {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (int i = 0; i < prop_imu_buffer.size(); i++)
      {
        double t_from_lidar_end_time = rclcpp::Time(prop_imu_buffer[i].header.stamp).seconds() - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        // cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].linear_acceleration.y, prop_imu_buffer[i].linear_acceleration.z);
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z);
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    }
    else
    {
      V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z);
      V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
      double t_from_lidar_end_time = rclcpp::Time(newest_imu.header.stamp).seconds() - latest_ekf_time;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate.pos_end;
    vel_i = imu_propagate.vel_end;
    q = Eigen::Quaterniond(imu_propagate.rot_end);
    imu_prop_odom.header.frame_id = "world";
    imu_prop_odom.header.stamp = newest_imu.header.stamp;
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    pubImuPropOdom->publish(imu_prop_odom);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

// void LIVMapper::standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstPtr &msg)
// {
//   if (!lidar_en) return;
//   mtx_buffer.lock();
//   // cout<<"got feature"<<endl;
//   if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar)
//   {
//     RCLCPP_ERROR(nh_->get_logger(),"lidar loop back, clear buffer");
//     lid_raw_data_buffer.clear();
//   }
//   // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
//   PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
//   p_pre->process(msg, ptr);
//   lid_raw_data_buffer.push_back(ptr);
//   lid_header_time_buffer.push_back(rclcpp::Time(msg->header.stamp).seconds());
//   last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();
//   mtx_buffer.unlock();
//   sig_buffer.notify_all();
// }

void LIVMapper::livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg_in)
{
  if (!lidar_en) return;
  mtx_buffer.lock();
  livox_ros_driver2::msg::CustomMsg::Ptr msg(new livox_ros_driver2::msg::CustomMsg(*msg_in));
  // if ((abs(msg->header.stamp.toSec() - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("lidar jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_lidar);
  //   sync_jump_flag = true;
  //   msg->header.stamp = ros::Time().fromSec(last_timestamp_lidar + 0.1);
  // }
  if (abs(last_timestamp_imu - rclcpp::Time(msg->header.stamp).seconds()) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - rclcpp::Time(msg->header.stamp).seconds();
    printf("\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  double cur_head_time = rclcpp::Time(msg->header.stamp).seconds();
  // RCLCPP_INFO(nh_->get_logger(),"Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    RCLCPP_ERROR(nh_->get_logger(),"lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->empty()) {
    RCLCPP_ERROR(nh_->get_logger(),"Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::imu_cbk(const sensor_msgs::msg::Imu::SharedPtr msg_in)
{
  if (!imu_en) return;

  if (last_timestamp_lidar < 0.0) return;
  // RCLCPP_INFO(nh_->get_logger(),"get imu at time: %.6f", rclcpp::Time(msg_in->header.stamp).seconds());
  sensor_msgs::msg::Imu::Ptr msg(new sensor_msgs::msg::Imu(*msg_in));

  rclcpp::Time msg_stamp(static_cast<int64_t>((rclcpp::Time(msg->header.stamp).seconds() - imu_time_offset) * 1e9));
  msg->header.stamp = msg_stamp;
  double timestamp = rclcpp::Time(msg->header.stamp).seconds();

  if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    RCLCPP_WARN(nh_->get_logger(),"IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en) timestamp += std::round(last_timestamp_lidar - timestamp);
  rclcpp::Time time_from_sec(static_cast<int64_t>(timestamp * 1e9));
  msg->header.stamp = time_from_sec;

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    RCLCPP_ERROR(nh_->get_logger(),"imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
    return;
  }

  // if (last_timestamp_imu > 0.0 && timestamp > last_timestamp_imu + 0.2)
  // {

  //   ROS_WARN("imu time stamp Jumps %0.4lf seconds \n", timestamp - last_timestamp_imu);
  //   mtx_buffer.unlock();
  //   sig_buffer.notify_all();
  //   return;
  // }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (lid_raw_data_buffer.empty() && lidar_en) return false;
  // if (img_buffer.empty() && img_en) return false;
  if (imu_buffer.empty() && imu_en) return false;


  if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
  if (!lidar_pushed)
  {
    // If not push the lidar into measurement data buffer
    meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
    if (meas.lidar->points.size() <= 1) return false;

    meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
    meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
    meas.pcl_proc_cur = meas.lidar;
    lidar_pushed = true;                                                                                       // flag
  }

  if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
  { // waiting imu message needs to be
    // larger than _lidar_frame_end_time,
    // make sure complete propagate.
    // ROS_ERROR("out sync");
    return false;
  }

  struct MeasureGroup m; // standard method to keep imu message.

  m.imu.clear();
  m.lio_time = meas.lidar_frame_end_time;
  mtx_buffer.lock();
  while (!imu_buffer.empty())
  {
    if (rclcpp::Time(imu_buffer.front()->header.stamp).seconds() > meas.lidar_frame_end_time) break;
    m.imu.push_back(imu_buffer.front());
    imu_buffer.pop_front();
  }
  lid_raw_data_buffer.pop_front();
  lid_header_time_buffer.pop_front();
  mtx_buffer.unlock();
  sig_buffer.notify_all();

  meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
  meas.measures.push_back(m);
  // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
  lidar_pushed = false; // sync one whole lidar scan.
  return true;

  RCLCPP_ERROR(nh_->get_logger(),"out sync");
}

void LIVMapper::publish_frame_world()
{
  if (pcl_w_wait_pub->empty()) return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());

  /*** Publish Frame ***/
  sensor_msgs::msg::PointCloud2 laserCloudmsg;

  pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg); 

  laserCloudmsg.header.stamp = nh_->now(); //.fromSec(last_timestamp_lidar);
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes->publish(laserCloudmsg);

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  if (pcd_save_en)
  {
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
    static int scan_wait_num = 0;

    *pcl_wait_save += *laserCloudWorldRGB;
    scan_wait_num++;

    if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      pcd_index++;
      string all_points_dir(string(string(ROOT_DIR) + "Log/PCD/") + to_string(pcd_index) + string(".pcd"));
      pcl::PCDWriter pcd_writer;
      if (pcd_save_en)
      {
        cout << "current scan saved to /PCD/" << all_points_dir << endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
        PointCloudXYZRGB().swap(*pcl_wait_save);
        Eigen::Quaterniond q(_state.rot_end);
        fout_pcd_pos << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.w() << " " << q.x() << " " << q.y()
                     << " " << q.z() << " " << endl;
        scan_wait_num = 0;
      }
    }
  }
  if(laserCloudWorldRGB->size() > 0) 
  {
    PointCloudXYZI().swap(*pcl_wait_pub); 
    PointCloudXYZRGB().swap(*laserCloudWorldRGB);
  }
  PointCloudXYZI().swap(*pcl_w_wait_pub);
}

void LIVMapper::publish_visual_sub_map()
{
  PointCloudXYZI::Ptr laserCloudFullRes(visual_sub_map);
  int size = laserCloudFullRes->points.size(); if (size == 0) return;
  PointCloudXYZI::Ptr sub_pcl_visual_map_pub(new PointCloudXYZI());
  *sub_pcl_visual_map_pub = *laserCloudFullRes;
  if (1)
  {
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = nh_->now();
    laserCloudmsg.header.frame_id = "camera_init";
    pubSubVisualMap->publish(laserCloudmsg);
  }
}

void LIVMapper::publish_effect_world(const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = nh_->now();
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudEffect->publish(laserCloudFullRes3);
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry()
{
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "aft_mapped";
  odomAftMapped.header.stamp = nh_->now(); //.ros::Time()fromSec(last_timestamp_lidar);
  set_posestamp(odomAftMapped.pose.pose);

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = odomAftMapped.header.stamp;
  transform_stamped.header.frame_id = "camera_init";
  transform_stamped.child_frame_id = "aft_mapped";
  transform_stamped.transform.translation.x = _state.pos_end(0);
  transform_stamped.transform.translation.y = _state.pos_end(1);
  transform_stamped.transform.translation.z = _state.pos_end(2);
  transform_stamped.transform.rotation.w = geoQuat.w;
  transform_stamped.transform.rotation.x = geoQuat.x;
  transform_stamped.transform.rotation.y = geoQuat.y;
  transform_stamped.transform.rotation.z = geoQuat.z;

  tf_broadcaster_->sendTransform(transform_stamped);

  pubOdomAftMapped->publish(odomAftMapped);
}

void LIVMapper::publish_mavros()
{
  msg_body_pose.header.stamp = nh_->now();
  msg_body_pose.header.frame_id = "camera_init";
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher->publish(msg_body_pose);
}

void LIVMapper::publish_path()
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = nh_->now();
  msg_body_pose.header.frame_id = "camera_init";
  path.poses.push_back(msg_body_pose);
  pubPath->publish(path);
}