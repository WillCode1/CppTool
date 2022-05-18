#include "front_camera_extrinsic_calib_base.h"
#include <cstdio>
#include <cstdlib>
#include <sys/stat.h>
#include <fstream>

using namespace nullmax_perception;

void PrintHelper()
{
  std::cout << "Helper: \n";
  std::cout << "usage: [config file path] [saveimage] [verify] [undistort]\n";
  std::cout << "  [config file path] input config file path \n";
  std::cout << "  [saveimage] if input saveimage then write undistort image in the work folder; default is not write \n";
  std::cout << "  [verify] according rotation angle to generate birdview without calibrate. \n";
  std::cout << "  [undistort] just do image undistortion without calibrate. \n";

}

void LoadOcamModelParam(const std::string &str_config, OcamModel &ocam_model_)
{
  std::ifstream input_file;
  std::string s;
  input_file.open(str_config.c_str(), std::ifstream::in);
  getline(input_file, s); // first line
  getline(input_file, s); // second line
  getline(input_file, s); // third line
  std::stringstream ss(s);
  std::string strlength_pol;
  ss >> strlength_pol;

  ocam_model_.length_pol = atoi(strlength_pol.c_str());

  ocam_model_.pol.resize(ocam_model_.length_pol);
  for (int i = 0; i < ocam_model_.length_pol; i++) {
    std::string coef;
    ss >> coef;
    ocam_model_.pol[i] = atof(coef.c_str());
  }

  getline(input_file, s); // 4th
  getline(input_file, s); // 5th
  getline(input_file, s); // 6th
  getline(input_file, s); // 7th
  std::stringstream ssinv(s);
  std::string strlength_invpol;
  ssinv >> strlength_invpol;
  ocam_model_.length_invpol = atoi(strlength_invpol.c_str());
  ocam_model_.invpol.resize(ocam_model_.length_invpol);
  for (int i = 0; i < ocam_model_.length_invpol; i++) {
    std::string coef;
    ssinv >> coef;
    ocam_model_.invpol[i] = atof(coef.c_str());
  }

  getline(input_file, s); // 8th
  getline(input_file, s); // 9th
  getline(input_file, s); // 10th
  getline(input_file, s); // 11th

  std::stringstream uv(s);
  std::string stry;
  std::string strx;

  uv >> stry;
  uv >> strx;
  ocam_model_.yc = atof(stry.c_str());
  ocam_model_.xc = atof(strx.c_str());

  getline(input_file, s); // 12th
  getline(input_file, s); // 13th
  getline(input_file, s); // 14th
  getline(input_file, s); // 15th

  std::stringstream affinecoef(s);
  std::string strc;
  std::string strd;
  std::string stre;
  affinecoef >> strc;
  affinecoef >> strd;
  affinecoef >> stre;

  ocam_model_.c = atof(strc.c_str());
  ocam_model_.d = atof(strd.c_str());
  ocam_model_.e = atof(stre.c_str());

  getline(input_file, s); // 16th
  getline(input_file, s); // 17th
  getline(input_file, s); // 18th
  getline(input_file, s); // 19th

  std::stringstream imagesize(s);
  std::string strwidth;
  std::string strheight;
  imagesize >> strheight;
  imagesize >> strwidth;
  ocam_model_.height = atoi(strheight.c_str());
  ocam_model_.width = atoi(strwidth.c_str());
  return;
}

void LoadParameters(const std::string &str_config, std::string &data_path,
                    CameraModel &camera_model, PinholeModel &pinhole_model, float &camera_height,
                    RoiData &roi_data, int &edge_threadhold, CameraRotationEuler &angler, std::vector<std::pair<int, int>> &image_sequences)
{
  cv::FileStorage settings(str_config, cv::FileStorage::READ);
  pinhole_model.intrinsic.fx = settings["Camera_fx"];
  pinhole_model.intrinsic.fy = settings["Camera_fy"];
  pinhole_model.intrinsic.cx = settings["Camera_cx"];
  pinhole_model.intrinsic.cy = settings["Camera_cy"];
  pinhole_model.distort.k1 = settings["Camera_k1"];
  pinhole_model.distort.k2 = settings["Camera_k2"];
  pinhole_model.distort.p1 = settings["Camera_p1"];
  pinhole_model.distort.p2 = settings["Camera_p2"];
  pinhole_model.distort.k3 = settings["Camera_k3"];
  camera_height = settings["Camera_height"];
  camera_model = (CameraModel)(int)settings["Camera_model"];
  data_path = std::string(settings["Calibration_Image_Paths"]);
  edge_threadhold = settings["Edge_threadhold"];
  for(auto image_sequence: settings["Image_Sequences"]){
    image_sequences.push_back(std::make_pair(image_sequence[0], image_sequence[1]));
  }

  std::cout << "------carameter parameters------" << std::endl;
  std::cout << " -Camera fx " << pinhole_model.intrinsic.fx << std::endl;
  std::cout << " -Camera fy " << pinhole_model.intrinsic.fy << std::endl;
  std::cout << " -Camera cx " << pinhole_model.intrinsic.cx << std::endl;
  std::cout << " -Camera cy " << pinhole_model.intrinsic.cy << std::endl;
  std::cout << " -Camera k1 " << pinhole_model.distort.k1 << std::endl;
  std::cout << " -Camera k2 " << pinhole_model.distort.k2 << std::endl;
  std::cout << " -Camera p1 " << pinhole_model.distort.p1 << std::endl;
  std::cout << " -Camera p2 " << pinhole_model.distort.p2 << std::endl;
  std::cout << " -Camera k3 " << pinhole_model.distort.k3 << std::endl;
  std::cout << " -Camera height  " << camera_height << std::endl;
  std::cout << " -Camera model  " << camera_model << std::endl;
  std::cout << " -Data Path " << data_path << std::endl;
  std::cout << " -Edge threadhold " << edge_threadhold << std::endl;
  std::cout << " -Image_Sequences " << "\n";
  for(auto image_sequence: image_sequences){
    std::cout << image_sequence.first << ", "<< image_sequence.second << "\n";
  }
  std::cout << "                     " << std::endl;

  if (camera_model == FISHEYE_OCAM_MODEL) {
    pinhole_model.intrinsic_undistort.fx = settings["Camera_undistort_fx"];
    pinhole_model.intrinsic_undistort.fy = settings["Camera_undistort_fy"];
    pinhole_model.intrinsic_undistort.cx = settings["Camera_undistort_cx"];
    pinhole_model.intrinsic_undistort.cy = settings["Camera_undistort_cy"];
    std::cout << " -Camera undistort fx " << pinhole_model.intrinsic_undistort.fx << std::endl;
    std::cout << " -Camera undistort fy " << pinhole_model.intrinsic_undistort.fy << std::endl;
    std::cout << " -Camera undistort cx " << pinhole_model.intrinsic_undistort.cx << std::endl;
    std::cout << " -Camera undistort cy " << pinhole_model.intrinsic_undistort.cy << std::endl;
    std::cout << "                     " << std::endl;
  }

  roi_data.width_top = settings["Roi_width_top"];
  roi_data.width_bottom = settings["Roi_width_bottom"];
  roi_data.height_min = settings["Roi_height_min"];
  roi_data.height_max = settings["Roi_height_max"];

  std::cout << "------roi parameters------" << std::endl;
  std::cout << " -roi width_top :" << roi_data.width_top << std::endl;
  std::cout << " -roi width_bottom :" << roi_data.width_bottom << std::endl;
  std::cout << " -roi height_min :" << roi_data.height_min << std::endl;
  std::cout << " -roi height_max :" << roi_data.height_max << std::endl;
  std::cout << "                     " << std::endl;

  angler.pitch = settings["Angle_pitch"];
  angler.yaw = settings["Angle_yaw"];  
  angler.roll = settings["Angle_roll"];
  std::cout << "------angler parameters raw [deg]------" << std::endl;
  std::cout << " -Angle pitch :" << angler.pitch << std::endl;
  std::cout << " -Angle yaw :" << angler.yaw << std::endl;
  std::cout << " -Angle roll :" << angler.roll << std::endl;
  std::cout << "                     " << std::endl;
}

void WriteParameters(const std::string &str_config,
                     CameraParam &camera_param,
                     const CameraRotationEuler &rotation_angle)
{
  const CameraModelParam &param = camera_param.getCameraModelParam();
  cv::FileStorage settings(str_config, cv::FileStorage::WRITE);
  // undistort
  settings << "Camera_undistort_fx" << param.pinhole.intrinsic_undistort.fx;
  settings << "Camera_undistort_fy" << param.pinhole.intrinsic_undistort.fy;
  settings << "Camera_undistort_cx" << param.pinhole.intrinsic_undistort.cx;
  settings << "Camera_undistort_cy" << param.pinhole.intrinsic_undistort.cy;
  // extrinsic (unit:deg)
  settings << "Angle_pitch" << rotation_angle.pitch * 180.0 / CV_PI;
  settings << "Angle_yaw" << rotation_angle.yaw * 180.0 / CV_PI;
  settings << "Angle_roll" << rotation_angle.roll * 180.0 / CV_PI;
}

inline bool HasFile(const std::string& file_path) {
  struct stat buffer;   
  return (stat (file_path.c_str(), &buffer) == 0); 
}

inline std::string GetImageExtension(std::string data_path){
  return data_path.substr(data_path.find_last_of('.'));
}

inline std::string GetImageDirectory(std::string data_path){
 return data_path.substr(0, data_path.find_last_of("/")+1); 
}


inline std::string GetImagePrefix(std::string data_path){
  std::string image_prefix =  data_path.substr(data_path.find_last_of('/')+1);
  return image_prefix.substr(0, image_prefix.find_last_of("*"));
}

inline bool CopyFile(const std::string& src_path, const std::string& dst_path){
  std::ifstream src(src_path, std::ios::binary);
  if(src.good() == false)
    return false;
  std::ofstream dst(dst_path, std::ios::binary);
  if(dst.good() == false)
    return false;
  dst << src.rdbuf();
  return true;
}

std::vector<std::string> GetImagePaths(const std::string& data_path, const std::vector<std::pair<int, int>>& image_sequences){
  const std::string image_directory = GetImageDirectory(data_path);
  const std::string image_extenstion = GetImageExtension(data_path);
  const std::string image_prefix = GetImagePrefix(data_path);
  std::vector<std::string> image_paths;
  std::string image_path;
  for(auto image_sequence: image_sequences){
    for(int i=image_sequence.first; i<=image_sequence.second; ++i){
      std::string image_id = std::to_string(i);
      if(i < 10)
        image_id = "0" + image_id;
      std::string image_path = image_directory + image_prefix + image_id + image_extenstion;
      if(HasFile(image_path))
        image_paths.push_back(image_path);
      else
        std::cout << "Unable to find image " << image_path << "\n";
    }
  }
  return image_paths;
}

bool LaunchAdjustmentProcess(std::string config_path, std::string para_data_path_result){
  std::cout << "Would you like to adjust the the rotation(especially Row) of the extrinsic(y/n)?\n";
  char response;
  std::cin >> response;
  if(response == 'y' or response== 'Y'){
    std::string launch_command = "./front-camera-extrinsic-adjustment-sample " + config_path + " " + para_data_path_result;
    std::cout << "$" << launch_command << std::endl;
    std::system(launch_command.c_str());
    return true;
  }
  return false;
}

bool BackupImages(std::vector<std::string> image_paths, const std::string& output_directory){
  std::cout << "Do you wish to backup the input images(y/n)? There are " << image_paths.size() << " images\n";
  char response;
  std::cin >> response;
  if(response == 'y' or response== 'Y'){
    std::string sys_command = "mkdir -p " + output_directory;
    std::cout << "$" << sys_command << std::endl;
    std::system(sys_command.c_str());
    for(auto src_path: image_paths){
      std::string dst_path = output_directory + src_path.substr(src_path.find_last_of("/")+1);
      if(CopyFile(src_path, dst_path) == false)
          std::cout << "Failed to copy " << src_path << " to " << dst_path << "\n";
    }
    return true;
  }
  return false;
}

int main(int argc, char **argv) {
  PrintHelper();
  if (argc < 2) {
    std::cout << " ARGS ERROR " << std::endl;
    std::cout << " ARGS : config_file " << std::endl;
    return 1;
  }

  //-----------------------------
  // read config
  //-----------------------------
  bool is_write_undistort_img = false;
  bool just_birdview = false;
  bool just_undistort = false;
  for (size_t i = 2; i < argc; i++) {
    if (std::string(argv[i]) == "saveimage") {
      is_write_undistort_img = true;
    }
    else if (std::string(argv[i]) == "verify") {
      just_birdview = true;
    }
    else if (std::string(argv[i]) == "undistort") {
      just_undistort = true;
    }
  }
  std::string config_path = (argv[1]);
  std::string para_data_path = config_path;
  std::string ocam_para_data_path = config_path;
  std::string para_data_path_result = config_path;
  std::string image_backup_dir = config_path;
  para_data_path.append("/config.yaml");
  ocam_para_data_path.append("/calib_results_fisheye.txt");
  para_data_path_result.append("/calibration_result.yaml");
  image_backup_dir.append("/picked_images/");

  CameraIntrinsic intrisic;
  CameraDistortCoef distort_coef;
  CameraModel camera_model;
  PinholeModel pinhole_model;
  RoiData roi_data;
  CameraRotationEuler rotation_angle_raw;
  std::string data_path;
  float camera_height;
  int edge_threadhold;
  std::vector<std::pair<int, int>> image_sequences;
  LoadParameters(para_data_path, data_path, camera_model, pinhole_model,
                 camera_height, roi_data, edge_threadhold, rotation_angle_raw, image_sequences);

  cv::String images_path = cv::String(data_path);
  std::vector<std::string> file_names = GetImagePaths(data_path, image_sequences);
  //Abort when none of the images can be found
  if(file_names.size() == 0){
    std::cout << "Unable to find any images under " << data_path<< "\nWith sequence:\n";
    for(auto image_sequence: image_sequences)
      std::cout << image_sequence.first<< " - " <<image_sequence.second << "\n"; 
    std::cout << "Please check the sequence or input directory\n";
    return -1;
  }

  // std::vector<cv::String> file_names;
  // cv::glob(images_path, file_names, true); // recursive, if you want

  //-----------------------------
  // init
  //-----------------------------
  CameraParam *camera_param = NULL;
  CameraModelParam camera_model_param;
  camera_model_param.pinhole = pinhole_model;
  if (CameraModel::FISHEYE_OCAM_MODEL == camera_model) {
    OcamModel ocam_model;
    LoadOcamModelParam(ocam_para_data_path, ocam_model);
    camera_model_param.ocam=ocam_model;
    camera_param=CreateCameraParamHandle(camera_model,camera_model_param);
  }
  else {
    camera_param=CreateCameraParamHandle(camera_model,camera_model_param);
  }

  FrontCameraExtrinsicCalibBase *front_camera_extrinsic_calib = CreateFrontCameraExtrinsicCalibHandle("UseLane");
  if (front_camera_extrinsic_calib == NULL) {
    std::cout << " CreateFrontCameraExtrinsicCalibHandle failed. " << std::endl;
    return 1;
  }
  front_camera_extrinsic_calib->InitCameraParam(camera_param, camera_height);

  //-----------------------------
  // just verify use birdview
  //-----------------------------
  if (just_birdview) {
    std::cout << " Generate Birdview " << std::endl;
    rotation_angle_raw.pitch = rotation_angle_raw.pitch * CV_PI / 180.0;
    rotation_angle_raw.yaw = rotation_angle_raw.yaw * CV_PI / 180.0;
    rotation_angle_raw.roll = rotation_angle_raw.roll * CV_PI / 180.0;
    cv::Mat bird_view;
    cv::Mat image_raw;
    for (size_t i = 0; i < file_names.size(); i++) {
      cv::Mat img = cv::imread(file_names[i], cv::IMREAD_GRAYSCALE);
      if (img.empty()){
        std::cout << "Unable to read image " << file_names[i] << "\n";
        continue;
      }

      cv::Mat image_data_undistort;
      front_camera_extrinsic_calib->UndistortImage(img, image_data_undistort);
      front_camera_extrinsic_calib->BirdviewGenerator(rotation_angle_raw, image_data_undistort, bird_view);
      cv::imshow("birdview", bird_view);
      cv::waitKey(5);
    }
    cv::destroyWindow("birdview");
    DestroyFrontCameraExtrinsicCalibHandle(front_camera_extrinsic_calib);
    DestroyCameraParamHandle(camera_param);
    cv::destroyAllWindows();
    return 0;
  }

  //-----------------------------
  // calibration
  //-----------------------------
  std::vector<std::vector<LaneCoef>> lane_coefs;
  std::vector<VanishPoint> vanish_point;
  std::vector<float> pitch_raw;
  for (size_t i = 0; i < file_names.size(); i++) {
    cv::Mat img = cv::imread(file_names[i], cv::IMREAD_GRAYSCALE);
    if (img.empty())
      continue;

    cv::Mat image_data_undistort;
    front_camera_extrinsic_calib->UndistortImage(img, image_data_undistort);

    if (is_write_undistort_img) {
      std::string save_image_path = "";
      save_image_path += file_names[i].substr(file_names[i].find_last_of("/") + 1, file_names[i].size());
      bool imwrite_ok = cv::imwrite(save_image_path, image_data_undistort);
      std::cout << "image save path [" << imwrite_ok << "]:" << save_image_path << std::endl;
    }

    if (just_undistort) {
      cv::waitKey(5);
      cv::Mat image_data_undistort_resize;
      cv::resize(image_data_undistort, image_data_undistort_resize,
                 cv::Size(image_data_undistort.cols / 2, image_data_undistort.rows / 2));
      cv::imshow("image_undistort", image_data_undistort_resize);
      continue;
    }

    VanishPoint vanish_point_tmp;
    std::vector<LaneCoef> lane_coefs_each_image;
    float pitch_raw_tmp;
    bool detect_ok = front_camera_extrinsic_calib->DetectLane(image_data_undistort,
                                                             roi_data, edge_threadhold,
                                                             lane_coefs_each_image,
                                                             vanish_point_tmp,
                                                             pitch_raw_tmp);
    if(detect_ok) {
      lane_coefs.push_back(lane_coefs_each_image);
      vanish_point.push_back(vanish_point_tmp);
      pitch_raw.push_back(pitch_raw_tmp);
    }else {
      std::cout << "failed to detect lane: index "<<i << std::endl;
      continue;
    }
    
    cv::Mat image_lane;
    cv::cvtColor(image_data_undistort, image_lane, CV_GRAY2RGB);
    // draw lane
    for (size_t j = 0; j < lane_coefs_each_image.size(); j++) {
      if (lane_coefs_each_image[j].valid) {
        const int ky0 = 310;
        const int ky1 = 1280;
        float x0 = (ky0 - lane_coefs_each_image[j].coef_b) / lane_coefs_each_image[j].coef_k;
        cv::Point2f pt5(x0, ky0);
        float x1 = (ky1 - lane_coefs_each_image[j].coef_b) / lane_coefs_each_image[j].coef_k;
        cv::Point2f pt6(x1, ky1);
        cv::line(image_lane, pt5, pt6, cv::Scalar(255, 255, 0), 1);
      }
    }

    // draw vanish point
    if (vanish_point[i].valid) {
      cv::circle(image_lane, cv::Point2f(vanish_point[i].x, vanish_point[i].y),
                 3, cv::Scalar(0, 0, 255), 3);
      cv::circle(image_lane, cv::Point2f(pinhole_model.intrinsic.cx, pinhole_model.intrinsic.cy), 2,
                 cv::Scalar(0, 255, 0), 2);

      cv::putText(image_lane, "img_index:" + std::to_string(i), cv::Point(30, 30),
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
    }
    cv::resize(image_lane, image_lane, cv::Size(image_lane.cols / 2, image_lane.rows / 2));
    cv::imshow("lane_detect", image_lane);
    cv::waitKey(5);
  }
  cv::destroyWindow("lane_detect");
  if (just_undistort) {
    DestroyFrontCameraExtrinsicCalibHandle(front_camera_extrinsic_calib);
    DestroyCameraParamHandle(camera_param);
    cv::destroyAllWindows();
    return 0;
  }

  if (lane_coefs.size() == 0) {
    std::cout << " Can not detect lane " << std::endl;
    return 1;
  }
  std::cout << " Calculate Front Camera Extrinsic " << std::endl;

  CameraRotationEuler rotation_angle;
  front_camera_extrinsic_calib->CalcFrontCameraExtrinsic(
      lane_coefs,
      vanish_point,
      pitch_raw,
      rotation_angle);
  std::cout << " result [rad] " << std::endl;
  std::cout << " pitch " << rotation_angle.pitch << std::endl;
  std::cout << " yaw " << rotation_angle.yaw << std::endl;
  std::cout << " roll " << rotation_angle.roll << " not calculate " << std::endl;
  if (CameraModel::FISHEYE_OCAM_MODEL == camera_param->getCameraModelType()) {
    const PinholeModel &pinhole_model_new = camera_param->getCameraModelParam().pinhole;
    std::cout << " Ocam model convert to pinhole \n";
    std::cout << " fx: " << pinhole_model_new.intrinsic_undistort.fx << std::endl;
    std::cout << " fy: " << pinhole_model_new.intrinsic_undistort.fy << std::endl;
    std::cout << " cx: " << pinhole_model_new.intrinsic_undistort.cx << std::endl;
    std::cout << " cy: " << pinhole_model_new.intrinsic_undistort.cy << std::endl;
  }
  WriteParameters(para_data_path_result, *camera_param, rotation_angle);

  //-----------------------------
  // vertify by birdview
  //-----------------------------
  std::cout << " Generate Birdview " << std::endl;
  cv::Mat bird_view;
  cv::Mat image_raw;
  for (size_t i = 0; i < file_names.size(); i++) {
    cv::Mat img = cv::imread(file_names[i], cv::IMREAD_GRAYSCALE);
    if (img.empty())
      continue;

    cv::Mat image_data_undistort;
    front_camera_extrinsic_calib->UndistortImage(img, image_data_undistort);

    front_camera_extrinsic_calib->BirdviewGenerator(rotation_angle, image_data_undistort, bird_view);
    cv::cvtColor(image_data_undistort, image_raw, CV_GRAY2RGB);
    if (vanish_point[i].valid) {
      float v = vanish_point[i].y;
      cv::line(image_raw, cv::Point2f(0, v), cv::Point2f(1280, v),
               cv::Scalar(0, 255, 0));
      cv::resize(image_raw, image_raw, cv::Size(640, 480));
      char pitch_info[100];
      sprintf(pitch_info, "current_pitch : %f", rotation_angle.pitch);
      cv::putText(image_raw, pitch_info, cv::Point(5, 20), cv::FONT_HERSHEY_PLAIN, 1,
                  cv::Scalar(0, 255, 0), 1, 8);

      char horizon_line[100];
      sprintf(horizon_line, "HORIZON LINE");
      cv::putText(image_raw, horizon_line, cv::Point2f(0, v / 2),
                  cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 1, 8);
    }
    
    cv::imshow("raw", image_raw);
    cv::imshow("birdview", bird_view);
    cv::waitKey(5);
  }
  cv::destroyWindow("raw");
  cv::destroyWindow("birdview");
  DestroyFrontCameraExtrinsicCalibHandle(front_camera_extrinsic_calib);
  DestroyCameraParamHandle(camera_param);
  cv::destroyAllWindows();

  LaunchAdjustmentProcess(config_path, para_data_path_result);
  BackupImages(file_names, image_backup_dir);

  return 0;
}
