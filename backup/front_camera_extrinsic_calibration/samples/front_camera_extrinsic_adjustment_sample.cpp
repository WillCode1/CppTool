#include <cstdlib>
#include <cctype>
#include "front_camera_extrinsic_adjustment_base.h"

using namespace nullmax_perception;

void help() {
  std::cout << "this code support functions: " << std::endl;
  std::cout << "1. undistortion pinhole camera image" << std::endl;
  std::cout << "2. verify extrinsic parameters" << std::endl;
  std::cout << "Note: only support pinhole model" << std::endl;
  std::cout << "input format: ./front_camera_extrinsic_adjustment_sample [config path]"
            << std::endl;
  std::cout << "example:" << std::endl;
  std::cout
      << "     ./front_camera_extrinsic_adjustment_sample /home/user/proj/calibration/front_camera_extrinsic_adjustment/data"
      << std::endl;
}

void AddTextReminder(cv::Mat& image){
  std::vector<std::string> texts = {"helper(press 'q' to save result):", "Pitch: +[p]-[o]", "Roll:  +[r]-[e]", "Yaw:  +[y]-[t]"};
  for(int i=0; i<texts.size(); ++i){
    std::string text = texts[i];
    cv::Point text_position(5, 20+i*20);
    cv::putText(image,
            text,
            text_position, // Coordinates (Bottom-left corner of the text string in the image)
            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            0.8, // Scale. 2.0 = 2x bigger
            cv::Scalar(210,20,0), // BGR Color
            1, // Line Thickness (Optional)
            cv::LINE_AA); // Anti-alias (Optional, see version note)
  }
}

CameraRotationEuler LoadRotationParameter(const std::string &rotation_config){
  cv::FileStorage settings(rotation_config, cv::FileStorage::READ);
  CameraRotationEuler rotation_angle;
  rotation_angle.pitch = settings["Angle_pitch"];
  rotation_angle.yaw = settings["Angle_yaw"];
  rotation_angle.roll = settings["Angle_roll"];
  std::cout << " -Angle_pitch  " << rotation_angle.pitch << std::endl;
  std::cout << " -Angle_yaw  " << rotation_angle.yaw << std::endl;
  std::cout << " -Angle_roll  " << rotation_angle.roll << std::endl;
  return rotation_angle;
}

int LoadCamreaParameter(const std::string &str_config,
                        std::string &str_image_path, float &camera_height,
                        CameraIntrinsic &intrinsic, CameraDistortCoef &distort_coef,
                        CameraRotationEuler &rotation_angle, int &image_width, int &image_height,
                        nullmax_perception::UVIPMRoi &uv_ipm_roi)
{

  cv::FileStorage settings(str_config, cv::FileStorage::READ);
  CameraModel camera_model = (CameraModel)(int)settings["Camera_model"];
  if (camera_model == FISHEYE_OCAM_MODEL)
  {
    intrinsic.fx = settings["Camera_undistort_fx"];
    intrinsic.fy = settings["Camera_undistort_fy"];
    intrinsic.cx = settings["Camera_undistort_cx"];
    intrinsic.cy = settings["Camera_undistort_cy"];
    distort_coef.k1 = 0;
    distort_coef.k2 = 0;
    distort_coef.p1 = 0;
    distort_coef.p2 = 0;
    distort_coef.k3 = 0;
  }
  else
  {
    intrinsic.fx = settings["Camera_fx"];
    intrinsic.fy = settings["Camera_fy"];
    intrinsic.cx = settings["Camera_cx"];
    intrinsic.cy = settings["Camera_cy"];
    distort_coef.k1 = settings["Camera_k1"];
    distort_coef.k2 = settings["Camera_k2"];
    distort_coef.p1 = settings["Camera_p1"];
    distort_coef.p2 = settings["Camera_p2"];
    distort_coef.k3 = settings["Camera_k3"];
  }
  camera_height = settings["Camera_height"];
  image_width = settings["Image_width"];
  image_height = settings["Image_height"];
  rotation_angle.pitch = settings["Angle_pitch"];
  rotation_angle.yaw = settings["Angle_yaw"];
  rotation_angle.roll = settings["Angle_roll"];
  str_image_path = std::string(settings["Adjustment_Image_Path"]);

  std::cout << "------carameter parameters------" << std::endl;
  std::cout << " -Camera model " << camera_model << std::endl;
  std::cout << " -Camera fx " << intrinsic.fx << std::endl;
  std::cout << " -Camera fy " << intrinsic.fy << std::endl;
  std::cout << " -Camera cx " << intrinsic.cx << std::endl;
  std::cout << " -Camera cy " << intrinsic.cy << std::endl;
  std::cout << " -Camera k1 " << distort_coef.k1 << std::endl;
  std::cout << " -Camera k2 " << distort_coef.k2 << std::endl;
  std::cout << " -Camera p1 " << distort_coef.p1 << std::endl;
  std::cout << " -Camera p2 " << distort_coef.p2 << std::endl;
  std::cout << " -Camera k3 " << distort_coef.k3 << std::endl;
  std::cout << "                     " << std::endl;
  std::cout << " -Camera height  " << camera_height << std::endl;
  std::cout << " -Angle_pitch  " << rotation_angle.pitch << std::endl;
  std::cout << " -Angle_yaw  " << rotation_angle.yaw << std::endl;
  std::cout << " -Angle_roll  " << rotation_angle.roll << std::endl;
  std::cout << "                     " << std::endl;
  std::cout << " -Iamge Path " << str_image_path << std::endl;
  std::cout << "                     " << std::endl;

  uv_ipm_roi.left = settings["Roi_uv_left"];
  uv_ipm_roi.right = settings["Roi_uv_right"];
  uv_ipm_roi.top = settings["Roi_uv_top"];
  uv_ipm_roi.bottom = settings["Roi_uv_bottom"];

  std::cout << "------roi parameters------" << std::endl;
  std::cout << " -Roi_uv_left :" << uv_ipm_roi.left << std::endl;
  std::cout << " -Roi_uv_right :" << uv_ipm_roi.right << std::endl;
  std::cout << " -Roi_uv_top :" << uv_ipm_roi.top << std::endl;
  std::cout << " -Roi_uv_bottom :" << uv_ipm_roi.bottom << std::endl;
  std::cout << "                     " << std::endl;

  return 0;
}

int WriteParam(const std::string &str_config, const CameraRotationEuler &rotation_angle)
{
  cv::FileStorage settings(str_config, cv::FileStorage::WRITE);
  // extrinsic deg
  settings << "Angle_pitch" << rotation_angle.pitch;
  settings << "Angle_yaw" << rotation_angle.yaw;
  settings << "Angle_roll" << rotation_angle.roll;
}

int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cout << " ARGS ERROR " << std::endl;
    std::cout << " ARGS : not enough ARGS " << std::endl;
    return 1;
  }


  int err;
  std::string base_path = argv[1];
  std::string str_camera_info = base_path+"/config.yaml";
  std::string write_param_path = base_path+"/adjustment_result.yaml";
  float camera_height;
  CameraIntrinsic intrinsic;
  CameraDistortCoef distort_coef;
  CameraRotationEuler rotation_angle;
  int image_width, image_height;
  std::string str_image_path;
  UVIPMRoi uv_ipm_roi;
  err = LoadCamreaParameter(str_camera_info, str_image_path,
                            camera_height, intrinsic, distort_coef,
                            rotation_angle, image_width, image_height, uv_ipm_roi);
  if (err)
    return 1;

  if(argc >= 2){
    std::string rotation_file_path = base_path + "/calibration_result.yaml";
    std::cout << "Overloading rotation from " << rotation_file_path << std::endl;
    rotation_angle = LoadRotationParameter(rotation_file_path);
  }

  cv::Mat image_color = cv::imread(str_image_path, cv::IMREAD_COLOR);
  FrontCameraExtrinsicAdjustmentBase* front_camera_extrinsic_adjustment = CreateFrontCameraExtrinsicAdjustmentHandle("Manual");
  front_camera_extrinsic_adjustment->InitCameraParam(intrinsic, distort_coef, camera_height,image_width,image_height);

  cv::Mat image_undistort;
  front_camera_extrinsic_adjustment->UndistortImage(image_color, image_undistort);
  //cv::imshow("undistort image", image_undistort);
  //cv::waitKey(2);

  front_camera_extrinsic_adjustment->VerifyCameraInit(image_undistort, uv_ipm_roi, rotation_angle);
  char cmd_selected;
  cv::Mat image;
  const int width=640;
  const int height=480;
  std::cout << "helper(press 'q' to save result):\n"
            << " Pitch: +[p]-[o]\n"
            << " Roll:  +[r]-[e]\n"
            << " Yaw:   +[y]-[t]\n";
  do
  {
    front_camera_extrinsic_adjustment->VerifyCameraFlush(width, height, image);
    AddTextReminder(image);
    cv::imshow("image-uv-ipm", image);
    cmd_selected = std::tolower(cv::waitKey(0));
    std::cout << "response: " << cmd_selected << std::endl;
    
    if (cmd_selected=='p') {
      front_camera_extrinsic_adjustment->VerifyCameraAdjustPitch(true);
    }
    else if (cmd_selected=='r') {
      front_camera_extrinsic_adjustment->VerifyCameraAdjustRoll(true);
    }
    else if (cmd_selected=='y') {
      front_camera_extrinsic_adjustment->VerifyCameraAdjustYaw(true);
    }
    else if (cmd_selected=='o') {
      front_camera_extrinsic_adjustment->VerifyCameraAdjustPitch(false);
    }
    else if (cmd_selected=='e') {
      front_camera_extrinsic_adjustment->VerifyCameraAdjustRoll(false);
    }
    else if (cmd_selected=='t') {
      front_camera_extrinsic_adjustment->VerifyCameraAdjustYaw(false);
    }

  } while (cmd_selected !='q');
  const CameraRotationEuler &angle = front_camera_extrinsic_adjustment->GetRotationAngle();
  WriteParam(write_param_path, angle);

  DestroyFrontCameraExtrinsicAdjustmentHandle(front_camera_extrinsic_adjustment);
  cv::destroyAllWindows();
  return 0;
}
