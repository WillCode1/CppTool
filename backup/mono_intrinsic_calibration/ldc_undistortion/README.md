# LDC Pinhole Camera Undistortion

## 功能
生成TDA4支持的前视针孔相机去畸变文件


## 脚本

pinhole_undistortion_mesh.py

## 运行

python ./script/gen_avp_label.py [img_path] [intrinsic_file_path] [output_file_path]

example:

python pinhole_undistortion_mesh.py frame_vc1_2844.bmp calib_results_tda4_mondeo1_left.txt output.txt

**[Note]**

脚本53 ~ 56行的相机参数可根据具体需求进行自定义调整