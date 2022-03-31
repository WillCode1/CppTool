python associate.py rgb.txt depth.txt > associations.txt
python evaluate_ate.py groundtruth.txt CameraTrajectory.txt --verbose --plot ate.png
python evaluate_rpe.py groundtruth.txt CameraTrajectory.txt --verbose --fixed_delta --plot rpe.png


