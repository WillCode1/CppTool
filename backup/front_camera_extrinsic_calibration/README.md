# Front camera calibration using lane marking and manual adjustment

## 1.Project description
 The project is to estimate the pitch and yaw parameters of the front camera in AVM(around view monitor).

 AVM system has four cameras located at the centers of the front and rear bumpers and under two side view mirrors.

 Generally speaking camera calibration can be categorized into three approches:calibration pattern-based, interest point-based, and lane marking-based, this project selects lane marking-based approch to estimate these parameters.

## 2.Manage
 The original version is created by "mjlin@nullmax.ai".And the final-period workis managed by "slyang@nullmax.ai" and "zhangjunhao@nullmax.ai".

## 3. Calibration and Adjustment modules
  This project is composed by two modules, **Calibration** and  **Adjustment**.  
  The camera extrinsic is usually computed firstly by Calibration module(front_camera_extrinsic_calib), which gives you the Pitch and Yaw angles. Secondly, the Adjustment module(front_camera_extrinsic_adjustment) helps the developer tune the Roll angle. You can also run the two modules indpendently.



Please refer to **[calibration.md](./doc/calibration.md)** and **[adjustment.md](./doc/adjustment.md)** for more details.