#include "utils.h"
using namespace std;
using namespace cv;

void print_point(cv::Point2f&pt){
    cout<<"(x,y) -> ("<<pt.x<<", "<<pt.y<<")"<<std::endl;
}

bool parse_calib_params(const std::string &config_path,
                        std::vector<std::string> &imagelist,
                        cv::Size &boardSize,float &squareSize){
  imagelist.clear();
  std::ifstream conf_jsonfile(config_path, std::ios::binary);
  if(!conf_jsonfile.is_open()){
      std::cout<<"Input intrinsic json file missing: "<<config_path<<std::endl;
      return false;
  }
  try
  {
      Json::Reader reader;
      Json::Value root;
      Json::Value value;
      if(reader.parse(conf_jsonfile,root)){
          value = root["chessboard"];
          boardSize.width = value["width"].asInt();
          boardSize.height = value["height"].asInt();
          squareSize = value["square_size"].asFloat();
          value = root["img_list"];
          std::cout<<value["left"].asString()<<std::endl;
          std::vector<cv::String> img_list[2];
          // cv::FileStorage settings(value["left"],cv::FileStorage::READ);
          cv::glob(value["left"].asString(),img_list[0],true);
          cv::glob(value["right"].asString(),img_list[1],true);
          if(fabs(img_list[0].size()-img_list[1].size())>1e-3){
              printf("left img and right img don't have equal length!");
              return false;
          }
          for(int i=0,len=int(img_list[0].size());i<len;++i){
              imagelist.push_back(img_list[0][i]);
              imagelist.push_back(img_list[1][i]);
          }
      }
      conf_jsonfile.close();
  }
  catch(const std::exception& e)
  {
      printf("configuration json file parse failed!");
      std::cerr << e.what() << '\n';
      return false;
  }
  return true;
}

static void
SaveStereoIntrinsic(const Mat &leftCameraMatrix, const Mat &rightCameraMatrix,
        const Mat &leftDistCoeffs, const Mat &rightDistCoeffs){
  try {
    Json::Value root;
    Json::Value value;
    std::string output_filepath = "./stereo_intrinsic.json";
    std::ofstream out;
    Json::StreamWriterBuilder builder;
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());

    value["fx"] = leftCameraMatrix.at<double>(0, 0);
    value["fy"] = leftCameraMatrix.at<double>(1, 1);
    value["cx"] = leftCameraMatrix.at<double>(0, 2);
    value["cy"] = leftCameraMatrix.at<double>(1, 2);
    root["left_intrinsic"] = value;

    value.clear();
    value["fx"] = rightCameraMatrix.at<double>(0, 0);
    value["fy"] = rightCameraMatrix.at<double>(1, 1);
    value["cx"] = rightCameraMatrix.at<double>(0, 2);
    value["cy"] = rightCameraMatrix.at<double>(1, 2);
    root["right_intrinsic"] = value;

    value.clear();
    value["k1"] = leftDistCoeffs.at<double>(0, 0);
    value["k2"] = leftDistCoeffs.at<double>(0, 1);
    value["p1"] = leftDistCoeffs.at<double>(0, 2);
    value["p2"] = leftDistCoeffs.at<double>(0, 3);
    root["left_distortion"] = value;

    value.clear();
    value["k1"] = rightDistCoeffs.at<double>(0, 0);
    value["k2"] = rightDistCoeffs.at<double>(0, 1);
    value["p1"] = rightDistCoeffs.at<double>(0, 2);
    value["p2"] = rightDistCoeffs.at<double>(0, 3);
    root["right_distortion"] = value;

    out.open(output_filepath,
             std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      std::cout << "Failed to open " << output_filepath << std::endl;
      return;
    }
    writer->write(root, &out);
    out << std::endl;
    out.close();

  } catch (std::exception &ex) {
    std::cout << "Save json file error" << std::endl;
    return;
  }
}

static void
SaveStereoExtrinsic(const Mat &R, const Mat &T,
        const Mat &R1, const Mat &R2,
        const Mat &P1, const Mat &P2, const Mat &Q) {
  try {
    Json::Value root;
    Json::Value value;
    std::string output_filepath = "./stereo_extrinsic.json";
    std::ofstream out;
    Json::StreamWriterBuilder builder;
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());

    value["R"] = R.data;
    value["T"] = T.data;
    value["R1"] = R1.data;
    value["R2"] = R2.data;
    value["P1"] = P1.data;
    value["P2"] = P2.data;
    value["Q"] = Q.data;

    root["stereo_extrinsic"] = value;
    out.open(output_filepath,
             std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      std::cout << "Failed to open " << output_filepath << std::endl;
      return;
    }
    writer->write(root, &out);
    out << std::endl;
    out.close();

  } catch (std::exception &ex) {
    std::cout << "Save json file error" << std::endl;
    return;
  }
}

bool
StereoCalib(const vector<string>& imagelist, Size boardSize, float squareSize,
            bool displayCorners, bool useCalibrated,
            bool showRectified, bool fix_intrinstic){
    if( imagelist.size() % 2 != 0 )
    {
        cout << "Error: the image list contains odd (non-even) number of elements\n";
        return -1;
    }

    const int maxScale = 2;
    // ARRAY AND VECTOR STORAGE:
    
    // save image points from left and right img
    vector<vector<Point2f> > imagePoints[2];
    // save world points
    vector<vector<Point3f> > objectPoints;
    Size imageSize;

    int nimages = (int)imagelist.size()/2;

    imagePoints[0].clear();
    imagePoints[1].clear();
    vector<string> goodImageList;
    vector<Point2f> single_pair_img_pts[2];
    cv::Mat pair_img[2];
    std::cout<<"--------Total nimages: "<<nimages<<std::endl;
    for(int i=0; i<nimages; ++i){
        bool pair_available_flag = true;
        for(int k=0;k<2;++k){
            single_pair_img_pts[k].clear();
            const string &filename = imagelist[i*2+k];
            cv::Mat img = imread(filename,0);
            if(img.empty()){
                cout<<"load img error: "<<filename<<std::endl;
                break;
            }
            // save img to display
            pair_img[k]=img.clone();

            // img size -> imageSize
            if( imageSize == Size() ){
                imageSize = img.size();
            }else if( img.size() != imageSize )
            {
                cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
                break;
            }
            bool found = false;
            std::vector<cv::Point2f> &corners = single_pair_img_pts[k];
            // if img too small, resize big img
            for(int scale=1; scale<maxScale; ++scale){
                cv::Mat timg;
                if(scale==1){
                    timg = img.clone();
                }else{
                    cv::resize(img,timg,Size(),scale,scale,cv::INTER_LINEAR_EXACT);
                }
                // return true, if detect successful
                found = cv::findChessboardCorners(timg, boardSize, corners,
                    CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
                if(found){
                    if(scale>1){
                        cv::Mat cornersMat(corners);
                        cornersMat *= 1.0/scale;
                    }
                    break;
                }
            }
            // 
            if(!found){
                pair_available_flag = false;
                std::cout<<">>>>>"<<filename<<" don't detect corners"<<std::endl;
                break;
            }else{
                // for sub-pixel corner for image
                cv::cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
                            TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
                                        30, 0.01));
            }
        }
        if(!pair_available_flag){
            std::cout<<"--------"<<imagelist[i*2]<<","
                     <<"This pair discard because failure detecting corners!"<<std::endl;
            continue;
        }
        // discard rotation bigger than 45 degree(polar metric)
        float angle_diff = judge_polar_metric(single_pair_img_pts[0],single_pair_img_pts[1],boardSize);
        std::cout<<">>>>"<<imagelist[i*2]<<" pair,polar angle(degree):"<<angle_diff*180/3.14159<<std::endl;
        if(angle_diff<0){
            std::cout<<imagelist[i*2]<<", ";
            std::cout<<"This pair discard don't have same points,discard!"<<std::endl;
            continue;
        }
        if(angle_diff>45.0/180.0*3.14159){
            std::cout<<imagelist[i*2]<<", ";
            std::cout<<"This pair have polar error,discard!"<<std::endl;
            continue;
        }
        // show detect corner result
        if( displayCorners )
        {
            cv::Mat res_img[2];
            for(int kk=0;kk<2;++kk){
                cv::Mat temp_img;
                cv::cvtColor(pair_img[kk],temp_img,cv::COLOR_GRAY2BGR);
                cv::drawChessboardCorners(temp_img,boardSize,single_pair_img_pts[kk],true);
                for(int kki=0;kki<21;kki++){
                    // print_point(corners[kki]);
                    cv::putText(temp_img,to_string(kki),single_pair_img_pts[kk][kki],1,3,{0,0,255},2);
                }
                double sf = 640./MAX(temp_img.rows, temp_img.cols);
                resize(temp_img, temp_img, Size(), sf, sf, INTER_LINEAR_EXACT);
                res_img[kk]=temp_img.clone();
            }
            cv::imshow("left_img_corner",res_img[0]);
            cv::imshow("right_img_corner",res_img[1]);
            cv::waitKey(0);
            // cv::destroyAllWindows();
        }
        // save image points
        imagePoints[0].push_back(single_pair_img_pts[0]);
        imagePoints[1].push_back(single_pair_img_pts[1]);
        // save good image path
        goodImageList.push_back(imagelist[i*2]);
        goodImageList.push_back(imagelist[i*2+1]);
    }

    nimages = int(imagePoints[0].size());
    if(nimages<2){
        std::cout<<"Available pair: "<<nimages<<", "<<"Error: too little pairs to calibration.\n";
        return -1;
    }
    std::cout<<nimages<<" pairs have been successfully detected.\n";
    
    // start calib
    // parpare board pts
    objectPoints.resize(nimages);
    for (int ii = 0; ii < nimages; ii++){
        for( int jj = 0; jj < boardSize.height; jj++ )
            for( int kk = 0; kk < boardSize.width; kk++ )
                objectPoints[ii].push_back(Point3f(kk*squareSize, jj*squareSize, 0));
    }
    cout << "<<<<<<<<<Running stereo calibration ...>>>>>>>>>>>\n";

    Mat cameraMatrix[2], distCoeffs[2];
    Mat R, T, E, F;
    double rms = 0.0f;
    if(fix_intrinstic){
        std::cout<<"<<<<<<Fix intrinsic parameters and calibrate extrinsic parameters!>>>>>>"<<std::endl;
        cv::FileStorage intrinsic_in("intrinsics_seperate.yml", cv::FileStorage::READ);
        if(!intrinsic_in.isOpened())
        {
        std::cout << "Failed to open intrinsic file" << std::endl;
        std::cout << "param file name:" << "/intrinsics_seperate.yml" << std::endl;
        exit(1);
        }
        intrinsic_in["M1"] >> cameraMatrix[0];
        intrinsic_in["M2"] >> cameraMatrix[1];
        intrinsic_in["D1"] >> distCoeffs[0];
        intrinsic_in["D2"] >> distCoeffs[1];
        std::cout<<"\n>>>>camera1 K:\n"<<cameraMatrix[0]<<"\n"<<"  >>distortion coeff(k1,k2,p1,p2,k3):\n"<<distCoeffs[0]<<std::endl;
        std::cout<<"\n>>>>camera2 K:\n"<<cameraMatrix[1]<<"\n"<<"  >>distortion coeff(k1,k2,p1,p2,k3):\n"<<distCoeffs[1]<<"\n"<<std::endl;
        rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                            cameraMatrix[0], distCoeffs[0],
                            cameraMatrix[1], distCoeffs[1],
                            imageSize, R, T, E, F,
                            CALIB_FIX_INTRINSIC, 
                            TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-5) );
    }else{
        std::cout<<"<<<<<<Calibrate intrinsic and extrinsic parameters!>>>>>>"<<std::endl;
        cameraMatrix[0] = initCameraMatrix2D(objectPoints,imagePoints[0],imageSize,0);
        cameraMatrix[1] = initCameraMatrix2D(objectPoints,imagePoints[1],imageSize,0);
        rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                        cameraMatrix[0], distCoeffs[0],
                        cameraMatrix[1], distCoeffs[1],
                        imageSize, R, T, E, F,
                        CALIB_FIX_ASPECT_RATIO +
                        CALIB_USE_INTRINSIC_GUESS +
                        CALIB_SAME_FOCAL_LENGTH +
                        CALIB_RATIONAL_MODEL +
                        CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5 + CALIB_FIX_K6,
                        TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-5) );
    }
    cout << "done with RMS error=" << rms << endl;

    // CALIBRATION QUALITY CHECK
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for( int i = 0; i < nimages; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        for( int k = 0; k < 2; k++ )
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        double tmp =0.0f;
        for( int j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            errij/=2.0;
            err += errij;
            tmp+=errij;
        }
        std::cout<<imagelist[i*2]<<" epipolar err: "<<tmp/npt<<std::endl;
        npoints += npt;
    }
    cout << "average epipolar err = " <<  err/npoints << endl;

    // save intrinsic parameters
    // SaveStereoIntrinsic(cameraMatrix[0], cameraMatrix[1], distCoeffs[0], distCoeffs[1]);

    FileStorage fs("intrinsics.yml", FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
            "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q);
    // stereoRectify(cameraMatrix[0], distCoeffs[0],
    //               cameraMatrix[1], distCoeffs[1],
    //               imageSize, R, T, R1, R2, P1, P2, Q,
    //               1, 1, imageSize, &validRoi[0], &validRoi[1]);
    // stereoRectify(cameraMatrix[0], distCoeffs[0],
    //               cameraMatrix[1], distCoeffs[1],
    //               imageSize, R, T, R1, R2, P1, P2, Q,
    //               CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
    // SaveStereoExtrinsic(R, T, R1, R2, P1, P2, Q);
    // cv::stereoRectify( K_new_cvmat, D, K_new_cvmat, D, imsize, R, T,
    //                     rm_R1, rm_R2, rm_P1, rm_P2, rm_Q
    //                 );
    fs.open("extrinsics.yml", FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    }
    else
        cout << "Error: can not save the extrinsic parameters\n";

    // OpenCV can handle left-right
    // or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

    // COMPUTE AND DISPLAY RECTIFICATION
    if( !showRectified )
        return -1;

    Mat rmap[2][2];
    // IF BY CALIBRATED (BOUGUET'S METHOD)
    if( useCalibrated )
    {
        // we already computed everything
    }
    // OR ELSE HARTLEY'S METHOD
    else
    // use intrinsic parameters of each camera, but
    // compute the rectification transformation directly
    // from the fundamental matrix
    {
        vector<Point2f> allimgpt[2];
        for( int k = 0; k < 2; k++ )
        {
            for( int i = 0; i < nimages; i++ )
                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
        }
        F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
        Mat H1, H2;
        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

        R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
        R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }

    //Precompute maps for cv::remap()
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    Mat canvas;
    double sf;
    int w, h;
    if( !isVerticalStereo )
    {
        sf = 600./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h, w*2, CV_8UC3);
    }
    else
    {
        sf = 300./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h*2, w, CV_8UC3);
    }

    for( int i = 0; i < nimages; i++ )
    {
        for( int k = 0; k < 2; k++ )
        {
            Mat img = imread(goodImageList[i*2+k], 0), rimg, cimg;
            remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
            cvtColor(rimg, cimg, COLOR_GRAY2BGR);
            Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
            resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
            if( useCalibrated )
            {
                Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
                          cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
                rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
            }
        }

        if( !isVerticalStereo )
            for( int j = 0; j < canvas.rows; j += 16 )
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
        else
            for( int j = 0; j < canvas.cols; j += 16 )
                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
        imshow("rectified", canvas);
        char c = (char)waitKey(0);
        if( c == 27 || c == 'q' || c == 'Q' )
            break;
    }
}

float judge_polar_metric(const std::vector<cv::Point2f>& l_pts,
                        const std::vector<cv::Point2f>& r_pts,
                        const cv::Size &boardSize){
  int count = int(l_pts.size());
  if((count-int(r_pts.size()))>1e-3){
      return -1;
  }
  float delta_x = fabs(l_pts[0].x-l_pts[1].x);
  float delta_y = fabs(l_pts[0].y-r_pts[1].y);
  int num = (delta_x>delta_y)?boardSize.height:boardSize.width;
  num -=1;
  float theta1,theta2;

  delta_x = l_pts[num].x - l_pts[0].x;
  delta_y = l_pts[num].y - l_pts[0].y;
  theta1 = atanf(delta_y/(delta_x+1e-3));

  delta_x = r_pts[num].x - r_pts[0].x;
  delta_y = r_pts[num].y - r_pts[0].y;
  theta2 = atanf(delta_y/(delta_x+1e-3));
  return fabs(theta1-theta2);
}
