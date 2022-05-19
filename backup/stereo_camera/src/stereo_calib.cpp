#include "utils.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  if(argc!=3){
      printf("please input two params:\n1)config_path\n2)flag(0->calib intrinsic and extrinsic,1->only calib extrinsic)");
      return 0;
  }
  Size boardSize;
  float squareSize;
  bool showRectified = true;
  int flag;
  sscanf(argv[1],"%d",&flag);
  vector<string> imagelist;
  if(!parse_calib_params(argv[2],imagelist,boardSize,squareSize)){
      printf("parse json file error!");
      return -1;
  }
  StereoCalib(imagelist, boardSize, squareSize, true, true, showRectified, bool(flag));
  return 0;
}
