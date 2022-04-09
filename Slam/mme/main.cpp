#include "MeanMapEntropyMetrics.h"
#include "Timer.hpp"
#include <iostream>

int main()
{
    MeanMapEntropyMetrics mme;
    mme.LoadGtMap("/home/will/data/test/old_updated_map_filtered/pcd/filtered_map.pcd");
    mme.LoadCurMap("/home/will/data/test/filtered_pointcloud_map/pcd/filtered_map.pcd");
    Timer timer;
    timer.start();
    mme.Metrics(0.4, 0.2);
    timer.elapsedByLast();
    return 0;
}
