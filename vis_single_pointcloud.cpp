#include "Digest.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv) {
  Digest::Cloud::Ptr pointcloud(new Digest::Cloud);
  std::shared_ptr<Digest::Parameters> params(new Digest::Parameters);

  // Load pointcloud
  pcl::io::loadPCDFile(argv[1], *pointcloud);
  Digest digest(pointcloud, params);
  pcl::visualization::PCLVisualizer vis("Single Cloud Digest Visualization");
  vis.setBackgroundColor(0,0,0);
  vis.addPointCloud<Digest::PointType>(digest.getReducedCloud(), "reduced_cloud");
  vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "reduced_cloud");

  while (!vis.wasStopped()) {
    vis.spinOnce(100);
  }
  return 0;
}
