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

  // Add the reduced pointcloud
  vis.addPointCloud<Digest::PointType>(digest.getReducedCloud(), "reduced_cloud");
  vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "reduced_cloud");

  // Add the normals of the reduced pointcloud
  vis.addPointCloudNormals<Digest::PointType, Digest::NormalType>(digest.getReducedCloud(), digest.getNormalCloud(), 1, 0.13, "reduced_cloud_normals");

  // Add the keypoints with bigger dots

  while (!vis.wasStopped()) {
    vis.spinOnce(100);
  }
  return 0;
}
