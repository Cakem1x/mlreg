//----------------------------------------------------------------------
/*!\file
 *
 * \author  Matthias Holoch <mholoch@gmail.com>
 * \date    2014-12-26
 *
 */
//----------------------------------------------------------------------
#include "Digest.hpp"
#include "DigestMatch.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv) {
  Digest::Cloud::Ptr pointcloud_source(new Digest::Cloud);
  Digest::Cloud::Ptr pointcloud_target(new Digest::Cloud);
  std::shared_ptr<Digest::Parameters> params_digest(new Digest::Parameters);
  std::shared_ptr<DigestMatch::Parameters> params_digest_match(new DigestMatch::Parameters);

  // Load pointclouds
  pcl::io::loadPCDFile(argv[1], *pointcloud_source);
  pcl::io::loadPCDFile(argv[2], *pointcloud_target);

  // Digest the pointclouds!
  std::shared_ptr<Digest> digest_source(new Digest(pointcloud_source, params_digest));
  std::shared_ptr<Digest> digest_target(new Digest(pointcloud_target, params_digest));

  // Put the Digests into a DigestMatch!
  DigestMatch digest_match(digest_source, digest_target, params_digest_match);

  //--------------------------------------------------------------------
  // Visualization-stuff from here:
  //--------------------------------------------------------------------
  pcl::visualization::PCLVisualizer vis("Digest Matcher Visualization");
  vis.setBackgroundColor(0,0,0);

  // Add the reduced pointclouds
  pcl::visualization::PointCloudColorHandlerCustom<Digest::PointType> source_cloud_color_handler(digest_source->getReducedCloud(), 255, 255, 0);
  vis.addPointCloud<Digest::PointType>(digest_source->getReducedCloud(), source_cloud_color_handler, "reduced_cloud_source");
  vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "reduced_cloud_source");
  pcl::visualization::PointCloudColorHandlerCustom<Digest::PointType> target_cloud_color_handler(digest_target->getReducedCloud(), 255, 0, 255);
  vis.addPointCloud<Digest::PointType>(digest_target->getReducedCloud(), "reduced_cloud_target");
  vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "reduced_cloud_target");

  // Draw lines for the correspondences

  while (!vis.wasStopped()) {
    vis.spinOnce(100);
  }
  return 0;
}
