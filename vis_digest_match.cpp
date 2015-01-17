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
#include "MLMSVM.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

typedef MLMSVM MLMType;

int main(int argc, char** argv) {
  Digest::Cloud::Ptr pointcloud_source(new Digest::Cloud);
  Digest::Cloud::Ptr pointcloud_target(new Digest::Cloud);
  Digest::Parameters params_digest;
  DigestMatch<MLMType>::Parameters params_digest_match;
  std::shared_ptr<MLMType> mlm(new MLMType());

  // Load pointclouds
  pcl::io::loadPCDFile(argv[1], *pointcloud_source);
  pcl::io::loadPCDFile(argv[2], *pointcloud_target);

  // Digest the pointclouds!
  std::shared_ptr<Digest> digest_source(new Digest(pointcloud_source, params_digest));
  std::shared_ptr<Digest> digest_target(new Digest(pointcloud_target, params_digest));

  // Put the Digests into a DigestMatch!
  DigestMatch<MLMType> digest_match(digest_source, digest_target, params_digest_match, mlm);

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

  // Add the keypoints of the reduced pointclouds
  pcl::visualization::PointCloudColorHandlerCustom<Digest::KeypointType> source_keypoint_color_handler(digest_source->getKeypointCloud(), 0, 255, 0);
  vis.addPointCloud<Digest::KeypointType>(digest_source->getKeypointCloud(), source_keypoint_color_handler, "keypoint_cloud_source");
  vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "keypoint_cloud_source");
  pcl::visualization::PointCloudColorHandlerCustom<Digest::KeypointType> target_keypoint_color_handler(digest_target->getKeypointCloud(), 0, 0, 255); 
  vis.addPointCloud<Digest::KeypointType>(digest_target->getKeypointCloud(), target_keypoint_color_handler, "keypoint_cloud_target");
  vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "keypoint_cloud_target");


  // Draw lines for the correspondences between the reduced pointclouds

  while (!vis.wasStopped()) {
    vis.spinOnce(100);
  }
  return 0;
}
