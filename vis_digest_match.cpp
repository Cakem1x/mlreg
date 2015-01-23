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

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

typedef MLMSVM MLMType;
namespace po = boost::program_options;
namespace pclvis = pcl::visualization;

int main(int argc, char** argv) {
  //--------------------------------------------------------------------
  // Get the parameters:
  //--------------------------------------------------------------------
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce this message")
    ("pointcloud-source,ps", po::value<std::string>(), "Path to the source pointcloud")
    ("pointcloud-target,pt", po::value<std::string>(), "Path to the target pointcloud")
  ;

  //--------------------------------------------------------------------
  // Create the digest and the digest match:
  //--------------------------------------------------------------------
  Digest::Cloud::Ptr pointcloud_source(new Digest::Cloud);
  Digest::Cloud::Ptr pointcloud_target(new Digest::Cloud);
  Digest::Parameters params_digest;
  DigestMatch<MLMType>::Parameters params_digest_match;
  // Create the machine learning module
  DigestMatch<MLMType>::MLMPtr mlm(new MLMType);

  // Load pointclouds
  pcl::io::loadPCDFile(argv[1], *pointcloud_source);
  pcl::io::loadPCDFile(argv[2], *pointcloud_target);

  // Digest the pointclouds
  Digest::Ptr digest_source(new Digest(pointcloud_source, params_digest));
  Digest::Ptr digest_target(new Digest(pointcloud_target, params_digest));

  // Put the Digests into a DigestMatch
  DigestMatch<MLMType> digest_match(digest_source, digest_target, mlm, params_digest_match);

  //--------------------------------------------------------------------
  // Visualization:
  //--------------------------------------------------------------------
  pclvis::PCLVisualizer vis("Digest Matcher Visualization");
  vis.setBackgroundColor(0,0,0);

  // Add the reduced pointclouds
  pclvis::PointCloudColorHandlerCustom<Digest::PointType> source_cloud_color_handler(digest_source->getReducedCloud(), 255, 255, 0);
  vis.addPointCloud<Digest::PointType>(digest_source->getReducedCloud(), source_cloud_color_handler, "reduced_cloud_source");
  vis.setPointCloudRenderingProperties (pclvis::PCL_VISUALIZER_POINT_SIZE, 2, "reduced_cloud_source");
  pclvis::PointCloudColorHandlerCustom<Digest::PointType> target_cloud_color_handler(digest_target->getReducedCloud(), 255, 0, 255);
  vis.addPointCloud<Digest::PointType>(digest_target->getReducedCloud(), "reduced_cloud_target");
  vis.setPointCloudRenderingProperties (pclvis::PCL_VISUALIZER_POINT_SIZE, 2, "reduced_cloud_target");

  // Add the location of all keypoints which were over the threshold
  pclvis::PointCloudColorHandlerCustom<Digest::PointType> source_keypoint_color_handler(digest_source->getDescriptorCloudPoints(), 0, 255, 0);
  vis.addPointCloud<Digest::PointType>(digest_source->getDescriptorCloudPoints(), source_keypoint_color_handler, "keypoint_cloud_source");
  vis.setPointCloudRenderingProperties (pclvis::PCL_VISUALIZER_POINT_SIZE, 6, "keypoint_cloud_source");
  pclvis::PointCloudColorHandlerCustom<Digest::PointType> target_keypoint_color_handler(digest_target->getDescriptorCloudPoints(), 0, 0, 255); 
  vis.addPointCloud<Digest::PointType>(digest_target->getDescriptorCloudPoints(), target_keypoint_color_handler, "keypoint_cloud_target");
  vis.setPointCloudRenderingProperties (pclvis::PCL_VISUALIZER_POINT_SIZE, 6, "keypoint_cloud_target");

  // Draw lines for the correspondences between the reduced pointclouds
  DigestMatch<MLMSVM>::Correspondences corrs = digest_match.getCorrespondences();
  for (DigestMatch<MLMSVM>::Correspondences::iterator it = corrs.begin(); it != corrs.end(); ++it) {
    std::stringstream corr_name;
    int d_src = it->source_id;
    int d_trg = it->target_id;
    int p_src = digest_source->getDescriptorCloudIndices()->at(d_src);
    int p_trg = digest_target->getDescriptorCloudIndices()->at(d_trg);
    corr_name << "Point " << p_src << " -> Point " << p_trg << " (Descriptor IDs " << d_src << " -> " << d_trg;
    vis.addLine<pcl::PointXYZ>(digest_source->getReducedCloud()->at(p_src), digest_target->getReducedCloud()->at(p_trg), corr_name.str());
  }

  while (!vis.wasStopped()) {
    vis.spinOnce(100);
  }
  return 0;
}
