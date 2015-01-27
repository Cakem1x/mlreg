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
#include <pcl/common/angles.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
namespace pclvis = pcl::visualization;

/*
 * Gets data from two ground truth files and returns a transformation for the second 
 * coordinate-system into the first coordinate-system.
 */
Eigen::Affine3f getRelativeTransformation(std::ifstream &ground_truth_file_0, std::ifstream &ground_truth_file_1)
{
  Eigen::Affine3f gt_trans_0;
  Eigen::Affine3f gt_trans_1;
  Eigen::Affine3f alpha;
  Eigen::Affine3f beta;
  Eigen::Affine3f gamma;
  float p_x, p_y, p_z, r_a, r_b, r_c;
  ground_truth_file_0.ignore(std::numeric_limits<std::streamsize>::max(), ':');
  ground_truth_file_0 >> p_x >> p_y >> p_z;
  ground_truth_file_0.ignore(std::numeric_limits<std::streamsize>::max(), ':');
  ground_truth_file_0 >> r_a >> r_b >> r_c;
  //dirty hack to reverse the angle-application-order
  alpha = pcl::getTransformation(0, 0, 0, pcl::deg2rad(r_a), 0, 0);
  beta = pcl::getTransformation(0, 0, 0, 0, pcl::deg2rad(r_b), 0);
  gamma = pcl::getTransformation(0, 0, 0, 0, 0, pcl::deg2rad(r_c));
  gt_trans_0 = pcl::getTransformation(p_x, p_y, p_z, 0, 0, 0) * alpha * beta * gamma;
  ground_truth_file_0.close();
  ground_truth_file_1.ignore(std::numeric_limits<std::streamsize>::max(), ':');
  ground_truth_file_1 >> p_x >> p_y >> p_z;
  ground_truth_file_1.ignore(std::numeric_limits<std::streamsize>::max(), ':');
  ground_truth_file_1 >> r_a >> r_b >> r_c;
  ground_truth_file_1.close();
  //dirty hack to reverse the angle-application-order
  alpha = pcl::getTransformation(0, 0, 0, pcl::deg2rad(r_a), 0, 0);
  beta = pcl::getTransformation(0, 0, 0, 0, pcl::deg2rad(r_b), 0);
  gamma = pcl::getTransformation(0, 0, 0, 0, 0, pcl::deg2rad(r_c));
  gt_trans_1 = pcl::getTransformation(p_x, p_y, p_z, 0, 0, 0) * alpha * beta * gamma; 
  return gt_trans_0.inverse() * gt_trans_1;
}

int main(int argc, char** argv) {
  //--------------------------------------------------------------------
  // Get the parameters:
  //--------------------------------------------------------------------
  std::string ps_path;
  std::string pt_path;
  std::string gs_path;
  std::string gt_path;

  po::options_description params_main_description("Allowed options");
  params_main_description.add_options()
    ("help", "produce this message")
    ("pointcloud-source,s", po::value<std::string>(&ps_path), "Path to the source pointcloud (necessary)")
    ("pointcloud-target,t", po::value<std::string>(&pt_path), "Path to the target pointcloud (necessary)")
    ("groundtruth-source", po::value<std::string>(&gs_path), "Path to the groundtruth for the source pointcloud (optional)")
    ("groundtruth-target", po::value<std::string>(&gt_path), "Path to the groundtruth for the target pointcloud (optional)")
  ;
  po::variables_map params_main;
  po::store(po::parse_command_line(argc, argv, params_main_description), params_main);
  po::notify(params_main);

  if (params_main.count("help") || !params_main.count("pointcloud-source") || !params_main.count("pointcloud-target")) {
    // Print the help message and terminate
    std::cout << params_main_description << std::endl;
    return 1;
  }

  // Create TransformationHints datastructure
  TransformationHints tf_hints;
  if (params_main.count("groundtruth-source") && params_main.count("groundtruth-target")) {
    std::ifstream ground_truth_file_source(gs_path);
    std::ifstream ground_truth_file_target(gt_path);
    if (ground_truth_file_source.is_open() && ground_truth_file_target.is_open()) {
      tf_hints.push_back(TransformationHint(getRelativeTransformation(ground_truth_file_source,
                                                                      ground_truth_file_target), 
                                            1.0)); // Confidence is 100%, since it's the ground-truth!
    }
  }

  //--------------------------------------------------------------------
  // Create the digest and the digest match:
  //--------------------------------------------------------------------
  Digest::Cloud::Ptr pointcloud_source(new Digest::Cloud);
  Digest::Cloud::Ptr pointcloud_target(new Digest::Cloud);
  Digest::Parameters params_digest;
  DigestMatch::Parameters params_digest_match;
  // Create the machine learning module
  DigestMatch::MLMPtr mlm(new MLMSVM);

  // Load pointclouds
  pcl::io::loadPCDFile(ps_path, *pointcloud_source);
  pcl::io::loadPCDFile(pt_path, *pointcloud_target);

  // Digest the pointclouds
  Digest::Ptr digest_source(new Digest(pointcloud_source, params_digest));
  Digest::Ptr digest_target(new Digest(pointcloud_target, params_digest));

  // Put the Digests into a DigestMatch
  DigestMatch digest_match(digest_source, digest_target, mlm, params_digest_match, tf_hints);

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
  DigestMatch::Correspondences corrs = digest_match.getCorrespondences();
  for (DigestMatch::Correspondences::iterator it = corrs.begin(); it != corrs.end(); ++it) {
    std::stringstream corr_name;
    int d_src = it->source_id;
    int d_trg = it->target_id;
    int p_src = digest_source->getDescriptorCloudIndices()->at(d_src);
    int p_trg = digest_target->getDescriptorCloudIndices()->at(d_trg);
    pcl::PointXYZ src = digest_source->getReducedCloud()->at(p_src);
    pcl::PointXYZ trg = digest_target->getReducedCloud()->at(p_trg);
    pcl::PointXYZ midpoint(src.x + ((trg.x - src.x) / 2),
                           src.y + ((trg.y - src.y) / 2),
                           src.z + ((trg.z - src.z) / 2));
    corr_name << "Point " << p_src << " -> Point " << p_trg << " (Descriptor IDs " << d_src << " -> " << d_trg << ")";
    vis.addLine<pcl::PointXYZ>(src, trg, corr_name.str());
    vis.addText3D<pcl::PointXYZ>(corr_name.str(), midpoint, 0.05, 255, 255, 255, corr_name.str() + "txt");
  }

  //TODO: Make transforming the pointclouds possible

  while (!vis.wasStopped()) {
    vis.spinOnce(100);
  }
  return 0;
}
