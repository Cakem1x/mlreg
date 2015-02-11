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
#include "DigestMatchViewer.hpp"
#include "MLMSVM.hpp"

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
  Digest::Parameters params_digest;
  MLMSVM::Parameters params_mlmsvm;
  std::string ps_path;
  std::string pt_path;
  std::string gs_path;
  std::string gt_path;

  po::options_description params_digest_description("Digest parameters");
  params_digest_description.add_options()
    ("voxelgrid-size", po::value<float>(&params_digest.voxelgrid_size)->default_value(0.2), "The size of the voxels for the voxel grid filter (in meters)")
    ("normal-radius", po::value<float>(&params_digest.normal_radius)->default_value(0.6), "The radius for the normal search (in meters)")
    ("ransac-threshold", po::value<float>(&params_digest.ransac_threshold)->default_value(1), 
     "The maximum distance of two corresponding points so that SAC will consider them as an inlier (in meters)")
    ("keypoint-radius", po::value<float>(&params_digest.keypoint_radius)->default_value(1), "The radius which is used for the keypoint search (in meters)")
    ("keypoint-threshold", po::value<float>(&params_digest.keypoint_threshold)->default_value(0.01), "Keypoints with intensity lower than this threshold will be discarded. (in 'magic unit'... :( )")
    ("descriptor-radius", po::value<float>(&params_digest.descriptor_radius)->default_value(4), "Radius for the descriptor calculation (in meters)")
    ("non-max-supression", po::value<bool>(&params_digest.non_max_supression)->default_value(0), "Sets the non maximum supression for the keypoint estimation (bool value)")
    ("refinement", po::value<bool>(&params_digest.refinement)->default_value(0), "Sets the refinement for the keypoint estimation (bool value)")
  ;

  po::options_description params_mlmsvm_description("MLMSVM parameters");
  params_mlmsvm_description.add_options()
    ("max-corr-distance-squared", po::value<float>(&params_mlmsvm.max_corr_distance_squared)->default_value(0.04), "The maximum distance between corresponding points so that they're still considered a valid correspondence (in meters)")
    ("model-store-path", po::value<std::string>(&params_mlmsvm.model_store_path)->default_value("svm_model.yaml"), "Path to where the svm model should be saved")
  ;

  po::options_description params_main_description("Allowed options");
  params_main_description.add_options()
    ("help", "produce this message")
    ("pointcloud-source,s", po::value<std::string>(&ps_path), "Path to the source pointcloud (necessary)")
    ("pointcloud-target,t", po::value<std::string>(&pt_path), "Path to the target pointcloud (necessary)")
    ("groundtruth-source", po::value<std::string>(&gs_path), "Path to the groundtruth for the source pointcloud (optional)")
    ("groundtruth-target", po::value<std::string>(&gt_path), "Path to the groundtruth for the target pointcloud (optional)")
  ;
  params_main_description.add(params_digest_description).add(params_mlmsvm_description);
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
  DigestMatch::Parameters params_digest_match;
  // Create the machine learning module
  DigestMatch::MLMPtr mlm(new MLMSVM(params_mlmsvm));

  // Load pointclouds
  pcl::io::loadPCDFile(ps_path, *pointcloud_source);
  pcl::io::loadPCDFile(pt_path, *pointcloud_target);

  // Digest the pointclouds
  Digest::Ptr digest_source(new Digest(pointcloud_source, params_digest));
  Digest::Ptr digest_target(new Digest(pointcloud_target, params_digest));

  // Put the Digests into a DigestMatch
  DigestMatch::Ptr digest_match(new DigestMatch(digest_source, digest_target, mlm, params_digest_match, tf_hints));

  //--------------------------------------------------------------------
  // Visualization:
  //--------------------------------------------------------------------
  DigestMatchViewer vis("Digest Matcher Visualization");

  vis.addDigestMatch(digest_match);

  while (!vis.wasStopped()) {
    vis.spinOnce(100);
  }

  //--------------------------------------------------------------------
  // Save the svm's model:
  //--------------------------------------------------------------------
  if (mlm->isReady()) {
    mlm->saveModel();
  }

  return 0;
}
