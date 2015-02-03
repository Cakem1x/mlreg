//----------------------------------------------------------------------
/*!\file
 *
 * \author  Matthias Holoch <mholoch@gmail.com>
 * \date    2015-01-17
 *
 */
//----------------------------------------------------------------------
#ifndef MLMSVM_HPP_INCLUDED
#define MLMSVM_HPP_INCLUDED

#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>

#include <pcl/registration/transforms.h>

#include "shared_types.hpp"
#include "MLModule.hpp"

/*! 
 * This class wraps OpenCV's Support Vector Machine for use on correspondences inside mlreg.
 * It uses the FPFH-distance correspondences for its classification.
 * The classified classes are "true correspondence" or "false correspondence".
 * For training, it'll use correspondences from a TransformationHint, which also contains 
 * a transformation. This transformation can come from multiple sources:
 *    ground-truth data when training the  algorithm for a known environment
 *    some intialization cloud obtained without moving the sensor and artificially put noise on the cloud
 *    robot odometry while it has high preicison
 *    ...
 */
class MLMSVM : public MLModule {
  public:
    /*!
     * This struct is used for storing the parameters used by the MLMSVM class.
     */
    struct Parameters {
      float correct_corr_max_distance_squared = 0.01;
    };

    /*!
     * Default constructor with parameters.
     */
    MLMSVM(struct Parameters& params)
     : params_(params), svm_()
    {

    }

    void train(const Digest::Ptr& digest_source, const Digest::Ptr& digest_target, const TransformationHint& transformation_hint, const Correspondences& correspondences) {
      // Stores the source cloud
      Digest::Cloud::ConstPtr cloud_source = digest_source->getReducedCloud();
      // Stores the transformed target cloud
      Digest::Cloud::Ptr cloud_target(new Digest::Cloud);

      // Transform the target cloud with the tf from the TransformationHint:
      pcl::transformPointCloud(*(digest_target->getReducedCloud()), *cloud_target, transformation_hint.transformation);

      // Iterate through all correspondences and define their class by the distance between the transformed corresponding points
      for (Correspondences::const_iterator it = correspondences.begin(); it != correspondences.end(); ++it) {
        bool correct_correspondence;
        Digest::PointType p_src = cloud_source->at(digest_source->getDescriptorCloudIndices()->at(it->source_id));
        Digest::PointType p_trg = cloud_target->at(digest_target->getDescriptorCloudIndices()->at(it->target_id));
        Digest::PointType p_diff(p_src.x - p_trg.x, p_src.y - p_trg.y, p_src.z - p_trg.z);
        // set the class of the correspondence depending on their euclidean distance
        correct_correspondence = p_diff.x * p_diff.x + p_diff.y * p_diff.y + p_diff.z * p_diff.z <= params_.correct_corr_max_distance_squared;
      }
    }

    int classify(Correspondence& correspondence) const {
      return 0;
    }

  protected:
    cv::SVM svm_;
    struct Parameters params_;
};

#endif
