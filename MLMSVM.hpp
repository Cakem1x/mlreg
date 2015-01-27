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
     * Default constructor.
     */
    MLMSVM()
     : svm_()
    {

    };

    void train(const Digest::Ptr& digest_source, const Digest::Ptr& digest_target, const TransformationHint& transformation_hint) {
      // Will store the source cloud
      Digest::Cloud::ConstPtr cloud_source = digest_source->getReducedCloud();
      // Will store the transformed target cloud
      Digest::Cloud::Ptr cloud_target(new Digest::Cloud);

      // Transform the target cloud with the tf from the TransformationHint:
      pcl::transformPointCloud(*(digest_target->getReducedCloud()), *cloud_target, transformation_hint.transformation);

      // Search for corresponding point in close proximity, set found (=true/false) as class for svm
    };

    int classify(Correspondence& correspondence) const {
      return 0;
    };

  protected:
    cv::SVM svm_;
};

#endif
