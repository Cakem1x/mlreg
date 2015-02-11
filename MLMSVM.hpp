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
      float correct_corr_max_distance_squared = 1.0;
    };

    /*!
     * Default constructor with parameters.
     */
    MLMSVM(struct Parameters& params)
     : svm_(),
       params_(params)
    {
    }

    void train(const Digest::Ptr& digest_source, const Digest::Ptr& digest_target, const TransformationHint& transformation_hint, const Correspondences& correspondences) {
      // Stores the source cloud
      Digest::Cloud::ConstPtr cloud_source = digest_source->getReducedCloud();
      // Stores the transformed target cloud
      Digest::Cloud::Ptr cloud_target(new Digest::Cloud);
      // Stores the training data
      cv::Mat trainingData(correspondences.size(), 33, CV_32FC1);
      // Stores the training labels
      cv::Mat labels(correspondences.size(), 1, CV_32SC1);

      // Transform the target cloud with the tf from the TransformationHint:
      pcl::transformPointCloud(*(digest_target->getReducedCloud()), *cloud_target, transformation_hint.transformation);

      // Iterate through all correspondences and define their class by the distance between the transformed corresponding points
      for (unsigned int i = 0; i < correspondences.size(); ++i) {
        Digest::PointType p_src = cloud_source->at(digest_source->getDescriptorCloudIndices()->at(correspondences[i].source_id));
        Digest::PointType p_trg = cloud_target->at(digest_target->getDescriptorCloudIndices()->at(correspondences[i].target_id));
        Digest::PointType p_diff(p_src.x - p_trg.x, p_src.y - p_trg.y, p_src.z - p_trg.z);
        // set the class of the correspondence depending on their squared euclidean distance
        labels.at<int>(i,0) = (p_diff.x * p_diff.x + p_diff.y * p_diff.y + p_diff.z * p_diff.z <= params_.correct_corr_max_distance_squared);
        // add the feature vector to the training data
        for (unsigned int j = 0; j < 33; ++j) {
          trainingData.at<float>(i,j) = correspondences[i].distance.histogram[j];
        }
      }
      // train the svm
      svm_.train(trainingData, labels, cv::Mat(), cv::Mat(), cv::SVMParams());
    }

    float classify(const Correspondence& correspondence) const {
      cv::Mat sample(1, 33, CV_32FC1);
      for (unsigned int j = 0; j < 33; ++j) {
        sample.at<float>(0,j) = correspondence.distance.histogram[j];
      }
      return svm_.predict(sample);
    }

  protected:
    cv::SVM svm_;
    struct Parameters params_;
};

#endif
