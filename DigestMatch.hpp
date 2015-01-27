//----------------------------------------------------------------------
/*!\file
 *
 * \author  Matthias Holoch <mholoch@gmail.com>
 * \date    2014-12-29
 *
 */
//----------------------------------------------------------------------
#ifndef DIGEST_MATCH_HPP_INCLUDED
#define DIGEST_MATCH_HPP_INCLUDED

#include "shared_types.hpp"
#include "Digest.hpp"
#include "MLModule.hpp"

/*!
 * The DigestMatch is used to match two Digests.
 * This class uses the descriptors of the Digest to calculate a transformation between the two Digest's pointclouds when they overlap enough.
 * It uses a machine learning module (MLModule) to filter the correspondences.
 */
class DigestMatch {

  public:
    typedef std::shared_ptr<DigestMatch> Ptr;
    typedef std::shared_ptr<MLModule> MLMPtr;
    typedef std::vector<Correspondence> Correspondences;

    /*!
     * This struct is used for storing the parameters used by the algorithms which create the digest match.
     */
    struct Parameters {
      float hint_confidence_threshold = 0.9;
    };

    /*!
     * Most general Constructor.
     * The machine learning module will be trained with each TransformationHint which has a big enough confidence value.
     */
    DigestMatch(Digest::Ptr digest_source, Digest::Ptr digest_target, MLMPtr& mlm, struct Parameters& params, TransformationHints transformation_hints)
      : digest_source_(digest_source), 
        digest_target_(digest_target),
        mlm_(mlm),
        params_(params),
        transformation_hints_(transformation_hints)
    {
      // Create a correspondence between each valid keypoint of the digests ("n²" correspondences)
      Digest::DescriptorCloud::Ptr descr_source = digest_source_->getDescriptorCloud();
      Digest::DescriptorCloud::Ptr descr_target = digest_target_->getDescriptorCloud();
      pcl::IndicesPtr descr_i_source = digest_source_->getDescriptorCloudIndices();
      pcl::IndicesPtr descr_i_target = digest_target_->getDescriptorCloudIndices();
      for (unsigned int i = 0; i < descr_source->size(); ++i) {
        for (unsigned int j = 0; j < descr_target->size(); ++j) {
          correspondences_.push_back(Correspondence(i, j, descriptorDistance(descr_source->at(i), descr_target->at(j))));
        }
      }

      // Train the MLM when there are TransformationHints with suitable confidence
      for (TransformationHints::iterator it = transformation_hints_.begin(); it < transformation_hints_.end(); ++it) {
        if (it->confidence >= params_.hint_confidence_threshold) {
          mlm_->train(digest_source_, digest_target_, *it);
        }
      }
      
      //TODO: Filter correspondences
    };

    /*!
     * Constructor with a transformation_hint.
     * The machine learning module will be trained with the transformation_hint
     * when its confidence value is bigger than the threshold from the parameters.
     */
    DigestMatch(Digest::Ptr digest_source, Digest::Ptr digest_target, MLMPtr mlm, struct Parameters& params, TransformationHint& transformation_hint)
      : DigestMatch(digest_source, digest_target, mlm, params, TransformationHints(1, transformation_hint))
    { };

    /*!
     * Constructor without a transformation_hint.
     * The machine learning module will not be trained.
     */
    DigestMatch(Digest::Ptr digest_source, Digest::Ptr digest_target, MLMPtr mlm, struct Parameters& params)
      : DigestMatch(digest_source, digest_target, mlm, params, TransformationHints())
    { };

    /*!
     * Destructor.
     */
    virtual ~DigestMatch() {
    };

    /*!
     * Getter for the current correspondences of this DigestMatch.
     * Each correspondence contains the index of both source and target descriptors and their distance.
     * To get the corresponding point from the original reduced point cloud, use the Digest's method getDescriptorCloudIndices().
     * With descr_cloud_indices->at[correspondces.source_id] = n,
     * the original point can be found at reduced_cloud[n].
     * descr_cloud->at[correspondences.source_id] will get you the actual descriptor.
     */
    const Correspondences getCorrespondences() const {
      return correspondences_;
    }

    /*!
     * Returns this DigestMatch's TransformationHints.
     */
    const TransformationHints getTransformationHints() const {
      return transformation_hints_;
    }

  protected:
    Digest::Ptr digest_source_;
    Digest::Ptr digest_target_;
    MLMPtr mlm_;
    Correspondences correspondences_;
    struct Parameters params_;
    TransformationHints transformation_hints_;

    /*!
     * Returns the distance of two FPFHSignatur33 descriptors.
     */
    Digest::DescriptorType descriptorDistance(Digest::DescriptorType& d1, Digest::DescriptorType& d2) const {
      Digest::DescriptorType dr;
      for (unsigned int i = 0; i < 33; ++i) {
        dr.histogram[i] = d1.histogram[i] - d2.histogram[i];
      }
      return dr;
    };
};

#endif
