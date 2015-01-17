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

#include "Digest.hpp"

/*!
 * The DigestMatch is used to match two Digests.
 * This class uses the descriptors of the Digest to calculate a transformation between the two Digest's pointclouds when they overlap enough.
 * It's templated with a machine learning module (MLMType) which is be used to filter the correspondences.
 */
template <typename MLMType>
class DigestMatch {

  public:
    /*!
     * This struct is used for storing the parameters used by the algorithms which create the digest match.
     */
    struct Parameters {
      float hint_threshold_ = 0.9;
    };

    struct TransformationHint {
      typedef Eigen::Affine3f Transformation;

      TransformationHint(Transformation transformation, float hint_confidence)
        : transformation_(transformation), hint_confidence_(hint_confidence)
      { }

      Transformation transformation_;
      float hint_confidence_;
    };

    /*!
     * This struct defines the relation between a pair of descriptors of two pointclouds
     */
    struct Correspondence {
      Correspondence(int source_id, int target_id, Digest::DescriptorType distance)
        : source_id(source_id), target_id(target_id), distance(distance)
      { }

      int source_id;
      int target_id;
      Digest::DescriptorType distance;
    };

    typedef std::vector<Correspondence> Correspondences;
    typedef std::vector<struct TransformationHint> TransformationHints;

    /*!
     * Most general Constructor.
     * The machine learning module will be trained with each TransformationHint which has a big enough confidence value.
     */
    DigestMatch(std::shared_ptr<Digest> digest_source, std::shared_ptr<Digest> digest_target, std::shared_ptr<MLMType> mlm, struct Parameters &params, TransformationHints transformation_hints)
      : digest_source_(digest_source), 
        digest_target_(digest_target),
        mlm_(mlm),
        params_(params),
        transformation_hints_(transformation_hints)
    {
      // Create a correspondence between each valid keypoint of the digests ("n²" correspondences)
      Digest::DescriptorCloud::Ptr descr_source = digest_source_->getDescriptorCloud();
      Digest::DescriptorCloud::Ptr descr_target = digest_target_->getDescriptorCloud();
      for (unsigned int i = 0; i < descr_source->size(); ++i) {
        for (unsigned int j = 0; j < descr_target->size(); ++j) {
          correspondences_.push_back(Correspondence(i, j, descriptorDistance(descr_source->at(i), descr_target->at(i))));
        }
      }

      //TODO: Do training (if possible)
      
      //TODO: Filter correspondences
    };

    /*!
     * Constructor with a transformation_hint.
     * The machine learning module will be trained with the transformation_hint
     * when its confidence value is bigger than the threshold from the parameters.
     */
    DigestMatch(std::shared_ptr<Digest> digest_source, std::shared_ptr<Digest> digest_target, std::shared_ptr<MLMType> mlm, struct Parameters &params, TransformationHint& transformation_hint)
      : DigestMatch(digest_source, digest_target, mlm, params, TransformationHints(1, transformation_hint))
    { };

    /*!
     * Constructor without a transformation_hint.
     * The machine learning module will not be trained.
     */
    DigestMatch(std::shared_ptr<Digest> digest_source, std::shared_ptr<Digest> digest_target, std::shared_ptr<MLMType> mlm, struct Parameters &params)
      : DigestMatch(digest_source, digest_target, mlm, params, TransformationHints())
    { };

    /*!
     * Destructor.
     */
    virtual ~DigestMatch() {
    };

  protected:
    std::shared_ptr<Digest> digest_source_;
    std::shared_ptr<Digest> digest_target_;
    std::shared_ptr<MLMType> mlm_;
    Correspondences correspondences_;
    struct Parameters params_;
    TransformationHints transformation_hints_;

    /*!
     * Returns the distance of two FPFHSignatur33 descriptors.
     */
    Digest::DescriptorType descriptorDistance(Digest::DescriptorType &d1, Digest::DescriptorType &d2) const {
      Digest::DescriptorType dr;
      for (unsigned int i = 0; i < 33; ++i) {
        dr.histogram[i] = d1.histogram[i] - d2.histogram[i];
      }
      return dr;
    };
};

#endif
