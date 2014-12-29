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
 * This means that this class uses the descriptors of the Digest to calculate a transformation between the two Digest's pointclouds when they overlap enough.
 */
class DigestMatch {

  public:
    /*!
     * This struct is used for storing the parameters used by the algorithms which create the digest match.
     */
    struct Parameters {
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

    DigestMatch(std::shared_ptr<Digest> digest_source, std::shared_ptr<Digest> digest_target, std::shared_ptr<struct Parameters> params)
      : digest_source_(digest_source), 
        digest_target_(digest_target),
        correspondences_(new Correspondences),
        params_(params)
    {
      // Create a correspondence between each valid keypoint of the digests ("n²" correspondences)
      Digest::DescriptorCloud::Ptr descr_source = digest_source_->getDescriptorCloud();
      Digest::DescriptorCloud::Ptr descr_target = digest_target_->getDescriptorCloud();
      for (unsigned int i = 0; i < descr_source->size(); ++i) {
        for (unsigned int j = 0; j < descr_target->size(); ++j) {
          correspondences_->push_back(Correspondence(i, j, descriptorDistance(descr_source->at(i), descr_target->at(i))));
        }
      }
    };

  protected:
    std::shared_ptr<Digest> digest_source_;
    std::shared_ptr<Digest> digest_target_;
    std::shared_ptr<Correspondences> correspondences_;
    std::shared_ptr<struct Parameters> params_;

    /*!
     * Returns the distance of two FPFHSignatur33 descriptors.
     */
    Digest::DescriptorType descriptorDistance(Digest::DescriptorType d1, Digest::DescriptorType d2) {
      Digest::DescriptorType dr;
      for (unsigned int i = 0; i < 33; ++i) {
        dr.histogram[i] = d1.histogram[i] - d2.histogram[i];
      }
      return dr;
    };
};

#endif
