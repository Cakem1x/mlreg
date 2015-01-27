//----------------------------------------------------------------------
/*!\file
 *
 * \author  Matthias Holoch <mholoch@gmail.com>
 * \date    2015-01-17
 *
 */
//----------------------------------------------------------------------
#ifndef SHARED_TYPES_HPP_INCLUDED
#define SHARED_TYPES_HPP_INCLUDED

#include <eigen3/Eigen/Geometry>
#include "Digest.hpp"

/*!
 * Data structor to define a TransformationHint.
 */
struct TransformationHint {
      typedef Eigen::Affine3f Transformation;

      TransformationHint(Transformation transformation, float confidence)
        : transformation(transformation), confidence(confidence)
      { }

      Transformation transformation;
      float confidence;
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

typedef std::vector<TransformationHint> TransformationHints;
#endif
