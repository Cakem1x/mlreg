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
};

#endif
