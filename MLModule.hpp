//----------------------------------------------------------------------
/*!\file
 *
 * \author  Matthias Holoch <mholoch@gmail.com>
 * \date    2015-01-27
 *
 */
//----------------------------------------------------------------------
#ifndef ML_MODULE_HPP_INCLUDED
#define ML_MODULE_HPP_INCLUDED

#include "DigestMatch.hpp"
#include "shared_types.hpp"

/*! 
 * The MLModule class is the interface which all machine learning modules 
 * have to implement to be used with DigestMatch.
 */
class MLModule {
  public:
    /*!
     * Destructor.
     */
    virtual ~MLModule() { };

    /*!
     */
    virtual void train(const Digest::Ptr& digest_source, const Digest::Ptr& digest_target, const TransformationHint& transformation_hint) = 0;

    virtual int classify(Correspondence& correspondence) const = 0;
};

#endif
