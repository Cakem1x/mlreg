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
     * This method is used to implement some kind of training on correspondences with a given TransformationHint.
     */
    virtual void train(const Digest::Ptr& digest_source, const Digest::Ptr& digest_target, const TransformationHint& transformation_hint, const Correspondences& correspondences) = 0;

    /*!
     * This method is used to implement some kind of classification of a correspondence.
     */
    virtual float classify(const Correspondence& correspondence) const = 0;

    /*!
     * Returns whether the MLModule is ready to classify.
     */
    virtual bool isReady() const = 0;

    /*!
     * Loads the model from some file.
     */
    virtual void loadModel() = 0;

    /*
     * Saves the svm model some file.
     */
    virtual void saveModel() const = 0;

};

#endif
