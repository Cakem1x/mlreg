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
#include "shared_types.hpp"

using namespace cv;

class MLMSVM {
  public:
    /*!
     * Default constructor.
     */
    MLMSVM()
     : svm_()
    {

    };

    void train(std::shared_ptr<Digest>& digest_source, std::shared_ptr<Digest>& digest_target, TransformationHint& transformation_hint) {
    };

    bool classify(Correspondence correspondence) {
      return true;
    };

  protected:
    SVM svm_;
};

#endif
