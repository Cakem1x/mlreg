//----------------------------------------------------------------------
/*!\file
 *
 * \author  Matthias Holoch <mholoch@gmail.com>
 * \date    2015-01-27
 *
 */
//----------------------------------------------------------------------
#ifndef DIGEST_MATCH_VIEWER_HPP_INCLUDED
#define DIGEST_MATCH_VIEWER_HPP_INCLUDED

#include "DigestMatch.hpp"
#include "Digest.hpp"
#include "shared_types.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include <boost/bind.hpp>

namespace pclvis = pcl::visualization;

/*!
 * The DigestMatchViewer is a specialization of the PCLVisualizer.
 * It can be used to easily visualize the results of a DigestMatch.
 */
class DigestMatchViewer : public pclvis::PCLVisualizer {
  public:
    DigestMatchViewer(const std::string& name = "", const bool create_interactor = true)
      : pclvis::PCLVisualizer(name, create_interactor)
    {
      setBackgroundColor(0,0,0);

      // Register the keyboard event handler
      boost::function<void (const pclvis::KeyboardEvent&)> f = boost::bind(&DigestMatchViewer::keyboardEventHandler, this, _1);
      registerKeyboardCallback(f);
    }

    virtual ~DigestMatchViewer() { }

    /*!
     * Adds another DigestMatch to the visualization.
     */
    void addDigestMatch(DigestMatch::Ptr& digest_match) {
      digest_match_ = digest_match;
      // Add the reduced pointclouds
      Digest::Ptr digest_source = digest_match->getDigestSource();
      Digest::Ptr digest_target = digest_match->getDigestTarget();
      pclvis::PointCloudColorHandlerCustom<Digest::PointType> source_cloud_color_handler(digest_source->getReducedCloud(), 255, 255, 0);
      addPointCloud<Digest::PointType>(digest_source->getReducedCloud(), source_cloud_color_handler, "reduced_cloud_source");
      setPointCloudRenderingProperties (pclvis::PCL_VISUALIZER_POINT_SIZE, 2, "reduced_cloud_source");
      pclvis::PointCloudColorHandlerCustom<Digest::PointType> target_cloud_color_handler(digest_target->getReducedCloud(), 255, 0, 255);
      addPointCloud<Digest::PointType>(digest_target->getReducedCloud(), "reduced_cloud_target");
      setPointCloudRenderingProperties (pclvis::PCL_VISUALIZER_POINT_SIZE, 2, "reduced_cloud_target");

      // Add the location of all keypoints which were over the threshold
      pclvis::PointCloudColorHandlerCustom<Digest::PointType> source_keypoint_color_handler(digest_source->getDescriptorCloudPoints(), 0, 255, 0);
      addPointCloud<Digest::PointType>(digest_source->getDescriptorCloudPoints(), source_keypoint_color_handler, "keypoint_cloud_source");
      setPointCloudRenderingProperties (pclvis::PCL_VISUALIZER_POINT_SIZE, 6, "keypoint_cloud_source");
      pclvis::PointCloudColorHandlerCustom<Digest::PointType> target_keypoint_color_handler(digest_target->getDescriptorCloudPoints(), 0, 0, 255); 
      addPointCloud<Digest::PointType>(digest_target->getDescriptorCloudPoints(), target_keypoint_color_handler, "keypoint_cloud_target");
      setPointCloudRenderingProperties (pclvis::PCL_VISUALIZER_POINT_SIZE, 6, "keypoint_cloud_target");

      // Draw lines for the correspondences between the reduced pointclouds
      drawCorrespondences(Eigen::Affine3f::Identity());
    }

    /*!
     * A new keyboard event handler which handles the following commands:
     *   -y: Display the pointclouds transformed with the first tf hint.
     *   -a: Display the pointclouds transformed with the calculated tf.
     *   -x: Display the pointclouds untransformed.
     */
    void keyboardEventHandler(const pclvis::KeyboardEvent& event) {
      // -y: Display the pointclouds transformed with the first tf hint.
      if (event.getKeySym() == "y" && event.keyDown() && digest_match_->getTransformationHints().size() > 0) {
        Eigen::Affine3f tf = digest_match_->getTransformationHints()[0].transformation;
        const Digest::Cloud target = *(digest_match_->getDigestTarget()->getReducedCloud());
        Digest::Cloud target_transformed;

        // Update the poses of all displayed clouds
        updatePointCloudPose("reduced_cloud_target", tf);
        updatePointCloudPose("keypoint_cloud_target", tf);
        // Update the correspondences
        drawCorrespondences(tf);
      }

      // -a: Display the pointclouds transformed with the calculated tf.
      if (event.getKeySym() == "a" && event.keyDown() && digest_match_->getCorrespondences().size() > 0) {
        Eigen::Affine3f tf = digest_match_->getTransformation();
        const Digest::Cloud target = *(digest_match_->getDigestTarget()->getReducedCloud());
        Digest::Cloud target_transformed;
        // Update the poses of all displayed clouds
        updatePointCloudPose("reduced_cloud_target", tf);
        updatePointCloudPose("keypoint_cloud_target", tf);
        // Update the correspondences
        drawCorrespondences(tf);
      }

      // -x: Display the pointclouds untransformed.
      if (event.getKeySym() == "x" && event.keyDown()) {
        Eigen::Affine3f tf = Eigen::Affine3f::Identity();
        const Digest::Cloud target = *(digest_match_->getDigestTarget()->getReducedCloud());
        Digest::Cloud target_transformed;
        // Update the poses of all displayed clouds
        updatePointCloudPose("reduced_cloud_target", tf);
        updatePointCloudPose("keypoint_cloud_target", tf);
        // Update the correspondences
        drawCorrespondences(tf);
      }
    }

  protected:
    DigestMatch::Ptr digest_match_;

    /*!
     * Redraws the correspondences of the currently displayed DigestMatch.
     * The target cloud will be transformed by tf.
     */
    void drawCorrespondences(Eigen::Affine3f tf) {
      removeAllShapes();
      Correspondences corrs = digest_match_->getCorrespondences();
      for (Correspondences::iterator it = corrs.begin(); it != corrs.end(); ++it) {
        std::stringstream corr_name;
        int d_src = it->source_id;
        int d_trg = it->target_id;
        int p_src = digest_match_->getDigestSource()->getDescriptorCloudIndices()->at(d_src);
        int p_trg = digest_match_->getDigestTarget()->getDescriptorCloudIndices()->at(d_trg);
        pcl::PointXYZ src = digest_match_->getDigestSource()->getReducedCloud()->at(p_src);
        pcl::PointXYZ trg = digest_match_->getDigestTarget()->getReducedCloud()->at(p_trg);
        trg = pcl::transformPoint(trg, tf);
        pcl::PointXYZ midpoint(src.x + ((trg.x - src.x) / 2),
                               src.y + ((trg.y - src.y) / 2),
                               src.z + ((trg.z - src.z) / 2));
        corr_name << "Point " << p_src << " -> Point " << p_trg << " (Descriptor IDs " << d_src << " -> " << d_trg << ")";
        addLine<Digest::PointType>(src, trg, corr_name.str());
        addText3D<Digest::PointType>(corr_name.str(), midpoint, 0.05, 255, 255, 255, corr_name.str() + "txt");
      }
    }
};

#endif
