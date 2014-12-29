//----------------------------------------------------------------------
/*!\file
 *
 * \author  Matthias Holoch <mholoch@gmail.com>
 * \date    2014-12-26
 *
 */
//----------------------------------------------------------------------
#ifndef DIGEST_HPP_INCLUDED
#define DIGEST_HPP_INCLUDED

#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/voxel_grid.h>

/*!
 * Class to create and store the important features and descriptors for one pointcloud.
 * Two digests can be used by the DigestMatcher to check whether the pointclouds overlap and to get a tranformation between the two pointclouds.
 */
class Digest {

  public:
    /*!
     * This struct is used for storing the parameters used by the algorithms which create the digest.
     */
    struct Parameters {
      float voxelgrid_size = 0.2;
      float normal_radius = 0.6;
      float ransac_threshold = 1;
      float keypoint_radius = 1;
      float keypoint_threshold = 0.001;
      float descriptor_radius = 4;
    };
    typedef pcl::PointXYZ PointType;
    typedef pcl::Normal NormalType;
    typedef pcl::PointXYZI KeypointType;
    typedef pcl::HarrisKeypoint3D<PointType, KeypointType> Detector;
    typedef pcl::FPFHSignature33 DescriptorType;
    typedef pcl::PointCloud<PointType> Cloud;
    typedef pcl::PointCloud<NormalType> NormalCloud;
    typedef pcl::PointCloud<KeypointType> KeypointCloud;
    typedef pcl::PointCloud<DescriptorType> DescriptorCloud;

    /*!
     * Constructor which creates a Digest from a pointcloud stored in the filesystem
     */
    Digest(Cloud::Ptr cloud, std::shared_ptr<Digest::Parameters> params)
      : cloud_(cloud),
        reduced_cloud_(new Cloud),
        normal_cloud_(new NormalCloud), 
        valid_normal_cloud_indices_(new std::vector<int>),
        keypoint_cloud_(new KeypointCloud),
        valid_keypoint_cloud_indices_(new std::vector<int>),
        descriptor_cloud_(new DescriptorCloud),
        params_(params)
    {
      // Apply voxel grid filter
      voxelGrid();
      // Get the normals of the pointcloud
      calcNormals();
      // Calculate the Harris3D Keypoints
      calcHarris3D();
      // Calculate the indices of the keypoints we want to use (e.g. those over a threshold)
      calcKeypointIndices();
      // Calculate the descriptor for all points with the above indices
      calcDescriptors();
    };

    /*! 
     * Destructor.
     */
    virtual ~Digest() {
    };

    /*!
     * Returns the pointer to the pointcloud.
     */
    Cloud::Ptr getCloud() const {
      return cloud_;
    }

    /*!
     * Returns the pointer to the reduced pointcloud.
     */
    Cloud::Ptr getReducedCloud() const {
      return reduced_cloud_;
    }

    /*!
     * Returns the pointer to the normals of the reduced pointcloud.
     */
    NormalCloud::Ptr getNormalCloud() const {
      return normal_cloud_;
    }

    /*!
     * Returns the pointer to the indices for the valid normals.
     */
    pcl::IndicesPtr getNormalCloudIndices() const {
      return valid_normal_cloud_indices_;
    }

    /*!
     * Returns the pointer to the keypoints of the reduced pointcloud.
     */
    KeypointCloud::Ptr getKeypointCloud() const {
      return keypoint_cloud_;
    }

    /*!
     * Returns the pointer to the indices for the valid keypoints of the reduced pointcloud.
     */
    pcl::IndicesPtr getKeypointCloudIndices() const {
      return valid_keypoint_cloud_indices_;
    }

    /*!
     * Returns the pointer to the descriptors of the reduced pointcloud (see getKeypointCloudIndices to find out for which points the descriptors are calculated).
     */
    DescriptorCloud::Ptr getDescriptorCloud() const {
      return descriptor_cloud_;
    }

  protected:
    Cloud::Ptr cloud_;
    Cloud::Ptr reduced_cloud_;
    NormalCloud::Ptr normal_cloud_;
    pcl::IndicesPtr valid_normal_cloud_indices_;
    KeypointCloud::Ptr keypoint_cloud_;
    pcl::IndicesPtr valid_keypoint_cloud_indices_;
    DescriptorCloud::Ptr descriptor_cloud_;
    std::shared_ptr<struct Parameters> params_;

    void voxelGrid() {
      pcl::VoxelGrid<PointType> vg;
      vg.setInputCloud(cloud_);
      vg.setLeafSize(params_->voxelgrid_size, params_->voxelgrid_size, params_->voxelgrid_size);
      vg.filter(*reduced_cloud_);
    };

    void calcNormals() {
    int not_finite_count = 0;
      pcl::NormalEstimation<PointType, NormalType> normal_estimation;
      pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
      normal_estimation.setInputCloud(reduced_cloud_);
      normal_estimation.setSearchMethod(tree);
      normal_estimation.setRadiusSearch(params_->normal_radius);
      normal_estimation.compute(*normal_cloud_);
      for (size_t i = 0; i < normal_cloud_->size(); ++i)
      {
        if (!pcl::isFinite<NormalType>(normal_cloud_->points[i])) 
        {
          ++not_finite_count;
        }
        else
        {
          valid_normal_cloud_indices_->push_back(i);
        }
      }
      if (not_finite_count > 0)
      {
        std::cout << not_finite_count << " of " << normal_cloud_->size() << " cloud normals are not finite!" << std::endl;
      }
      normal_cloud_->sensor_orientation_ = cloud_->sensor_orientation_;
      normal_cloud_->sensor_origin_ = cloud_->sensor_origin_;
    };

    void calcHarris3D() {
      Detector::Ptr detector(new pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>);
      detector->setNonMaxSupression(true);
      detector->setRefine(false);
      detector->setNormals(normal_cloud_);
      detector->setRadius(params_->keypoint_radius);
      //detector->setRadiusSearch(params_->keypoint_radius);
      detector->setIndices(valid_normal_cloud_indices_);
      detector->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::CURVATURE);
      detector->setInputCloud(reduced_cloud_);
      detector->compute(*keypoint_cloud_);

      keypoint_cloud_->sensor_orientation_ = cloud_->sensor_orientation_;
      keypoint_cloud_->sensor_origin_ = cloud_->sensor_origin_;
      std::cout << "Extracted " << keypoint_cloud_->size() << " Harris3D keypoints." << std::endl;
    };

    void calcKeypointIndices() {
      for (std::vector<int>::iterator it = valid_normal_cloud_indices_->begin(); it != valid_normal_cloud_indices_->end(); ++it)
      {
        // TODO: Instead of a fixed threshold, choose a number of keypoints you want and let the algorithm decide the threshold needed for that.
        if (keypoint_cloud_->points[*it].intensity >= params_->keypoint_threshold)
        {
          valid_keypoint_cloud_indices_->push_back(*it);
        }
      }
    };

    void calcDescriptors() {
      pcl::FPFHEstimation<pcl::PointXYZ, NormalType, pcl::FPFHSignature33> fpfh_estimation;
      fpfh_estimation.setInputNormals(normal_cloud_);
      fpfh_estimation.setIndices(valid_keypoint_cloud_indices_);
      fpfh_estimation.setInputCloud(reduced_cloud_);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      fpfh_estimation.setSearchMethod(tree);
      fpfh_estimation.setRadiusSearch(params_->descriptor_radius);
      // Compute the features
      fpfh_estimation.compute(*descriptor_cloud_);
      descriptor_cloud_->sensor_orientation_ = cloud_->sensor_orientation_;
      descriptor_cloud_->sensor_origin_ = cloud_->sensor_origin_;
      // Debug-output
      std::cout << "Extracted " << descriptor_cloud_->size() << " descriptors." << std::endl;
    };
};

#endif

