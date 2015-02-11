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

#include <eigen3/Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/voxel_grid.h>

/*!
 * Class to create and store the important features and descriptors for one pointcloud.
 * Two digests can be used by the DigestMatch to check whether the pointclouds overlap and to get a tranformation between the two pointclouds.
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
      float keypoint_threshold = 0.01;
      float descriptor_radius = 4;
    };

    typedef std::shared_ptr<Digest> Ptr;
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
    Digest(Cloud::Ptr cloud, struct Parameters& params)
      : cloud_(cloud),
        reduced_cloud_(new Cloud),
        normal_cloud_(new NormalCloud), 
        valid_normal_cloud_indices_(new std::vector<int>),
        keypoint_cloud_(new KeypointCloud),
        descriptor_cloud_indices_(new std::vector<int>),
        descriptor_cloud_(new DescriptorCloud),
        params_(params)
    {
      std::cout << "Creating digest of pointcloud with " << cloud_->size() << " points." << std::endl;
      // Apply voxel grid filter
      voxelGrid();
      std::cout << "Voxel grid filter reduced the pointcloud to " << reduced_cloud_->size() << " points." << std::endl;
      // Get the normals of the pointcloud
      calcNormals();
      std::cout << "Got " << valid_normal_cloud_indices_->size() << " valid normals." << std::endl;
      // Calculate the Harris3D Keypoints
      calcHarris3D();
      std::cout << "Extracted " << keypoint_cloud_->size() << " Harris3D keypoints." << std::endl;
      // Calculate the indices of the keypoints we want to use (e.g. those over a threshold)
      calcDescriptorIndices();
      std::cout << descriptor_cloud_indices_->size() << " Keypoints are above threshold." << std::endl;
      // Calculate the descriptor for all points with the above indices
      calcDescriptors();
      std::cout << "Extracted " << descriptor_cloud_->size() << " descriptors." << std::endl;
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
     * Returns a pointer a point cloud containing only those points for which
     * descriptors were calculated. (so which keypoints were above the threshold)
     * The descriptor for point[n] is descriptor[n].
     */
    Cloud::Ptr getDescriptorCloudPoints() const {
      Cloud::Ptr descriptor_cloud_points(new Cloud());
      for (std::vector<int>::iterator it = descriptor_cloud_indices_->begin(); it != descriptor_cloud_indices_->end(); ++it) {
        descriptor_cloud_points->push_back(reduced_cloud_->at(*it));
      }
      return descriptor_cloud_points;
    }

    /*!
     * Returns the pointer to the indices for the keypoints of the reduced pointcloud.
     * Those indices can be used to get the XYZ points or normals for each keypoint.
     * The index m of the point and normal of keypoint[n] is indices[n] = m.
     */
    pcl::IndicesPtr getKeypointCloudIndices() const {
      return keypoint_cloud_indices_;
    }

    /*!
     * Returns the pointer to the descriptors of the reduced pointcloud (see getKeypointCloudIndices to find out for which points the descriptors are calculated).
     */
    DescriptorCloud::Ptr getDescriptorCloud() const {
      return descriptor_cloud_;
    }

    /*!
     * Returns the pointer to the indices for the descriptors of the reduced pointcloud.
     * Those indices can be used to get the XYZ points or normals for each descriptor, 
     * The index m of the point and normal of descriptor[n] is indices[n] = m.
     */
    pcl::IndicesPtr getDescriptorCloudIndices() const {
      return descriptor_cloud_indices_;
    }

  protected:
    //! Stores the full point cloud
    Cloud::Ptr cloud_;
    //! Stores the reduces point cloud
    Cloud::Ptr reduced_cloud_;
    //! Stores the normals, indices corresponding to the reduced point cloud
    NormalCloud::Ptr normal_cloud_;
    //! Contains all indices to valid normals
    pcl::IndicesPtr valid_normal_cloud_indices_;
    //! Stores the keypoints (containing their position in space, redundant in reduced_cloud_)
    KeypointCloud::Ptr keypoint_cloud_;
    //! Contains the indices to get from a keypoint to its original XYZ-point (or its normal)
    pcl::IndicesPtr keypoint_cloud_indices_;
    //! Contains all indices to points with valid keypoints
    pcl::IndicesPtr descriptor_cloud_indices_;
    //! Contains the descriptors, indices corresponding to the descriptor_cloud_indices_
    DescriptorCloud::Ptr descriptor_cloud_;
    //! Stores the parameters of the digest
    struct Parameters params_;

    void voxelGrid() {
      pcl::VoxelGrid<PointType> vg;
      vg.setInputCloud(cloud_);
      vg.setLeafSize(params_.voxelgrid_size, params_.voxelgrid_size, params_.voxelgrid_size);
      vg.filter(*reduced_cloud_);
    };

    void calcNormals() {
      int not_finite_count = 0;
      pcl::NormalEstimation<PointType, NormalType> normal_estimation;
      pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
      normal_estimation.setInputCloud(reduced_cloud_);
      normal_estimation.setSearchMethod(tree);
      normal_estimation.setRadiusSearch(params_.normal_radius);
      normal_estimation.compute(*normal_cloud_);
      assert(normal_cloud_->size() == reduced_cloud_->size());
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
      detector->setInputCloud(reduced_cloud_);
      detector->setNormals(normal_cloud_);
      detector->setIndices(valid_normal_cloud_indices_);
      detector->setNonMaxSupression(false);
      detector->setRefine(false);
      detector->setRadius(params_.keypoint_radius);
      detector->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::CURVATURE);
      //detector->setRadiusSearch(params_.keypoint_radius);
      detector->compute(*keypoint_cloud_);
      keypoint_cloud_indices_.reset(new std::vector<int>(detector->getKeypointsIndices()->indices));
      assert(keypoint_cloud_indices_->size() == keypoint_cloud_->size());

      keypoint_cloud_->sensor_orientation_ = cloud_->sensor_orientation_;
      keypoint_cloud_->sensor_origin_ = cloud_->sensor_origin_;
    };

    void calcDescriptorIndices() {
      for (unsigned int i = 0; i < keypoint_cloud_indices_->size(); ++i)
      {
        // TODO: Instead of a fixed threshold, choose a number of keypoints you want and let the algorithm decide the threshold needed for that.
        if (keypoint_cloud_->points[i].intensity >= params_.keypoint_threshold)
        {
          descriptor_cloud_indices_->push_back(i);
        }
      }
    };

    void calcDescriptors() {
      pcl::FPFHEstimation<pcl::PointXYZ, NormalType, pcl::FPFHSignature33> fpfh_estimation;
      fpfh_estimation.setInputNormals(normal_cloud_);
      fpfh_estimation.setInputCloud(reduced_cloud_);
      fpfh_estimation.setIndices(descriptor_cloud_indices_);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      fpfh_estimation.setSearchMethod(tree);
      fpfh_estimation.setRadiusSearch(params_.descriptor_radius);
      // Compute the features
      fpfh_estimation.compute(*descriptor_cloud_);
      descriptor_cloud_->sensor_orientation_ = cloud_->sensor_orientation_;
      descriptor_cloud_->sensor_origin_ = cloud_->sensor_origin_;
      // Debug-output
      assert(descriptor_cloud_->size() == descriptor_cloud_indices_->size());
    };
};

#endif

