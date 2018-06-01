#include "usertracking.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#define PARTICLES_NR	400
void removeZeroPoints (const CloudConstPtr &cloud,
                       Cloud &result)
{
  for (size_t i = 0; i < cloud->points.size (); i++)
  {
    PointType point = cloud->points[i];
    if (!(fabs(point.x) < 0.01 &&
          fabs(point.y) < 0.01 &&
          fabs(point.z) < 0.01) &&
        !pcl_isnan(point.x) &&
        !pcl_isnan(point.y) &&
        !pcl_isnan(point.z))
      result.points.push_back(point);
  }

  result.width = static_cast<pcl::uint32_t> (result.points.size ());
  result.height = 1;
  result.is_dense = true;
}
void
gridSample (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
{
  pcl::VoxelGrid<PointType> grid;
  //pcl::ApproximateVoxelGrid<PointType> grid;
  grid.setLeafSize (float (leaf_size), float (leaf_size), float (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (result);
  //result = *cloud;
}

void
gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
{
  //pcl::VoxelGrid<PointType> grid;
  pcl::ApproximateVoxelGrid<PointType> grid;
  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (result);
  //result = *cloud;
}

using namespace pcl::tracking;
UserTracking::UserTracking(QObject *parent, int thread_nr)
    : QObject(parent),
      tracking_time_(1),
      hasTarget_(false),
      downsampling_grid_size_(0.01)
{
    boost::shared_ptr<ParticleFilterOMPTracker<PointType, ParticleT> > tracker
        (new ParticleFilterOMPTracker<PointType, ParticleT> (thread_nr));

    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;

    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

    tracker_ = tracker;
    tracker_->setTrans (Eigen::Affine3f::Identity ());
    tracker_->setStepNoiseCovariance (default_step_covariance);
    tracker_->setInitialNoiseCovariance (initial_noise_covariance);
    tracker_->setInitialNoiseMean (default_initial_mean);
    tracker_->setIterationNum (1);

    tracker_->setParticleNum (PARTICLES_NR);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal (false);


    // setup coherences
    ApproxNearestPairPointCloudCoherence<PointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<PointType>::Ptr
      (new ApproxNearestPairPointCloudCoherence<PointType> ());
    // NearestPairPointCloudCoherence<PointType>::Ptr coherence = NearestPairPointCloudCoherence<PointType>::Ptr
    //   (new NearestPairPointCloudCoherence<PointType> ());

    boost::shared_ptr<DistanceCoherence<PointType> > distance_coherence
      = boost::shared_ptr<DistanceCoherence<PointType> > (new DistanceCoherence<PointType> ());
    coherence->addPointCoherence (distance_coherence);

    boost::shared_ptr<HSVColorCoherence<PointType> > color_coherence
      = boost::shared_ptr<HSVColorCoherence<PointType> > (new HSVColorCoherence<PointType> ());
    color_coherence->setWeight (0.1);
    coherence->addPointCoherence (color_coherence);

    //boost::shared_ptr<pcl::search::KdTree<PointType> > search (new pcl::search::KdTree<PointType> (false));
    boost::shared_ptr<pcl::search::Octree<PointType> > search (new pcl::search::Octree<PointType> (0.01));
    //boost::shared_ptr<pcl::search::OrganizedNeighbor<PointType> > search (new pcl::search::OrganizedNeighbor<PointType>);
    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (0.01);
    tracker_->setCloudCoherence (coherence);
}

UserTracking::~UserTracking()
{

}

void
UserTracking::setReferenceCloud(const CloudConstPtr &target)
{
    if(target->points.size() == 0)
    {
        emit errorHandler("Input target invalid",QMSGBOX_Critical);
        return;
    }
    // Create reference cloud for particle filter
    CloudPtr nonzero_ref(new Cloud);
    removeZeroPoints(target,*nonzero_ref);

    PCL_INFO ("calculating cog\n");

    CloudPtr transed_ref (new Cloud);
    Eigen::Vector4f c;
    pcl::compute3DCentroid<PointType> (*nonzero_ref, c);
    Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
    trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
    //pcl::transformPointCloudWithNormals<PointType> (*ref_cloud, *transed_ref, trans.inverse());
    pcl::transformPointCloud<PointType> (*nonzero_ref, *transed_ref, trans.inverse());
    tracker_->setReferenceCloud (transed_ref);
    tracker_->setTrans (trans);
    reference_ = transed_ref;
    tracker_->setMinIndices (int (target->points.size ()) / 2);

    hasTarget_ = true;
}

void
UserTracking::trackingRoutine(const CloudPtr &cloud_pass)
{
    CloudPtr cloud_pass_downsampled(new Cloud);
    gridSampleApprox (cloud_pass, *cloud_pass_downsampled, downsampling_grid_size_);
    tracking(cloud_pass_downsampled);
}

void
UserTracking::tracking(const CloudPtr &cloud)
{
    double start = pcl::getTime ();
    tracker_->setInputCloud (cloud);
    tracker_->compute ();
    double end = pcl::getTime ();
    tracking_time_ = end - start;
}

