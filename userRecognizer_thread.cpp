#include "userRecognizer_thread.h"

#define ONLY_CLUSTERING

UserRecognizer_Thread::UserRecognizer_Thread
        (CloudPtr &scene_pass,
         CloudPtr &model,
         QObject *parent)
       : QThread(parent)
{
   scene_pass_.reset(new Cloud);
   scene_pass_.swap(scene_pass);
   model_.reset(new Cloud);
   model_.swap(model);
}

UserRecognizer_Thread::~UserRecognizer_Thread()
{
    std::cout << "~UserRecognizer_Thread()" << std::endl;;
}

#ifdef OLD_SEGMENTATION_CLUSTERING_
/*
 * @brief: remove planars from scene and store the planars;
 * @in: vector to store planars
 */
void
UserRecognizer_Thread::extractPlanars(CloudPtr &scene,std::vector<CloudPtr> &planars)
{
    pcl::SACSegmentation<PointType> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);


    int nr_points = (int) scene->points.size ();

    // Loop continuously extract and remove the largest planar found
    while (scene->points.size () > 0.3 * nr_points)
    {
      CloudPtr cloudPlanar(new Cloud);
      CloudPtr cloudRemaining(new Cloud);
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (scene);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<PointType> extract;
      extract.setInputCloud (scene);
      extract.setIndices (inliers);
      extract.setNegative (false);

      // Get the points associated with the planar surface
      extract.filter (*cloudPlanar);
      planars.push_back(cloudPlanar);

      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloudRemaining);
      scene.reset(new Cloud);
      scene = cloudRemaining->makeShared();
    }
}

/*
 * @brief: cluster object from scene and store the objects;
 * @in: vector to store object
 */
void
UserRecognizer_Thread::cluster(CloudPtr &scene_removedPlanars, std::vector<CloudPtr> &clusteredObjects)
{
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree->setInputCloud (scene_removedPlanars);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance (0.05); // 2cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (scene_removedPlanars);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      CloudPtr cloud_cluster (new Cloud);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
        cloud_cluster->points.push_back (scene_removedPlanars->points[*pit]); //*
      }

      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      clusteredObjects.push_back(cloud_cluster);
    }
}
#endif

#ifdef OPENNI_TRACKING_BASED_SEGMENATION_CLUSTERING
/*
 * @brief: segmentation the cloud into planars
 * @in[1] cloud: pointer to cloud
 * @in[2] coefficients: holder to store plane cof
 * @in[3] inliers: indices of point referred to planars in orignal cloud
 */
void
UserRecognizer_Thread::planeSegmentation (const CloudConstPtr &cloud,
                        pcl::ModelCoefficients &coefficients,
                        pcl::PointIndices &inliers)
{
  pcl::SACSegmentation<PointType> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud);
  seg.segment (inliers, coefficients);
}

/*
 * @brief: project cloud into a single plane
 * @in[1]: cloud: pointer to cloud
 * @in[2]: result: plane
 * @in[3]: coefficients:
 */
void
UserRecognizer_Thread::planeProjection (const CloudConstPtr &cloud,
                                        Cloud &result,
                                        const pcl::ModelCoefficients::ConstPtr &coefficients)
{
  pcl::ProjectInliers<PointType> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (result);
}

/*
 * @brief: calculate the convex hull of the cloud
 */
void
UserRecognizer_Thread::convexHull (const CloudConstPtr &cloud,
                                   CloudPtr &cloud_hull,
                                   std::vector<pcl::Vertices> &hull_vertices)
{
  pcl::ConvexHull<PointType> chull;
  chull.setInputCloud (cloud);
  chull.reconstruct (*cloud_hull, hull_vertices);
}

/*
 * @brief: extract non planar points using convex hull
 */
void
UserRecognizer_Thread::extractNonPlanePoints (const CloudConstPtr &cloud,
                                              const CloudConstPtr &cloud_hull,
                                              Cloud &result)
{
  pcl::ExtractPolygonalPrismData<PointType> polygon_extract;
  pcl::PointIndices::Ptr inliers_polygon (new pcl::PointIndices ());
  polygon_extract.setHeightLimits (0.01, 10.0);
  polygon_extract.setInputPlanarHull (cloud_hull);
  polygon_extract.setInputCloud (cloud);
  polygon_extract.segment (*inliers_polygon);
  {
    pcl::ExtractIndices<PointType> extract_positive;
    extract_positive.setNegative (false);
    extract_positive.setInputCloud (cloud);
    extract_positive.setIndices (inliers_polygon);
    extract_positive.filter (result);
  }
}
/*
 * @brief: segmentation using Euclidean distance
 */
void
UserRecognizer_Thread::euclideanSegment (const CloudConstPtr &cloud,
                       std::vector<pcl::PointIndices> &cluster_indices)
{
  pcl::EuclideanClusterExtraction<PointType> ec;
  KdTreePtr tree (new KdTree ());

  ec.setClusterTolerance (0.05); // 2cm
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (25000);
  //ec.setMaxClusterSize (400);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);
}
/*
 * @brief: extract the clustered object from segmentated clustered
 */
void
UserRecognizer_Thread::extractSegmentCluster (const CloudConstPtr &cloud,
                            const std::vector<pcl::PointIndices> cluster_indices,
                            const int segment_index,
                            Cloud &result)
{
  pcl::PointIndices segmented_indices = cluster_indices[segment_index];
  for (size_t i = 0; i < segmented_indices.indices.size (); i++)
  {
    PointType point = cloud->points[segmented_indices.indices[i]];
    result.points.push_back (point);
  }
  result.width = pcl::uint32_t (result.points.size ());
  result.height = 1;
  result.is_dense = true;
}

/*
 * @brief: remove planars from scene and store the planars;
 * @in: vector to store planars
 */
void
UserRecognizer_Thread::removePlanars(const CloudConstPtr &scene_pass_downSampled,
                                     CloudPtr &scene_without_planars)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    CloudPtr target_cloud(new Cloud);
    CloudPtr cloud_hull;
    {
      planeSegmentation (scene_pass_downSampled, *coefficients, *inliers);
      if (inliers->indices.size () > 3)
      {
        CloudPtr cloud_projected (new Cloud);
        cloud_hull.reset (new Cloud);
        scene_without_planars.reset (new Cloud);

        std::vector<pcl::Vertices> hull_vertices_;
        planeProjection (scene_pass_downSampled, *cloud_projected, coefficients);
        convexHull (cloud_projected,cloud_hull, hull_vertices_);

        extractNonPlanePoints (scene_pass_downSampled, cloud_hull, *scene_without_planars);
      }
      else
      {
        PCL_WARN ("cannot segment plane\n");
      }
    }
}

/*
 * @brief: cluster object from scene and store the objects;
 * @in: vector to store object
 */
void
UserRecognizer_Thread::cluster(CloudPtr &scene_removedPlanars,
                               std::vector<CloudPtr> &clusteredObjects)
{
    std::vector<pcl::PointIndices> cluster_indices;
    euclideanSegment (scene_removedPlanars, cluster_indices);

    for(unsigned i = 0; i < cluster_indices.size(); i++)
    {
       CloudPtr segmentedCloud(new Cloud);
       extractSegmentCluster(scene_removedPlanars,cluster_indices,i,*segmentedCloud);
       clusteredObjects.push_back(segmentedCloud);
    }
}
#endif

/*
 * @brief: down sample the cloud
 * @in[1] cloud: input cloud pointer
 * @in[2] result: pointer to store result
 */
void
UserRecognizer_Thread::gridSample(const CloudConstPtr &cloud,
                                  CloudPtr &result, double leaf_size)
{
   pcl::VoxelGrid<PointType> grid;
   //pcl::ApproximateVoxelGrid<PointType> grid;
   grid.setLeafSize (float (leaf_size), float (leaf_size), float (leaf_size));
   grid.setInputCloud (cloud);
   grid.filter (*result);
   //result = *cloud; (note: result as Cloud)
}
/*
 * @brief Compute cloudn normals utilising OpenMP library
 */
void
UserRecognizer_Thread::computeNormals(const CloudConstPtr &cloud,
                                      pcl::PointCloud<pcl::Normal>::Ptr &result)
{
   pcl::NormalEstimationOMP<PointType,pcl::Normal> ne;
   ne.setKSearch(10);
   ne.setInputCloud(cloud);
   ne.compute(*result);
}
void
UserRecognizer_Thread::computeSHOTs(const CloudConstPtr &cloud_downSampled,
                                    const pcl::PointCloud<pcl::Normal>::ConstPtr &cloud_normals,
                                    pcl::PointCloud<DescriptorType>::Ptr &desc)
{
    pcl::SHOTEstimationOMP<PointType,pcl::Normal,DescriptorType> descr_est;
    descr_est.setRadiusSearch(0.02f);
    descr_est.setInputCloud(cloud_downSampled);
    descr_est.setInputNormals(cloud_normals);
    descr_est.compute(*desc);
}
/*
 * @brief: inherited as QThread::run, automatically run after this->start
 */
void
UserRecognizer_Thread::run()
{
    // do the hard work
    // maybe emit a signal to parentl

#ifdef ONLY_CLUSTERING
    scene_pass_downSampled_.reset(new Cloud);
    gridSample(scene_pass_,scene_pass_downSampled_);
    model_downSampled_.reset(new Cloud);
    gridSample(model_,model_downSampled_);

#ifdef OPENNI_TRACKING_BASED_SEGMENATION_CLUSTERING
    removePlanars(scene_pass_downSampled_,scene_without_planars_);
    cluster(scene_without_planars_,clusteredObjects_);
#endif //OPENNI_TRACKING_BASED_SEGMENATION_CLUSTERING

#ifdef OLD_SEGMENTATION_CLUSTERING

    extractPlanars(scene_pass_downSampled_,planars_);
    cluster(scene_pass_downSampled_,clusteredObjects_);

#endif //OLD_SEGMENTATION_CLUSTERING

    std::cout << "Planars size: " << planars_.size() << std::endl;
    std::cout << "Object size: " << clusteredObjects_.size() << std::endl;
#else

#endif
}
