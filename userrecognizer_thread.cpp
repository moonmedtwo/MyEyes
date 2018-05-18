#include "userrecognizer_thread.h"

#include <pcl/io/pcd_io.h>

#define xONLY_CLUSTERING

UserRecognizer_Thread::UserRecognizer_Thread
        (CloudPtr &scene_pass,
         const std::vector<std::string> &model_list,
         QObject *parent)
       : QThread(parent),
         found_(false),
         model_list_(model_list)
{
    std::cout << "--- " << __FUNCTION__ << " ---------" << std::endl;
    PRINT_CURRENT_TIME(__FUNCTION__);
    scene_pass_.reset(new Cloud);
    scene_pass_.swap(scene_pass);
    if(model_list.size() == 0)
        emit(errorHandler("No models found",QMSGBOX_Warning));
}

UserRecognizer_Thread::~UserRecognizer_Thread()
{
    std::cout << "--- " << __FUNCTION__ << " ---------" << std::endl;
}

/*
 * @brief: find correspondences between 2 pointcloud features
 * @brief: using KdTree and Euclidean distance
 */
pcl::CorrespondencesPtr
UserRecognizer_Thread::correspondencesGrouping(const DescCloudConstPtr &model_descriptors,
                                               const DescCloudConstPtr &scene_descriptors,
                                               std::vector<int> &model_good_keypoints_indices,
                                               std::vector<int> &scene_good_keypoints_indices)
{
    std::cout << "--- " << __FUNCTION__ << " ---------" << std::endl;
    pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());
    pcl::KdTreeFLANN<DescriptorType> match_search;
    match_search.setInputCloud(model_descriptors);
    for (size_t i = 0; i < scene_descriptors->size (); ++i)
    {
      std::vector<int> neigh_indices (1);
      std::vector<float> neigh_sqr_dists (1);
      if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0]))  //skipping NaNs
      {
        continue;
      }
      int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
      if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)//0.25f)
      {
        pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
        model_scene_corrs->push_back (corr);
        model_good_keypoints_indices.push_back (corr.index_query);
        scene_good_keypoints_indices.push_back (corr.index_match);
      }
    }
    return model_scene_corrs;
}

/*
 * @brief: align the suspected generated pointclouds with the scene cloud
 * param [in1]: scene cloud
 * param [in2]: vector of generated pointclouds
 * param [out]: vector of align clouds
 */
std::vector<CloudConstPtr>
UserRecognizer_Thread::ICP_align(const CloudConstPtr &inputTarget,
                                 const std::vector<CloudConstPtr> &inputSources)
{
    std::cout << "--- " << __FUNCTION__ << " ---------" << std::endl;
    std::vector<CloudConstPtr> registered_instances;
    for (size_t i = 0; i < inputSources.size (); ++i)
    {
      pcl::IterativeClosestPoint<PointType, PointType> icp;
      icp.setMaximumIterations (5);
      icp.setMaxCorrespondenceDistance (0.05f);
      icp.setInputTarget (inputTarget);
      icp.setInputSource (inputSources[i]);
      CloudPtr registered (new Cloud);
      icp.align (*registered);
      registered_instances.push_back (registered);
      std::cout << "Instance " << i << " ";
      if (icp.hasConverged ())
      {
        std::cout << "Aligned!" << std::endl;
        std::cout << "--- icp.hasConverged: " << icp.hasConverged()
                  << " --- icp.getFitnessScore: " << icp.getFitnessScore()
                  << std::endl;
      }
      else
      {
        std::cout << "Not Aligned!" << std::endl;
      }
    }
    return registered_instances;
}
/*
 * @brief: Global Hypothesis verification
 */
std::vector<CloudConstPtr>
UserRecognizer_Thread::HypothesisVerification(const CloudPtr &scene_cloud,
                                              std::vector<CloudConstPtr> &models)
{
    std::cout << "--- " << __FUNCTION__ << " ---------" << std::endl;
    std::vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses

    pcl::GlobalHypothesesVerification<PointType, PointType> GoHv;

    GoHv.setSceneCloud (scene_cloud);
    GoHv.addModels (models, true);  //Models to verify

    GoHv.setInlierThreshold (0.01f);//(hv_inlier_th_);
    GoHv.setOcclusionThreshold (0.01f);
    GoHv.setRegularizer (3.0f);
    GoHv.setRadiusClutter (0.03f);
    GoHv.setClutterRegularizer (5.0f);
    GoHv.setDetectClutter (true);
    GoHv.setRadiusNormals (0.05f);

    GoHv.verify ();
    GoHv.getMask (hypotheses_mask);  // i-element TRUE if hvModels[i] verifies hypotheses
    std::vector<CloudConstPtr> founds;

    for (unsigned i = 0; i < hypotheses_mask.size (); i++)
    {
      if (hypotheses_mask[i])
      {
        std::cout << "Instance " << i << " is GOOD! <---" << std::endl;
        founds.push_back(models[i]);
      }
      else
      {
        std::cout << "Instance " << i << " is bad!" << std::endl;
      }
    }
    std::cout << "-------------------------------" << std::endl;
    return founds;
}

/*
 * @brief: regroup/filter the correspondences using geometric
 * @param [out]: rototranslations of filtered objects
 */
std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >
UserRecognizer_Thread::
GeoConGrouping(pcl::CorrespondencesPtr &model_scene_corrs,
               const CloudConstPtr &model_keypoints,
               const CloudConstPtr &scene_keypoints)
{
    std::cout << "--- " << __FUNCTION__ << " ---------" << std::endl;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector < pcl::Correspondences > clustered_corrs;
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    gc_clusterer.setGCSize (0.01f);
    gc_clusterer.setGCThreshold (5.0f);
    gc_clusterer.setInputCloud (model_keypoints);
    gc_clusterer.setSceneCloud (scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);
    gc_clusterer.recognize (rototranslations, clustered_corrs);

    return rototranslations;
}
/*
 * @brief: wrapper for recognition pipeline with SHOT
 * @param [out]: rototranslation of detectable objects
 */
std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >
UserRecognizer_Thread::
w_SHOT_recognizer()
{
    std::cout << "--- " << __FUNCTION__ << " ---------" << std::endl;
    scene_normals_.reset(new pcl::PointCloud<pcl::Normal>);
    computeNormals_OMP(scene_pass_,scene_normals_);
    model_normals_.reset(new pcl::PointCloud<pcl::Normal>);
    computeNormals_OMP(model_,model_normals_);

    /* choose keypoints */
    scene_keypoints_.reset(new Cloud);
    gridSample(scene_pass_,scene_keypoints_);
    model_keypoints_.reset(new Cloud);
    gridSample(model_,model_keypoints_);
    std::cout << "Swap scene_kp by the one with all planars removed" << std::endl;
    removePlanars(scene_keypoints_,scene_without_planars_);
    scene_keypoints_.swap(scene_without_planars_);
    std::cout << "Scene total points: " << scene_pass_->points.size() << "; "
              << "Seleted Keypoints: " << scene_keypoints_->points.size()
              << std::endl;
    std::cout << "Model total points: " << model_->points.size() << "; "
              << "Seleted Keypoints: " << model_keypoints_->points.size()
              << std::endl;
    /* choose keypoints */

    /* calculate descriptors */
    DescCloudPtr scene_descriptors(new DescCloud);
    computeSHOTs_OMP(scene_keypoints_,scene_pass_,scene_normals_,
                     scene_descriptors);
    DescCloudPtr model_descriptors(new DescCloud);
    computeSHOTs_OMP(model_keypoints_,model_,model_normals_,
                     model_descriptors);
    /* calculate desriptors */

    /* find correspondences */
    std::vector<int> model_good_keypoints_indices;
    std::vector<int> scene_good_keypoints_indices;
    pcl::CorrespondencesPtr model_scene_corrs = \
            correspondencesGrouping(model_descriptors, scene_descriptors,
                                    model_good_keypoints_indices, scene_good_keypoints_indices);
    CloudPtr model_good_kp (new Cloud ());
    CloudPtr scene_good_kp (new Cloud ());
    pcl::copyPointCloud (*model_keypoints_, model_good_keypoints_indices, *model_good_kp);
    pcl::copyPointCloud (*scene_keypoints_, scene_good_keypoints_indices, *scene_good_kp);
    std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;
    /* find correspondences */

    /* GC clustering */
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >\
            rototranslations = GeoConGrouping(model_scene_corrs,
                                              model_keypoints_,
                                              scene_keypoints_);
    /* GC clustering */
   return rototranslations;
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
    std::cout << "--- " << __FUNCTION__ << " ---------" << std::endl;
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
    std::cout << "--- " << __FUNCTION__ << " ---------" << std::endl;
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
    std::cout << "--- " << __FUNCTION__ << " ---------" << std::endl;
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
    std::cout << "--- " << __FUNCTION__ << " ---------" << std::endl;
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
    std::cout << "--- " << __FUNCTION__ << " ---------" << std::endl;
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
    std::cout << "--- " << __FUNCTION__ << " ---------" << std::endl;
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
    std::cout << "--- " << __FUNCTION__ << " ---------" << std::endl;
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
    std::cout << "--- " << __FUNCTION__ << " ---------" << std::endl;
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
                                  const CloudPtr &result, double leaf_size)
{
    std::cout << "--- " << __FUNCTION__ << " ---------" << std::endl;
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
UserRecognizer_Thread::computeNormals_OMP(const CloudConstPtr &cloud,
                                      pcl::PointCloud<pcl::Normal>::Ptr &result)
{
    std::cout << "--- " << __FUNCTION__ << " ---------" << std::endl;
    pcl::NormalEstimationOMP<PointType,pcl::Normal> ne;
    ne.setKSearch(10);
    ne.setInputCloud(cloud);
    ne.compute(*result);
}
/*
 * @brief: compute shot descriptor utilising OpenMP library
 */
void
UserRecognizer_Thread::computeSHOTs_OMP(const CloudConstPtr &cloud_downSampled,
                                        const CloudConstPtr &search_surface,
                                        const pcl::PointCloud<pcl::Normal>::ConstPtr &cloud_normals,
                                        DescCloudPtr &desc, float radius)
{
    std::cout << "--- " << __FUNCTION__ << " ---------" << std::endl;
    pcl::SHOTEstimationOMP<PointType,pcl::Normal,DescriptorType> descr_est;
    descr_est.setRadiusSearch(radius);
    descr_est.setInputCloud(cloud_downSampled);
    descr_est.setSearchSurface(search_surface);
    descr_est.setInputNormals(cloud_normals);
    descr_est.compute(*desc);
}
/*
 * @brief: compute shot utilising OpenMP
 * note: normal size must equal to cloud size
 * note: and this can be achieve ball_clusteredy using the non-downsampled cloud
 */
void
UserRecognizer_Thread::computeSHOTs_OMP(const CloudConstPtr &cloud_downSampled,
                                        const pcl::PointCloud<pcl::Normal>::ConstPtr &cloud_normals,
                                        DescCloudPtr &desc, float radius)
{
    std::cout << "--- " << __FUNCTION__ << " ---------" << std::endl;
    pcl::SHOTEstimationOMP<PointType,pcl::Normal,DescriptorType> descr_est;
    descr_est.setRadiusSearch(radius);
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
    std::cout << "--- " << __FUNCTION__ << " ---------" << std::endl;
    for(const auto &model_path : model_list_)
    {
        std::cout << "model at " << model_path << std::endl;
        model_.reset(new Cloud);
        pcl::PCDReader reader;
        reader.read(model_path,*model_);
        if(model_->points.size() <= 0)
        {
            std::stringstream ss;
            ss << "invalid input" << std::endl;
            ss << "--- model: " << model_->points.size() << std::endl;
            // Skip to next model
            continue;
        }
        scene_keypoints_.reset(new Cloud);
        gridSample(scene_pass_,scene_keypoints_);
#ifdef OPENNI_TRACKING_BASED_SEGMENATION_CLUSTERING
        removePlanars(scene_keypoints_,scene_without_planars_);
        cluster(scene_without_planars_,clusteredObjects_);
#endif //OPENNI_TRACKING_BASED_SEGMENATION_CLUSTERING
#ifdef OLD_SEGMENTATION_CLUSTERING
            extractPlanars(scene_keypoints_,planars_);
            cluster(scene_keypoints_,clusteredObjects_);
#endif //OLD_SEGMENTATION_CLUSTERING
         std::cout << "Planars extracted: " << planars_.size() << std::endl;
         std::cout << "Objects found: " << clusteredObjects_.size() << std::endl;

#ifdef ONLY_CLUSTERING
            // Signal the main app
            emit resultReady(clusteredObjects_);
            return;
#else
         std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >\
                 rototranslations = w_SHOT_recognizer();
         if (rototranslations.size () == 0)
         {
           // Skip to next model
           continue;
         }
         else std::cout << "Recognized Instances: " << rototranslations.size () << std::endl << std::endl;

         /**
          * Generates clouds for each instances found
          */
         std::vector<CloudConstPtr> instances;
         for (size_t i = 0; i < rototranslations.size (); ++i)
         {
           pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
           pcl::transformPointCloud (*model_keypoints_, *rotated_model, rototranslations[i]);
           instances.push_back (rotated_model);
         }

         std::vector<CloudConstPtr> aligned_instances = ICP_align(scene_keypoints_,instances);
         if (aligned_instances.size () == 0)
         {
           // Skip to next model
           continue;
         }

         /**
          * Hypothesis Verification
          */
         std::vector<CloudConstPtr>
                 results = HypothesisVerification(scene_keypoints_,aligned_instances);

         if(results.size() > 0)
         {
            std::vector<CloudPtr> tmpResult;
            for(const auto &p :results)
            {
               CloudPtr a(new Cloud);
               pcl::copyPointCloud(*p,*a);
               tmpResult.push_back(a);
            }
            found_ = true;
            emit(resultReady(tmpResult));
         }
        // Exit if result is ready
        if(found_)
        {
            // TODO change this, emit progess bar
//            std::stringstream ss;
//            ss << "Found with model @ " << model_path;
//            emit(errorHandler(QString::fromStdString(ss.str()),QMSGBOX_Info));
            return;
        }

#endif // ONLY_CLUSTERING
    }

    if(!found_) emit(errorHandler("No instance found",QMSGBOX_Warning));
}
