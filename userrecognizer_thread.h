/*
 * Author: Vu Quoc Anh - 1410149@hcmut.edu.vn/vu.quoc.anh.ee@gmail.com
 * @brief: a thread to recognize a model in scene, works independently in a different thread using QThread
 * @note[1]: There are 2 modes of segmentation, just define one or another
 * input: cloud_pass_
 * output: list of centroid of jrecognized object
 */
#ifndef USERRECOGNIZER_THREAD_H
#define USERRECOGNIZER_THREAD_H

#include <QThread>
#include "commons.h"
#include <vector>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>

#ifdef forever // Solve macro conflict from Qt and metlibs
#undef forever
#include <pcl/recognition/hv/hv_go.h>
#define forever Q_FOREVER
#endif

#include <pcl/registration/icp.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>

#include <pcl/search/search.h>

#include <pcl/Vertices.h>

#include <pcl/surface/convex_hull.h>

#define xOLD_SEGMENTATION_CLUSTERING // Segmentation implemented in pcl_clustering tutorials
#define OPENNI_TRACKING_BASED_SEGMENATION_CLUSTERING // Segmentation implemented in openni_tracking tutorials


class UserRecognizer_Thread : public QThread
{
    Q_OBJECT
public:
    UserRecognizer_Thread(CloudPtr &scene_pass,
                          const std::vector<std::string> &model_list,
                          QObject *parent = 0);
    ~UserRecognizer_Thread();

    void
    gridSample(const CloudConstPtr &cloud, const CloudPtr &result,
               double leaf_size = 0.01);
    void
    computeNormals_OMP(const CloudConstPtr &cloud,
                   pcl::PointCloud<pcl::Normal>::Ptr &result);
    void
    computeSHOTs_OMP(const CloudConstPtr &cloud_downSampled,
                     const CloudConstPtr &search_surface,
                     const pcl::PointCloud<pcl::Normal>::ConstPtr &cloud_normals,
                     DescCloudPtr &desc, float radius = 0.02);
    void
    computeSHOTs_OMP(const CloudConstPtr &cloud_downSampled,
                     const pcl::PointCloud<pcl::Normal>::ConstPtr &cloud_normals,
                     DescCloudPtr &desc, float radius = 0.02);

    pcl::CorrespondencesPtr
    correspondencesGrouping(const DescCloudConstPtr &model_descriptors,
                            const DescCloudConstPtr &scene_descriptors,
                            std::vector<int> &model_good_keypoints_indices,
                            std::vector<int> &scene_good_keypoints_indices);

    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >
    GeoConGrouping(pcl::CorrespondencesPtr &model_scene_corrs,
                   const CloudConstPtr &model_keypoints,
                   const CloudConstPtr &scene_keypoints);

    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >\
    w_SHOT_recognizer();

#ifdef OPENNI_TRACKING_BASED_SEGMENATION_CLUSTERING
    void
    planeSegmentation (const CloudConstPtr &cloud,
                       pcl::ModelCoefficients &coefficients,
                       pcl::PointIndices &inliers);
    void
    planeProjection (const CloudConstPtr &cloud,
                     Cloud &result,
                     const pcl::ModelCoefficients::ConstPtr &coefficients);
    void
    convexHull (const CloudConstPtr &cloud,
                CloudPtr &cloud_hull,
                std::vector<pcl::Vertices> &hull_vertices);
    void
    extractNonPlanePoints (const CloudConstPtr &cloud,
                           const CloudConstPtr &cloud_hull,
                           Cloud &result);
    void
    euclideanSegment (const CloudConstPtr &cloud,
                      std::vector<pcl::PointIndices> &cluster_indices);
    void
    extractSegmentCluster (const CloudConstPtr &cloud,
                           const std::vector<pcl::PointIndices> cluster_indices,
                           const int segment_index,
                           Cloud &result);
    void
    removePlanars(const CloudConstPtr &scene_pass_downSampled,
                  CloudPtr &scene_without_planars);
    void
    cluster(CloudPtr &scene_removedPlanars,
            std::vector<CloudPtr> &clusteredObjects);

    std::vector<CloudConstPtr>
    ICP_align(const CloudConstPtr &inputTarget,
              const std::vector<CloudConstPtr> &inputSources);

    std::vector<CloudConstPtr>
    HypothesisVerification(const CloudPtr &scene_cloud,
                           std::vector<CloudConstPtr> &models);

//    std::vector<CloudPtr>
//    HypothesisVerification(const CloudPtr &scene_cloud,
//                           std::vector<CloudPtr> &models);
#endif //OPENNI_TRACKING_BASED_SEGMENATION_CLUSTERING

#ifdef OLD_SEGMENATION_CLUSTERING
    void
    extractPlanars(CloudPtr &scene, std::vector<CloudPtr> &planars);

    void
    cluster(CloudPtr &scene_removedPlanars, std::vector<CloudPtr> &clusteredObjects);
#endif //OLD_SEGMENTATION_CLUSTERING

signals:
    void
    resultReady(std::vector<CloudPtr> clusteredObjects);

    void
    progress(int percent, int model, int totalmodel);

    void
    errorHandler(const QString &error, int type);

protected:
    void run() override;

    bool found_;

    CloudPtr scene_pass_;
    CloudPtr model_;
    CloudPtr scene_keypoints_;
    CloudPtr model_keypoints_;

    pcl::PointCloud<pcl::Normal>::Ptr scene_normals_;
    pcl::PointCloud<pcl::Normal>::Ptr model_normals_;

    std::vector<CloudPtr> planars_;
    std::vector<CloudPtr> clusteredObjects_;

    std::vector<std::string> model_list_;

#ifdef OPENNI_TRACKING_BASED_SEGMENATION_CLUSTERING
    CloudPtr scene_without_planars_;
    CloudPtr cloud_hull_;
#endif

};

#endif // USERRECOGNIZER_THREAD_H
