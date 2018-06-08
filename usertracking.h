#ifndef USERTRACKING_H
#define USERTRACKING_H


#include <QObject>
#include <QMutex>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include "user_commons.h"

typedef pcl::tracking::ParticleXYZRPY ParticleT;
typedef pcl::tracking::ParticleFilterOMPTracker<PointType, ParticleT> ParticleFilter;
typedef typename ParticleFilter::CoherencePtr CoherencePtr;

class UserTracking : public QObject
{
    Q_OBJECT
public:
    explicit UserTracking(QObject *parent = nullptr, int thread_nr = 4);
    ~UserTracking();

    void
    setReferenceCloud(const CloudConstPtr &target);

    void
    trackingRoutine(const CloudPtr &cloud_pass);

    void
    tracking(const CloudPtr &cloud);

    void
    stopTracking()
    {
        if(tracker_mutex_.tryLock(500))
        {
            tracker_->resetTracking();
            hasTarget_ = false;
            tracker_mutex_.unlock();
        }
        else
        {
            emit errorHandler("Cannot stop tracker",QMSGBOX_Critical);
        }
    }

    bool
    hasTarget()
    {
       return hasTarget_;
    }

    double
    getTrackingTime()
    {
        return tracking_time_;
    }

    void
    getBoundingBox(double &x_min, double &x_max,
                   double &y_min, double &y_max,
                   double &z_min, double &z_max)
    {
        tracker_->calcBoundingBox(x_min, x_max,
                                  y_min, y_max,
                                  z_min, z_max);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    getParticleCloud()
    {
        ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles();
        if(particles)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t i = 0; i < particles->points.size (); i++)
            {
              pcl::PointXYZ point;

              point.x = particles->points[i].x;
              point.y = particles->points[i].y;
              point.z = particles->points[i].z;
              particle_cloud->points.push_back (point);
            }
            return particle_cloud;
        }
        else
        {
           std::cerr << "No particle found" << std::endl;
           return nullptr;
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    getParticleCentroid()
    {
        ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles();
        if(particles)
        {
            pcl::PointXYZ centroid;
            for (size_t i = 0; i < particles->points.size (); i++)
            {

              centroid.x += particles->points[i].x;
              centroid.y += particles->points[i].y;
              centroid.z += particles->points[i].z;
            }
            centroid.x = centroid.x/particles->points.size();
            centroid.y = centroid.y/particles->points.size();
            centroid.z = centroid.z/particles->points.size();
            pcl::PointCloud<pcl::PointXYZ>::Ptr particle_centroid(new pcl::PointCloud<pcl::PointXYZ>);
            particle_centroid->points.push_back(centroid);
            return particle_centroid;
        }
        else
        {
           std::cerr << "No particle found" << std::endl;
           return nullptr;
        }
    }

    double
    getFitRatio()
    {
        return tracker_->getFitRatio();
    }

    int
    getReferenceCloudSize()
    {
        return tracker_->getReferenceCloud()->points.size();
    }

    int
    getParticleSize()
    {
        return tracker_->getParticles()->points.size();
    }

    double
    getTrackingFPS()
    {
        return 1.0/tracking_time_;
    }

    boost::shared_ptr<ParticleFilter> tracker_;
    double tracking_time_;

    CloudPtr reference_;
    QMutex tracker_mutex_;

signals:
    void
    errorHandler(const QString &error, int type);

private:
    bool hasTarget_;
    double downsampling_grid_size_;

};

#endif // USERTRACKING_H
