#ifndef COMMONS_H
#define COMMONS_H

/*
 *  Vu Quoc Anh - vu.quoc.anh.ee@gmail.com
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>

#include <ctime>

#include <boost/format.hpp>


#define QMSGBOX_NoICon 		0
#define QMSGBOX_Info 		1
#define QMSGBOX_Warning 	2
#define QMSGBOX_Critical	3
#define QMSGBOX_Question	4

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef typename pcl::search::KdTree<PointType> KdTree;
typedef typename KdTree::Ptr KdTreePtr;

typedef pcl::SHOT352 DescriptorType;
typedef pcl::PointCloud<DescriptorType> DescCloud;
typedef DescCloud::Ptr DescCloudPtr;
typedef DescCloud::ConstPtr DescCloudConstPtr;

// Enable std::vector<CloudPtr> to be used in QSignal
Q_DECLARE_METATYPE(std::vector<CloudPtr>);

#define DATABASE_ROOT "/home/vuquocanh/Documents/pcl/user_database/"
#define SCENE_ROOT	  "/home/vuquocanh/Documents/pcl/user_scene/"
#define DEFAULT_MODEL_DIR "/home/vuquocanh/Documents/pcl/user_database/sapporo_blue/"

#define DOWN_SAMPLE_RESOLUTION	(0.01f)
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

#define PRINT_CURRENT_TIME(_WHAT_) \
do \
{  \
   std::time_t t = std::time(NULL);\
   std::cout << "----------- " <<_WHAT_ << " compiled at " << std::ctime(&t) << std::endl;\
}while(false)
#endif // COMMONS_H
