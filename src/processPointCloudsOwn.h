
#ifndef PROCESSPOINTCLOUDS_OWN_H_
#define PROCESSPOINTCLOUDS_OWN_H_
#include "processPointClouds.h"
#include <unordered_set>

template<typename PointT>
class ProcessPointCloudsOwn : public ProcessPointClouds<PointT>{
    public:
    //template<typename PointT>
    class Ransac
    {
        static std::vector<float> findLineCoefficents(float x1, float y1, float z1, float x2, float y2, float z2, float x3, float y3, float z3);
        static float getPointDistance(float a, float b, float c, float d, float x, float y, float z);
        public:
        static std::unordered_set<int> ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,float distanceTol);
        /*static std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(
            typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold); */
    };
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(
        typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold) override;

};
#endif