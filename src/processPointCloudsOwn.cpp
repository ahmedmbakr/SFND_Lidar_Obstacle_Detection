#include "processPointCloudsOwn.h"
#include "processPointClouds.cpp"

template<typename PointT>
std::vector<float> ProcessPointCloudsOwn<PointT>::Ransac::findLineCoefficents(float x1, float y1, float z1,
 float x2, float y2, float z2, float x3, float y3, float z3)
 {
     //plane equation Ax + By + Cz + D = 0
	float a = (y2-y1) * (z3-z1) - (z2-z1) * (y3-y1);
	float b = (z2-z1) * (x3-x1) - (x2-x1) * (z3-z1);
	float c = (x2-x1) * (y3-y1) - (y2-y1) * (x3-x1);
	float d = -(a * x1 + b * y1 + c * z1);
	std::vector<float> lineCoefficents = {a, b, c, d};
	return lineCoefficents;
 }

 //plane coefficents are a, b, c, d
//point to compare the distance with the line is defined by: (x, y, z)
template<typename PointT>
float ProcessPointCloudsOwn<PointT>::Ransac::getPointDistance(float a, float b, float c, float d, float x, float y, float z)
{
	//apply the equation: d = |Ax + By + Cz + D| / sqrt(A^2 + B^2 + C^2)
	float dist = fabs(a * x + b * y + c * z + d) / sqrt(a * a + b * b + c * c);
	return dist;
}

template<typename PointT>
std::unordered_set<int> ProcessPointCloudsOwn<PointT>::Ransac::ransac(typename pcl::PointCloud<PointT>::Ptr cloud,
 int maxIterations,float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// For max iterations 
	float point1Idx, point2Idx, point3Idx, maxCountPointsOnLine = 0;
	float bestPoint1Idx = 0, bestPoint2Idx = 0, bestPoint3Idx = 0;
	for(int j = 0; j < maxIterations; ++j)
	{
		// Randomly sample subset and fit line
		point1Idx = rand() % cloud->points.size();
		point3Idx = point2Idx = point1Idx;
		while(point1Idx == point2Idx)//make sure that the two indices are not equal
			point2Idx = rand() % cloud->points.size(); 
		
		while(point3Idx == point2Idx || point3Idx == point1Idx)//make sure that the two indices are not equal
			point3Idx = rand() % cloud->points.size(); 

		PointT point1 = cloud->points[point1Idx];
		PointT point2 = cloud->points[point2Idx];
		PointT point3 = cloud->points[point3Idx];
		std::vector<float> lineCoeffs = findLineCoefficents(point1.x, point1.y, point1.z,
		 point2.x, point2.y, point2.z , point3.x, point3.y, point3.z);
		float a = lineCoeffs[0];
		float b = lineCoeffs[1];
		float c = lineCoeffs[2];
		float d = lineCoeffs[3];

		float countPointsOnLine = 0;
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for(int i = 0; i < cloud->points.size(); ++i)
		{
			PointT aPoint = cloud->points[i];
			float x = aPoint.x, y = aPoint.y, z = aPoint.z;
			float dist = getPointDistance(a, b, c, d, x, y, z);
			if(dist < distanceTol)
			{
				countPointsOnLine++;
			}
		}
		if(countPointsOnLine > maxCountPointsOnLine)
		{
			maxCountPointsOnLine = countPointsOnLine;
			bestPoint1Idx = point1Idx;
			bestPoint2Idx = point2Idx;
			bestPoint3Idx = point3Idx;
		}
	}
	
	// Return indicies of inliers from fitted line with most inliers
	PointT bestPoint1 = cloud->points[bestPoint1Idx];
	PointT bestPoint2 = cloud->points[bestPoint2Idx];
	PointT bestPoint3 = cloud->points[bestPoint3Idx];
	std::vector<float> bestLineCoeffs = findLineCoefficents(bestPoint1.x, bestPoint1.y, bestPoint1.z,
	  bestPoint2.x, bestPoint2.y, bestPoint2.z, bestPoint3.x, bestPoint3.y, bestPoint3.z);
	float bestA = bestLineCoeffs[0];
	float bestB = bestLineCoeffs[1];
	float bestC = bestLineCoeffs[2];
	float bestD = bestLineCoeffs[3];
	for(int i = 0; i < cloud->points.size(); ++i)
	{
		PointT aPoint = cloud->points[i];
		float x = aPoint.x, y = aPoint.y, z = aPoint.z;
		float dist = getPointDistance(bestA, bestB, bestC, bestD, x, y, z);
		if(dist < distanceTol)
		{
			inliersResult.insert(i);
		}
	}
	return inliersResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointCloudsOwn<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, 
    float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inliers = Ransac::ransac(cloud, maxIterations, distanceThreshold);
    if (inliers.size () == 0)
    {
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    static unsigned long long countNumTimesOfExecution = 0;
    static unsigned long long accumelativeTimeTaken = 0;
    countNumTimesOfExecution++;
    accumelativeTimeTaken += elapsedTime.count();
    std::cout << "My own Average plane segmentation took " << accumelativeTimeTaken / 1.0 * countNumTimesOfExecution << " milliseconds" << std::endl;
    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
    return std::make_pair(cloudOutliers, cloudInliers); 
}