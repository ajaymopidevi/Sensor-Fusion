// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr filteredCloud (new pcl::PointCloud<PointT>);
    
    //Downsample using a leaf size filterRes(m)
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*filteredCloud);
    
    //Limit the pcd with in the box
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filteredCloud);
    region.filter(*cloudRegion);
    

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for(int point: indices)
        inliers->indices.push_back(point);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	while(maxIterations--)
	{
		std::unordered_set<int>inliers;

		while (inliers.size() <3)
		{
			inliers.insert(rand()%(cloud->points.size()));
		}

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;

		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;

        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        float A = ((y2 - y1)* (z3-z1)) - ((z2 - z1)*(y3-y1));
        float B = ((z2 - z1)* (x3-x1)) - ((x2 - x1)*(z3-z1));
        float C = ((x2 -x1)* (y3-y1)) - ((y2 -y1)*(x3 -x1));
        float D = -((A*x1) + (B*y1) + (C*z1));



		for(int index=0; index < cloud->points.size(); index++)
		{
			if (inliers.count(index) > 0)
			{
				continue;

			}

			PointT point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
            float z4 = point.z;

			float d = fabs(A*x4 + B*y4 +C*z4 + D)/ sqrt(A*A + B*B + C*C);

			if(d<distanceTol)
			{
				inliers.insert(index);
			}
		}

		if (inliers.size()> inliersResult.size())
		{
			inliersResult = inliers;
		}

	}

    typename pcl::PointCloud<PointT>::Ptr  cloudInliers {new pcl::PointCloud<PointT>()};
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers {new pcl::PointCloud<PointT>()};

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);

    return segResult;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr obsCloud {new pcl::PointCloud<PointT> ()};
    typename pcl::PointCloud<PointT>::Ptr planeCloud {new pcl::PointCloud<PointT> ()};

    for(int index:inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);

    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obsCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obsCloud, planeCloud);

    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};

    //Segmentation Using pcl RANSAC
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if(inliers->indices.size()==0)
    {
        std::cout<<"Could not estimate a planar model forthe given dataset."<<std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout<<"Inliers: "<<segResult.second->points.size()<<" Outliers: "<<segResult.first->points.size()<<std::endl;
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneRansac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = RansacPlane(cloud, maxIterations, distanceThreshold);
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout<<"Inliers: "<<segResult.second->points.size()<<" Outliers: "<<segResult.first->points.size()<<std::endl;
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int i, typename pcl::PointCloud<PointT>::Ptr cloud, typename pcl::PointCloud<PointT>::Ptr &cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
    if(processed[i]){
        return;
    }
    processed[i] = true;
	cluster->points.push_back(cloud->points[i]);
    std::vector<float> point;
    point.push_back(cloud->points[i].x);
    point.push_back(cloud->points[i].y);
    point.push_back(cloud->points[i].z);
    
	std::vector<int> nearby = tree->search(point,distanceTol);
	
	for(int id: nearby)
	{
		if(!processed[id])
		{
			
			clusterHelper(id, cloud, cluster, processed, tree, distanceTol);
		}
	}
	
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>ProcessPointClouds<PointT>:: euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize)
{

	
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<std::pair<std::vector<float>, int>> points;
    
    KdTree* tree = new KdTree;
  
    for (int i=0; i<cloud->points.size(); i++) 
    {
        std::vector<float> point;
        point.push_back(cloud->points[i].x);
        point.push_back(cloud->points[i].y);
        point.push_back(cloud->points[i].z);
        points.push_back(std::pair<std::vector<float>,int>(point,i));
        
    }
    //build a balanced KD-Tree
    //balanced KD-Tree reduces the effective time for each search
    tree->buildTree(points.begin(), points.end());
    points.clear();

    auto startTime = std::chrono::steady_clock::now();


	std::vector<bool> processed(cloud->points.size(), false);
	int i=0;
	while(i < cloud->points.size())
	{
		if(processed[i])
		{
			i++;
			continue;
		}
        typename pcl::PointCloud<PointT>::Ptr cluster {new pcl::PointCloud<PointT>};
		clusterHelper(i, cloud, cluster, processed, tree, distanceTol);

        if(cluster->points.size()>minSize && cluster->points.size()<maxSize)
        {
		    clusters.push_back(cluster);
        }
		i++;
        
	}
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

	return clusters;

}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    //Using PCL standard libraries
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(pcl::PointIndices getIndices:clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        for(int index:getIndices.indices)
        {
            cloudCluster->points.push_back(cloud->points[index]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);


    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}