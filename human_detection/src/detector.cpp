#include <human_detection/detector.hpp>


Detector::Detector(ros::NodeHandle& nh){
    std::cout << "detector constructor called" << std::endl;
    node_handle = nh;
    // call callback function in constructor
    sub_pointCloud = nh.subscribe("/point_cloud", 1000, &Detector::detect_callback, this);
}

void Detector::detect_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){

    std::cout << "Point cloud before filtering has: " << cloud->size () << " data points." << std::endl;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    std::cout << "Point cloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl;

    // 
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);
    int nr_points = (int) cloud_filtered->size ();
    /*
    while (cloud_filtered->size () > 0.3 * nr_points){
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0){
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
 
        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }
    */
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);
 
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);
    std::cout << "there are " << cluster_indices.size() << " clusters" << std::endl;
    /*
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : it->indices){
            cloud_cluster->push_back ((*cloud_filtered)[idx]); //*
        }
 
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
 
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        j++;
    }
    */
}