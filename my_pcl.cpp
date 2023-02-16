#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/boundary.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
 
int main(int argc,char** argv)
{
    // create PointCloud<PointXYZ> boost share pointer and instantiation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered0 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2::Ptr cloud_filtered0_blob (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered1_blob (new pcl::PCLPointCloud2 ());

    // load pointcloud data from disk to cloud object
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/mls/DEMO/my_pcl/data/my_test2.pcd",*cloud)==-1)
    {
        PCL_ERROR("Couldn't read the PCD file\n");
        return(-1);
    }

    // Create the filtering object
    // Eliminate point in positive X
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.0,FLT_MAX);
    pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered0);

    // transform pointcloud<pointT> to pointcloud
    pcl::toPCLPointCloud2(*cloud_filtered0,*cloud_filtered0_blob);

    // Downsampling a PointCloud using a VoxelGrid filter
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud_filtered0_blob);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered1_blob);

    pcl::fromPCLPointCloud2(*cloud_filtered1_blob,*cloud_filtered1);

    std::cout << "Point amount before filtering: " << cloud->points.size() << std::endl;
    std::cout << "Point amount after filtering: " << cloud_filtered1->points.size() << std::endl;

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads(8);
    ne.setInputCloud (cloud_filtered1);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    // Compute the features
    ne.compute (*cloud_normals);

    // Compute the boundary
    pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary (new pcl::PointCloud<pcl::PointXYZ>);
	boundaries->resize(cloud_filtered1->size()); 
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_estimation;

	boundary_estimation.setInputCloud(cloud_filtered1);
	boundary_estimation.setInputNormals(cloud_normals);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_ptr(new pcl::search::KdTree<pcl::PointXYZ>); 
	boundary_estimation.setSearchMethod(kdtree_ptr);
	boundary_estimation.setKSearch(30); 
	boundary_estimation.setAngleThreshold(M_PI * 0.6);
	boundary_estimation.compute(*boundaries);

    for(int i = 0; i < cloud_filtered1->points.size(); i++) 
	{ 
		
		if(boundaries->points[i].boundary_point != 0) 
		{ 
			cloud_boundary->push_back(cloud_filtered1->points[i]); 
		} 
	} 

    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

    // add pointcloud to viewer
    viewer.addCoordinateSystem(1.0);
    viewer.setBackgroundColor(0.05,0.05,0.05);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_boundary);
    //viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud_boundary,cloud_normals);

    while(!viewer.wasStopped())
    {
        viewer.spin();
    }
 
    return(0);
}
