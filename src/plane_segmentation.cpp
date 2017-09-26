#include <iostream>

#include <ros/ros.h>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/features/normal_3d.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>

#include <sensor_msgs/Image.h>

class ObjectSegmentation {
    public:
        ObjectSegmentation() {}
        ~ObjectSegmentation() {}

        void init() {
            nh_ = ros::NodeHandle("~");
            int queueSize = 5;

            pc_sub_ = nh_.subscribe ("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 
                queueSize, &ObjectSegmentation::pointCloudCallBack, this);

            filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/victor_filtered_pointcloud", 1);
            plane_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/victor_plane", 1);
            obj_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/victor_objects", 1);
            cylinder_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/victor_cylinder", 1);
            regions_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/victor_regions", 1);
        }
 
        void pointCloudCallBack(const sensor_msgs::PointCloud2ConstPtr cloud) {
            sensor_msgs::PointCloud2 output_cloud;

            // Filter cloud with low pass filter and voxel
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_blob (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*cloud, *cloud_blob);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            filterCloud(cloud_blob, cloud_filtered);
            
            pcl::toROSMsg(*cloud_filtered, output_cloud);
            filtered_pub_.publish(output_cloud);

            // Segment plane and object from filtered cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>), cloud_objects (new pcl::PointCloud<pcl::PointXYZ>);
            segmentPlane(cloud_filtered, cloud_plane, cloud_objects);
            
            pcl::toROSMsg(*cloud_plane, output_cloud);
            plane_pub_.publish(output_cloud);

            pcl::toROSMsg(*cloud_objects, output_cloud);
            obj_pub_.publish(output_cloud);

            /*
            pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_coloured_regions (new pcl::PointCloud<pcl::PointXYZRGB>);
            regionGrowing(cloud_objects, cloud_coloured_regions);

            pcl::toROSMsg(*cloud_coloured_regions, output_cloud);

            output_cloud.header.frame_id = cloud->header.frame_id;

            regions_pub_.publish(output_cloud); 
            */

            /*
            // Find objects from object cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_coloured_regions (new pcl::PointCloud<pcl::PointXYZRGB>);
            findCylinder(cloud_objects, cloud_cylinder, cloud_coloured_regions);

            pcl::toROSMsg(*cloud_cylinder, output_cloud);
            cylinder_pub_.publish(output_cloud);

            pcl::toROSMsg(*cloud_coloured_regions, output_cloud);
            regions_pub_.publish(output_cloud);
            */
        }   

        void filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_blob, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel) {
            // http://docs.pointclouds.org/trunk/classpcl_1_1_pass_through.html#a4c852c876b5ed3d9235d5b4e59353883

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_pass_through (new pcl::PointCloud<pcl::PointXYZ>);

            // Filter based on z values

            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud (cloud_blob);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (0, 1.5);
            //pass.setFilterLimitsNegative (true);
            pass.filter (*cloud_pass_through); 
 
            // Create the filtering object

            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud (cloud_pass_through);
            sor.setMeanK (50);
            sor.setStddevMulThresh (1.0);
            sor.filter (*cloud_filtered);

            // Downsample to voxels

            pcl::VoxelGrid<pcl::PointXYZ> vox;
            vox.setInputCloud (cloud_filtered);
            vox.setLeafSize (0.01f, 0.01f, 0.01f);
            vox.filter (*cloud_voxel);

            // Maybe use supervoxels
            // http://pointclouds.org/documentation/tutorials/supervoxel_clustering.php#supervoxel-clustering 1.8
            // http://docs.pointclouds.org/1.7.1/classpcl_1_1_supervoxel_clustering.html 1.7
      }

        void segmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_objects) {
            // https://gitlab.iri.upc.edu/perception/tos_supervoxels/blob/master/trunk/src/tos_supervoxels.cpp 
            // detectObjectsOnTable

            // http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
            
            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            
            // Optional
            seg.setOptimizeCoefficients (true);
            
            // Mandatory
            seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setMaxIterations (1000);
            seg.setDistanceThreshold (0.01);

            // Victor: Maybe set this to SACMODEL_NORMAL_PLANE

            // Victor
            // https://answers.ros.org/question/61811/pcl-sacsegmentation-setaxis-and-setmodeltype-has-no-effect-in-output/
            // Filter only horizontal

            // y axis
            // THIS DOESNT SEEM TO WORK
            seg.setAxis(Eigen::Vector3f(0.0, 1.0, 0.0));
            seg.setEpsAngle(30.0f * (M_PI/180.0f));

            // Create the filtering object
            pcl::ExtractIndices<pcl::PointXYZ> extract;

            int i = 0, nr_points = (int) cloud_filtered->points.size ();
            // While 30% of the original cloud is still there

            //while (cloud_filtered->points.size () > 0.3 * nr_points)
            {
                // Segment the largest planar component from the remaining cloud
                seg.setInputCloud (cloud_filtered);
                seg.segment (*inliers, *coefficients);
                if (inliers->indices.size () == 0)
                {
                    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
                    //break;
                }

                // Extract the inliers
                extract.setInputCloud (cloud_filtered);
                extract.setIndices (inliers);
                extract.setNegative (false);
                extract.filter (*cloud_p);

                // Create the filtering object
                extract.setNegative (true);
                extract.filter (*cloud_f);
                cloud_filtered.swap (cloud_f);
                i++;
            }

            *cloud_plane = *cloud_p;
            *cloud_objects = *cloud_filtered;
        }


        void regionGrowing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_objects, 
                pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_coloured_regions) {
            
            // Create the segmentation object
            
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
            pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
            pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);

            normal_estimator.setSearchMethod (tree);
            normal_estimator.setInputCloud (cloud_objects);
            normal_estimator.setKSearch (50);
            normal_estimator.compute (*normals);

            // Region growing 
            // http://pointclouds.org/documentation/tutorials/region_growing_segmentation.php#region-growing-segmentation

            pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
            reg.setMinClusterSize (50);
            reg.setMaxClusterSize (1000000);

            reg.setSearchMethod (tree);
            reg.setNumberOfNeighbours (30);
            reg.setInputCloud (cloud_objects);
            //reg.setIndices (indices);
            reg.setInputNormals (normals);

            reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
            reg.setCurvatureThreshold (1.0);

            std::vector <pcl::PointIndices> clusters;
            reg.extract (clusters);

            *cloud_coloured_regions = *reg.getColoredCloud ();

            std::cout << "Coloured: " << (*cloud_coloured_regions).size() << "\n";

            /*
            #include <pcl/visualization/cloud_viewer.h>

            pcl::visualization::CloudViewer viewer ("Cluster viewer");
            viewer.showCloud(colored_cloud);
            while (!viewer.wasStopped ())
            {
            }
            */
        }

        void findCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_objects, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder,
                pcl::PointCloud <pcl::PointXYZRGB>::Ptr &cloud_coloured_regions) {
            // https://gitlab.iri.upc.edu/perception/tos_supervoxels/blob/master/trunk/src/tos_supervoxels.cpp 
            // detectObjectsOnTable

            // http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
            pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices ());
            
            // Create the segmentation object
            
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
            pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
            pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);

            normal_estimator.setSearchMethod (tree);
            normal_estimator.setInputCloud (cloud_objects);
            normal_estimator.setKSearch (50);
            normal_estimator.compute (*normals);

            pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg; 

            // Optional
            seg.setOptimizeCoefficients (true);
            
            // Mandatory
            seg.setModelType (pcl::SACMODEL_CYLINDER);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setNormalDistanceWeight (0.1);
            seg.setMaxIterations (1000);
            seg.setDistanceThreshold (0.05);

            // Radius: TODO: set to radius of bbox
            seg.setRadiusLimits (0.2, 0.5);

            seg.setInputCloud (cloud_objects);
            seg.setInputNormals (normals);
            seg.segment (*inliers_cylinder, *coefficients);

            // Maybe only look for standing cylinders
            seg.setAxis(Eigen::Vector3f(0.0, 1.0, 0.0));
            seg.setEpsAngle(30.0f * (M_PI/180.0f));

            // Create the filtering object
            pcl::ExtractIndices<pcl::PointXYZ> extract;

            extract.setInputCloud (cloud_objects);
            extract.setIndices (inliers_cylinder);
            extract.setNegative (false);
            extract.filter (*cloud_cylinder);

            // Region growing 
            // http://pointclouds.org/documentation/tutorials/region_growing_segmentation.php#region-growing-segmentation

            /*
            pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
            reg.setMinClusterSize (50);
            reg.setMaxClusterSize (1000000);

            reg.setSearchMethod (tree);
            reg.setNumberOfNeighbours (30);
            reg.setInputCloud (cloud_objects);
            //reg.setIndices (indices);
            reg.setInputNormals (normals);

            reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
            reg.setCurvatureThreshold (1.0);

            std::vector <pcl::PointIndices> clusters;
            reg.extract (clusters);

            cloud_coloured_regions = reg.getColoredCloud ();

            std::cout << "Coloured: " << (*cloud_coloured_regions).size() << "\n";
            */

            /*
            #include <pcl/visualization/cloud_viewer.h>

            pcl::visualization::CloudViewer viewer ("Cluster viewer");
            viewer.showCloud(colored_cloud);
            while (!viewer.wasStopped ())
            {
            }
            */
        }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber pc_sub_;

        ros::Publisher filtered_pub_;
        ros::Publisher plane_pub_;
        ros::Publisher obj_pub_;
        ros::Publisher cylinder_pub_;
        ros::Publisher regions_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_segmentation");
    ROS_INFO("Node started");

    ObjectSegmentation os = ObjectSegmentation();
    os.init(); 

    ros::spin();
    ROS_INFO("Shutting down");

    ros::shutdown();
    return 0;
}