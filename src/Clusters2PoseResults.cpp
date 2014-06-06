#include <ecto/ecto.hpp>
#include <object_recognition_core/common/pose_result.h>

#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using ecto::tendrils;
using ecto::spore;

using object_recognition_core::common::PoseResult;

struct Clusters2PoseResults {
private:
  spore< std::vector< std::vector< std::vector<cv::Vec3f> > > > clusters_;
  spore< sensor_msgs::ImageConstPtr > image_message_;
  spore< std::vector< PoseResult > > pose_results_;

public:
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs){
    inputs.declare(&Clusters2PoseResults::clusters_,  "clusters3d", "The object clusters").required(true);
    inputs.declare(&Clusters2PoseResults::image_message_,  "image_message", "The input image to extract the header");

    outputs.declare(&Clusters2PoseResults::pose_results_, "pose_results", "The resulting 'objects'");
  }

  void Cluster2PoseResult( const std::vector<cv::Vec3f> cluster, PoseResult& result ){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for( const cv::Vec3f &point : cluster )
      cloud->push_back( pcl::PointXYZ(point[0], point[1], point[2]) );

    sensor_msgs::PointCloud2Ptr cloudmsg(new sensor_msgs::PointCloud2() );
    pcl::toROSMsg(*cloud, *cloudmsg);

    if( *image_message_ ){
      cloudmsg->header= (*image_message_)->header;
    }

    std::vector<sensor_msgs::PointCloud2Ptr> cloudmsgs(1);
    cloudmsgs[0]= cloudmsg;
    result.set_clouds(cloudmsgs);
  }

  int process(const tendrils& input, const tendrils& output){
    pose_results_->clear();
    size_t clusters_n(0);

    for( const std::vector< std::vector<cv::Vec3f> > &table : *clusters_ ){
      clusters_n+= table.size();
      pose_results_->reserve(clusters_n);

      for( const std::vector<cv::Vec3f> &cluster : table ){
        PoseResult result;
        Cluster2PoseResult(cluster, result);
        pose_results_->push_back(result);
      }
    }

    return ecto::OK;
  };
};

ECTO_CELL(raw_clusters, Clusters2PoseResults, "Clusters2PoseResults",
          "Given clusters on a table, generate PoseResult messages without any recognition")
