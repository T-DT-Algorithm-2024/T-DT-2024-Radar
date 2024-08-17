#include "dynamic_cloud.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <rclcpp/time.hpp>
namespace tdt_radar{
DynamicCloud::DynamicCloud(const rclcpp::NodeOptions& node_options):rclcpp::Node("dynamic_cloud_node",node_options),tf_buffer_(this->get_clock()),tf_listener_(tf_buffer_)
{
    RCLCPP_INFO(this->get_logger(), "Dynamic_cloud Node start");
    //从pcd读取map
    auto temp_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("config/RM2024.pcd", *temp_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file map.pcd \n");
    }
    //下采样
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(temp_cloud);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    auto result = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*result);
    map_cloud = result;
    kd_Tree.setInputCloud(map_cloud);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar", 10, std::bind(&DynamicCloud::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_dynamic", 10);
    other_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_other", 10);
    detect_pub_ = this->create_publisher<vision_interface::msg::RadarWarn>("/lidar_detect", 10);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Dynamic Cloud Launch!");
}

void DynamicCloud::GetDynamicCloud(pcl::PointCloud<pcl::PointXYZ> &input_cloud,pcl::PointCloud<pcl::PointXYZ> &output_cloud,float threshold,int thread_num){
    int K=1;
    std::vector<std::thread> threads;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clouds(thread_num);
    auto start=std::chrono::system_clock::now();
    int cloud_size=input_cloud.points.size();
    int step=cloud_size/thread_num;
    for(int i=0;i<thread_num;i++){
        threads.push_back(std::thread([i,step,cloud_size,&clouds,&input_cloud,this,threshold,K](){
            for(int j=i*step;j<(i+1)*step;j++){
                std::vector<int> pointIdxNKNSearch(K);
                std::vector<float> pointNKNSquaredDistance(K);
                if(kd_Tree.nearestKSearch(input_cloud.points[j],K,pointIdxNKNSearch,pointNKNSquaredDistance)>0){
                    if(pointNKNSquaredDistance[0]>threshold){
                        clouds[i].points.push_back(input_cloud.points[j]);
                    }
                }
            }
        }));            
    }
    for(auto &t:threads){
        t.join();
    }
    for(auto &cloud:clouds){
        output_cloud+=cloud;
    }
    auto end=std::chrono::system_clock::now();
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "kd_tree search time: %f", std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()/1000.0);
}

void TransformCloud(pcl::PointCloud<pcl::PointXYZ> &input_cloud, pcl::PointCloud<pcl::PointXYZ> &output_cloud, geometry_msgs::msg::TransformStamped transform_stamped, int thread_num) {
    std::vector<std::thread> threads;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clouds(thread_num);
    auto start = std::chrono::system_clock::now();
    int cloud_size = input_cloud.points.size();
    int step = cloud_size / thread_num;

    // 将geometry_msgs::msg::TransformStamped转换为Eigen::Affine3f
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << transform_stamped.transform.translation.x,
                               transform_stamped.transform.translation.y,
                               transform_stamped.transform.translation.z;
    Eigen::Quaternionf rotation(
        transform_stamped.transform.rotation.w,
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z);
    transform.rotate(rotation);

    for (int i = 0; i < thread_num; i++) {
        threads.push_back(std::thread([i, step, cloud_size, &clouds, &input_cloud, transform]() {
            for (int j = i * step; j < (i + 1) * step && j < cloud_size; j++) {
                pcl::PointXYZ point = input_cloud.points[j];
                Eigen::Vector3f point_vec(point.x, point.y, point.z);
                Eigen::Vector3f point_out = transform * point_vec;

                pcl::PointXYZ transformed_point;
                transformed_point.x = point_out.x();
                transformed_point.y = point_out.y();
                transformed_point.z = point_out.z();

                clouds[i].points.push_back(transformed_point);
            }
        }));
    }

    for (auto &t : threads) {
        t.join();
    }
    for (auto &cloud : clouds) {
        output_cloud += cloud;
    }
    auto end = std::chrono::system_clock::now();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "transform time: %f", std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0);
}

void DynamicCloud::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{    
    vision_interface::msg::RadarWarn lidar_detect;
    auto dart_cloud_filter = [](pcl::PointXYZ &point) {
        return (point.x > 28 - 0.5889 - 0.1885 && point.x < 28 - 0.5889) &&
               (point.y > 3.925 && point.y < 4.525) &&
               (point.z > 2.4722 - 0.859 +0.1&& point.z < 2.4722);
    };//飞镖检测
    auto fly_safe_filter = [](pcl::PointXYZ &point) {
        return (point.x > 28-2.775 && point.x < 27.5) &&
                (point.y > 0.2 && point.y < 2.2) &&
                (point.z > 1.7 && point.z < 3);
    };//飞机起飞

    auto fly_warn_filter = [](pcl::PointXYZ &point) {
        return (point.x > 19.83 && point.x < 28-2.7) &&
               (point.y > 0.2 && point.y < 1.356 + 2.4 + 0.8) &&
               (point.z > 1.7 && point.z < 3);
    };//飞机飞到半场

    auto fly_alarm_filter = [](pcl::PointXYZ &point) {
        return (point.x > 13 && point.x < 20.5) &&
               (point.y > 0.2 && point.y < 1.356 + 2.4 + 0.8) &&
               (point.z > 1.7 && point.z < 3);
    };//飞机飞到中场

    auto engine_filter = [](pcl::PointXYZ &point) {
        return (point.x > 28-2.0234 && point.x < 28-1.0234) &&
               (point.y > 10.955+0.1 && point.y < 10.955 + 1.6 - 0.1) &&
               (point.z > 0.4 && point.z < 1.5);
    };//兑换站

    auto little_engine_filter = [](pcl::PointXYZ &point) {
        //小资源岛方程：
        // y=tan55°*x - 20.2563
        // y=tan55°*x -18.0736
        // y=-1/tan55°*x -5.9628
        // y=-1/tan55°*x -7.4988
        double xminusy = point.y - point.x*tan(55.0/180.0*M_PI);
        double xplusy = point.y + point.x / tan(55.0/180.0*M_PI);    
        return (xminusy < -21.9555 && xminusy > -23.3419) &&
               (xplusy > 16.7456 && xplusy < 18.1448)&&(point.z>0&&point.z<1.2);
               //构建方程之后微调一下
};

    auto receive_cloud = pcl::PointCloud<pcl::PointXYZ>();
    pcl::fromROSMsg(*msg, receive_cloud);

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    geometry_msgs::msg::TransformStamped transform_stamped;
    // 获取目标坐标系的变换
    auto ta=std::chrono::steady_clock::now();
    try{
    transform_stamped = tf_buffer_.lookupTransform("rm_frame", msg->header.frame_id, tf2::TimePointZero);}
    catch (tf2::TransformException &ex){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Transform error: %s", ex.what());
        return;
    }
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    auto transform = Eigen::Affine3f::Identity();
    transform.translation() << transform_stamped.transform.translation.x,
                               transform_stamped.transform.translation.y,
                               transform_stamped.transform.translation.z;
    Eigen::Quaternionf rotation(
        transform_stamped.transform.rotation.w,
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z);
    transform.rotate(rotation);
    pcl::transformPointCloud(receive_cloud, transformed_cloud, transform);

    //只保留x在3-28，y在0-15，z在0-2.4的点
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
    pcl::PointCloud<pcl::PointXYZ> other_filtered_cloud;
    for (size_t i = 0; i < transformed_cloud.size(); i++)
    {
        auto &point = transformed_cloud.points[i];
        if (point.x < 3 || point.x > 28 || point.y < 0 || point.y > 15 || point.z < 0 || point.z > 1.4 ||
            //或者y(0,5),x(25,28)不要
            (point.y > 0 && point.y < 5 && point.x > 25) ||
            //或者y(11,12),x(23,24)不要
            // (point.y > 11 && point.y < 12 && point.x > 23 && point.x < 24) 
            //画四个直线切割大资源岛
            ((21.5-0.9/sqrt(2))<(point.x + point.y) &&(point.x + point.y) <(21.5+0.9/sqrt(2))&&
            (-6.5-2.9/sqrt(2))<(point.y-point.x)&&(point.y-point.x)<(-6.5+2.9/sqrt(2)))||
            ((12<point.y&&point.y<13.5)&&(17<point.x&&point.x<18))||
            ((11<point.y&&point.y<12.25)&&(23<point.x&&point.x<24.1)&&(point.z<0.535))||
            (point.x>28-2.0234&&point.x<28-1.0234)&&(point.y > 10.955+0.1 && point.y < 10.955 + 1.6 - 0.1)&&(point.z>0.4&&point.z<1.5)||
            little_engine_filter(point)
            ///TODO: 此处代码混乱，需要重构，全部替换成Lambda表达式的过滤器形式
        )
        {
            // 如果在飞镖识别范围内：x(28-0.5889-0.1885,28-0.5889) y(3.925,4.525),z(2.7422-0.859,2.7422)
            // 如果在飞机识别范围内：x(14,28-3.024) y(0,1.356+2.4) z(1.7,2.5)
            // 如果在兑换站范围内   x(28-2.0234,28-1.0234) y(10.955,10.955+1.6) z(0.4,1.5)
            if((point.x>28-0.5889-0.1885&&point.x<28-0.5889)&&(point.y>3.925&&point.y<4.525)&&(point.z>2.4722-0.859+0.1&&point.z<2.4722)||
            (point.x>13&&point.x<27.5)&&(point.y>0.2&&point.y<1.356+2.4+0.8)&&(point.z>1.7&&point.z<3)||
            (point.x>28-2.0234&&point.x<28-1.0234)&&(point.y > 10.955+0.1 && point.y < 10.955 + 1.6 - 0.1)&&(point.z>0.4&&point.z<1.5)||
            little_engine_filter(point)
            ){
                other_filtered_cloud.push_back(point);
                }
            continue;
        }
        filtered_cloud.push_back(point);
    }
    // std::cout << "filter time: " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-ta).count()/1000.0 << std::endl;
    pcl::PointCloud<pcl::PointXYZ> dynamic_pointcloud;
    GetDynamicCloud(filtered_cloud,dynamic_pointcloud,0.1,12);
    ///TODO: 点云积分的代码比较重复，需要重构成一个class，维护那些积分的点云

    if(accumulate_count<accumulate_time){
        accumulated_clouds_.push_back(dynamic_pointcloud.makeShared());
        other_accumulated_clouds_.push_back(other_filtered_cloud.makeShared());
        accumulate_count++;
    }else{
        accumulated_clouds_.erase(accumulated_clouds_.begin());
        accumulated_clouds_.push_back(dynamic_pointcloud.makeShared());
        other_accumulated_clouds_.erase(other_accumulated_clouds_.begin());
        other_accumulated_clouds_.push_back(other_filtered_cloud.makeShared());
    }
    pcl::PointCloud<pcl::PointXYZ> accumulated_cloud;
    for(auto it = accumulated_clouds_.begin(); it != accumulated_clouds_.end(); ++it)
    {
        accumulated_cloud += **it;
    }
    pcl::PointCloud<pcl::PointXYZ> other_accumulated_cloud;
    for(auto it = other_accumulated_clouds_.begin(); it != other_accumulated_clouds_.end(); ++it)
    {
        other_accumulated_cloud += **it;
    }//因为other的点少，就先累积再处理
    ta = std::chrono::steady_clock::now();
    sensor_msgs::msg::PointCloud2 output;
    accumulated_cloud.header.frame_id = "rm_frame";
    other_accumulated_cloud.header.frame_id = "rm_frame";
    pcl::toROSMsg(accumulated_cloud, output);
    output.header.frame_id = "rm_frame";
    output.header.stamp = msg->header.stamp;
    pub_->publish(output);

    pcl::toROSMsg(other_accumulated_cloud, output);
    other_pub_->publish(output);
    // 筛出飞镖点云
    pcl::PointCloud<pcl::PointXYZ> dart_cloud;

    for (size_t i = 0; i < other_accumulated_cloud.size(); i++)
    {
        auto &point = other_accumulated_cloud.points[i];
        if (dart_cloud_filter(point))
        {
            dart_cloud.push_back(point);
        }
    }

    if(dart_cloud.size()>5){
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Find dart cloud!");
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Dart cloud size: %d", dart_cloud.size());
        lidar_detect.dart_state = 1;
    }

    pcl::PointCloud<pcl::PointXYZ> fly_safe_cloud;
    pcl::PointCloud<pcl::PointXYZ> fly_warn_cloud;
    pcl::PointCloud<pcl::PointXYZ> fly_alarm_cloud;

    for (size_t i = 0; i < other_accumulated_cloud.size(); i++)
    {
        auto &point = other_accumulated_cloud.points[i];
        if (fly_safe_filter(point))
        {
            fly_safe_cloud.push_back(point);
        }
        if (fly_warn_filter(point))
        {
            fly_warn_cloud.push_back(point);
        }
        if (fly_alarm_filter(point))
        {
            fly_alarm_cloud.push_back(point);
        }
    }
    if(fly_safe_cloud.size()>40){
        lidar_detect.fly_state = 1;
    }

    if(fly_warn_cloud.size()>40){
        lidar_detect.fly_state = 2;
    }

    if(fly_alarm_cloud.size()>40){
        lidar_detect.fly_state = 3;
    }
    
    switch (lidar_detect.fly_state)
    {
    case 0:
        break;
    case 1:
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Safe fly object detected!");
        break;
    case 2:
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Warn fly object detected!");
        break;
    case 3:
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Alarm fly object detected!");
        break;
    default:
        break;
    }

    pcl::PointCloud<pcl::PointXYZ> engine_cloud;
    for (size_t i = 0; i < other_accumulated_cloud.size(); i++)
    {
        auto &point = other_accumulated_cloud.points[i];
        if (engine_filter(point))
        {
            engine_cloud.push_back(point);
        }
    }
    if(engine_cloud.size()>5){
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Find engine cloud!");
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Engine cloud size: %d", engine_cloud.size());
        lidar_detect.engine_state = 2;
    }
    
    pcl::PointCloud<pcl::PointXYZ> little_engine_cloud;
    for (size_t i = 0; i < other_accumulated_cloud.size(); i++)
    {
        auto &point = other_accumulated_cloud.points[i];
        if (little_engine_filter(point))
        {
            little_engine_cloud.push_back(point);
        }
    }
    if(little_engine_cloud.size()>5){
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Find little engine cloud!");
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Little engine cloud size: %d", little_engine_cloud.size());
        lidar_detect.engine_state = 1;
    }
    
    detect_pub_->publish(lidar_detect);
    // std::cout << "publish time: " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-ta).count()/1000.0 << std::endl;
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "cloud size: %d", accumulated_cloud.points.size());
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Dynamic callback time: %f", std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()/1000.0);
}
}//namespace tdt_radar

RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::DynamicCloud)
