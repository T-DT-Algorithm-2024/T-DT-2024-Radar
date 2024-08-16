#include "resolve.h"

#define TDT_INFO(msg) std::cout << msg << std::endl

namespace tdt_radar {

Resolve::Resolve(const rclcpp::NodeOptions& node_options)
    : Node("radar_resolve_node", node_options) {
      parser_ = new parser();
      minimap=cv::imread("configa/RM2024.png");
  point_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
      "camera_point2D", rclcpp::SensorDataQoS(),
      std::bind(&Resolve::callback, this, std::placeholders::_1));

  pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "camera_point3D", rclcpp::SensorDataQoS());
      match_info_sub = this->create_subscription<vision_interface::msg::MatchInfo>(
      "match_info", rclcpp::SensorDataQoS(),
      std::bind(&Resolve::MatchInfoCallback, this, std::placeholders::_1));
  
  detect_sub = this->create_subscription<vision_interface::msg::DetectResult>(
      "detect_result", rclcpp::SensorDataQoS(),
      std::bind(&Resolve::DetectCallback, this, std::placeholders::_1));
      pub_radar=this->create_publisher<vision_interface::msg::DetectResult>("resolve_result",rclcpp::SensorDataQoS());
    TDT_INFO("Load radar resolve node success!");
}

void Resolve::MatchInfoCallback(const vision_interface::msg::MatchInfo::SharedPtr msg) {
  for(int i=0;i<6;i++){
    markers[i]=msg->marks[i];
  }
  for(int i=0;i<16;i++){
    robot_hp[i]=msg->robot_hp[i];
  }
  match_time=msg->match_time;
}

void Resolve::callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
  cv::Point2f point;
  point.x = msg->x;
  point.y = msg->y;
  auto center_point=parser_->parse(point);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ send_point;
  send_point.x = center_point.x;
  send_point.y = 15-center_point.y;
  std::cout<<center_point<<std::endl;
  send_point.z = 1;
  cloud->points.push_back(send_point);
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  output.header.frame_id = "rm_frame";
  pub->publish(output);
  }

void Resolve::DetectCallback(const vision_interface::msg::DetectResult::SharedPtr msg) {
  cv::Mat Map_clone=minimap.clone();
  std::vector<map_car> cars;
  //这个节点的功能是
  //1.解算
  //2.可视化
  vision_interface::msg::DetectResult send_data;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  for(int i=0;i<6;i++){
    cv::Point2f blue_point;
    blue_point.x = msg->blue_x[i];
    blue_point.y = msg->blue_y[i];
    if(blue_point.x*blue_point.y){
      auto center_point=parser_->parse(blue_point);
      center_point.y=15+center_point.y;
      // center_point.x=center_point.x+=6;
      // center_point.y=center_point.y-=4;
      send_data.blue_x[i]=center_point.x;
      send_data.blue_y[i]=center_point.y;
      pcl::PointXYZRGBA send_point;
      send_point.x = center_point.x;
      send_point.y = center_point.y;
      send_point.z = 1;
      send_point.r = 0;
      send_point.g = 0;
      send_point.b = 255;
      send_point.a = 255;
      cloud->points.push_back(send_point);
      cv::circle(Map_clone,cv::Point((Map_clone.cols*center_point.x)/28,Map_clone.rows*(15-center_point.y)/15),20,cv::Scalar(200,0,0),-1);
      cv::putText(Map_clone,std::to_string(i+1),cv::Point((Map_clone.cols*center_point.x)/28-10,Map_clone.rows*(15-center_point.y)/15+10),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,255),2);
    }
    cv::Point2f red_point;
    red_point.x = msg->red_x[i];
    red_point.y = msg->red_y[i];
    if(red_point.x*red_point.y){
      auto center_point=parser_->parse(red_point);
      center_point.y=15+center_point.y;
      // center_point.x=center_point.x+=6;
      // center_point.y=center_point.y-=4;
      send_data.red_x[i]=center_point.x;
      send_data.red_y[i]=center_point.y;
      pcl::PointXYZRGBA send_point;
      send_point.x = center_point.x;
      send_point.y = center_point.y;
      send_point.z = 1;
      send_point.r = 255;
      send_point.g = 0;
      send_point.b = 0;
      send_point.a = 255;
      cloud->points.push_back(send_point);
      cv::circle(Map_clone,cv::Point((Map_clone.cols*center_point.x)/28,Map_clone.rows*(15-center_point.y)/15),20,cv::Scalar(0,0,200),-1);
      cv::putText(Map_clone,std::to_string(i+1),cv::Point((Map_clone.cols*center_point.x)/28-10,Map_clone.rows*(15-center_point.y)/15+10),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,255),2);
    }
  }
  // std::vector<cv::Point2f> red_car;
  // red_car.resize(6);

  // std::vector<cv::Point2f> blue_car;
  // blue_car.resize(6);
  // for(int i=0;i<6;i++){
  //   if(msg->blue_x[i]*msg->blue_y[i]){
  //     cv::Point2f point;
  //     point.x = msg->blue_x[i];
  //     point.y = msg->blue_y[i];
  //     auto center_point=parser_->parse(point);
  //     center_point.y=15+center_point.y;
  //     // center_point.x=center_point.x+=6;
  //     // center_point.y=center_point.y-=4;
  //     blue_car[i]=center_point;
  //     pcl::PointXYZRGBA send_point;
  //     send_point.x = center_point.x;
  //     send_point.y = center_point.y;
  //     send_point.z = 1;
  //     send_point.r = 0;
  //     send_point.g = 0;
  //     send_point.b = 255;
  //     send_point.a = 255;
  //     cloud->points.push_back(send_point);
  //     cv::putText(Map_clone,std::to_string(i+1),cv::Point((Map_clone.cols*center_point.x)/28,Map_clone.rows*(15-center_point.y)/15),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,128,0),2);

  //   }
  //   if(msg->red_x[i]*msg->red_y[i]){
  //     cv::Point2f point;
  //     point.x = msg->red_x[i];
  //     point.y = msg->red_y[i];
  //     auto center_point=parser_->parse(point);
  //     center_point.y=15+center_point.y;
  //     // center_point.x=center_point.x+=6;
  //     // center_point.y=center_point.y-=4;
  //     red_car[i]=center_point;
  //     pcl::PointXYZRGBA send_point;
  //     send_point.x = center_point.x;
  //     send_point.y = center_point.y;
  //     send_point.z = 1;
  //     send_point.r = 255;
  //     send_point.g = 0;
  //     send_point.b = 0;
  //     send_point.a = 255;
  //     cloud->points.push_back(send_point);
  //     cv::putText(Map_clone,std::to_string(i+1),cv::Point((Map_clone.cols*center_point.x)/28,Map_clone.rows*(15-center_point.y)/15),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,128,255),2);
  //   }
  // }
  // //获取当前时间用来确定文件名
  // time_t now = time(0);
  // tm *ltm = localtime(&now);
  // std::string filename=std::to_string(ltm->tm_year+1900)+"_"+std::to_string(ltm->tm_mon+1)+"_"+std::to_string(ltm->tm_mday)+"_"+std::to_string(ltm->tm_hour)+"_"+std::to_string(ltm->tm_min)+"_"+std::to_string(ltm->tm_sec);
  // //保存到时间.txt
  // std::ofstream out("save_config/"+filename+".txt");
  // out<<match_time<<std::endl;
  // out<<EnemyColor<<std::endl;
  // for(int i=0;i<6;i++){
  //   out<<blue_car[i].x<<" "<<blue_car[i].y<<std::endl;
  // }
  // for(int i=0;i<6;i++){
  //   out<<red_car[i].x<<" "<<red_car[i].y<<std::endl;
  // }
  // for(int i=0;i<16;i++){
  //   out<<(uint16_t)robot_hp[i]<<" ";
  // }
  // out.close();

  // if(EnemyColor==0){//Enemy blue
  //     for(int i=0;i<6;i++){
  //       send_data.radar_enemy_x[i]=blue_car[i].x+0.5;
  //       send_data.radar_enemy_y[i]=blue_car[i].y;
  //       if(blue_car[i].x*blue_car[i].y){
  //         map_car temp;
  //         temp.x=blue_car[i].x;
  //         temp.y=blue_car[i].y;
  //         temp.id=i+1;
  //         if(temp.id==6)temp.id+=1;
  //         temp.id+=100;
  //         // std::cout<<"send id"<<temp.id<<std::endl;
  //         cars.push_back(temp);
  //       }
  //     }
  // }
  // if(EnemyColor==2){
  //     for(int i=0;i<6;i++){
  //       send_data.radar_enemy_x[i]=red_car[i].x;
  //       send_data.radar_enemy_y[i]=red_car[i].y;
  //       if(red_car[i].x*red_car[i].y){
  //         map_car temp;
  //         temp.x=red_car[i].x;
  //         temp.y=red_car[i].y;
  //         temp.id=i+1;
  //         if(temp.id==6)temp.id+=1;
  //         cars.push_back(temp);
  //       }
  //     }
  // }
  
  /*
  for(int i=0;i<6;i++){
    if(EnemyColor==0&&msg->blue_x[i]*msg->blue_y[i]){
      cv::Point2f point;
      point.x = msg->blue_x[i];
      point.y = msg->blue_y[i];
      auto center_point=parser_->parse(point);
      center_point.y=15+center_point.y;

      center_point.x=center_point.x+=6;
      center_point.y=center_point.y-=4;
      cv::putText(Map_clone,std::to_string(i+1),cv::Point((Map_clone.cols*center_point.x)/28,Map_clone.rows*(15-center_point.y)/15),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,0),2);

      send_data.radar_enemy_x[i]=center_point.x;
      send_data.radar_enemy_y[i]=center_point.y;
      pcl::PointXYZRGBA send_point;
      send_point.x = center_point.x;
      send_point.y = center_point.y;
      send_point.z = 1;
      send_point.r = 0;
      send_point.g = 0;
      send_point.b = 255;
      send_point.a = 255;
      cloud->points.push_back(send_point);
      if(i!=5&&msg->red_x[i]*msg->red_y[i]){
      cv::Point2f point;
      point.x = msg->red_x[i];
      point.y = msg->red_y[i];
      std::cout<<"pixel"<<point.x<<" "<<point.y<<std::endl;
      auto center_point=parser_->parse(point);
      center_point.y=15-center_point.y;
      center_point.x=center_point.x+=3;
      // center_point.y=center_point.y-=4;
      cv::putText(Map_clone,std::to_string(i+1),cv::Point((Map_clone.cols*center_point.x)/28,Map_clone.rows*(15-center_point.y)/15),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,0),2);
      send_data.radar_enemy_x[i]=center_point.x;
      send_data.radar_enemy_y[i]=center_point.y;


      pcl::PointXYZRGBA send_point;
      send_point.x = center_point.x;
      send_point.y = center_point.y;

      send_point.z = 1;
      send_point.r = 255;
      send_point.g = 0;
      send_point.b = 0;
      send_point.a = 255;
      cloud->points.push_back(send_point);
      }
    }
    if(EnemyColor==2&&msg->red_x[i]*msg->red_y[i]){
      cv::Point2f point;
      point.x = msg->red_x[i];
      point.y = msg->red_y[i];
      auto center_point=parser_->parse(point);
      center_point.x=center_point.x+=3;
      center_point.y=center_point.y-=4;
      send_data.radar_enemy_x[i]=center_point.x;
      send_data.radar_enemy_y[i]=15-center_point.y;
      pcl::PointXYZRGBA send_point;
      send_point.x = center_point.x;
      send_point.y = 15-center_point.y;
      send_point.z = 1;
      send_point.r = 255;
      send_point.g = 0;
      send_point.b = 0;
      send_point.a = 255;
      cloud->points.push_back(send_point);
      if(i!=5){
      cv::Point2f point;
      point.x = msg->blue_x[i];
      point.y = msg->blue_y[i];
      auto center_point=parser_->parse(point);
      send_data.radar_enemy_x[i]=center_point.x;
      send_data.radar_enemy_y[i]=15-center_point.y;
      pcl::PointXYZRGBA send_point;
      send_point.x = center_point.x;
      send_point.y = 15-center_point.y;
      send_point.z = 1;
      send_point.r = 0;
      send_point.g = 0;
      send_point.b = 255;
      send_point.a = 255;
      cloud->points.push_back(send_point);
      }
    }
  }
  */
  // 决定发哪个车
  // if(cars.empty()){return;}
  // if(!cars.empty()){
  //   srand(unsigned(time(nullptr)));
  //   shuffle(cars.begin(), cars.end(), std::mt19937(std::random_device()()));
  //   // send_data.target_position_x=cars[0].x;
  //   // send_data.target_position_y=cars[0].y;
  //   // send_data.target_robot_id=cars[0].id;
  // }
  //如果只有一个车，那么就发这个车
  //如果有多个车并且mark都大于115，那么随机发一个车
  //如果有多个车并且mark不都大于115，那么首先删掉mark大于115的车
  //然后如果有英雄，那么发英雄
  //如果没有英雄，那么随机发一个车
  // if(cars.size()==1){
  //   send_data.target_position_x=cars[0].x;
  //   send_data.target_position_y=cars[0].y;
  //   send_data.target_robot_id=cars[0].id;
  // }
  // if(cars.size()>1){
  //   std::vector<map_car> temp=cars;
  //   if(EnemyColor==0){
  //     for(int i=cars.size()-1;i>=0;i--){
  //       int real_id=cars[i].id-101;
  //       if(real_id==6)real_id-=1;
  //       if(markers[real_id]>115){
  //         temp.erase(temp.begin()+i);
  //       }
  //     }
  //   }
  //   else{
  //     for(int i=cars.size()-1;i>=0;i--){
  //       int real_id=cars[i].id-1;
  //       if(real_id==6)real_id-=1;
  //       if(markers[real_id]>115){
  //         temp.erase(temp.begin()+i);
  //       }
  //     }
  //   }
  //   if(temp.empty()){
  //     send_data.target_position_x=cars[0].x;
  //     send_data.target_position_y=cars[0].y;
  //     send_data.target_robot_id=cars[0].id;
  //   }
  //   if(temp.size()!=0){
  //     send_data.target_position_x=temp[0].x;
  //     send_data.target_position_y=temp[0].y;
  //     send_data.target_robot_id=temp[0].id;
  //     for(int i=0;i<temp.size();i++){
  //       if(temp[i].id==101||temp[i].id==1){
  //         send_data.target_position_x=temp[i].x;
  //         send_data.target_position_y=temp[i].y;
  //         send_data.target_robot_id=temp[i].id;
  //         break;
  //       }
  //     }
  //   }
  // }
  send_data.header.stamp=msg->header.stamp;
  pub_radar->publish(send_data);
  auto cloud_msg = sensor_msgs::msg::PointCloud2();
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = "rm_frame";
  cloud_msg.header.stamp = msg->header.stamp;
  pub->publish(cloud_msg);
  // cv::imshow("resolve",Map_clone);
  // cv::waitKey(100);
} 
} // namespace tdt_radar
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::Resolve)