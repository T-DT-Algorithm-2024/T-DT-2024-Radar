#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <vision_interface/msg/detect_result.hpp>
#include <vision_interface/msg/radar2_sentry.hpp>
#include <vision_interface/msg/radar_warn.hpp>
#include <vision_interface/msg/match_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
namespace tdt_radar {
    class DebugMap : public rclcpp::Node {
    public:
        explicit DebugMap(const rclcpp::NodeOptions & options)
            : Node("debug_map", options){
            detect_result_sub = this->create_subscription<vision_interface::msg::DetectResult>(
                "/kalman_detect", 10, std::bind(&DebugMap::callback, this, std::placeholders::_1));
            camera_detect_sub = this->create_subscription<vision_interface::msg::DetectResult>(
                "/resolve_result", rclcpp::SensorDataQoS(), std::bind(&DebugMap::camera_callback, this, std::placeholders::_1));
            map = cv::imread("config/RM2024.png");
            match_info_sub = this->create_subscription<vision_interface::msg::MatchInfo>(
                "/match_info", 10, std::bind(&DebugMap::save_match_info, this, std::placeholders::_1));
            radar_warn_pub = this->create_publisher<vision_interface::msg::RadarWarn>("/hero_state", 10);
            debug_map_pub = this->create_publisher<sensor_msgs::msg::Image>("/map_2d", 10);
            radar2sentry_pub = this->create_publisher<vision_interface::msg::Radar2Sentry>("/Radar2Sentry", rclcpp::SensorDataQoS());
            cv::resize(map, map, cv::Size(28*25, 15*25));
        }
        void save_match_info(const std::shared_ptr<vision_interface::msg::MatchInfo> msg){
            this->match_info = *msg;
            if(msg->self_color==1)
            match_info.self_color = 2;
        }

        void show_map(){
            auto now_time = std::chrono::system_clock::now();
            double time = std::chrono::duration_cast<std::chrono::milliseconds>(now_time.time_since_epoch()).count()/1000.0;
            auto clone_map = map.clone();
            for(int i=0;i<6;i++){
                int number = i+1;
                if(number==6)number++;
                if(blue_point[i].x*blue_point[i].y&&time-blue_update[i]<2){
                    cv::Point2f point = cv::Point2f(clone_map.cols*blue_point[i].x/28,clone_map.rows*(15-blue_point[i].y)/15);
                    cv::circle(clone_map,point,10,cv::Scalar(200,0,0),-1);
                    cv::putText(clone_map,std::to_string(number),cv::Point(point.x-6,point.y+5),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255));
                }
                if(red_point[i].x*red_point[i].y&&time-red_update[i]<2){
                    cv::Point2f point = cv::Point2f(clone_map.cols*red_point[i].x/28,clone_map.rows*(15-red_point[i].y)/15);
                    cv::circle(clone_map,point,10,cv::Scalar(0,0,200),-1);
                    cv::putText(clone_map,std::to_string(number),cv::Point(point.x-6,point.y+5),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255));
                }
            }
            cv::imshow("map", clone_map);
            cv::waitKey(1);
        }
        void camera_callback(const std::shared_ptr<vision_interface::msg::DetectResult> msg){
            auto now = std::chrono::system_clock::now();
            double time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count()/1000.0;
            for(int i=0;i<6;i++){
                if(msg->blue_x[i]*msg->blue_y[i]){//如果是蓝色，所有坐标都要转换
                    if(time - blue_time[i] > 5){
                        blue_point[i] = cv::Point2f(msg->blue_x[i], msg->blue_y[i]);
                        if(!match_info.self_color){
                            blue_point[i] = cv::Point2f(28-msg->blue_x[i], 15-msg->blue_y[i]);
                        }
                        blue_update[i] = time;
                    }
                }
                if(msg->red_x[i]*msg->red_y[i]){
                    if(time - red_time[i] > 5){
                        red_point[i] = cv::Point2f(msg->red_x[i], msg->red_y[i]);
                        if(!match_info.self_color){
                            red_point[i] = cv::Point2f(28-msg->red_x[i], 15-msg->red_y[i]);
                        }
                        red_update[i] = time;
                    }
                }
            }
            show_map();
        }

        void callback(const std::shared_ptr<vision_interface::msg::DetectResult> msg){
            auto now = std::chrono::system_clock::now();
            double time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count()/1000.0;
            for(int i = 0; i<6; i++){
                if(msg->blue_x[i]*msg->blue_y[i]){
                    blue_point[i] = cv::Point2f(msg->blue_x[i], msg->blue_y[i]);
                    blue_time[i] = time;
                    blue_update[i] = time;

                }
                if(msg->red_x[i]*msg->red_y[i]){
                    red_point[i] = cv::Point2f(msg->red_x[i], msg->red_y[i]);
                    red_time[i] = time;
                    red_update[i] = time;
                }
            }
            show_map();
            vision_interface::msg::RadarWarn radar_warn;
            if(hero_count1>10){
                radar_warn.hero_state = 1;
                radar_warn_pub->publish(radar_warn);
            }
            else if(hero_count2>10){
                radar_warn.hero_state = 2;
                radar_warn_pub->publish(radar_warn);
            }
            //1是吊射 2是自己家
            if(match_info.self_color==0){//自己是蓝色 发送红色信息
                if(red_point[0].x>(28-8.668)){
                    hero_count1++;
                    hero_count2--;
                }else
                if(red_point[0].x<(28-20.3)&&red_point[0].x>(28-25.075)&&red_point[0].y<15&&red_point[0].y>10.3){
                    hero_count1--;
                    hero_count2++;
                }else{
                    hero_count1--;
                    hero_count2--;
                }
            }
            if(match_info.self_color==2){//自己是红色 发送蓝色信息
                if(blue_point[0].x<8.668){
                    hero_count1++;
                    hero_count2--;
                }else
                if(blue_point[0].x>20.3&&blue_point[0].x<25.075&&blue_point[0].y>0&&blue_point[0].y<(15-10.3)){
                    hero_count1--;
                    hero_count2++;
                }else{
                    hero_count1--;
                    hero_count2--;
                }
            }

            vision_interface::msg::Radar2Sentry radar2sentry;
            if(match_info.self_color==0){//自己是蓝色 发送红色信息
                for(int i=0;i<6;i++){
                    if(!relax[i]){
                        if(match_info.marks[i]>=117){
                            relax[i]=true;
                            relax_time[i] = time;
                        } 
                        else{
                            if(time-red_update[i]<2){
                            radar2sentry.radar_enemy_x[i] = red_point[i].x;
                            radar2sentry.radar_enemy_y[i] = red_point[i].y;
                            }
                            // else if(match_info.match_time<420&&match_info.match_time>360&&i==1){
                            //     radar2sentry.radar_enemy_x[i] = 28-14.556;
                            //     radar2sentry.radar_enemy_y[i] = 6.947;
                            // }
                        }
                    }else{
                        //当mark在(105,117)间隔0.4s发送一次
                        if(match_info.marks[i]<105){
                            relax[i]=false;
                            if(time-red_update[i]<2){
                            radar2sentry.radar_enemy_x[i] = red_point[i].x;
                            radar2sentry.radar_enemy_y[i] = red_point[i].y;
                            } 
                        }else if(time-relax_time[i]>0.35){
                            relax_time[i] = time;
                            if(time-red_update[i]<2){
                            radar2sentry.radar_enemy_x[i] = red_point[i].x;
                            radar2sentry.radar_enemy_y[i] = red_point[i].y; 
                            }
                        }
                    }
                }
            }
            if(match_info.self_color==2){//自己是红色 发送蓝色信息
                for(int i=0;i<6;i++){
                    if(!relax[i]){
                        if(match_info.marks[i]>=117){
                            relax[i]=true;
                            relax_time[i] = time;
                        } 
                        else{
                            if(time-blue_update[i]<2){
                            radar2sentry.radar_enemy_x[i] = blue_point[i].x;
                            radar2sentry.radar_enemy_y[i] = blue_point[i].y;
                            }
                            // else if(match_info.match_time<420&&match_info.match_time>360&&i==1){
                            //     radar2sentry.radar_enemy_x[i] = 14.556;
                            //     radar2sentry.radar_enemy_y[i] = 8.057;
                            // }                    
                        }
                    }else{
                        //当mark在(105,117)间隔0.4s发送一次
                        if(match_info.marks[i]<105){
                            relax[i]=false;
                            if(time-blue_update[i]<2){
                            radar2sentry.radar_enemy_x[i] = blue_point[i].x;
                            radar2sentry.radar_enemy_y[i] = blue_point[i].y;
                            } 
                        }else if(time-relax_time[i]>0.35){
                            relax_time[i] = time;
                            if(time-blue_update[i]<2){
                            radar2sentry.radar_enemy_x[i] = blue_point[i].x;
                            radar2sentry.radar_enemy_y[i] = blue_point[i].y;
                            } 
                        }
                    }
                }
            }
            radar2sentry_pub->publish(radar2sentry);
            // auto clone_map = map.clone();
            // for (int i=0;i<6;i++){
            //     int number = i+1;
            //     if(number==6)number++;
            //     cv::Point2f blue_point(msg->blue_x[i], msg->blue_y[i]);
            //     if(blue_point.x*blue_point.y){
            //         cv::circle(clone_map,cv::Point((clone_map.cols*blue_point.x)/28,clone_map.rows*(15-blue_point.y)/15),10,cv::Scalar(200,0,0),-1);
            //         cv::putText(clone_map,std::to_string(number),cv::Point((clone_map.cols*blue_point.x)/28-6,clone_map.rows*(15-blue_point.y)/15+5),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255));
            //     }
            //     cv::Point2f red_point(msg->red_x[i], msg->red_y[i]);
            //     if(red_point.x*red_point.y){
            //         cv::circle(clone_map,cv::Point((clone_map.cols*red_point.x)/28,clone_map.rows*(15-red_point.y)/15),10,cv::Scalar(0,0,200),-1);
            //         cv::putText(clone_map,std::to_string(number),cv::Point((clone_map.cols*red_point.x)/28-6,clone_map.rows*(15-red_point.y)/15+5),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255));
            //     }
            // }
            // auto send_image = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", clone_map).toImageMsg();
            // debug_map_pub->publish(*send_image);
            // count++;
            // if(count==20){
            // // time_t now = time(0);
            // // tm *ltm = localtime(&now);
            // // std::string filename=std::to_string(ltm->tm_year+1900)+"_"+
            // // std::to_string(ltm->tm_mon+1)+"_"+
            // // std::to_string(ltm->tm_mday)+"_"+
            // // std::to_string(ltm->tm_hour)+"_"+
            // // std::to_string(ltm->tm_min)+"_"+
            // // std::to_string(ltm->tm_sec);
            // // std::ofstream out("/home/tdt/Label/big/test1/"+filename+".txt");
            // // out<<match_info.match_time<<std::endl;
            // // out<<(uint16_t)match_info.self_color<<std::endl;
            // // for(int i=0;i<16;i++){
            // //   out<<(uint16_t)match_info.robot_hp[i]<<" ";
            // // }
            // // out.close();   
            // // cv::imwrite("/home/tdt/Label/big/test1"+filename +".png", clone_map);
            // count = 0;
            // }
        }
        rclcpp::Subscription<vision_interface::msg::DetectResult>::SharedPtr detect_result_sub;
        rclcpp::Subscription<vision_interface::msg::DetectResult>::SharedPtr camera_detect_sub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_map_pub;
        rclcpp::Publisher<vision_interface::msg::RadarWarn>::SharedPtr radar_warn_pub;
        rclcpp::Publisher<vision_interface::msg::Radar2Sentry>::SharedPtr radar2sentry_pub;
        rclcpp::Subscription<vision_interface::msg::MatchInfo>::SharedPtr match_info_sub;//打标用

        double blue_time[6];//单位s
        double red_time[6];//单位s

        bool relax[6];
        double relax_time[6];

        double blue_update[6];
        double red_update[6];

        int hero_count1;
        int hero_count2;
        
        cv::Point2f blue_point[6];
        cv::Point2f red_point[6];

        vision_interface::msg::MatchInfo match_info;
        cv::Mat map;
        int count = 0;//20帧保存一次
        };
}  // namespace tdt_radar

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node_options = rclcpp::NodeOptions(); // 创建NodeOptions实例
    rclcpp::spin(std::make_shared<tdt_radar::DebugMap>(node_options)); // 传递NodeOptions实例
    rclcpp::shutdown();
    return 0;
}