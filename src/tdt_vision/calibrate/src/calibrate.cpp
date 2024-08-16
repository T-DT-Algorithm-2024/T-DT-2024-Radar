#include "calibrate.h"
#include <string>

namespace tdt_radar {

    Calibrate::Calibrate(const rclcpp::NodeOptions &options)
            : Node("radar_calibrate_node", options)

    {
        std::cout<<"Calibrate start"<<std::endl;
        cv::namedWindow("calibrate", cv::WINDOW_AUTOSIZE);
        cv::resizeWindow("calibrate",1440,900);
        cv::moveWindow("calibrate", 1920-1440, 1080-900);
        cv::namedWindow("ROI", cv::WINDOW_AUTOSIZE);
        cv::resizeWindow("ROI", 400, 400);
        cv::moveWindow("ROI", 0,0);
        cv::setMouseCallback("calibrate", mousecallback, 0);
        cv::FileStorage fs;
        fs.open("./config/camera_params.yaml", cv::FileStorage::READ);
        fs["camera_matrix"] >> camera_matrix;
        fs["dist_coeffs"] >> dist_coeffs;
        int usechessborad=0;
        fs["usechessborad"] >> usechessborad;
        fs.release();
        // std::string param_name = "radar";
        // std::string param_path = "./config/radar_param.jsonc";
        // LoadParam::InitParam(param_name, param_path);
        // LoadParam::ReadParam(param_name, "camera_matrix", camera_matrix);
        // LoadParam::ReadParam(param_name, "dist_coeffs", dist_coeffs);
        // LoadParam::ReadParam(param_name, "EnemyColor", EnemyColor);
        // LoadParam::ReadParam(param_name, "usechessborad", usechessborad);


        std::cout<<"Calibrate 1"<<std::endl;


        real_points.push_back(self_R0TL);
        real_points.push_back(self_R0TR);
        real_points.push_back(self_Tower);
        real_points.push_back(enemy_Base);
        real_points.push_back(enemy_Tower);
        parser_ = new parser();
        

        // publish_tf();
        RCLCPP_INFO(this->get_logger(),"\n" 
                                  "      ┏━┓       ┏━┓\n"
                                  "    ┏━┛ ┻━━━━━━━┛ ┻━┓\n"
                                  "    ┃               ┃\n"
                                  "    ┃       ━       ┃\n"
                                  "    ┃  ┳━┛     ┗━┳  ┃\n"
                                  "    ┃               ┃\n"
                                  "    ┃      ━┻━      ┃\n"
                                  "    ┃               ┃\n"
                                  "    ┗━━━┓       ┏━━━┛\n"
                                  "        ┃       ┃\n"
                                  "        ┃       ┃\n"
                                  "        ┃       ┃\n"
                                  "        ┃       ┃\n"
                                  "        ┃       ┗━━━━━━━┓\n"
                                  "        ┃               ┣━━┓\n"
                                  "        ┃              ┏┛\n"
                                  "        ┗━┓┓┏━━━━━━━┳┓┏┛\n"
                                  "          ┃┫┫       ┃┫┫\n"
                                  "          ┗┻┛       ┗┻┛");
        // cv::waitKey(100);
        this->broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "rm_frame";
        transformStamped.child_frame_id = "camera_frame";

        std::cout<<"Calibrate 2"<<std::endl;

        if(usechessborad){
        image_sub = this->create_subscription<sensor_msgs::msg::Image>(
                "camera_image", rclcpp::SensorDataQoS(),
                std::bind(&Calibrate::ChessboardCallback, this, std::placeholders::_1));}
        else{
        image_sub = this->create_subscription<sensor_msgs::msg::Image>(
                "camera_image", rclcpp::SensorDataQoS(),
                std::bind(&Calibrate::callback, this, std::placeholders::_1));}
        compressed_image_sub = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                "compressed_image", rclcpp::SensorDataQoS(),
                std::bind(&Calibrate::compressed_callback, this, std::placeholders::_1));
        std::cout<<"Calibrate end"<<std::endl;
    }

    void Calibrate::publish_tf(){
  
        cv::Mat R;
        cv::Rodrigues(parser_->world_rvec, R);
        Eigen::Matrix3d eigen_R;               
        cv::cv2eigen(R, eigen_R);
        Eigen::Quaterniond eigen_quat(eigen_R);
        transformStamped.transform.rotation.x = eigen_quat.x();
        transformStamped.transform.rotation.y = eigen_quat.y();
        transformStamped.transform.rotation.z = eigen_quat.z();
        transformStamped.transform.rotation.w = eigen_quat.w();
        //将parser的平移矩阵和旋转矩阵转换成tf2的TransformStamped
        auto RT = R.t();
        cv::Mat tvec = -RT * parser_->world_tvec;
        transformStamped.transform.translation.x = tvec.at<double>(0, 0);
        transformStamped.transform.translation.y = tvec.at<double>(1, 0);
        transformStamped.transform.translation.z = tvec.at<double>(2, 0);
        transformStamped.header.stamp = this->now();
        broadcaster_->sendTransform(transformStamped);
    }

    void Calibrate::change_outmatrix(double x,double y,double z){
        cv::FileStorage fs;
        fs.open("./config/out_matrix.yaml", cv::FileStorage::READ);
        cv::Mat world_rvec;
        fs["world_rvec"] >> world_rvec;
        // std::cout<<world_rvec<<std::endl;
        fs.release();
        fs.open("./config/out_matrix.yaml", cv::FileStorage::WRITE);
        //将xyz变换加到rvec上
        cv::Mat input(3, 1, CV_64F);
        input.at<double>(0, 0) = x;
        input.at<double>(1, 0) = y;
        input.at<double>(2, 0) = z;
        world_rvec += input;
        fs << "world_rvec" << world_rvec;
        cv::Mat tvec(3, 1, CV_64F);
        if(EnemyColor==2){
            tvec.at<double>(0, 0) = -1.2;
            tvec.at<double>(1, 0) = -5.4;
            tvec.at<double>(2, 0) = 4.1;

        }else{
            tvec.at<double>(0, 0) = 29.2;
            tvec.at<double>(1, 0) = -9.6;
            tvec.at<double>(2, 0) = 4.1;
        }
        // std::cout<<tvec<<std::endl;
        cv::Mat Rmat;
        cv::Rodrigues(world_rvec, Rmat);
        // auto Rmat_inv=Rmat.t();
        auto tvec_inv=-Rmat*tvec;
        std::cout<<tvec_inv<<std::endl;
        fs << "world_tvec" << tvec_inv;
        fs.release();
        parser_->Change_Matrix();
        // publish_tf();
    }
    void Calibrate::callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat calib_img;
        cv::resize(img, calib_img, cv::Size(1536, 1125));
        cvimage_ = calib_img;
        if(is_calibrating){
            cv::putText(img, std::to_string(pick_points.size()), cv::Point(50, 200), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(0, 0, 255), 2);
            if(pick_points.size() == real_points.size()){
            solve();
            parser_->Change_Matrix();
            // publish_tf();
            }
        }
        else{
            parser_->draw_ui(img);
            cv::putText(img,"Press Enter to Calibrate !!!",cv::Point(50,200),cv::FONT_HERSHEY_SIMPLEX,3,cv::Scalar(0,0,255),2);
        }
        auto temp = img.clone();
        cv::resize(img, img, cv::Size(1536, 1125));
        cv::imshow("calibrate", img);
        //按下回车键，开始标定
        auto key =cv::waitKey(10);
        // auto key = cv::pollKey();
        switch (key)
        {
            case 13:
                is_calibrating = true;
                break;
            // case 'w':
            //     change_outmatrix(0,0,0.01);
            //     break;
            // case 'a':
            //     change_outmatrix(0,0.01,0);
            //     break;
            // case 's':
            //     change_outmatrix(0,0,-0.01);
            //     break;
            // case 'd':
            //     change_outmatrix(0,-0.01,0);
            //     break;
            // case 'q':
            //     change_outmatrix(0.01,0,0);
            //     break;
            // case 'e':
            //     change_outmatrix(-0.01,0,0);
            //     break;
            default:
                break;
        }       
    }
        void Calibrate::compressed_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        auto img = cv::imdecode(msg->data, cv::IMREAD_COLOR);
        cv::Mat calib_img;
        cv::resize(img, calib_img, cv::Size(1536, 1125));
        cvimage_ = calib_img;
        if(is_calibrating){
            cv::putText(img, std::to_string(pick_points.size()), cv::Point(50, 200), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(0, 0, 255), 2);
            if(pick_points.size() == real_points.size()){
            solve();
            parser_->Change_Matrix();
            // publish_tf();
            }
        }
        else{
            parser_->draw_ui(img);
            cv::putText(img,"Press Enter to Calibrate !!!",cv::Point(50,200),cv::FONT_HERSHEY_SIMPLEX,3,cv::Scalar(0,0,255),2);
        }
        auto temp = img.clone();
        cv::resize(img, img, cv::Size(1536, 1125));
        cv::imshow("calibrate", img);
        //按下回车键，开始标定
        auto key =cv::waitKey(10);
        switch (key)
        {
            case 13:
                is_calibrating = true;
                break;
            default:
                break;
        }       
    }
    
    void Calibrate::ChessboardCallback(const sensor_msgs::msg::Image::SharedPtr msg){
        auto img = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::putText(img,"Press Enter to Calibrate !!!",cv::Point(50,200),cv::FONT_HERSHEY_SIMPLEX,3,cv::Scalar(0,0,255),2);
        cv::Mat show_img;
        // parser_->draw_ui(img);
        cv::resize(img, show_img, cv::Size(1536, 1125));
        cv::imshow("calibrate", show_img);
        if(cv::waitKey(10) == 13){
            std::vector<cv::Point2f> corners;
            cv::Mat gray;
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
            cv::findChessboardCorners(gray, cv::Size(11, 7), corners);
            cv::drawChessboardCorners(img, cv::Size(11, 7), corners, true);
            if (corners.size() != 77)
            {
                std::cout << "ChessBoard Error" << std::endl;
                return;
            }
            std::reverse(corners.begin(), corners.end());
            for(int i=0;i<corners.size();i++)
            {
                std::cout<<corners[i]<<std::endl;
                cv::putText(img, std::to_string(i), corners[i], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1, 8, 0);
            }
            std::vector<cv::Point3f> ChessBoardPoints;
            float size = 0.25;
            for (int i = 0; i < 7; i++)
            {
                for (int j = 0; j < 11; j++)
                {
                    ChessBoardPoints.push_back(cv::Point3f(j * size, -i * size, 0));
                }
            }
            // cout<<ChessBoardPoints.size()<<endl;
            // cout<<corners.size()<<endl;
            cv::Mat world_rvec, world_tvec;
            cv::solvePnP(ChessBoardPoints, corners, camera_matrix, dist_coeffs, world_rvec, world_tvec, 0, cv::SOLVEPNP_EPNP); // duo点
            std::string file_address_ = "config/out_matrix.yaml";
            cv::FileStorage fs(file_address_, cv::FileStorage::WRITE);
            fs << "world_rvec" << world_rvec;
            fs << "world_tvec" << world_tvec;
            fs.release();
            cv::resize(img, show_img, cv::Size(1536, 1125));
            cv::imshow("calibrate", show_img);
            cv::waitKey(0);
        }
    }

    void mousecallback(int event, int x, int y, int flags, void *userdata)
    {
        int temp_key = 0; // 初始化temp_key

        switch (event)
        {
        case cv::EVENT_LBUTTONDOWN:
            if (is_calibrating) {

            do {
                temp_key = cv::waitKey(10);
                switch (temp_key)
                {
                case 'w': y -= 1; break; // 向上移动
                case 'a': x -= 1; break; // 向左移动
                case 's': y += 1; break; // 向下移动
                case 'd': x += 1; break; // 向右移动
                }

                // 确保x和y在安全区域内
                x = std::max(50, std::min(x, cvimage_.cols - 50));
                y = std::max(50, std::min(y, cvimage_.rows - 50));

                // 更新ROI和显示
                cv::Mat roi = cvimage_(cv::Rect(x - 50, y - 50, 100, 100));
                cv::Mat dst;
                cv::resize(roi, dst, cv::Size(400, 400));
                cv::line(dst, cv::Point(200, 100), cv::Point(200, 300), cv::Scalar(0, 0, 255), 1);
                cv::line(dst, cv::Point(100, 200), cv::Point(300, 200), cv::Scalar(0, 0, 255), 1);
                cv::imshow("ROI", dst);

            } while (temp_key != 'n'); // 按'n'退出循环
                x *= 1.3333333333 * 2;
                y *= 1.3333333333 * 2;
                std::cout << "x:" << x << " y:" << y << std::endl;
                pick_points.push_back(cv::Point2f(x, y));
            }
            break;

        case cv::EVENT_MOUSEMOVE:
            // 如果x,y不落在安全区域内，则break
            if (x > cvimage_.cols - 50 || y > cvimage_.rows - 50 || x < 50 || y < 50)
                break;
            cv::Mat roi = cvimage_(cv::Rect(x - 50, y - 50, 100, 100));
            cv::Mat dst;
            cv::resize(roi, dst, cv::Size(400, 400));
            cv::line(dst, cv::Point(200, 100), cv::Point(200, 300), cv::Scalar(0, 0, 255), 1);
            cv::line(dst, cv::Point(100, 200), cv::Point(300, 200), cv::Scalar(0, 0, 255), 1);
            cv::imshow("ROI", dst);
            break;
        }
    }

    void Calibrate::solve(){
        cv::solvePnP(real_points, pick_points, camera_matrix, dist_coeffs, rvec, tvec, 0, cv::SOLVEPNP_EPNP);
        std::cout << "rvec:" << rvec << std::endl;
        std::cout << "tvec:" << tvec << std::endl;
        cv::FileStorage fs;
        fs.open("./config/out_matrix.yaml", cv::FileStorage::WRITE);
        fs << "world_rvec" << rvec;
        fs << "world_tvec" << tvec;
        fs.release();
        pick_points.clear();
        is_calibrating = false;
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::Calibrate);
