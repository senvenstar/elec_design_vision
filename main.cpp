#include "main.hpp"

using namespace cv;
using namespace std;

int totalFrameCounter = 0;

std::chrono::high_resolution_clock::time_point last_tp;

uint8_t imageData[LCD_W*LCD_H*2];

// 相机内参矩阵 K（根据你的标定结果填写）
cv::Mat K = (cv::Mat_<double>(3, 3) << 
    1.44521186e+03, 0.00000000e+00, 5.33904882e+02,
    0.00000000e+00, 1.44396638e+03, 3.28810013e+02,
    0.00000000e+00, 0.00000000e+00, 1.00000000e+00);

double fx = 1.44521186e+03;
double cx = 5.33904882e+02;
double fy = 1.44396638e+03;
double cy = 3.28810013e+02;


// 畸变系数 D（k1, k2, p1, p2, k3）
cv::Mat D = (cv::Mat_<double>(5, 1) << 
    2.11194249e-01, -1.30918347e+00,  3.39484720e-04, -1.36706102e-03,
  1.50127996e+00);

std::vector<cv::Point3f> target_corners = {cv::Point3f(-0.01305 , -0.0087, 0),
                                            cv::Point3f(0.01305 , -0.0087, 0),
                                            cv::Point3f(0.01305 , 0.0087, 0),
                                            cv::Point3f(-0.01305 , 0.0087, 0)};  // 左上，右上，右下，左下


Mat R_cl = (Mat_<double>(3, 3) <<
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
);

// int rad = 353; // 584;

// int t_y = -30+500;
// int t_z = 30+500;

int rad = 0;

int t_y = 500;
int t_z = 500;


// int binary_threshold = 128;

// 定义绿色范围（HSV 范围）
int lower_green_h = 0;
int lower_green_s = 0;
int lower_green_v = 0;
int upper_green_h = 130;
int upper_green_s = 255;
int upper_green_v = 255;

int red_upper_l = 255;
int red_upper_a = 255;
int red_upper_b = 200;
int red_lower_l = 0;
int red_lower_a = 151;
int red_lower_b = 96;

int purple_upper_l = 255;
int purple_upper_a = 255;
int purple_upper_b = 200;
int purple_lower_l = 0;
int purple_lower_a = 151;
int purple_lower_b = 96;

int rect_size_threshold = 5000;

int mode = 1;
int step = 0;

// 定义圆的参数
float radius = 0.06f;         // 半径
float angleStep = 0.1f;       // 角度步长（弧度）

int same_point_threshold = 100;

// 比较 Point 的 y 坐标
bool compareY(const Point2f& a, const Point2f& b) {
    return a.y < b.y;
}

// 比较 Point 的 x 坐标
bool compareX(const Point2f& a, const Point2f& b) {
    return a.x < b.x;
}

// 排序函数
vector<Point2f> orderPointsClockwise(vector<Point> points) {
    bool y_equal_flag = false;

    for (int i=0; i!= 4; i++) {
        if (norm(points[i].y - points[(i+1)%4].y) < 30) {
            y_equal_flag = true;
        }
    }

    vector<Point2f> orderedPoints;

    if (y_equal_flag) {
        // 2. 按 y 值排序，前两个是上边两个点，后两个是下边两个点
        sort(points.begin(), points.end(), compareY);

        vector<Point2f> topPoints(points.begin(), points.begin() + 2);
        vector<Point2f> bottomPoints(points.begin() + 2, points.end());

        // 3. 上边两个点按 x 排序：左上、右上
        sort(topPoints.begin(), topPoints.end(), compareX);

        // 4. 下边两个点按 x 排序：左下、右下
        sort(bottomPoints.begin(), bottomPoints.end(), compareX);

        // 5. 按顺时针排列：左上、右上、右下、左下
        orderedPoints = {
            topPoints[0],     // 左上
            topPoints[1],     // 右上
            bottomPoints[1],  // 右下
            bottomPoints[0]   // 左下
        };
    }
    else {
        // 2. 按 y 值排序，前两个是上边两个点，后两个是下边两个点
        sort(points.begin(), points.end(), compareY);

        vector<Point2f> Points(points.begin()+1, points.begin() + 3);

        sort(Points.begin(), Points.end(), compareX);

        // 5. 按顺时针排列：左上、右上、右下、左下
        orderedPoints = {
            points[0],     // 左上
            Points[1], 
            points[3],    // 右上
            Points[0],  // 右下
        };
    }

    return orderedPoints;
}

int main(void)
{
    wiringPiSetup();    // 初始化
    
    key_board_init();
    lcd_init();

    // init
    cmd_parser parser;
    map<string, string> info;
    map<string, bool> display;
    screen = new Log();
    try
    {
        parser.parse("launch.cfg", info, display);
    }
    catch (const char *msg)
    {
        LOGE(screen, "%s", msg);
        return false;
    }

    LOGM_F("open log file success!");
    LOGM_S("open log file success!");

    LOGM_S("[sensor] comm I/O on %s", info["port"].c_str());
    LOGM_S("[sensor] video input from %s", info["source"].c_str());

    cv::VideoCapture cap("v4l2src device=/dev/video0 ! image/jpeg,width=1024,height=768,framerate=30/1 ! jpegdec ! videoconvert ! appsink", cv::CAP_GSTREAMER);
    
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Could not open camera." << std::endl;
        return -1;
    }

    LOGM_S("[senosr] video ready");
    LOGM_S("[senosr] IMU input from %s", info["imu"].c_str());

    UartIMU *imu = new UartIMU(info["port"]);

    if (imu == nullptr || !imu->init())
    {
        LOGE_S("[sensor]Error: IMU init failed");
        return -1;
    }

    bool is_image_input_flipped = false;

    if (info["flip"] == "1")
    {
        is_image_input_flipped = true;
        LOGW_S("[sensor] Input image will be flipped");
    }
    else 
    {
        LOGW_S("[sensor] Input image will not be flipped");
    }

    LOGM_S("[senosr] IMU ready");
    LOGM_S("[sensor] ready");

    cv::Mat frame;

    double width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double fps = cap.get(cv::CAP_PROP_FPS);
    std::cout << "Resolution: " << width << "x" << height << ", FPS: " << fps << std::endl;

    Attitude attitude;
    RobotStatus robotstatus;

    pc_mcu_data_t mcu_data;

    // 定义 ROI 的位置和大小（x, y, width, height）
    // cv::Rect tracking_roi_rect1(150, 140, 340, 40);


    // 创建窗口
    namedWindow("trackbar", WINDOW_AUTOSIZE);

    // 创建滑动条
    // createTrackbar("binary_threshold", "trackbar", &binary_threshold, 255, NULL);
    createTrackbar("lower_green_h", "trackbar", &lower_green_h, 255, NULL);
    createTrackbar("lower_green_s", "trackbar", &lower_green_s, 255, NULL);
    createTrackbar("lower_green_v", "trackbar", &lower_green_v, 255, NULL);
    createTrackbar("upper_green_h", "trackbar", &upper_green_h, 255, NULL);
    createTrackbar("upper_green_s", "trackbar", &upper_green_s, 255, NULL);
    createTrackbar("upper_green_v", "trackbar", &upper_green_v, 255, NULL);
    createTrackbar("t_y", "trackbar", &t_y, 1000, NULL);
    createTrackbar("t_z", "trackbar", &t_z, 1000, NULL);
    createTrackbar("purple_upper_l", "trackbar", &purple_upper_l, 255, NULL);
    createTrackbar("purple_upper_a", "trackbar", &purple_upper_a, 255, NULL);
    createTrackbar("purple_upper_b", "trackbar", &purple_upper_b, 255, NULL);
    createTrackbar("purple_lower_l", "trackbar", &purple_lower_l, 255, NULL);
    createTrackbar("purple_lower_a", "trackbar", &purple_lower_a, 255, NULL);
    createTrackbar("purple_lower_b", "trackbar", &purple_lower_b, 255, NULL);
    createTrackbar("rad", "trackbar", &rad, 2000, NULL);
    // createTrackbar("rect_size_threshold", "trackbar", &rect_size_threshold, 4000, NULL);




    while (true) {
        std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();

        cout << "dt: " << std::chrono::duration_cast<std::chrono::microseconds>(tp - last_tp).count() / 1e6 << endl;

        // key board
        // if (key_pressed_down(K1, tp)) {
        //     cout << "pressed" << endl;
        // }

        // key_update_last_pressed(tp);

        // int sw_status = get_switch_status(SW2);
        // if (sw_status == SW_A_ON) {
        //     cout << "A" << endl;
        // }
        // else if (sw_status == SW_B_ON) {
        //     cout << "B" << endl;
        // }
        // else if (sw_status == SW_MID) {
        //     cout << "mid" << endl;
        // }
        // else {
        //     cout << "error" << endl;
        // }

        // get picture
        cap >> frame;

        //  // 去畸变
        // cv::Mat undistorted;
        // cv::undistort(frame, undistorted, K, D);

        // cv::imshow("Camera Feed", frame);
        // cv::imshow("undistorted", undistorted);
        // cv::waitKey(1);

        // lcd screen
        // cv::Mat resizedImage;
        // cv::resize(frame, resizedImage, cv::Size(LCD_W, LCD_H),cv::INTER_CUBIC);
        // //转换为RGB565格式
        // cv::Mat rgb565Image;
        // cv::cvtColor(resizedImage, rgb565Image, cv::COLOR_BGR2BGR565);
        // // 获取图像的宽度和高度
        // int width = rgb565Image.cols;
        // int height = rgb565Image.rows;
        // for (int y = 0; y < height; y++) {
        //     for (int x = 0; x < width; x++) {
        //         // 获取RGB565值
        //         uint16_t rgb565Value = rgb565Image.at<uint16_t>(y, x);
        //         // 分开高位和低位，并写入数组
        //         uint8_t highByte = (rgb565Value >> 8) & 0xFF;
        //         uint8_t lowByte = rgb565Value & 0xFF;
        //         imageData[(y * width + x) * 2]=highByte;
        //         imageData[(y * width + x) * 2 + 1]=lowByte;
        //     }
        // }

        // LCD_ShowPicture2(0,0,LCD_W,LCD_H,imageData);

        totalFrameCounter++;

        // std::cout << totalFrameCounter << std::endl;

        if (frame.empty()) {
            LOGW_S("empty image");
            continue;
        }

        if (imu != nullptr)
            imu->start();

        if (is_image_input_flipped)
        {
            cv::flip(frame, frame, -1);
        }



        // proceed picture
        // 检测按键
        // char key = static_cast<char>(cv::waitKey(1));
        // if (key == ' ' && last_key_pressed == false) {
        //     track_step++;
        //     last_key_pressed = true;
        // }

        // if (key != ' ') {
        //     last_key_pressed = false;
        // }




        // 转换为 HSV 颜色空间
        cv::Mat lab;
        cv::cvtColor(frame, lab, cv::COLOR_BGR2Lab);

        cv::Scalar lower_green(lower_green_h, lower_green_s, lower_green_v);
        cv::Scalar upper_green(upper_green_h, upper_green_s, upper_green_v);

        // 提取绿色区域
        cv::Mat mask;
        cv::inRange(lab, lower_green, upper_green, mask);

         // 定义结构元素大小和形状
        int morph_size = 5; // 结构元素的尺寸
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morph_size + 1, 2 * morph_size + 1), cv::Point(morph_size, morph_size));

        // 执行闭操作
        cv::Mat closed;
        cv::morphologyEx(mask, closed, cv::MORPH_CLOSE, element);

        // // 定义高斯核大小和标准差
        // int kernel_size = 5; // 必须是正奇数
        // double sigma_x = 0; // X方向上的高斯核标准差。如果设为0，则根据内核大小自动计算
        
        // // 对图像应用高斯模糊
        // cv::Mat blurred;
        // cv::GaussianBlur(closed, blurred, cv::Size(kernel_size, kernel_size), sigma_x);

        // 寻找轮廓
        int target_index = -1;
        double maxArea = 0;
        vector<Point> bestContour;

        // 查找轮廓
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(closed, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        // 用于保存外轮廓和内轮廓
        
        vector<int> inner_rect_indexes; 

        // 遍历轮廓，寻找外轮廓和内轮廓（假设结构为一个外矩形包含一个内矩形）
        for (size_t i = 0; i < contours.size(); ++i) {
            // drawContours(frame, contours, i, Scalar(0, 255, 0), 2);

            if (contourArea(contours[i]) < rect_size_threshold) {
                continue;
            }

            // 如果该轮廓有子轮廓（可能是外矩形）
            if (hierarchy[i][2] >= 0) {
                inner_rect_indexes.push_back(hierarchy[i][2]);
            }

            // 如果该轮廓是子轮廓（可能是内矩形）
            if (hierarchy[i][3] >= 0) {
                inner_rect_indexes.push_back(i);
            }
        }

        int max_size = 0;
        int inner_rect_index = -1;

        for (int i=0; i!=inner_rect_indexes.size(); i++) {
            int size = contourArea(contours[inner_rect_indexes[i]]);
            if (size > max_size) {
                max_size = size;
                inner_rect_index = inner_rect_indexes[i];
            }
        }

        vector<Point> corners;

        if (inner_rect_index != -1) {
            drawContours(frame, contours, inner_rect_index, Scalar(255, 0, 0), 2);

            approxPolyDP(contours[inner_rect_index], corners, arcLength(contours[inner_rect_index], true) * 0.03, true);
        }
        else {
            cout << "no inner_rect_index" << endl;
        }

        cv::Point2f target_center;
        double yaw;
        double pitch;

        if (corners.size() == 4) {
            vector<Point2f> img_corners = orderPointsClockwise(corners);

            cv::Mat rvec, tvec;

            if (mode == 0) {
                cv::solvePnP(target_corners, img_corners, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE);

                cout << "目标点相机坐标系中的位置 P_l: " << tvec.t() << endl;

                // 6. 将世界坐标系原点 (0,0,0) 投影回图像（这正是 tvec 对应的点）
                vector<Point3f> pointsToProject;
                pointsToProject.push_back(Point3f(0, 0, 0)); // 世界坐标原点

                vector<Point2f> projectedPoints;
                projectPoints(pointsToProject, rvec, tvec, K, D, projectedPoints);

                // 7. 在图像上画出投影点
                Point2f p = projectedPoints[0];
                circle(frame, p, 5, Scalar(0, 0, 255), -1);           // 红色实心圆

                cout << "project: " << p << endl;
            }
            else if (mode == 1) {
                cv::Point3f circle_target_on_plane;
                circle_target_on_plane.x = radius * cos(angleStep*step);
                circle_target_on_plane.y = radius * sin(angleStep*step);
                circle_target_on_plane.z = 0;

                std::vector<cv::Point3f> circle_target_corners = {cv::Point3f(-0.01305 , -0.0087, 0) - circle_target_on_plane,
                                            cv::Point3f(0.01305 , -0.0087, 0) - circle_target_on_plane,
                                            cv::Point3f(0.01305 , 0.0087, 0) - circle_target_on_plane,
                                            cv::Point3f(-0.01305 , 0.0087, 0) - circle_target_on_plane};  // 左上，右上，右下，左下

                cv::solvePnP(circle_target_corners, img_corners, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE);

                cout << "目标点相机坐标系中的位置 P_l: " << tvec.t() << endl;

                // 6. 将世界坐标系原点 (0,0,0) 投影回图像（这正是 tvec 对应的点）
                vector<Point3f> pointsToProject;
                pointsToProject.push_back(Point3f(0, 0, 0)); // 世界坐标原点

                vector<Point2f> projectedPoints;
                projectPoints(pointsToProject, rvec, tvec, K, D, projectedPoints);

                // 7. 在图像上画出投影点
                Point2f p = projectedPoints[0];
                circle(frame, p, 5, Scalar(0, 0, 255), -1);           // 红色实心圆

                cout << "project: " << p << endl;
            }
            

            float pitch_bias = (rad - 1000) / 1000.0f;
            R_cl = (cv::Mat_<double>(3,3) <<
                1,              0,               0,
                0,  cos(pitch_bias), -sin(pitch_bias),
                0,  sin(pitch_bias),  cos(pitch_bias));
            Mat t_cl = (Mat_<double>(3, 1) << 0.00, (t_y-500)/1000.0f, (t_z-500)/1000.0f);

            Mat P_c = tvec.clone();
            Mat P_l = R_cl * P_c + t_cl;

            cout << "目标点在激光坐标系中的位置 P_l: " << P_l.t() << endl;

            Vec3d target_laser(P_l.at<double>(0), P_l.at<double>(1), P_l.at<double>(2));

            if (mode == 1) {
                if (sqrt(pow(target_laser(0), 2) + pow(target_laser(0), 2)) < same_point_threshold/1000.0f) {
                    step++;
                }
            }

            
            yaw = atan2(target_laser[0], target_laser[2])/M_PI*180.0f;
            pitch = atan2(target_laser[1], target_laser[2])/M_PI*180.0f;

            cout << "yaw: " << yaw << "pitch: " << pitch << endl;
        }
        else {
            cout << "no rect" << endl;
        }
        

        uint8_t valid;
        if (inner_rect_index != -1)
        {
            valid = 1;
        }
        else {
            valid = 0;
        }


        circle(frame, cv::Point(533, 328), 5, Scalar(0, 0, 255), -1);


        




        // cv::Point2f target_center;
        // double yaw;
        // double pitch;

        // // 3. 定义红色在 Lab 空间的阈值范围
        // // Scalar lower_red = Scalar(red_lower_l, red_lower_a, red_lower_b);   // L, a, b 下限
        // // Scalar upper_red = Scalar(red_upper_l, red_upper_a, red_upper_b); // L, a, b 上限

        // Scalar lower_red = Scalar(purple_lower_l, purple_lower_a, purple_lower_b);   // L, a, b 下限
        // Scalar upper_red = Scalar(purple_upper_l, purple_upper_a, purple_upper_b); // L, a, b 上限

        // // 4. 使用 inRange 进行颜色阈值分割
        // Mat mask_r;
        // inRange(lab, lower_red, upper_red, mask_r);

        // // 6. 查找轮廓
        // vector<vector<Point>> contours_r;
        // findContours(mask_r, contours_r, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Point centroid;
        // vector<Point> filtered_contour;

        // for (auto contour : contours_r) {
        //     if (contourArea(contour) > 1) {
        //         filtered_contour = contour;
        //     }
        // }


        // if (filtered_contour.size() > 0) {
        //     // 1. 计算矩
        //     Moments m = moments(filtered_contour);

        //     // 2. 计算质心坐标
        //     if (m.m00 != 0) {
        //         centroid.x = m.m10 / m.m00;
        //         centroid.y = m.m01 / m.m00;
        //     }

        //     // 3. 打印结果
        //     cout << "current coordinate: (" << centroid.x << ", " << centroid.y << ")" << endl; 
        // }
        
        // drawContours(frame, contours_r, 0, Scalar(0, 255, 0), 2);
        // circle(frame, centroid, 5, Scalar(0, 255, 0), 2);




        // if (bestContour.size() > 0 && filtered_contour.size() > 0) {
        //     // 获取矩形中心点
        //     target_center = (bestContour[0]+bestContour[1]+bestContour[2]+bestContour[3])/4;

        //     // 绘制矩形
        //     for (int i = 0; i < 4; ++i)
        //         cv::line(frame, bestContour[i], bestContour[(i+1)%4], cv::Scalar(0, 0, 255), 2);

        //     // 显示中心点
        //     cv::circle(frame, target_center, 5, cv::Scalar(0, 255, 0), -1);

        //     std::cout << "Center of green object: (" << target_center.x << ", " << target_center.y << ")" << std::endl;

        //     yaw = target_center.x - centroid.x;
        //     pitch = target_center.y - centroid.y;

        //     cout << "yaw: " << yaw << "pitch: " << pitch << endl;
            
        // }
        // else {
        //     std::cout << "No green object found!" << std::endl;
        // }



        cv::imshow("Original", frame);
        cv::imshow("mask", closed);
        // cv::imshow("Green Mask", blurred);
        cv::waitKey(1);


        


        // 检查是否需要停止程序
        // if (temp == true) {
        //     std::cout << "Stopping the program." << std::endl;
        //     break; // 退出循环，从而结束程序
        // }



        // show picture
        // 在原图上画出ROI区域的框
        // if (display["predic_show"]) {
        //     cv::imshow("Camera Feed", frame);

        //     cv::waitKey(1);
        // }




        // cboard communication
        if (imu != nullptr && imu->is_open())
        {
            imu->transmit_cmd(
                valid,
                yaw,
                pitch
                );

            LOGM_S("[transmit] status:%d | x_error:%f | y_error:%f",
                valid,
                yaw,
                pitch
                );
        }

        if (imu != nullptr)
        {
            // mcu_data.start_track_flag = imu->mcu_data.start_track_flag;
        }

        last_tp = tp;
    }

    // 释放资源
    cap.release();
    cv::destroyAllWindows();

    return 0;



}
