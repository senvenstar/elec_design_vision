#include "main.hpp"

using namespace cv;
using namespace std;

int totalFrameCounter = 0;

std::chrono::high_resolution_clock::time_point last_tp;

int threshold_value = 111;     // 阈值
int black_pixel_threshold = 1000;
int black_block_pixel_threshold = 50;
int black_block_num_threshold = 2;

int main(void)
{
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

    cv::VideoCapture cap(0, cv::CAP_V4L2);

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

    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    // cap.set(cv::CAP_PROP_FPS, 30);

    cv::Mat frame;

    double width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double fps = cap.get(cv::CAP_PROP_FPS);
    std::cout << "Resolution: " << width << "x" << height << ", FPS: " << fps << std::endl;

    Attitude attitude;
    RobotStatus robotstatus;

    // 定义 ROI 的位置和大小（x, y, width, height）
    cv::Rect tracking_roi_rect1(150, 140, 340, 40);
    cv::Rect tracking_roi_rect2(150, 180, 340, 40);
    cv::Rect tracking_roi_rect3(150, 220, 340, 40);
    cv::Rect tracking_roi_rect4(150, 260, 340, 40);
    cv::Rect tracking_roi_rect5(150, 300, 340, 40);

    vector<cv::Rect> tracking_roi_rects;
    tracking_roi_rects.push_back(tracking_roi_rect1);
    tracking_roi_rects.push_back(tracking_roi_rect2);
    tracking_roi_rects.push_back(tracking_roi_rect3);
    tracking_roi_rects.push_back(tracking_roi_rect4);
    tracking_roi_rects.push_back(tracking_roi_rect5);

    vector<float> tracking_roi_weight;
    tracking_roi_weight.push_back(0.05);
    tracking_roi_weight.push_back(0.15);
    tracking_roi_weight.push_back(0.2);
    tracking_roi_weight.push_back(0.25);
    tracking_roi_weight.push_back(0.35);

    cv::Rect left_roi_rect(0, 150, 50, 100);
    cv::Rect right_roi_rect(590, 150, 50, 100);
    cv::Rect up_roi_rect(150, 0, 340, 150);

    // 创建窗口
    namedWindow("trackbar", WINDOW_AUTOSIZE);

    // 创建滑动条
    createTrackbar("Binary Threshold", "trackbar", &threshold_value, 255, NULL);
    createTrackbar("Black Pixel Count Threshold", "trackbar", &black_pixel_threshold, 5000, NULL);
    createTrackbar("black_block_pixel_threshold", "trackbar", &black_block_pixel_threshold, 1000, NULL);
    createTrackbar("black_block_num_threshold", "trackbar", &black_block_num_threshold, 20, NULL);

    while (true) {
        // get picture
        cap >> frame;

        totalFrameCounter++;

        // std::cout << totalFrameCounter << std::endl;

        std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();

        cout << "dt: " << std::chrono::duration_cast<std::chrono::microseconds>(tp - last_tp).count() / 1e6 << endl;

        if (frame.empty()) {
            LOGW_S("empty image");
            continue;
        }

        if (imu != nullptr)
            // imu->start();

        if (is_image_input_flipped)
        {
            cv::flip(frame, frame, -1);
        }





        // proceed picture
        // 创建灰度图
        cv::Mat gray_frame;
        cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);

        // 二值化处理
        cv::Mat binary_frame;
        
        cv::threshold(gray_frame, binary_frame, threshold_value, 255, cv::THRESH_BINARY_INV);

        vector<cv::Mat> tracking_rois;
        // 获取 ROI 区域
        for (int i=0; i!=5; i++) {
            cv::Mat tracking_roi = binary_frame(tracking_roi_rects[i]);
            tracking_rois.push_back(tracking_roi);
        }

        // 创建一个副本，避免显示原始图像上的 ROI 被修改
        // cv::Mat tracking_roi_copy = tracking_roi.clone();

        // 获取 ROI 区域
        cv::Mat left_roi = binary_frame(left_roi_rect);

        // 创建一个副本，避免显示原始图像上的 ROI 被修改
        // cv::Mat left_roi_copy = left_roi.clone();

        // 获取 ROI 区域
        cv::Mat right_roi = binary_frame(right_roi_rect);

        // 创建一个副本，避免显示原始图像上的 ROI 被修改
        // cv::Mat right_roi_copy = right_roi.clone();

        // 获取 ROI 区域
        cv::Mat up_roi = binary_frame(up_roi_rect);

        // 创建一个副本，避免显示原始图像上的 ROI 被修改
        // cv::Mat up_roi_copy = up_roi.clone();


        // 计算黑色像素数量
        int left_black_pixel_count = countNonZero(left_roi == 255);
        int right_black_pixel_count = countNonZero(right_roi == 255);
        int up_black_pixel_count = countNonZero(up_roi == 255);

        bool reach_cross_flag = false;

        // 判断是否出现大量黑色
        if (left_black_pixel_count > black_pixel_threshold && right_black_pixel_count > black_pixel_threshold) {
            reach_cross_flag = true;
            cout << "is a cross" << endl;
        } else {
            cout << "not a cross" << endl;
        }

        bool reach_destination_flag = false;

        if (!reach_cross_flag) {
            // 查找轮廓
            vector<vector<Point>> contours;
            findContours(up_roi, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            int blackBlocksCount = 0;
            for (size_t i = 0; i < contours.size(); i++)
            {
                // 计算轮廓面积，根据实际需要调整最小面积大小
                double area = contourArea(contours[i]);
                if (area > black_block_pixel_threshold) { // 假设最小色块面积为50像素
                    blackBlocksCount++;
                }
            }      

            cout << "black block number: " << black_block_num_threshold << endl;
            
            if (blackBlocksCount > black_block_num_threshold) {
                reach_destination_flag = true;
                cout << "reach destination" << endl;
            } else {
                cout << "not reach destination" << endl;
            }
        
        }

        double fit_centerX = 0;
        double trace_center_error = 0;
        double angle_deg = 90;

        if (!reach_cross_flag && !reach_destination_flag) {
            vector<Point2d> trace_centers;

            for (int i=0; i!=5; i++) {
                // 查找所有轮廓
                vector<vector<Point>> contours;
                vector<Vec4i> hierarchy;
                findContours(tracking_rois[i], contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

                // 寻找最大面积轮廓
                double maxArea = 0;
                int maxContourIdx = -1;

                for (size_t i = 0; i < contours.size(); ++i) {
                    double area = contourArea(contours[i]);

                    if (area > maxArea) {
                        maxArea = area;
                        maxContourIdx = i;
                    }
                }

                // 判断是否找到有效轮廓
                if (maxContourIdx == -1) {
                    continue;
                }

                Moments m = moments(contours[maxContourIdx]);
                trace_centers.push_back(Point2f(static_cast<float>(m.m10 / m.m00 + tracking_roi_rects[i].x),
                                        static_cast<float>(m.m01 / m.m00) + tracking_roi_rects[i].y));
            }
            
            bool fit_center_valid = false;
            for (int i=0; i!=trace_centers.size(); i++) {
                fit_centerX += trace_centers[i].x/5;
                fit_center_valid = true;
            }

            if (!fit_center_valid) {
                fit_centerX = 320;
            }

            trace_center_error = 320 - fit_centerX;

            if (trace_centers.size() >= 3) {
                // 用于存储拟合结果：(vx, vy, x0, y0)
                Vec4f line;

                // 进行直线拟合
                fitLine(trace_centers, line, DIST_L2, 0, 1e-2, 1e-2);

                // 提取拟合结果
                float vx = line[0];  // 方向向量 x 分量
                float vy = line[1];  // 方向向量 y 分量
                float x0 = line[2];  // 直线上某一点 x 坐标
                float y0 = line[3];  // 直线上某一点 y 坐标

                // 计算倾斜角度（相对于 y 轴）
                angle_deg = atan2(vx, vy) * 180 / CV_PI;

                if (angle_deg > 90) {
                    angle_deg -= 180;
                }
            }   
         
            // 输出结果
            cout << "fit center x: " << fit_centerX << endl;
            cout << "拟合直线的角度（度）: " << angle_deg << "°" << endl;
        }

        



        // show picture
        // 在原图上画出ROI区域的框
        if (display["predic_show"]) {
            for (int i=0; i!=5; i++) {
                cv::rectangle(frame, tracking_roi_rects[i], cv::Scalar(0, 255, 0), 2);
            }
            cv::rectangle(frame, left_roi_rect, cv::Scalar(0, 255, 0), 2);
            cv::rectangle(frame, right_roi_rect, cv::Scalar(0, 255, 0), 2);
            cv::rectangle(frame, up_roi_rect, cv::Scalar(255, 0, 0), 2);

            circle(frame, Point2d(fit_centerX, 240), 5, Scalar(0, 0, 255), FILLED);

            // 定义线段长度
            double length = 200.0;

            // 计算方向向量
            double dx = length * sin(angle_deg/180*M_PI);
            double dy = length * cos(angle_deg/180*M_PI);

            // 计算两个端点
            Point pt1(fit_centerX - dx, 240 - dy);
            Point pt2(fit_centerX + dx, 240 + dy);

            // 在图像上画线
            line(frame, pt1, pt2, Scalar(0, 255, 0), 2);

            cv::imshow("Camera Feed", frame);


            for (int i=0; i!=5; i++) {
                cv::rectangle(binary_frame, tracking_roi_rects[i], cv::Scalar(0, 255, 0), 2);
            }
            cv::rectangle(binary_frame, left_roi_rect, cv::Scalar(0, 255, 0), 2);
            cv::rectangle(binary_frame, right_roi_rect, cv::Scalar(0, 255, 0), 2);
            cv::rectangle(binary_frame, up_roi_rect, cv::Scalar(0, 255, 0), 2);

            cv::imshow("Binary frame", binary_frame);

            cv::waitKey(1);
        }




        // cboard communication
        if (imu != nullptr && imu->is_open())
        {
            imu->transmit_cmd(
                reach_cross_flag,
                reach_destination_flag,
                trace_center_error,
                angle_deg
                );

            LOGM_S("[transmit] cross:%d | dest:%d | center_e:%6.2f | angle:%6.2f",
                reach_cross_flag,
                reach_destination_flag,
                trace_center_error,
                angle_deg
                );
        }

        if (imu != nullptr)
        {
            // imu->get_attitude(attitude);
            // imu->get_robotstatus(robotstatus);
        }

        last_tp = tp;
    }

    // 释放资源
    cap.release();
    cv::destroyAllWindows();

    return 0;



}
