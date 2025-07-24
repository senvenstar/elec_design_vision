#include "main.hpp"

using namespace cv;
using namespace std;

int totalFrameCounter = 0;

std::chrono::high_resolution_clock::time_point last_tp;

int binary_threshold = 128;
int rect_size_threshold = 200;
int trace_point_num_for_long_edge = 10;
int trace_point_num_for_short_edge = 6;

int red_upper_l = 255;
int red_upper_a = 255;
int red_upper_b = 200;
int red_lower_l = 0;
int red_lower_a = 151;
int red_lower_b = 96;

int same_point_threshold = 8;


int q3_status = 100;

// 比较 Point 的 y 坐标
bool compareY(const Point& a, const Point& b) {
    return a.y < b.y;
}

// 比较 Point 的 x 坐标
bool compareX(const Point& a, const Point& b) {
    return a.x < b.x;
}

// 排序函数
vector<Point> orderPointsClockwise(vector<Point> points) {
    bool y_equal_flag = false;

    for (int i=0; i!= 4; i++) {
        if (norm(points[i].y - points[(i+1)%4].y) < 30) {
            y_equal_flag = true;
        }
    }

    vector<Point> orderedPoints;

    if (y_equal_flag) {
        // 2. 按 y 值排序，前两个是上边两个点，后两个是下边两个点
        sort(points.begin(), points.end(), compareY);

        vector<Point> topPoints(points.begin(), points.begin() + 2);
        vector<Point> bottomPoints(points.begin() + 2, points.end());

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

        vector<Point> Points(points.begin()+1, points.begin() + 3);

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

// 插值获取从 p1 到 p2 的 n 个点（含 p1，不含 p2）
vector<Point> getPointsBetween(const Point& p1, const Point& p2, int n) {
    vector<Point> points;
    for (int i = 0; i < n; ++i) {
        double alpha = (double)i / n;
        Point p = p1 + (p2 - p1) * alpha;
        points.push_back(p);
    }
    return points;
}

// 获取矩形每条边上的点，按顺时针排列
vector<Point> getOrderedEdgePoints(const vector<Point>& corners) {
    vector<Point> allPoints;

    int long_edge_index = 0;
    Point p1 = corners[0];
    Point p2 = corners[1];
    Point p3 = corners[2];
    if (norm(p1-p2) < norm(p2-p3)) {
        long_edge_index = 1;
    }

    // 总共 4 条边
    for (int i = 0; i < 4; ++i) {
        Point p1 = corners[i];
        Point p2 = corners[(i + 1) % 4];

        vector<Point> edgePoints;

        if (i == long_edge_index || i == long_edge_index + 2) {
            edgePoints = getPointsBetween(p1, p2, trace_point_num_for_long_edge);
        }
        else {
            edgePoints = getPointsBetween(p1, p2, trace_point_num_for_short_edge);
        }

        // 添加当前边上的点（不重复添加角点）
        allPoints.insert(allPoints.end(), edgePoints.begin(), edgePoints.end());
    }

    return allPoints;
}

int main(void)
{
    wiringPiSetup();    // 初始化
    
    key_board_init();

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

    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 960);
    // cap.set(cv::CAP_PROP_FPS, 30);

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
    createTrackbar("binary_threshold", "trackbar", &binary_threshold, 255, NULL);
    createTrackbar("rect_size_threshold", "trackbar", &rect_size_threshold, 4000, NULL);
    createTrackbar("trace_point_num_for_long_edge", "trackbar", &trace_point_num_for_long_edge, 40, NULL);
    createTrackbar("trace_point_num_for_short_edge", "trackbar", &trace_point_num_for_short_edge, 40, NULL);
    createTrackbar("red_upper_l", "trackbar", &red_upper_l, 255, NULL);
    createTrackbar("red_upper_a", "trackbar", &red_upper_a, 255, NULL);
    createTrackbar("red_upper_b", "trackbar", &red_upper_b, 255, NULL);
    createTrackbar("red_lower_l", "trackbar", &red_lower_l, 255, NULL);
    createTrackbar("red_lower_a", "trackbar", &red_lower_a, 255, NULL);
    createTrackbar("red_lower_b", "trackbar", &red_lower_b, 255, NULL);
    createTrackbar("same_point_threshold", "trackbar", &same_point_threshold, 30, NULL);

    vector<Point> trace_points;
    int track_step = 0;
    int x_error = 0;
    int y_error = 0;

    bool temp=false;

    bool last_pressed = false;
    std::chrono::high_resolution_clock::time_point last_pressed_tp;

    while (true) {
        std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();

        cout << "dt: " << std::chrono::duration_cast<std::chrono::microseconds>(tp - last_tp).count() / 1e6 << endl;

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
        if (q3_status == 100) {
            if (mcu_data.start_track_flag == (uint8_t)1) {
                q3_status = 0;
            }
        }


        if (q3_status == 0) {
            // 转为灰度图
            Mat gray;
            cvtColor(frame, gray, COLOR_BGR2GRAY);

            // 使用高斯模糊减少噪声
            GaussianBlur(gray, gray, Size(9, 9), 0);

            // 二值化处理
            Mat binary;
            threshold(gray, binary, binary_threshold, 255, THRESH_BINARY_INV);

            // // 定义结构元素大小
            // Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));

            // // 开运算：去除小的外部噪点
            // morphologyEx(binary, binary, MORPH_OPEN, element);

            // // 闭运算：填补内部孔洞
            // morphologyEx(binary, binary, MORPH_CLOSE, element);


            // 查找轮廓
            vector<vector<Point>> contours;
            vector<Vec4i> hierarchy;
            findContours(binary, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

            // 用于保存外轮廓和内轮廓
            vector<Point> outerRect, innerRect;
            int outer_rect_index = -1;
            int inner_rect_index = -1;

            // int counter = 0;

            // 遍历轮廓，寻找外轮廓和内轮廓（假设结构为一个外矩形包含一个内矩形）
            for (size_t i = 0; i < contours.size(); ++i) {
                if (contourArea(contours[i]) < rect_size_threshold) {
                    continue;
                }

                // counter++;

                // 如果该轮廓有子轮廓（可能是外矩形）
                if (hierarchy[i][2] >= 0) {
                    outerRect = contours[i];
                    innerRect = contours[hierarchy[i][2]];

                    outer_rect_index = i;
                    inner_rect_index = hierarchy[i][2];
                    break;
                }

                // 如果该轮廓是子轮廓（可能是内矩形）
                if (hierarchy[i][3] >= 0) {
                    innerRect = contours[i];
                    outerRect = contours[hierarchy[i][3]];

                    outer_rect_index = hierarchy[i][3];
                    inner_rect_index = i;
                    break;
                }
            }

            // cout << "counter: " << counter << endl;

            vector<Point> approxOuter;
            vector<Point> approxInner;

            if (!outerRect.empty() && !innerRect.empty()) {
                // 拟合外矩形
                approxPolyDP(outerRect, approxOuter, arcLength(outerRect, true) * 0.03, true);

                // 拟合内矩形
                approxPolyDP(innerRect, approxInner, arcLength(innerRect, true) * 0.03, true);

                cout << "approxOuter" << approxOuter <<endl;
                cout << "approxInner" << approxInner <<endl;
            }
            else {
                cout << "no contour" << endl;
            }
            
            vector<Point> sorted_outer_rect_points;
            vector<Point> sorted_inner_rect_points;
            vector<Point> sorted_trace_corners;

            if (approxOuter.size() == 4 && approxInner.size() == 4) {
                sorted_outer_rect_points = orderPointsClockwise(approxOuter);
                // sorted_inner_rect_points = orderPointsClockwise(approxInner);

                for (int i=0; i!=4; i++) {
                    for (int j=0; j!=4; j++) {
                        if (norm(sorted_outer_rect_points[i]-approxInner[j]) < 100) {
                            sorted_inner_rect_points.push_back(approxInner[j]);
                            break;
                        }
                    }
                }

                cout << "sorted_outer_rect_points" << sorted_outer_rect_points << endl;
                cout << "sorted_inner_rect_points" << sorted_inner_rect_points << endl;

                if (sorted_inner_rect_points.size() == 4) {
                    for (int i=0; i!=4; i++) {
                        Point corner;
                        corner = (sorted_outer_rect_points[i] + sorted_inner_rect_points[i]) / 2;

                        sorted_trace_corners.push_back(corner);

                        cout << "sorted_trace_corners" << sorted_trace_corners << endl;

                        trace_points = getOrderedEdgePoints(sorted_trace_corners);

                        trace_points.push_back(trace_points[0]);
                    }
                }
                else {
                    cout << "fail to match inner corners" << endl;
                }

                if (!trace_points.empty()) {
                    q3_status = 1;
                }
            }
            else {
                cout << "not a rect" << endl;
            }

            if (display["predic_show"]) {
                for (size_t i = 0; i < contours.size(); ++i) {
                    drawContours(frame, contours, i, Scalar(255, 255, 255), 2);

                }

                if (outer_rect_index != -1 && inner_rect_index != -1 && sorted_inner_rect_points.size() == 4) {
                    
                    drawContours(frame, contours, outer_rect_index, Scalar(255, 255, 255), 2);
                    drawContours(frame, contours, inner_rect_index, Scalar(255, 255, 255), 2);

                    if (!sorted_outer_rect_points.empty() && !sorted_outer_rect_points.empty()) {
                        // 绘制外矩形角点
                        circle(frame, sorted_outer_rect_points[0], 5, Scalar(0, 0, 0), 2);
                        circle(frame, sorted_outer_rect_points[1], 5, Scalar(0, 0, 100), 2);
                        circle(frame, sorted_outer_rect_points[2], 5, Scalar(0, 0, 180), 2);
                        circle(frame, sorted_outer_rect_points[3], 5, Scalar(0, 0, 255), 2);

                        // 绘制内矩形角点
                        circle(frame, sorted_inner_rect_points[0], 5, Scalar(0, 0, 0), 2);
                        circle(frame, sorted_inner_rect_points[1], 5, Scalar(0, 0, 100), 2);
                        circle(frame, sorted_inner_rect_points[2], 5, Scalar(0, 0, 180), 2);
                        circle(frame, sorted_inner_rect_points[3], 5, Scalar(0, 0, 255), 2);
                    }
                    
                }

                if (!sorted_trace_corners.empty()) {
                    for (const auto& pt : sorted_trace_corners) {
                        circle(frame, pt, 5, Scalar(0, 0, 255), FILLED);
                    }

                    for (const auto& pt : trace_points) {
                        circle(frame, pt, 5, Scalar(255, 0, 0), FILLED);
                    }
                }

                cv::imshow("Camera Feed", frame);


                cv::imshow("Binary", binary);

                cv::waitKey(1);
            }
        }
        else if (q3_status == 1) {
            // 2. 转换到 Lab 颜色空间
            Mat lab;
            cvtColor(frame, lab, COLOR_BGR2Lab);

            // 3. 定义红色在 Lab 空间的阈值范围
            Scalar lower_red = Scalar(red_lower_l, red_lower_a, red_lower_b);   // L, a, b 下限
            Scalar upper_red = Scalar(red_upper_l, red_upper_a, red_upper_b); // L, a, b 上限

            // 4. 使用 inRange 进行颜色阈值分割
            Mat mask;
            inRange(lab, lower_red, upper_red, mask);

            // 6. 查找轮廓
            vector<vector<Point>> contours;
            findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            Point centroid;
            Point target;

            vector<Point> filtered_contour;

            for (auto contour : contours) {
                if (contourArea(contour) > 1) {
                    filtered_contour = contour;
                }
            }


            if (filtered_contour.size() > 0) {
                // 1. 计算矩
                Moments m = moments(filtered_contour);

                // 2. 计算质心坐标
                if (m.m00 != 0) {
                    centroid.x = m.m10 / m.m00;
                    centroid.y = m.m01 / m.m00;
                }

                if (centroid.x == 0) {
                    temp = true;
                }

                // 3. 打印结果
                cout << "current coordinate: (" << centroid.x << ", " << centroid.y << ")" << endl;   

                target = trace_points[track_step];

                if (track_step < (trace_point_num_for_long_edge + trace_point_num_for_short_edge)*2) {
                    if (norm(centroid - target) < same_point_threshold) {
                        track_step++;
                    }
                }
                else if (track_step == (trace_point_num_for_long_edge + trace_point_num_for_short_edge)*2){
                    if (norm(centroid - target) < 3) {
                        track_step++;
                    }
                }
                

                // 检测按键
                // char key = static_cast<char>(cv::waitKey(1));
                // if (key == ' ' && last_key_pressed == false) {
                //     track_step++;
                //     last_key_pressed = true;
                // }

                // if (key != ' ') {
                //     last_key_pressed = false;
                // }

                if (track_step < (trace_point_num_for_long_edge + trace_point_num_for_short_edge)*2+1) {
                    target = trace_points[track_step];
                }
                else {
                    q3_status = 2;
                }

                x_error = target.x - centroid.x;
                y_error = target.y - centroid.y;
            }
            else {
                cout << "no red point" << endl;

                x_error = 0;
                y_error = 0;
            }

            


            if (display["predic_show"]) {
                drawContours(frame, contours, 0, Scalar(0, 255, 0), 2);
                circle(frame, target, 5, Scalar(0, 255, 0), 2);

                cv::imshow("Camera Feed", frame);

                cv::imshow("mask", mask);

                cv::waitKey(1);
            }

        }
        
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
                q3_status,
                x_error,
                y_error
                );

            LOGM_S("[transmit] status:%d | x_error:%d | y_error:%d",
                q3_status,
                x_error,
                y_error
                );
        }

        if (imu != nullptr)
        {
            mcu_data.start_track_flag = imu->mcu_data.start_track_flag;
        }

        last_tp = tp;
    }

    // 释放资源
    cap.release();
    cv::destroyAllWindows();

    return 0;



}
