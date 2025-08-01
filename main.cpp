#include "main.hpp"

using namespace cv;
using namespace std;
using namespace predict;

using _filter = Kalman<1, 2>;
using Matx1 = _filter::Matrix_x1d;
using Matxx = _filter::Matrix_xxd;
using Matxz = _filter::Matrix_xzd;
using Matz1 = _filter::Matrix_z1d;
using Matzx = _filter::Matrix_zxd;
using Matzz = _filter::Matrix_zzd;
using Pos3D = Eigen::Vector3d;


int totalFrameCounter = 0;

std::chrono::high_resolution_clock::time_point last_tp;

uint8_t imageData[LCD_W*LCD_H*2];

// 相机内参矩阵 K（根据你的标定结果填写）
cv::Mat K = (cv::Mat_<double>(3, 3) << 
    // 1.44521186e+03, 0.00000000e+00, 5.33904882e+02,
    // 0.00000000e+00, 1.44396638e+03, 3.28810013e+02,
    // 0.00000000e+00, 0.00000000e+00, 1.00000000e+00);

    1.41551675e+03, 0.00000000e+00, 5.21014820e+02,
    0.00000000e+00, 1.41327590e+03, 3.25974100e+02,
    0.00000000e+00, 0.00000000e+00, 1.00000000e+00);

// 畸变系数 D（k1, k2, p1, p2, k3）
cv::Mat D = (cv::Mat_<double>(5, 1) << 
//     2.11194249e-01, -1.30918347e+00,  3.39484720e-04, -1.36706102e-03,
//   1.50127996e+00);

  2.45758770e-01, -1.62955334e+00,  3.25746049e-04, -1.04823290e-03,
  2.15842987e+00);

std::vector<cv::Point3f> target_corners = {cv::Point3f(-0.1305 , -0.087, 0),
                                            cv::Point3f(0.1305 , -0.087, 0),
                                            cv::Point3f(0.1305 , 0.087, 0),
                                            cv::Point3f(-0.1305 , 0.087, 0)};  // 左上，右上，右下，左下

std::vector<cv::Point3f> two_rect_target_corners = {cv::Point3f(-0.1305 , -0.087, 0),
                                            cv::Point3f(0.1305 , -0.087, 0),
                                            cv::Point3f(0.1305 , 0.087, 0),
                                            cv::Point3f(-0.1305 , 0.087, 0),
                                            cv::Point3f(-0.1485 , -0.105, 0),
                                            cv::Point3f(0.1485 , -0.105, 0),
                                            cv::Point3f(0.1485 , 0.105, 0),
                                            cv::Point3f(-0.1485 , 0.105, 0)};  // 左上，右上，右下，左下


// Mat T_cl = (Mat_<double>(4, 4) <<
//     0., 9.9992705418350314e-01, 1.2078340610415808e-02,
//        2.3820959096292813e-02, -9.9999618897324494e-01,
//        -3.3345931638808607e-05, 2.7606026579383310e-03,
//        9.4282454524722923e-03, 2.7608040470437304e-03,
//        -1.2078294579536585e-02, 9.9992324343474659e-01,
//        -3.1376938094740403e-04, 0., 0., 0., 1. 
// );

Mat T_cl = (Mat_<double>(4, 4) <<
    1, 0, 0,
       0, 0,
       1, 0,
       0, 0, 0,
       1, 0, 0., 0., 0., 1.
);


// int binary_threshold = 128;

// 定义绿色范围（HSV 范围）
int lower_green_h = 0;
int lower_green_s = 0;
int lower_green_v = 0;
int upper_green_h = 125;
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

int pitch_deg = 1010;
int yaw_deg = 992;

int same_point_threshold = 100;
int same_center_threshold = 15;
int lw_ratio_lower = 8;
int lw_ratio_upper = 18;
int area_ratio_lower = 10;
int area_ratio_upper = 15;

int center_roi_area = 40000;
int whiteThreshold = 202; // 定义“偏白色”的亮度阈值 (0-255), 200 是一个较高的值
int whiteRatioThreshold = 7; // 定义“大部分”的比例阈值, 例如 70%

int shoot_threshold = 3;
float yaw_arrive_threshold = 0.5;
float pitch_arrive_threshold = 0.5;

int measurement_variable = 1;
int process_coord_variable = 200;
int process_speed_variable = 60;

// 使用自适应阈值方法（均值法）
int blockSize = 11; // 邻域大小
int C = 2;          // 常数，从计算出的均值或加权均值中减去

int comm_latency_int = 40; // m秒
float process_latency = 0;
int p_coord = 90;


int mode = 0;
int step = 0;

// 定义圆的参数
float radius = 0.06f;         // 半径
float angleStep = 0.1f;       // 角度步长（弧度）

bool last_key_pressed = false;

int shoot_detect_frame = 0;

_filter filter_x; // x轴滤波
_filter filter_y; // y轴滤波
_filter filter_z; // z轴滤波

bool last_track = false;

uint8_t valid = 0;

Matx1 last_px;
Matx1 last_py;
Matx1 last_pz;


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
    // createTrackbar("lower_green_s", "trackbar", &lower_green_s, 255, NULL);
    // createTrackbar("lower_green_v", "trackbar", &lower_green_v, 255, NULL);
    createTrackbar("upper_green_h", "trackbar", &upper_green_h, 255, NULL);
    // createTrackbar("upper_green_s", "trackbar", &upper_green_s, 255, NULL);
    // createTrackbar("upper_green_v", "trackbar", &upper_green_v, 255, NULL);
    createTrackbar("rect_size_threshold", "trackbar", &rect_size_threshold, 4000, NULL);
    createTrackbar("pitch_deg", "trackbar", &pitch_deg, 2000, NULL);
    createTrackbar("yaw_deg", "trackbar", &yaw_deg, 2000, NULL);
    createTrackbar("same_center_threshold", "trackbar", &same_center_threshold, 100, NULL);
    createTrackbar("lw_ratio_lower", "trackbar", &lw_ratio_lower, 50, NULL);
    createTrackbar("lw_ratio_upper", "trackbar", &lw_ratio_upper, 50, NULL);
    createTrackbar("center_roi_area", "trackbar", &center_roi_area, 100000, NULL);
    createTrackbar("whiteThreshold", "trackbar", &whiteThreshold, 255, NULL);
    createTrackbar("whiteRatioThreshold", "trackbar", &whiteRatioThreshold, 10, NULL);
    // createTrackbar("measurement_variable", "trackbar", &measurement_variable, 1000, NULL);
    // createTrackbar("process_coord_variable", "trackbar", &process_coord_variable, 1000, NULL);
    // createTrackbar("process_speed_variable", "trackbar", &process_speed_variable, 1000, NULL);
    // createTrackbar("blockSize", "trackbar", &blockSize, 100, NULL);
    // createTrackbar("C", "trackbar", &C, 100, NULL);
    createTrackbar("comm_latency_int", "trackbar", &comm_latency_int, 1000, NULL);
    createTrackbar("p_coord", "trackbar", &p_coord, 10000, NULL);



    ///初始化滤波器参数
    Matxx A = Matxx::Identity(); //转移矩阵
    Matzx H;                     //观测矩阵
    Matxx R;                     //过程噪声矩阵
    Matzz Q{0.0005};               //测量噪声矩阵
    Matx1 init{0, 0};            //初始值
    ///初始化观测矩阵
    H(0, 0) = 1;
    ///初始化过程方差
    R(0, 0) = 10;
    R(1, 1) = 10;
    ///初始化滤波器
    filter_x = _filter(A, H, R, Q, init, std::chrono::high_resolution_clock::now());
    filter_y = _filter(A, H, R, Q, init, std::chrono::high_resolution_clock::now());
    filter_z = _filter(A, H, R, Q, init, std::chrono::high_resolution_clock::now());

    bool debug = display["predic_debug"];

    while (true) {
        std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();

        auto dt = std::chrono::duration_cast<std::chrono::microseconds>(tp - last_tp).count() / 1e6;

        if (debug)
        cout << "dt: " << dt << endl;

        

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
        




        // 转换为 HSV 颜色空间
        cv::Mat lab;
        cv::cvtColor(frame, lab, cv::COLOR_BGR2Lab);

        cv::Scalar lower_green(lower_green_h, lower_green_s, lower_green_v);
        cv::Scalar upper_green(upper_green_h, upper_green_s, upper_green_v);

        // 提取绿色区域
        cv::Mat mask;
        cv::inRange(lab, lower_green, upper_green, mask);

        //  // 定义结构元素大小和形状
        // int morph_size = 5; // 结构元素的尺寸
        // cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morph_size + 1, 2 * morph_size + 1), cv::Point(morph_size, morph_size));

        // // 执行闭操作
        // cv::Mat closed;
        // cv::morphologyEx(mask, closed, cv::MORPH_CLOSE, element);

        // // 定义高斯核大小和标准差
        // int kernel_size = 5; // 必须是正奇数
        // double sigma_x = 0; // X方向上的高斯核标准差。如果设为0，则根据内核大小自动计算
        
        // // 对图像应用高斯模糊
        // cv::Mat blurred;
        // cv::GaussianBlur(closed, blurred, cv::Size(kernel_size, kernel_size), sigma_x);

        // cv::Mat gray;
        // cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // cv::Mat binary;
        // if (blockSize % 2 == 0) {
        //     cv::adaptiveThreshold(gray, binary, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, blockSize+1, C);
        // }
        // else {
        //     cv::adaptiveThreshold(gray, binary, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, blockSize, C);
        // }
        

        // 寻找轮廓
        int target_index = -1;
        double maxArea = 0;
        vector<Point> bestContour;

        // 查找轮廓
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        // 用于保存外轮廓和内轮廓
        
        vector<int> inner_rect_indexes; 
        vector<int> outer_rect_indexes; 

        // 遍历轮廓，寻找外轮廓和内轮廓（假设结构为一个外矩形包含一个内矩形）
        for (size_t i = 0; i < contours.size(); ++i) {
            // drawContours(frame, contours, i, Scalar(0, 255, 0), 2);

            if (contourArea(contours[i]) < rect_size_threshold) {
                continue;
            }

            // 如果该轮廓有子轮廓（可能是外矩形）
            if (hierarchy[i][2] >= 0) {
                inner_rect_indexes.push_back(hierarchy[i][2]);
                outer_rect_indexes.push_back(i);
            }

            // 如果该轮廓是子轮廓（可能是内矩形）
            if (hierarchy[i][3] >= 0) {
                inner_rect_indexes.push_back(i);
                outer_rect_indexes.push_back(hierarchy[i][3]);
            }
        }

        vector<Point> best_inner_corners;
        vector<Point> best_outer_corners;
        int max_size = 0;

        vector<int> rect_valid_indexes;
        vector<int> same_center_indexes;
        vector<int> white_valid_indexes;
        vector<int> area_valid_indexes;
        int best_index = -1;

        for (int i=0; i!=inner_rect_indexes.size(); i++) {
            // drawContours(frame, contours, inner_rect_indexes[i], Scalar(255, 0, 0), 2);

            // is two rect && length/width valid
            vector<Point> outer_corners;

            approxPolyDP(contours[outer_rect_indexes[i]], outer_corners, arcLength(contours[outer_rect_indexes[i]], true) * 0.03, true);
            
            float outer_lw_ratio = 0;

            if (norm(outer_corners[0]-outer_corners[1]) > norm(outer_corners[1]-outer_corners[2])) {
                outer_lw_ratio = norm(outer_corners[0]-outer_corners[1]) / norm(outer_corners[1]-outer_corners[2]);
            }
            else {
                outer_lw_ratio = norm(outer_corners[1]-outer_corners[2]) / norm(outer_corners[0]-outer_corners[1]);
            }

            if (debug)
            cout << "outer_lw_ratio: " << outer_lw_ratio << endl;

            if (outer_corners.size() == 4 && outer_lw_ratio > lw_ratio_lower/10.0f && outer_lw_ratio < lw_ratio_upper/10.0f) {

                vector<Point> inner_corners;

                approxPolyDP(contours[inner_rect_indexes[i]], inner_corners, arcLength(contours[inner_rect_indexes[i]], true) * 0.01, true);

                float inner_lw_ratio = 0;
                int inner_rect_length = 0;

                if (norm(inner_corners[0]-inner_corners[1]) > norm(inner_corners[1]-inner_corners[2])) {
                    inner_lw_ratio = norm(inner_corners[0]-inner_corners[1]) / norm(inner_corners[1]-inner_corners[2]);

                    inner_rect_length = norm(inner_corners[1]-inner_corners[2]);
                }
                else {
                    inner_lw_ratio = norm(inner_corners[1]-inner_corners[2]) / norm(inner_corners[0]-inner_corners[1]);

                    inner_rect_length = norm(inner_corners[0]-inner_corners[1]);
                }

                if (debug)
                cout << "inner_lw_ratio: " << inner_lw_ratio << endl;

                if (inner_corners.size() == 4 && inner_lw_ratio > lw_ratio_lower/10.0f && inner_lw_ratio < lw_ratio_upper/10.0f) {
                    rect_valid_indexes.push_back(inner_rect_indexes[i]);

                    // same center
                    Point inner_center = (inner_corners[0] + inner_corners[1] + inner_corners[2] + inner_corners[3])/4;
                    Point outer_center = (outer_corners[0] + outer_corners[1] + outer_corners[2] + outer_corners[3])/4;

                    if (debug)
                    cout << "norm(inner_center - outer_center): " << norm(inner_center - outer_center) << endl;

                    if (norm(inner_center - outer_center) < same_center_threshold) {
                        same_center_indexes.push_back(inner_rect_indexes[i]);

                        circle(frame, inner_center, 1, Scalar(0, 255, 0), -1);           // 红色实心圆

                        // center roi is full of white
                        uint16_t center_roi_length = inner_rect_length/2;
                        uint16_t center_roi_width = contourArea(contours[inner_rect_indexes[i]])/4/center_roi_length;

                        if (center_roi_length > 5 && center_roi_width > 5) {
                            int top_left_x = inner_center.x-center_roi_width/2;
                            int top_left_y = inner_center.y-center_roi_length/2;

                            if (debug)
                            cout << "roiRect_raw: " << top_left_x << "  " << top_left_y << "   " << center_roi_width << "  " << center_roi_length << endl;

                            if (top_left_x < 0) {
                                center_roi_width -= -top_left_x;

                                top_left_x = 0;

                            }
                            else if (top_left_x + center_roi_width > 1024) {
                                center_roi_width -= top_left_x + center_roi_width - 1024;

                                top_left_x = 1024 - center_roi_width;

                            }

                            if (top_left_y < 0) {
                                center_roi_length -= -top_left_y;

                                top_left_y = 0;

                            }
                            else if (top_left_y + center_roi_length > 768) {
                                center_roi_length -= top_left_y + center_roi_length - 768;

                                top_left_y = 768 - center_roi_length;

                            }

                            cv::Rect roiRect(top_left_x, top_left_y, center_roi_width, center_roi_length);

                            if (debug)
                            cout << "roiRect: " << top_left_x << "  " << top_left_y << "   " << center_roi_width << "  " << center_roi_length << endl;

                            // 统计灰度值大于 whiteThreshold 的像素数量
                            cv::Mat roi = frame(roiRect);

                            cv::Mat grayRoi;
                            cv::cvtColor(roi, grayRoi, cv::COLOR_BGR2GRAY);

                            cv::Mat mask;
                            cv::threshold(grayRoi, mask, whiteThreshold, 255, cv::THRESH_BINARY_INV); // 大于阈值的设为 255，否则 0

                            imshow("center_roi", mask);
                            
                            int whitePixels = cv::countNonZero(mask); // 统计非零像素（即“偏白色”像素）的数量
                            int totalPixels = roi.rows * roi.cols;
                            double whiteRatio = static_cast<double>(whitePixels) / totalPixels;

                            // double whiteRatio = 0.8;

                            if (debug)
                            cout << "whiteRatio: " << whiteRatio << endl;

                            if (whiteRatio >= whiteRatioThreshold/10.0f) {
                                white_valid_indexes.push_back(inner_rect_indexes[i]);

                                // valid area ratio
                                float area_ratio = contourArea(contours[outer_rect_indexes[i]])/contourArea(contours[inner_rect_indexes[i]]);

                                if (debug)
                                cout << "area_ratio: " << area_ratio << endl;

                                if (area_ratio > area_ratio_lower/10.0f && area_ratio < area_ratio_upper/10.0f) {
                                    area_valid_indexes.push_back(inner_rect_indexes[i]);

                                    // max size
                                    int size = contourArea(contours[inner_rect_indexes[i]]);

                                    if (size > max_size) {
                                        max_size = size;
                                        best_inner_corners = inner_corners;
                                        best_outer_corners = outer_corners;
                                        best_index = inner_rect_indexes[i];
                                    }
                                }  
                            }

                         
                        }
                    }
                    
                }
            }
        }
        
        if (imu != nullptr)
        {
            mcu_data.cur_yaw = imu->mcu_data.cur_yaw;
            mcu_data.cur_pitch = imu->mcu_data.cur_pitch;

            if (debug) {
                cout << "mcu_data.cur_yaw" << mcu_data.cur_yaw << endl;
                cout << "mcu_data.cur_pitch" << mcu_data.cur_pitch << endl;
            }
            
            
        }

        cv::Point2f target_center;
        double yaw;
        double pitch;
        bool shoot;

        double yaw_error;
        double pitch_error;

        double yaw_speed;
        double pitch_speed;

        if (best_inner_corners.size() > 0) {
            vector<Point2f> img_corners = orderPointsClockwise(best_inner_corners);

            cv::Mat rvec, tvec;

            if (mode == 0) {
                cv::solvePnP(target_corners, img_corners, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE);

                if (debug)
                cout << "目标点相机坐标系中的位置 P_l: " << tvec.t() << endl;

                // 6. 将世界坐标系原点 (0,0,0) 投影回图像（这正是 tvec 对应的点）
                vector<Point3f> pointsToProject;
                pointsToProject.push_back(Point3f(0, 0, 0)); // 世界坐标原点

                vector<Point2f> projectedPoints;
                projectPoints(pointsToProject, rvec, tvec, K, D, projectedPoints);

                // 7. 在图像上画出投影点
                Point2f p = projectedPoints[0];
                circle(frame, p, 1, Scalar(0, 0, 255), -1);           // 红色实心圆

                if (debug)
                cout << "project: " << p << endl;

            }
            else if (mode == 1) {
                if (debug)
                cout << step << endl;

                cv::Point3f circle_target_on_plane;
                circle_target_on_plane.x = radius * cos(angleStep*step);
                circle_target_on_plane.y = radius * sin(angleStep*step);
                circle_target_on_plane.z = 0;

                std::vector<cv::Point3f> circle_target_corners = {cv::Point3f(-0.1305 , -0.087, 0) - circle_target_on_plane,
                                            cv::Point3f(0.1305 , -0.087, 0) - circle_target_on_plane,
                                            cv::Point3f(0.1305 , 0.087, 0) - circle_target_on_plane,
                                            cv::Point3f(-0.1305 , 0.087, 0) - circle_target_on_plane};  // 左上，右上，右下，左下

                cv::solvePnP(circle_target_corners, img_corners, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE);

                if (debug)
                cout << "目标点相机坐标系中的位置 P_l: " << tvec.t() << endl;

                // 6. 将世界坐标系原点 (0,0,0) 投影回图像（这正是 tvec 对应的点）
                vector<Point3f> pointsToProject;
                pointsToProject.push_back(Point3f(0, 0, 0)); // 世界坐标原点

                vector<Point2f> projectedPoints;
                projectPoints(pointsToProject, rvec, tvec, K, D, projectedPoints);

                // 7. 在图像上画出投影点
                Point2f p = projectedPoints[0];
                circle(frame, p, 1, Scalar(0, 0, 255), -1);           // 红色实心圆

                if (debug)
                cout << "project: " << p << endl;
            }
            
            double rad = (pitch_deg-1000)/1000.0f;
            double rad_yaw = (yaw_deg-1000)/1000.0f;

            T_cl = (Mat_<double>(4, 4) << cos(rad_yaw), 0, sin(rad_yaw), -0.02403,
                                            sin(rad_yaw)*sin(rad), cos(rad), -sin(rad)*cos(rad_yaw), -0.01097,
                                            -cos(rad)*sin(rad_yaw), sin(rad), cos(rad)*cos(rad_yaw), 0,
                                            0, 0, 0, 1);

            Mat P_c = (Mat_<double>(4, 1) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2), 1);
            Mat P_l = T_cl * P_c;

            if (debug)
            cout << "目标点在激光坐标系中的位置 P_l: " << P_l.t() << endl;

            Vec3d target_laser(P_l.at<double>(0), P_l.at<double>(1), P_l.at<double>(2));

            double cur_yaw = -mcu_data.cur_yaw/180.0f*M_PI;
            double cur_pitch = mcu_data.cur_pitch/180.0f*M_PI;

            cv::Mat T_lw = (Mat_<double>(4, 4) << cos(cur_yaw), 0, sin(cur_yaw), 0,
                                            sin(cur_yaw)*sin(cur_pitch), cos(cur_pitch), -sin(cur_pitch)*cos(cur_yaw), 0,
                                            -cos(cur_pitch)*sin(cur_yaw), sin(cur_pitch), cos(cur_pitch)*cos(cur_yaw), 0,
                                            0, 0, 0, 1);
            Mat P_w = T_lw * P_l;

            if (debug)
            cout << "目标点在世界坐标系中的位置 P_w: " << P_w.t() << endl;

            Vec3d target_world(P_w.at<double>(0), P_w.at<double>(1), P_w.at<double>(2));

            if (last_track) {
                // kalman filter in pw
                Eigen::Matrix<double, 1, 1> z_k_x{target_world(0)};                                            // z_k_x: x轴滤波器观测量
                Eigen::Matrix<double, 1, 1> z_k_y{target_world(1)};  
                Eigen::Matrix<double, 1, 1> z_k_z{target_world(2)}; 
                
                Eigen::Matrix<double, 2, 2> R_x;
                R_x << pow(dt, 4)/4*p_coord/100.0f, pow(dt, 3)/2*p_coord/100.0f,
                        pow(dt, 3)/2*p_coord/100.0f, pow(dt, 2)*p_coord/100.0f;
                
                auto p_x = filter_x.update(z_k_x, tp, R_x);                                                     // p_x: x轴滤波器状态量
                auto p_y = filter_y.update(z_k_y, tp, R_x);   
                auto p_z = filter_z.update(z_k_z, tp, R_x);   

                if (debug)
                cout << "估计当前目标点在世界坐标系中的位置和速度 P_w: " << p_x << "   " << p_y << "   " << p_z << endl;

                auto now_t = std::chrono::high_resolution_clock::now();                                   //
                double process_latency = duration_cast<microseconds>(now_t - tp).count() / 1e6;            //

                double t_delay = comm_latency_int/1000.0f + process_latency;   

                if (debug)
                cout << "t_delay: " << t_delay << endl;

                

                Pos3D s_pw{p_x(0, 0) + t_delay * p_x(1, 0), p_y(0, 0) + t_delay * p_y(1, 0), p_z(0, 0) + t_delay * p_z(1, 0)}; // s_pw: ft后预测点

                if (debug)
                cout << "预测目标点在世界坐标系中的位置 P_w: " << s_pw << endl;

                

                // cal yaw pitch in pl
                cv::Mat T_lw_inv = (Mat_<double>(4, 4) << cos(cur_yaw), sin(cur_yaw)*sin(cur_pitch), -cos(cur_pitch)*sin(cur_yaw), 0,
                                            0, cos(cur_pitch), sin(cur_pitch), 0,
                                            sin(cur_yaw), -sin(cur_pitch)*cos(cur_yaw), cos(cur_pitch)*cos(cur_yaw), 0,
                                            0, 0, 0, 1);
            
                Mat s_w = (Mat_<double>(4, 1) << s_pw(0), s_pw(1), s_pw(2), 1);
                Mat s_l = T_lw_inv * s_w;
                Vec3d s_target_laser(s_l.at<double>(0), s_l.at<double>(1), s_l.at<double>(2));

                if (debug)
                cout << "预测目标点在激光坐标系中的位置 P_w: " << s_target_laser << endl;

                cv::Mat T_cl_inv = (Mat_<double>(4, 4) << cos(rad_yaw), sin(rad_yaw)*sin(rad), -cos(rad)*sin(rad_yaw), 0.02403,
                                            0, cos(rad), sin(rad), 0.01097,
                                            sin(rad_yaw), -sin(rad)*cos(rad_yaw), cos(rad)*cos(rad_yaw), 0,
                                            0, 0, 0, 1);

                Mat s_c = T_cl_inv * s_l;

                // 初始化旋转向量和平移向量。因为我们是直接从相机坐标系开始，
                // 所以旋转和平移都初始化为0。
                cv::Mat rvec = cv::Mat::zeros(3, 1, cv::DataType<double>::type); // 旋转向量
                cv::Mat tvec = cv::Mat::zeros(3, 1, cv::DataType<double>::type); // 平移向量

                vector<Point3f> pointsToProject;
                pointsToProject.push_back(Point3f(s_c.at<double>(0), s_c.at<double>(1), s_c.at<double>(2))); // 世界坐标原点

                vector<Point2f> projectedPoints;
                projectPoints(pointsToProject, rvec, tvec, K, D, projectedPoints);

                // 7. 在图像上画出投影点
                Point2f p = projectedPoints[0];
                circle(frame, p, 1, Scalar(255, 0, 0), -1);           // 红色实心圆

                Mat p_c = (Mat_<double>(4, 1) << p_x(0, 0), p_y(0, 0), p_z(0, 0), 1);
                Mat p_l = T_lw_inv * p_c;
                Vec3d p_target_laser(p_l.at<double>(0), p_l.at<double>(1), p_l.at<double>(2));

                if (debug)
                cout << "估计当前目标点在激光坐标系中的位置 P_w: " << p_target_laser << endl;

                double cur_yaw_target = atan2(p_target_laser(0), p_target_laser(2))/M_PI*180.0f;
                double cur_pitch_target = atan2(p_target_laser(1), p_target_laser(2))/M_PI*180.0f;

                double s_yaw_target = atan2(s_target_laser(0), s_target_laser(2))/M_PI*180.0f;
                double s_pitch_target = atan2(s_target_laser(1), s_target_laser(2))/M_PI*180.0f;

                yaw_error = atan2(p_target_laser[0], p_target_laser[2])/M_PI*180.0f;
                pitch_error = atan2(p_target_laser[1], p_target_laser[2])/M_PI*180.0f;

                if (debug)
                cout << "cur yaw_error: " << yaw_error << "cur pitch_error: " << pitch_error <<  endl;

                Eigen::Vector2d r_vec(p_x(0, 0), p_z(0, 0));                                              // 目标装甲板位矢
                Eigen::Vector2d v_vec(p_z(1, 0), -p_x(1, 0));

                yaw_speed = -(r_vec.dot(v_vec)) / (r_vec.norm() * r_vec.norm())/M_PI*180.0f;
                pitch_speed = 0;

                if (debug)
                cout << "yaw_speed: " << yaw_speed << "pitch_speed: " << pitch_speed << endl;

                if (s_yaw_target > 3) {
                    yaw = mcu_data.cur_yaw - 3;
                }
                else if (s_yaw_target < -3) {
                    yaw = mcu_data.cur_yaw + 3;
                }
                else {
                    yaw = mcu_data.cur_yaw - s_yaw_target;
                }   
                pitch = mcu_data.cur_pitch - s_pitch_target;

                if (debug)
                cout << "yaw: " << yaw << "pitch: " << pitch << endl;
            }
            else {
                if (debug)
                cout << "kalman reset" << endl;

                filter_x.reset(target_world(0), tp);
                filter_y.reset(target_world(1), tp);
                filter_z.reset(target_world(2), tp);

                yaw_speed = 0.0f;
                pitch_speed = 0.0f;

                yaw_error = atan2(target_laser[0], target_laser[2])/M_PI*180.0f;
                pitch_error = atan2(target_laser[1], target_laser[2])/M_PI*180.0f;

                if (debug)
                cout << "cur yaw_error: " << yaw_error << "cur pitch_error: " << pitch_error <<  endl;

                yaw = mcu_data.cur_yaw - yaw_error;
                pitch = mcu_data.cur_pitch - pitch_error;

                if (debug)
                cout << "yaw: " << yaw << "pitch: " << pitch << endl;
            }

            last_track = true;

            // if (mode == 1) {
            //     if (sqrt(pow(target_laser(0), 2) + pow(target_laser(0), 2)) < same_point_threshold/10000.0f) {
            //         step++;
            //     }
            // }

            if (mode == 1) {
                // 检测按键
                char key = static_cast<char>(cv::waitKey(1));
                if (key == ' ' && last_key_pressed == false) {
                    step++;
                    last_key_pressed = true;
                }

                if (key != ' ') {
                    last_key_pressed = false;
                }
            }
            
            if (valid == 0) {
                if (best_inner_corners.size() > 0)
                {
                    valid = 1;
                }

            }
            else if (valid == 1) {
                if (abs(yaw_error) < yaw_arrive_threshold && abs(pitch_error) < pitch_arrive_threshold) {
                    valid = 2;
                }
            }

        }
        else {
            if (debug)
            cout << "no rect" << endl;
        }

        circle(frame, cv::Point(K.at<double>(0, 2), K.at<double>(1, 2)), 1, Scalar(0, 0, 255), -1);


        




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




        // vector<int> rect_valid_indexes;
        // vector<int> same_center_indexes;
        // vector<int> white_valid_indexes;
        // vector<int> area_valid_indexes;
        // int best_index = -1;

        // if (rect_valid_indexes.size() > 0) {
        //     for (int i=0; i!=rect_valid_indexes.size(); i++)
        //         drawContours(frame, contours, rect_valid_indexes[i], Scalar(0, 255, 0), 2);
        // }

        // if (same_center_indexes.size() > 0) {
        //     for (int i=0; i!=rect_valid_indexes.size(); i++)

        //         drawContours(frame, contours, same_center_indexes[i], Scalar(0, 255, 0), 2);
        // }

        // if (white_valid_indexes.size() > 0) {
        //     for (int i=0; i!=rect_valid_indexes.size(); i++)

        //         drawContours(frame, contours, white_valid_indexes[i], Scalar(0, 255, 0), 2);
        // }

        // if (area_valid_indexes.size() > 0) {
        //     for (int i=0; i!=rect_valid_indexes.size(); i++)

        //         drawContours(frame, contours, area_valid_indexes[i], Scalar(0, 255, 0), 2);
        // }

        if (best_index > 0) {
            drawContours(frame, contours, best_index, Scalar(255, 0, 0), 2);
        }

        cv::imshow("Original", frame);
        cv::imshow("mask", mask);
        // cv::imshow("binary", binary);
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
                pitch,
                yaw_speed/180.0f*M_PI,
                pitch_speed/180.0f*M_PI
                );

            if (debug)
            LOGM_S("[transmit] status:%d | x_error:%f | y_error:%f | yaw_s:%f | pitch_s:%f",
                valid,
                yaw,
                pitch,
                yaw_speed/180.0f*M_PI,
                pitch_speed/180.0f*M_PI
                );

        }

        

        last_tp = tp;
    }

    // 释放资源
    cap.release();
    cv::destroyAllWindows();

    return 0;



}
