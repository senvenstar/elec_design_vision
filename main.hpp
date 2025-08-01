//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>
//Std
#include <vector>
#include <fstream>
#include <stdio.h>
#include <string>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "signal.h"
#include <thread>
#include "pthread.h"
#include <dirent.h>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <chrono>
#include <iostream>
#include <cmath>

//Common
#include "common.hpp"

//submodules
#include "UartIMU/uartimu.hpp"
#include "KeyBoard/key_board.hpp"
#include "LcdScreen/LcdScreen.hpp"
#include "LcdScreen/LcdDraw.hpp"
#include "LcdScreen/LcdFont.hpp"
#include "LcdScreen/LcdPic.hpp"
#include "kalman.h"

//modules
#include "common.hpp"

#include <wiringPi.h>



#define GPU

using pipeline::autoaim_pipeline;
