/**
 * TODO:
 * 1. vector closer to own
 *
 */
#include "potential/vector_attractive.h"
#include "potential/vector_repulsive.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <mutex>
#include <termios.h>
#include <sys/ioctl.h>
#include <cstdlib>
#include <fstream>
#include <string.h>

using namespace std;
using namespace cv;

#define REGIONAL_CFG

#define DEG2RAD 0.017452925
#define RAD2DEG 57.295780
#define DIV180 0.005555556

#define FIELD_X1_CFG 800
#define FIELD_Y1_CFG 1200
#define REGIONAL_FIELD_X1_CFG 600
#define REGIONAL_FIELD_Y1_CFG 450
#define FIELD_X_CFG 0
#define FIELD_Y_CFG 0

#define FIELD_X_THRESH 100
#define FIELD_Y_THRESH 240

#define xrobot 0
#define yrobot 1

void DrawField(Mat frame);

uint8_t total_point_x = 31; // Point of passing in x field
uint8_t total_point_y = 31; // Point of passing in y field
uint8_t robot_radius = 20;
int restricted_passing_areas = 0; // this is a radius from robot's passer position
int restricted_passing_areas_obs = 0;
int restricted_passing_areas_own = 0; // this is a radius from robot's passer position

//-----Vector
//===========
VectorAttractive vec_attractive;
VectorRepulsive vec_repulsive;

uint16_t attr_rad = 200;
uint16_t repl_rad = 600;

int robot_raw_vel = 100;
float pos_x_obs;
float pos_y_obs;
float pos_x1_obs;
float pos_y1_obs;

#ifdef REGIONAL_CFG
uint16_t virtual_obs_pose[17] = {150, 350, 150, 250, 150, 150, 300, 350, 300, 250, 450, 350, 450, 250, 450, 150};
int active_obs_idx[2] = {0, 4};
int clr_obs = 0;
#endif

int16_t att_pose[3] = {0, 200, 180};
int16_t ass_pose[3] = {600, 200, -180};

int8_t att_vel[2] = {0, 0};
int8_t ass_vel[2] = {0, 0};

//---Camera Resolution
//======================
uint16_t g_res_x = 360;
uint16_t g_res_y = 640;

Mat grapher_frame_2d = Mat::zeros(Size(700, 550), CV_8UC3);
Mat frame_display_vertical = Mat::zeros(700, 550, CV_8UC3);
Mat frame_display_passing = Mat::zeros(700, 550, CV_8UC3);

Mat display_line_out = Mat::zeros(Size(g_res_x, g_res_y), CV_8UC3);
Mat display_grapher_out = Mat::zeros(Size(g_res_x, g_res_y), CV_8UC3);
Mat display_passing_out = Mat::zeros(Size(g_res_x, g_res_y), CV_8UC3);

float cm2px_x(float x)
{
    return (50 + x);
}

float cm2px_y(float y)
{
#ifdef NATIONAL_CFG
    return (1250 - y);
#endif

#ifdef REGIONAL_CFG
    return (500 - y);
#endif
}

float px2cm_x(float x)
{
    return (x - 50);
}
float px2cm_y(float y)
{
#ifdef NATIONAL_CFG
    return (1250 - y);
#endif

#ifdef REGIONAL_CFG
    return (500 - y);
#endif
}

mutex mtx;

float target_x = 0;
float target_y = 0;

void DrawRobots();
int8_t kbhit();
void KeyboardHandler();
void RobotMovement();

int main(int argc, char **argv)
{
    namedWindow("Passing", WINDOW_NORMAL);
    createTrackbar("obs1", "Passing", &active_obs_idx[0], 8);
    createTrackbar("obs2", "Passing", &active_obs_idx[1], 8);
    createTrackbar("Clear Obstacles", "Passing", &clr_obs, 1);
    createTrackbar("vel", "Passing", &robot_raw_vel, 200);
    createTrackbar("rstrct", "Passing", &restricted_passing_areas, 200);
    createTrackbar("rstrct obs", "Passing", &restricted_passing_areas_obs, 200);
    createTrackbar("rstrct own", "Passing", &restricted_passing_areas_own, 200);

    //---> Robot Vector Init
    vec_attractive.init(attr_rad, robot_raw_vel);
    vec_repulsive.init(repl_rad, robot_raw_vel);
    for (;;)
    {
        RobotMovement();
        KeyboardHandler();
        DrawField(frame_display_passing);
        float prev_vector_trans = 10000;

        Mat buffer = Mat::zeros(Size(700, 550), CV_8UC3);
        for (int i = 0; i < 2; i++)
        {
            mtx.lock();
            circle(frame_display_passing, Point(cm2px_y(virtual_obs_pose[active_obs_idx[i] * 2 + 1]), cm2px_x(virtual_obs_pose[active_obs_idx[i] * 2])), 40, Scalar(50, 50, 200), -1);
            mtx.unlock();
        }

        DrawRobots();
        circle(frame_display_passing, Point(cm2px_y(target_y), cm2px_x(target_x)), 5, Scalar(0, 0, 0), -1);
        line(frame_display_passing, Point(cm2px_y(target_y), cm2px_x(target_x)), Point(cm2px_y(att_pose[yrobot]), cm2px_x(att_pose[xrobot])), Scalar(0, 0, 0), 2);
        if (target_x != 0 || target_y != 0)
        {
            frame_display_passing.copyTo(buffer);
            imshow("Passing", buffer);
            if (waitKey(1) == 27)
                break;
            // continue;
        }

        //----------------------------------------------------------//
        // calculate the potential field

#ifdef REGIONAL_CFG
        for (uint8_t x = 0; x <= total_point_x; x++)
        {
            for (uint8_t y = 0; y <= total_point_y; y++)
            {
                uint8_t x_thresh_range = REGIONAL_FIELD_X1_CFG / total_point_x;
                uint8_t y_thresh_range = REGIONAL_FIELD_Y1_CFG / total_point_y;
                float pos_x = x * x_thresh_range;
                float pos_y = y * y_thresh_range;

                // if (pos_x > 200 && pos_x < 400)
                //     continue;

                // if (att_pose[xrobot] < 300)
                //     if (pos_x < 300)
                //         continue;

                // if (ass_pose[yrobot] < 450 / 2)
                //     if (pos_y > 450 / 2)
                //         continue;
                // if (ass_pose[yrobot] > 450 / 2)
                //     if (pos_y < 450 / 2)
                //         continue;

                printf("assist pose: %d %d | %d %d\n", ass_pose[xrobot], ass_pose[yrobot], ass_pose[yrobot]<450 / 2, ass_pose[yrobot]> 450 / 2);

                if (pos_x < ass_pose[0] + (robot_radius + restricted_passing_areas + 100) && pos_x > ass_pose[0] - (robot_radius + restricted_passing_areas + 100) && pos_y < ass_pose[1] + (robot_radius + restricted_passing_areas + 100) && pos_y > ass_pose[1] - (robot_radius + restricted_passing_areas + 100))
                    continue;

                float v_x = pos_x;
                float v_y = pos_y;
                float r, theta;

                vec_attractive.update(pos_x, pos_y, ass_pose[0], ass_pose[1], r, theta);
                v_x += r * cos(theta);
                v_y += r * sin(theta);

                if (restricted_passing_areas_own > 100)
                {
                    vec_attractive.update(pos_x, pos_y, att_pose[0], att_pose[1], r, theta);
                    v_x += r * cos(theta);
                    v_y += r * sin(theta);
                }

                mtx.lock();
                pos_x_obs = virtual_obs_pose[active_obs_idx[0] * 2];
                pos_y_obs = virtual_obs_pose[active_obs_idx[0] * 2 + 1];

                pos_x1_obs = virtual_obs_pose[active_obs_idx[1] * 2];
                pos_y1_obs = virtual_obs_pose[active_obs_idx[1] * 2 + 1];

                circle(frame_display_passing, cv::Point(cm2px_y(pos_y_obs), cm2px_x(pos_x_obs)), 5, Scalar(0, 0, 0), -1);

                vec_repulsive.update(pos_x, pos_y, pos_x_obs, pos_y_obs, r, theta);
                v_x += r * cos(theta);
                v_y += r * sin(theta);
                mtx.unlock();

                if (pos_x < pos_x_obs + (26 + restricted_passing_areas_obs + 50) && pos_x > pos_x_obs - (26 + restricted_passing_areas_obs + 50) && pos_y < pos_y_obs + (26 + restricted_passing_areas_obs + 50) && pos_y > pos_y_obs - (26 + restricted_passing_areas_obs + 50))
                    continue;

                if (pos_x < pos_x1_obs + (26 + restricted_passing_areas_obs + 50) && pos_x > pos_x1_obs - (26 + restricted_passing_areas_obs + 50) && pos_y < pos_y1_obs + (26 + restricted_passing_areas_obs + 50) && pos_y > pos_y1_obs - (26 + restricted_passing_areas_obs + 50))
                    continue;

                float current_th_2_target = 0;
                current_th_2_target = atan2f(ass_pose[1] - pos_y, ass_pose[0] - pos_x) * RAD2DEG;
                float theta_output = atan2f(v_y - pos_y, v_x - pos_x) * RAD2DEG;

                // if (theta_output < current_th_2_target - 2 || theta_output > current_th_2_target + 2)
                //     continue;

                float vector_translation = sqrtf((pos_x - v_x) * (pos_x - v_x) + (pos_y - v_y) * (pos_y - v_y));

                line(frame_display_passing, cv::Point(cm2px_y(pos_y), cm2px_x(pos_x)), cv::Point(cm2px_y(v_y), cm2px_x(v_x)), cv::Scalar(255, 0, 0), 2);

                if (vector_translation < prev_vector_trans)
                {
                    prev_vector_trans = vector_translation;
                    target_x = pos_x;
                    target_y = pos_y;
                }

                if (vector_translation > prev_vector_trans)
                    continue;

                line(frame_display_passing, cv::Point(cm2px_y(pos_y), cm2px_x(pos_x)), cv::Point(cm2px_y(v_y), cm2px_x(v_x)), cv::Scalar(0, 0, 255), 2);
                putText(frame_display_passing, to_string(vector_translation), cv::Point(cm2px_y(pos_y), cm2px_x(pos_x)), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
            }
        }
#endif
        frame_display_passing.copyTo(buffer);
        imshow("Passing", buffer);
        if (waitKey(1) == 27)
            break;
    }
}

void DrawRobots()
{
    //---> Draw own robot
    circle(frame_display_passing, Point(cm2px_y(att_pose[1]), cm2px_x(att_pose[0])), 20, Scalar(12, 123, 234), 3);

    circle(frame_display_passing, Point(cm2px_y(ass_pose[1]), cm2px_x(ass_pose[0])), 20, Scalar(12, 0, 234), 3);
}

void DrawField(Mat frame)
{
#ifdef NATIONAL_CFG
    rectangle(frame, Rect(Point(0, 0), Point(900, 1300)), Scalar(0, 255, 0), -1);
    rectangle(frame, Rect(Point(50, 50), Point(850, 1250)), Scalar(0, 255, 0), 3);

    rectangle(frame, Rect(Point(50, 50), Point(850, 1250)), Scalar(255, 255, 255), 3);

    ellipse(frame, Point(50, 50), cv::Size(50, 50), 180, 270, 180, Scalar(255, 255, 255), 3, 0);
    ellipse(frame, Point(850, 50), cv::Size(50, 50), 180, 270, 360, Scalar(255, 255, 255), 3, 0);
    rectangle(frame, Rect(Point(200, 50), Point(700, 230)), Scalar(255, 255, 255), 3);
    rectangle(frame, Rect(Point(250, 50), Point(650, 100)), Scalar(255, 255, 255), 3);

    line(frame, Point(50, 650), Point(850, 650), Scalar(255, 255, 255), 3);
    circle(frame, Point(450, 650), 130, Scalar(255, 255, 255), 3);

    rectangle(frame, Rect(Point(200, 1250), Point(700, 1250 - 180)), Scalar(255, 255, 255), 3);
    rectangle(frame, Rect(Point(250, 1250), Point(650, 1200)), Scalar(255, 255, 255), 3);
    ellipse(frame, Point(50, 1250), cv::Size(50, 50), 180, 180, 90, Scalar(255, 255, 255), 3, 0);
    ellipse(frame, Point(850, 1250), cv::Size(50, 50), 180, 90, 0, Scalar(255, 255, 255), 3, 0);
#endif

#ifdef REGIONAL_CFG
    // 1300 iki y, regional 550
    rectangle(frame, Rect(Point(0, 0), Point(550, 700)), Scalar(0, 255, 0), -1);
    // rectangle(frame, Rect(Point(50, 50), Point(500, 650)), Scalar(0, 255, 0), 3);

    rectangle(frame, Rect(Point(50, 50), Point(500, 650)), Scalar(255, 255, 255), 3);
    // Goal pos outer
    rectangle(frame, Rect(Point(50, 200), Point(100, 500)), Scalar(37, 168, 249), 3);
    // Goal pos
    rectangle(frame, Rect(Point(50, 250), Point(25, 450)), Scalar(255, 255, 255), 3);
    // Ball Point
    circle(frame, Point(500, 350), 20, Scalar(51, 51, 255), -1);

    // Robot init pose
    rectangle(frame, Rect(Point(cm2px_y(170), cm2px_x(0)), Point(cm2px_y(230), cm2px_x(50))), Scalar(255, 255, 255), 3);
    rectangle(frame, Rect(Point(cm2px_y(170), cm2px_x(600)), Point(cm2px_y(230), cm2px_x(550))), Scalar(255, 255, 255), 3);

    line(frame, Point(100, 250), Point(500, 250), Scalar(37, 168, 249), 3);
    line(frame, Point(100, 450), Point(500, 450), Scalar(37, 168, 249), 3);
#endif
}

int8_t kbhit()
{
    static const int STDIN = 0;
    static bool initialized = false;

    if (!initialized)
    {
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

void KeyboardHandler()
{
    if (kbhit() > 0)
    {
        char key = std::cin.get();

        switch (key)
        {
        case 'w':
            att_vel[0] += 3;
            break;
        case 's':
            att_vel[0] -= 3;
            break;
        case 'a':
            att_vel[1] -= 3;
            break;
        case 'd':
            att_vel[1] += 3;
            break;

        case 'i':
            ass_vel[0] -= 3;
            break;
        case 'k':
            ass_vel[0] += 3;
            break;
        case 'j':
            ass_vel[1] += 3;
            break;
        case 'l':
            ass_vel[1] -= 3;
            break;

        case ' ':
            att_vel[0] = 0;
            ass_vel[0] = 0;
            att_vel[1] = 0;
            ass_vel[1] = 0;
            break;
        case 'x':
            target_x = 0;
            target_y = 0;
            break;
        }
    }
}

void RobotMovement()
{
    att_pose[0] += att_vel[0];
    att_pose[1] += att_vel[1];
    ass_pose[0] += ass_vel[0];
    ass_pose[1] += ass_vel[1];
}