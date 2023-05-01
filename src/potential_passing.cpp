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
#include <chrono>
#include <thread>

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
int restricted_passing_areas_own = 101; // this is a radius from robot's passer position
int gain_far_obs = 50;

uint16_t curr_gain = 0;
uint16_t prev_gain = 0;

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
int active_obs_idx[2] = {6, 3};
int clr_obs = 0;
#endif

// int16_t att_pose[3] = {299, 0, 180};
int16_t att_pose[3] = {0, 200, 180};
int16_t ass_pose[3] = {600, 200, -180};

int16_t ball_pose[2] = {300, 0};

int8_t att_vel[2] = {0, 0};
int8_t ass_vel[2] = {0, 0};

uint8_t has_ball = 0; // 1 is att, 2 is ass

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
bool CheckCollision(float pos_x, float pos_y);
void DrawBall();
void KickBall();
bool CheckLineCircleIntersection(float x1, float y1, float x2, float y2, float cx, float cy, float r);
bool isIntersecting(float target_x, float target_y, float att_pose_x, float att_pose_y,
                    float obs_x, float obs_y, float obs_width, float obs_height);
void setInterval(uint16_t interval);

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
    createTrackbar("gain far obs", "Passing", &gain_far_obs, 200);

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

        if (target_x == 300 && target_y == 450)
        {
            setInterval(1000);
            KickBall();
            circle(frame_display_passing, Point(cm2px_y(ball_pose[1]), cm2px_x(ball_pose[0])), 10, Scalar(0, 0, 0), -1);
            printf("\n\n===================================\n");
            printf("GOLLLL\n");
            printf("===================================\n");
            setInterval(10000);
            break;
        }

        circle(frame_display_passing, Point(cm2px_y(ball_pose[1]), cm2px_x(ball_pose[0])), 10, Scalar(0, 0, 0), -1);

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

                if (pos_x > 200 && pos_x < 400)
                    continue;

                if (att_pose[xrobot] < 300)
                    if (pos_x < 300)
                        continue;

                if (att_pose[xrobot] > 300)
                    if (pos_x > 300)
                        continue;

                // if (ass_pose[yrobot] < 450 / 2)
                //     if (pos_y > 450 / 2)
                //         continue;
                // if (ass_pose[yrobot] > 450 / 2)
                //     if (pos_y < 450 / 2)
                //         continue;

                // printf("assist pose: %d %d | %d %d\n", ass_pose[xrobot], ass_pose[yrobot], ass_pose[yrobot]<450 / 2, ass_pose[yrobot]> 450 / 2);

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

                // if (pos_y > pos_y_obs - 10 && pos_y < pos_y_obs + 10)
                // {
                //     printf("%f %f\n", pos_y, pos_y_obs);
                //     printf("HEHEHE\n");
                //     continue;
                // }

                circle(frame_display_passing, cv::Point(cm2px_y(pos_y_obs), cm2px_x(pos_x_obs)), 5, Scalar(0, 0, 0), -1);

                vec_repulsive.update(pos_x, pos_y, pos_x_obs, pos_y_obs, r, theta);
                v_x += r * cos(theta);
                v_y += r * sin(theta);
                mtx.unlock();

                // if (isIntersecting(cm2px_x(pos_x), cm2px_y(pos_y), cm2px_x(att_pose[xrobot]), cm2px_y(att_pose[yrobot]), cm2px_x(pos_x_obs), cm2px_y(pos_y_obs), 26, 26))
                //     continue;

                // if (isIntersecting(cm2px_x(pos_x), cm2px_y(pos_y), cm2px_x(att_pose[xrobot]), cm2px_y(att_pose[yrobot]), cm2px_x(pos_x1_obs), cm2px_y(pos_y1_obs), 26, 26))
                //     continue;

                if (CheckLineCircleIntersection(cm2px_x(pos_x), cm2px_y(pos_y), cm2px_x(att_pose[xrobot]), cm2px_y(att_pose[yrobot]), cm2px_x(pos_x_obs), cm2px_y(pos_y_obs), gain_far_obs))
                    continue;

                if (pos_x < pos_x_obs + (26 + restricted_passing_areas_obs + 20) && pos_x > pos_x_obs - (26 + restricted_passing_areas_obs + 20) && pos_y < pos_y_obs + (26 + restricted_passing_areas_obs + 20) && pos_y > pos_y_obs - (26 + restricted_passing_areas_obs + 20))
                    continue;

                if (pos_x < pos_x1_obs + (26 + restricted_passing_areas_obs + 20) && pos_x > pos_x1_obs - (26 + restricted_passing_areas_obs + 20) && pos_y < pos_y1_obs + (26 + restricted_passing_areas_obs + 20) && pos_y > pos_y1_obs - (26 + restricted_passing_areas_obs + 20))
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

void DrawBall()
{
    circle(frame_display_passing, Point(cm2px_y(ball_pose[1]), cm2px_x(ball_pose[0])), 10, Scalar(0, 0, 0), -1);
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
            att_vel[0] -= 3;
            break;
        case 's':
            att_vel[0] += 3;
            break;
        case 'a':
            att_vel[1] += 3;
            break;
        case 'd':
            att_vel[1] -= 3;
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

        case 'p':
            printf("\n\n===================================\n");
            printf("PASSING BALL\n");
            printf("===================================\n");
            KickBall();
            break;

        case 'b':
            // wait for 2 seconds
            target_x = 300;
            target_y = 450;
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

bool CheckCollision(float pos_x, float pos_y)
{
    // check if the attacker pose and the target passing is goung through obstacles
    // for (uint8_t i = 0; i < 2; i++)
    // {
    //     uint8_t idx = active_obs_idx[i];

    //     // extract the coordinates of the current obstacle
    //     uint16_t obs_x = virtual_obs_pose[2 * idx];
    //     uint16_t obs_y = virtual_obs_pose[2 * idx + 1];

    //     // calculate the distance and angle between attacker and target
    //     float dx = pos_x - att_pose[1];
    //     float dy = pos_y - att_pose[0];
    //     float dist = sqrt(dx * dx + dy * dy);
    //     float theta = atan2(dy, dx);

    //     float delta_theta = atan2(obs_y - att_pose[0], obs_x - att_pose[1]) - theta;

    //     if (delta_theta >= -M_PI / 2 && delta_theta <= M_PI / 2)
    //     {
    //         float intersection_y = att_pose[0] + (obs_x - att_pose[1]) * tan(theta);
    //         printf("Intesection %f | pose %f %f\n", intersection_y, pos_x, pos_y);
    //         if ((intersection_y >= obs_y - 100 && intersection_y <= obs_y + 100))
    //             return true;
    //     }
    // }

    // return false;

    // for (int i = 0; i < 2; i++)
    // {
    //     int idx = active_obs_idx[i];

    //     // extract the coordinates and radius of the current obstacle
    //     uint16_t obs_x = virtual_obs_pose[2 * idx];
    //     uint16_t obs_y = virtual_obs_pose[2 * idx + 1];
    //     uint8_t obs_r = 26;

    //     // calculate the distance between the attacker and obstacle centers
    //     float dx = att_pose[1] - obs_x;
    //     float dy = att_pose[0] - obs_y;
    //     float dist = sqrt(dx * dx + dy * dy);

    //     // check if there is collision between the attacker and obstacle
    //     if (dist <= obs_r)
    //     {
    //         return true;
    //     }

    //     // calculate the intersection points between the line and the circle
    //     float d = (target_y - att_pose[0]) * (att_pose[1] - obs_x) - (target_x - att_pose[1]) * (att_pose[0] - obs_y);
    //     float discr = obs_r * obs_r * (target_y - att_pose[0]) * (target_y - att_pose[0]) + d * d - (att_pose[1] - obs_x) * (att_pose[1] - obs_x) * (target_y - att_pose[0]) * (target_y - att_pose[0]);

    //     // if the discriminant is negative, there is no intersection between the line and the circle
    //     if (discr < 0)
    //     {
    //         continue;
    //     }

    //     // calculate the intersection points between the line and the circle
    //     float root = sqrt(discr);
    //     float sgn_dy = (target_y - att_pose[0]) < 0 ? -1 : 1;
    //     float x_int1 = (d * sgn_dy * (target_y - att_pose[0]) + sgn_dy * (att_pose[1] - obs_x) * root) / (dist * dist);
    //     float y_int1 = (-d * sgn_dy * (att_pose[1] - obs_x) + fabsf((target_y - att_pose[0])) * root) / (dist * dist);
    //     float x_int2 = (d * sgn_dy * (target_y - att_pose[0]) - sgn_dy * (att_pose[1] - obs_x) * root) / (dist * dist);
    //     float y_int2 = (-d * sgn_dy * (att_pose[1] - obs_x) - fabsf((target_y - att_pose[0])) * root) / (dist * dist);

    //     // check if any of the intersection points lie on the line segment between attacker and target
    //     float min_x = fminf(att_pose[1], target_x);
    //     float max_x = fmaxf(att_pose[1], target_x);
    //     float min_y = fminf(att_pose[0], target_y);
    //     float max_y = fmaxf(att_pose[0], target_y);
    //     if (x_int1 >= min_x && x_int1 <= max_x && y_int1 >= min_y && y_int1 <= max_y)
    //     {
    //         return true;
    //     }
    //     if (x_int2 >= min_x && x_int2 <= max_x && y_int2 >= min_y && y_int2 <= max_y)
    //     {
    //         return true;
    //     }

    //     // check if any of the intersection points lie on the line segment between attacker and obstacle
    //     min_x = fminf(att_pose[1], obs_x);
    //     max_x = fmaxf(att_pose[1], obs_x);
    //     min_y = fminf(att_pose[0], obs_y);
    //     max_y = fmaxf(att_pose[0], obs_y);
    //     if (x_int1 >= min_x && x_int1 <= max_x && y_int1 >= min_y && y_int1 <= max_y)
    //     {
    //         return true;
    //     }
    //     if (x_int2 >= min_x && x_int2 <= max_x && y_int2 >= min_y && y_int2 <= max_y)
    //     {
    //         return true;
    //     }

    //     // check if any of the intersection points lie on the line segment between obstacle and target
    //     min_x = fminf(obs_x, target_x);
    //     max_x = fmaxf(obs_x, target_x);
    //     min_y = fminf(obs_y, target_y);
    //     max_y = fmaxf(obs_y, target_y);

    //     if (x_int1 >= min_x && x_int1 <= max_x && y_int1 >= min_y && y_int1 <= max_y)
    //     {
    //         return true;
    //     }
    //     if (x_int2 >= min_x && x_int2 <= max_x && y_int2 >= min_y && y_int2 <= max_y)
    //     {
    //         return true;
    //     }
    // }
    // return false;

    for (uint8_t i = 0; i < 2; i++)
    {
        int idx = active_obs_idx[i];

        // extract the coordinates and radius of the current obstacle
        uint16_t obs_x = virtual_obs_pose[2 * idx];
        uint16_t obs_y = virtual_obs_pose[2 * idx + 1];
        uint8_t obs_r = 26;

        float dx = target_x - att_pose[xrobot];
        float dy = target_y - att_pose[yrobot];

        float fx = att_pose[xrobot] - obs_x;
        float fy = att_pose[yrobot] - obs_y;

        float a = dx * dx + dy * dy;
        float b = 2 * (fx * dx + fy * dy);
        float c = fx * fx + fy * fy - obs_r * obs_r;

        float discr = b * b - 4 * a * c;

        if (discr < 0)
            return false;

        float t1 = (-b + std::sqrt(discr)) / (2 * a);
        float t2 = (-b - std::sqrt(discr)) / (2 * a);

        // Check if either of the roots lie between 0 and 1 (i.e. the intersection point lies on the line segment)
        if (t1 >= 0 && t1 <= 1)
        {
            return true;
        }
        if (t2 >= 0 && t2 <= 1)
        {
            return true;
        }

        // If neither of the roots lie between 0 and 1, there is no intersection
    }
    return false;
}

void KickBall()
{
    ball_pose[0] = target_x;
    ball_pose[1] = target_y;

    int16_t buffer_ass_pose[2] = {ass_pose[0], ass_pose[1]};

    ass_pose[0] = att_pose[0];
    ass_pose[1] = att_pose[1];

    att_pose[0] = buffer_ass_pose[0];
    att_pose[1] = buffer_ass_pose[1];
}

bool CheckLineCircleIntersection(float x1, float y1, float x2, float y2, float cx, float cy, float r)
{
    // Calculate the distance between the center of the circle and the line
    float dist = std::abs((y2 - y1) * cx - (x2 - x1) * cy + x2 * y1 - y2 * x1) / std::sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));

    // Check if the distance is less than or equal to the radius of the circle
    if (dist <= r)
    {
        return true;
    }

    // If the distance is greater than the radius of the circle, there is no intersection
    return false;
}

bool isIntersecting(float target_x, float target_y, float att_pose_x, float att_pose_y,
                    float obs_x, float obs_y, float obs_width, float obs_height)
{
    // Calculate the minimum and maximum x and y coordinates of the obstacle.
    float obs_min_x = obs_x;
    float obs_max_x = obs_x + obs_width;
    float obs_min_y = obs_y;
    float obs_max_y = obs_y + obs_height;

    // Calculate the slope and y-intercept of the line formed by the target and attacker positions.
    float m = (att_pose_y - target_y) / (att_pose_x - target_x);
    float b = target_y - m * target_x;

    // Calculate the y-coordinate of the line at the minimum and maximum x values of the obstacle.
    float y_min = m * obs_min_x + b;
    float y_max = m * obs_max_x + b;

    // Check if the y-coordinate of the line at the minimum or maximum x value is within the range of the obstacle's y-coordinates.
    if ((y_min >= obs_min_y && y_min <= obs_max_y) || (y_max >= obs_min_y && y_max <= obs_max_y))
    {
        return true;
    }

    // Check if the line formed by the target and attacker positions intersects with the left or right edge of the obstacle.
    if ((target_x <= obs_min_x && att_pose_x >= obs_min_x) || (target_x >= obs_max_x && att_pose_x <= obs_max_x))
    {
        return true;
    }

    // The line does not intersect with the obstacle.
    return false;
}

void setInterval(uint16_t interval)
{
    // use chrono
    std::this_thread::sleep_for(std::chrono::milliseconds(interval));
}