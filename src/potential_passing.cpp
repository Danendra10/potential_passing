#include "potential/vector_attractive.h"
#include "potential/vector_repulsive.h"
#include "pid/pid.h"
#include "robot/robot.h"
#include "ball/ball.h"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <mutex>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <string.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <thread>

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
std::mutex mtx;

//--->>Frame
cv::Mat frame_display_passing = cv::Mat::zeros(700, 550, CV_8UC3);

//--->>Timer
const int TARGET_FREQUENCY_HZ = 50;

//--->>Init Robot
Robot_t robot_1;
Robot_t robot_2;

//--->>Init Ball
Ball_t ball;

//--->>Target
float target_x = 0;
float target_y = 0;

//-----Vector
//===========
VectorAttractive vec_attractive;
VectorRepulsive vec_repulsive;
uint16_t attr_rad = 200;
uint16_t repl_rad = 600;

//--->>Vars
uint16_t virtual_obs_pose[17] = {150, 150, 150, 250, 150, 350, 300, 150, 300, 250, 450, 150, 450, 250, 450, 350};
int active_obs_idx[2] = {2, 3};
int robot_raw_vel = 10;
uint8_t total_point_x = 31; // Point of passing in x field
uint8_t total_point_y = 31; // Point of passing in y field
uint8_t robot_radius = 20;
int restricted_passing_areas = 0; // this is a radius from robot's passer position
int restricted_passing_areas_obs = 0;
int restricted_passing_areas_own = 50; // this is a radius from robot's passer position
int gain_far_obs = 50;
int state_machine = 0;
bool calculate_target = false;
bool free_ball = true;

//===>>Prototype Functions
void Init();
int8_t kbhit();
void Routine();
void DrawObs();
void DrawBall();
void DrawField();
void DrawRobot();
void ChangeRole();
void DrawLegend();
void DrawSliders();
void StateMachine();
void KeyboardHandler();
float cm2px_x(float x);
float cm2px_y(float y);
float px2cm_x(float x);
float px2cm_y(float y);
void CalculatePassingPoint();
void SetTimerHz(int targetFrequencyHz);
float RobotToBallAngle(float robot_angle);
void RobotToPoint(Robot_t *robot, float target_x, float target_y, float target_th);
bool CheckLineCircleIntersection(float x1, float y1, float x2, float y2, float cx, float cy, float r);

int main()
{
    std::cout << "Loop started!" << std::endl;
    Init();
    while (true)
    {
        cv::Mat buffer = cv::Mat::zeros(cv::Size(700, 550), CV_8UC3);
        //==> ROS LIKE TIMER
        auto startTime = std::chrono::high_resolution_clock::now();
        auto endTime = std::chrono::high_resolution_clock::now();
        auto elapsedTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
        int remainingTimeMs = (1000 / TARGET_FREQUENCY_HZ) - elapsedTimeMs;

        if (remainingTimeMs > 0)
        {
            SetTimerHz(remainingTimeMs);
        }

        // if (target_x == 0 || target_y == 0)
        calculate_target = true;
        // else
        //     calculate_target = false;

        Routine();
        if (robot_2.role)
            cv::line(frame_display_passing, cv::Point(cm2px_y(robot_2.pos[0]), cm2px_x(robot_2.pos[1])), cv::Point(cm2px_y(target_y), cm2px_x(target_x)), cv::Scalar(0, 0, 0), 2);
        else
            cv::line(frame_display_passing, cv::Point(cm2px_y(robot_1.pos[0]), cm2px_x(robot_1.pos[1])), cv::Point(cm2px_y(target_y), cm2px_x(target_x)), cv::Scalar(0, 0, 0), 2);
        frame_display_passing.copyTo(buffer);
        imshow("Passing", buffer);

        printf("Robot role: %d %d\n", robot_1.role, robot_2.role);

        if (cv::waitKey(1) == 27)
            break;
    }
}

void Init()
{
    RobotInit(&robot_1, 0, 200, 20, 90);
    RobotInit(&robot_2, 1, 200, 580, -90);
    BallInit(&ball, 450, 300);
    DrawField();
    DrawBall();
    DrawRobot();
    DrawSliders();
    DrawObs();
    DrawLegend();
    vec_attractive.init(attr_rad, robot_raw_vel);
    vec_repulsive.init(repl_rad, robot_raw_vel);
}

void Routine()
{
    KeyboardHandler();
    RobotUpdate(&robot_2);
    RobotUpdate(&robot_1);
    DrawField();
    DrawBall();
    DrawRobot();
    DrawObs();
    StateMachine();
    ChangeRole();
    if (calculate_target)
        CalculatePassingPoint();
}

void SetTimerHz(int targetFrequencyHz)
{
    int targetIntervalMs = 1000 / targetFrequencyHz; // Calculate the target interval in milliseconds
    std::chrono::milliseconds duration(targetIntervalMs);
    std::this_thread::sleep_for(duration);
}

void DrawField()
{
    cv::rectangle(frame_display_passing, cv::Rect(cv::Point(0, 0), cv::Point(550, 700)), cv::Scalar(0, 255, 0), -1);
    cv::rectangle(frame_display_passing, cv::Rect(cv::Point(cm2px_y(0), cm2px_x(0)), cv::Point(cm2px_y(450), cm2px_x(600))), cv::Scalar(255, 255, 255), 3);

    // Goal pos outer
    cv::rectangle(frame_display_passing, cv::Rect(cv::Point(cm2px_y(0), cm2px_x(150)), cv::Point(cm2px_y(50), cm2px_x(450))), cv::Scalar(37, 168, 249), 3);
    // Goal pos
    cv::rectangle(frame_display_passing, cv::Rect(cv::Point(cm2px_y(0), cm2px_x(200)), cv::Point(cm2px_y(-25), cm2px_x(400))), cv::Scalar(255, 255, 255), 3);

    // Robot init pose
    cv::rectangle(frame_display_passing, cv::Rect(cv::Point(cm2px_y(170), cm2px_x(0)), cv::Point(cm2px_y(230), cm2px_x(50))), cv::Scalar(255, 255, 255), 3);
    cv::rectangle(frame_display_passing, cv::Rect(cv::Point(cm2px_y(170), cm2px_x(600)), cv::Point(cm2px_y(230), cm2px_x(550))), cv::Scalar(255, 255, 255), 3);

    line(frame_display_passing, cv::Point(cm2px_y(50), cm2px_x(200)), cv::Point(cm2px_y(450), cm2px_x(200)), cv::Scalar(37, 168, 249), 3);
    line(frame_display_passing, cv::Point(cm2px_y(50), cm2px_x(400)), cv::Point(cm2px_y(450), cm2px_x(400)), cv::Scalar(37, 168, 249), 3);
}

void DrawBall()
{
    static uint8_t ball_size = 10;
    cv::circle(frame_display_passing, cv::Point(cm2px_y(ball.pos[0]), cm2px_x(ball.pos[1])), ball_size, cv::Scalar(0, 0, 255), -1);
    if (robot_2.role)
    {
        if (sqrt(pow(robot_2.pos[0] - ball.pos[0], 2) + pow(robot_2.pos[1] - ball.pos[1], 2)) < 50)
        {
            ball.pos[0] = robot_2.pos[0] + 20 * cos(robot_2.pos[2] * DEG2RAD);
            ball.pos[1] = robot_2.pos[1] + 20 * sin(robot_2.pos[2] * DEG2RAD);
            ball_size = 15;
        }
        else
        {
            ball_size = 10;
        }
    }
    else if (robot_1.role)
    {
        if (sqrt(pow(robot_1.pos[0] - ball.pos[0], 2) + pow(robot_1.pos[1] - ball.pos[1], 2)) < 50)
        {
            ball.pos[0] = robot_1.pos[0] + 20 * cos(robot_1.pos[2] * DEG2RAD);
            ball.pos[1] = robot_1.pos[1] + 20 * sin(robot_1.pos[2] * DEG2RAD);
            ball_size = 15;
        }
        else
        {
            ball_size = 10;
        }
    }
}

void DrawRobot()
{
    uint8_t color_range[2][3] = {{0, 0, 255}, {255, 0, 0}};
    if (robot_1.role)
    {
        color_range[0][0] = 255;
        color_range[0][1] = 0;
        color_range[0][2] = 0;

        color_range[1][0] = 0;
        color_range[1][1] = 0;
        color_range[1][2] = 255;
    }
    else if (robot_2.role)
    {
        color_range[0][0] = 0;
        color_range[0][1] = 0;
        color_range[0][2] = 255;

        color_range[1][0] = 255;
        color_range[1][1] = 0;
        color_range[1][2] = 0;
    }
    cv::circle(frame_display_passing, cv::Point(cm2px_y(robot_1.pos[0]), cm2px_x(robot_1.pos[1])), 20, cv::Scalar(color_range[0][0], color_range[0][1], color_range[0][2]), 3);
    cv::line(frame_display_passing, cv::Point(cm2px_y(robot_1.pos[0]), cm2px_x(robot_1.pos[1])), cv::Point(cm2px_y(robot_1.pos[0] + 40 * cos(robot_1.pos[2] * DEG2RAD)), cm2px_x(robot_1.pos[1] + 40 * sin(robot_1.pos[2] * DEG2RAD))), cv::Scalar(color_range[0][0], color_range[0][1], color_range[0][2]), 3);

    cv::circle(frame_display_passing, cv::Point(cm2px_y(robot_2.pos[0]), cm2px_x(robot_2.pos[1])), 20, cv::Scalar(color_range[1][0], color_range[1][1], color_range[1][2]), 3);
    cv::line(frame_display_passing, cv::Point(cm2px_y(robot_2.pos[0]), cm2px_x(robot_2.pos[1])), cv::Point(cm2px_y(robot_2.pos[0] + 40 * cos(robot_2.pos[2] * DEG2RAD)), cm2px_x(robot_2.pos[1] + 40 * sin(robot_2.pos[2] * DEG2RAD))), cv::Scalar(color_range[1][0], color_range[1][1], color_range[1][2]), 3);
}

void DrawSliders()
{
    cv::namedWindow("Passing", cv::WINDOW_NORMAL);
    cv::createTrackbar("obs1", "Passing", &active_obs_idx[0], 8);
    cv::createTrackbar("obs2", "Passing", &active_obs_idx[1], 8);
    cv::createTrackbar("vel", "Passing", &robot_raw_vel, 200);
    cv::createTrackbar("rstrct", "Passing", &restricted_passing_areas, 200);
    cv::createTrackbar("rstrct obs", "Passing", &restricted_passing_areas_obs, 200);
    cv::createTrackbar("rstrct own", "Passing", &restricted_passing_areas_own, 200);
    cv::createTrackbar("gain far obs", "Passing", &gain_far_obs, 200);
}

void DrawObs()
{
    for (int i = 0; i < 2; i++)
    {
        mtx.lock();
        cv::circle(frame_display_passing, cv::Point(cm2px_y(virtual_obs_pose[active_obs_idx[i] * 2 + 1]), cm2px_x(virtual_obs_pose[active_obs_idx[i] * 2])), 40, cv::Scalar(50, 50, 200), -1);
        mtx.unlock();
    }
}

void DrawLegend()
{
    cv::putText(frame_display_passing, "Red: assist", cv::Point(cm2px_y(500), cm2px_x(-25)), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    cv::putText(frame_display_passing, "Blue: attacker", cv::Point(cm2px_y(250), cm2px_x(-25)), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
}

float cm2px_x(float x)
{
    return (50 + x);
}
float cm2px_y(float y)
{
    return (500 - y);
}
float px2cm_x(float x)
{
    return (x - 50);
}
float px2cm_y(float y)
{
    return (500 - y);
}

void RobotToPoint(Robot_t *robot, float target_x, float target_y, float target_th)
{
    static PID_t position_pid;
    static PID_t angle_pid;

    PIDInit(&position_pid, 10.5, 0, 0.1);
    PIDInit(&angle_pid, 0.5, 0, 0);

    float target_pos[3] = {target_x, target_y, target_th};

    float position_error = sqrt(pow(target_pos[0] - robot->pos[0], 2) + pow(target_pos[1] - robot->pos[1], 2));
    float angle_error = target_pos[2] - robot->pos[2];

    float position_output = PIDCalculate(&position_pid, position_error, robot_raw_vel);
    float angle_output = PIDCalculate(&angle_pid, angle_error, robot_raw_vel);

    printf("Out x: %f, y: %f\n", position_output * cos(angle_output * DEG2RAD), position_output * sin(angle_output * DEG2RAD));

    robot->pos[0] += position_output * cos(angle_output * DEG2RAD);
    robot->pos[1] += position_output * sin(angle_output * DEG2RAD);
    robot->pos[2] += angle_output;
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
            SetVelocity(&robot_2, 0, -10, 0);
            break;
        case 's':
            SetVelocity(&robot_2, 0, 10, 0);
            break;
        case 'a':
            SetVelocity(&robot_2, 10, 0, 0);
            break;
        case 'd':
            SetVelocity(&robot_2, -10, 0, 0);
            break;
        case '1':
            SetVelocity(&robot_2, 0, 0, 10);
            break;
        case '2':
            SetVelocity(&robot_2, 0, 0, -10);
            break;
        case 'i':
            SetVelocity(&robot_1, 0, -10, 0);
            break;
        case 'k':
            SetVelocity(&robot_1, 0, 10, 0);
            break;
        case 'j':
            SetVelocity(&robot_1, 10, 0, 0);
            break;
        case 'l':
            SetVelocity(&robot_1, -10, 0, 0);
            break;
        case '0':
            SetVelocity(&robot_1, 0, 0, 10);
            break;
        case '9':
            SetVelocity(&robot_1, 0, 0, -10);
            break;
        case 'p':
            state_machine = 1;
            break;
        case 'o':
            printf("OASU\n");
            break;

        case ' ':
            SetVelocity(&robot_2, 0, 0, 0);
            SetVelocity(&robot_1, 0, 0, 0);
            break;
        }
    }
}

void StateMachine()
{
    switch (state_machine)
    {
    case 0:
        break;
    case 1:
        RobotToPoint(&robot_1, ball.pos[0], ball.pos[1], RobotToBallAngle(robot_1.pos[2]));
        break;
    }
}

float RobotToBallAngle(float robot_angle)
{
    float angle = atan2(ball.pos[1] - robot_1.pos[1], ball.pos[0] - robot_1.pos[0]) * RAD2DEG;
    return angle;
}

void CalculatePassingPoint()
{
    float prev_vector_trans = 10000;
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

            if (robot_1.role)
            {
                if (robot_1.pos[1] > 300)
                    if (pos_x > 300)
                        continue;
                if (robot_1.pos[0] < 225)
                {
                    if (pos_y < 225)
                        continue;
                }
                else
                {
                    if (pos_y > 225)
                        continue;
                }

                float v_x = pos_x;
                float v_y = pos_y;
                float r, theta;

                vec_attractive.update(pos_x, pos_y, robot_2.pos[1], robot_2.pos[0], r, theta);
                v_x += r * cos(theta);
                v_y += r * sin(theta);

                if (pos_x < virtual_obs_pose[active_obs_idx[0] * 2] + (26 + restricted_passing_areas_obs + 20) && pos_x > virtual_obs_pose[active_obs_idx[0] * 2] - (26 + restricted_passing_areas_obs + 20) && pos_y < virtual_obs_pose[active_obs_idx[0] * 2 + 1] + (26 + restricted_passing_areas_obs + 20) && pos_y > virtual_obs_pose[active_obs_idx[0] * 2 + 1] - (26 + restricted_passing_areas_obs + 20))
                    continue;
                if (pos_x < virtual_obs_pose[active_obs_idx[1] * 2] + (26 + restricted_passing_areas_obs + 20) && pos_x > virtual_obs_pose[active_obs_idx[1] * 2] - (26 + restricted_passing_areas_obs + 20) && pos_y < virtual_obs_pose[active_obs_idx[1] * 2 + 1] + (26 + restricted_passing_areas_obs + 20) && pos_y > virtual_obs_pose[active_obs_idx[1] * 2 + 1] - (26 + restricted_passing_areas_obs + 20))
                    continue;

                if (pos_y < robot_2.pos[0] + (robot_radius + restricted_passing_areas + 100) && pos_y > robot_2.pos[0] - (robot_radius + restricted_passing_areas + 100) && pos_x < robot_2.pos[1] + (robot_radius + restricted_passing_areas + 100) && pos_x > robot_2.pos[1] - (robot_radius + restricted_passing_areas + 100))
                    continue;

                if (restricted_passing_areas_own > 100)
                {
                    vec_attractive.update(pos_x, pos_y, robot_1.pos[0], robot_1.pos[1], r, theta);
                    v_x += r * cos(theta);
                    v_y += r * sin(theta);
                }

                for (uint8_t i = 0; i < 2; i++)
                {
                    vec_repulsive.update(pos_x, pos_y, virtual_obs_pose[active_obs_idx[i] * 2], virtual_obs_pose[active_obs_idx[i] * 2 + 1], r, theta);
                    v_x += r * cos(theta);
                    v_y += r * sin(theta);
                    if (CheckLineCircleIntersection(pos_x, pos_y, robot_1.pos[1], robot_1.pos[0], virtual_obs_pose[active_obs_idx[i] * 2], virtual_obs_pose[active_obs_idx[i] * 2 + 1], 40))
                        continue;
                }

                float vector_translation = sqrtf((pos_x - v_x) * (pos_x - v_x) + (pos_y - v_y) * (pos_y - v_y));

                if (vector_translation < prev_vector_trans)
                {
                    prev_vector_trans = vector_translation;
                    target_x = pos_x;
                    target_y = pos_y;
                }

                if (vector_translation > prev_vector_trans)
                    continue;

                cv::line(frame_display_passing, cv::Point(cm2px_y(pos_y), cm2px_x(pos_x)), cv::Point(cm2px_y(v_y), cm2px_x(v_x)), cv::Scalar(0, 0, 255), 1);
            }
            else if (robot_2.role)
            {
                if (robot_2.pos[1] > 300)
                    if (pos_x > 300)
                        continue;
                if (robot_2.pos[0] < 225)
                {
                    if (pos_y < 225)
                        continue;
                }
                else
                {
                    if (pos_y > 225)
                        continue;
                }

                float v_x = pos_x;
                float v_y = pos_y;
                float r, theta;

                vec_attractive.update(pos_x, pos_y, robot_1.pos[1], robot_1.pos[0], r, theta);
                v_x += r * cos(theta);
                v_y += r * sin(theta);

                if (pos_x < virtual_obs_pose[active_obs_idx[0] * 2] + (26 + restricted_passing_areas_obs + 20) && pos_x > virtual_obs_pose[active_obs_idx[0] * 2] - (26 + restricted_passing_areas_obs + 20) && pos_y < virtual_obs_pose[active_obs_idx[0] * 2 + 1] + (26 + restricted_passing_areas_obs + 20) && pos_y > virtual_obs_pose[active_obs_idx[0] * 2 + 1] - (26 + restricted_passing_areas_obs + 20))
                    continue;
                if (pos_x < virtual_obs_pose[active_obs_idx[1] * 2] + (26 + restricted_passing_areas_obs + 20) && pos_x > virtual_obs_pose[active_obs_idx[1] * 2] - (26 + restricted_passing_areas_obs + 20) && pos_y < virtual_obs_pose[active_obs_idx[1] * 2 + 1] + (26 + restricted_passing_areas_obs + 20) && pos_y > virtual_obs_pose[active_obs_idx[1] * 2 + 1] - (26 + restricted_passing_areas_obs + 20))
                    continue;

                if (pos_y < robot_1.pos[0] + (robot_radius + restricted_passing_areas + 100) && pos_y > robot_1.pos[0] - (robot_radius + restricted_passing_areas + 100) && pos_x < robot_1.pos[1] + (robot_radius + restricted_passing_areas + 100) && pos_x > robot_1.pos[1] - (robot_radius + restricted_passing_areas + 100))
                    continue;

                if (restricted_passing_areas_own > 100)
                {
                    vec_attractive.update(pos_x, pos_y, robot_2.pos[0], robot_2.pos[1], r, theta);
                    v_x += r * cos(theta);
                    v_y += r * sin(theta);
                }

                for (uint8_t i = 0; i < 2; i++)
                {
                    vec_repulsive.update(pos_x, pos_y, virtual_obs_pose[active_obs_idx[i] * 2], virtual_obs_pose[active_obs_idx[i] * 2 + 1], r, theta);
                    v_x += r * cos(theta);
                    v_y += r * sin(theta);
                    if (CheckLineCircleIntersection(pos_x, pos_y, robot_2.pos[1], robot_2.pos[0], virtual_obs_pose[active_obs_idx[i] * 2], virtual_obs_pose[active_obs_idx[i] * 2 + 1], 40))
                        continue;
                }

                float vector_translation = sqrtf((pos_x - v_x) * (pos_x - v_x) + (pos_y - v_y) * (pos_y - v_y));

                if (vector_translation < prev_vector_trans)
                {
                    prev_vector_trans = vector_translation;
                    target_x = pos_x;
                    target_y = pos_y;
                }

                if (vector_translation > prev_vector_trans)
                    continue;

                cv::line(frame_display_passing, cv::Point(cm2px_y(pos_y), cm2px_x(pos_x)), cv::Point(cm2px_y(v_y), cm2px_x(v_x)), cv::Scalar(0, 0, 255), 1);
            }
            cv::circle(frame_display_passing, cv::Point(cm2px_y(pos_y), cm2px_x(pos_x)), 2, cv::Scalar(0, 0, 255), -1);
        }
    }
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

void CalculateBestPosition(Robot_t *robot)
{
}

void ChangeRole()
{
    // if the distance of robot_1 to ball is closer than the distance of robot_2 to ball
    if (sqrtf((robot_1.pos[0] - ball.pos[0]) * (robot_1.pos[0] - ball.pos[0]) + (robot_1.pos[1] - ball.pos[1]) * (robot_1.pos[1] - ball.pos[1])) < sqrtf((robot_2.pos[0] - ball.pos[0]) * (robot_2.pos[0] - ball.pos[0]) + (robot_2.pos[1] - ball.pos[1]) * (robot_2.pos[1] - ball.pos[1])))
    {
        robot_1.role = 1;
        robot_2.role = 0;
    }
    else
    {
        robot_1.role = 0;
        robot_2.role = 1;
    }
}