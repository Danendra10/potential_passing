#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "stdint.h"

typedef struct RobotTag
{
    float pos[3];
    float vel[3];
    float target[3];
    uint8_t role; // 0: att, 1: ass
} Robot_t;

void RobotInit(Robot_t *robot, uint8_t role, float pos_x, float pos_y, float pos_th)
{
    robot->pos[0] = pos_x;
    robot->pos[1] = pos_y;
    robot->pos[2] = pos_th;
    robot->role = role;
}

void SetTarget(Robot_t *robot, float target_x, float target_y, float target_th)
{
    robot->target[0] = target_x;
    robot->target[1] = target_y;
    robot->target[2] = target_th;
}

void SetVelocity(Robot_t *robot, float vel_x, float vel_y, float vel_th)
{
    robot->vel[0] = vel_x;
    robot->vel[1] = vel_y;
    robot->vel[2] = vel_th;
}

void ResetVelocity(Robot_t *robot)
{
    robot->vel[0] = 0;
    robot->vel[1] = 0;
    robot->vel[2] = 0;
}

void RobotUpdate(Robot_t *robot)
{
    robot->pos[0] += robot->vel[0];
    robot->pos[1] += robot->vel[1];
    robot->pos[2] += robot->vel[2];
}

#endif