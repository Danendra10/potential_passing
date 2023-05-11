#ifndef _BALL_H_
#define _BALL_H_

typedef struct BallTag
{
    float pos[2];
} Ball_t;

void BallInit(Ball_t *ball, float pos_x, float pos_y)
{
    ball->pos[0] = pos_x;
    ball->pos[1] = pos_y;
}

void BallUpdate(Ball_t *ball, float vel_x, float vel_y, float angle)
{
    ball->pos[0] += vel_x * cos(angle);
    ball->pos[1] += vel_y * sin(angle);
}

#endif