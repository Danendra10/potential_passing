/**
 * @file vector_repulsive.h
 * @author Danendra (you@domain.com)
 * @Danendra10
 * @brief
 * @version 0.1
 * @date 2023-03-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "math.h"

class VectorRepulsive
{
private:
    float _max_field_radius, _gain_field_gradient, _c;

public:
    VectorRepulsive();
    ~VectorRepulsive();

    void init(float max_field_radius, float gain_field_gradient);
    void update(float x1, float y1, float x2, float y2, float& r, float& theta);
};

//------------------------------------------------------------------------------
//==============================================================================

VectorRepulsive::VectorRepulsive()
{
}

VectorRepulsive::~VectorRepulsive()
{
}

//------------------------------------------------------------------------------
//==============================================================================

void VectorRepulsive::init(float max_field_radius, float gain_field_gradient)
{
    _max_field_radius = max_field_radius;
    _gain_field_gradient = gain_field_gradient * 400000;
}

void VectorRepulsive::update(float x1, float y1, float x2, float y2, float& r, float& theta)
{
    float distance_between_points = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    float angle_between_points = atan2(y2 - y1, x2 - x1);
    if (distance_between_points < _max_field_radius && distance_between_points > 20)
        r = -_gain_field_gradient * (1 / pow(distance_between_points, 3) - ((1 / pow(distance_between_points, 2)) * (1 / _max_field_radius)));
    else
        r = 0;
    theta = angle_between_points;
}