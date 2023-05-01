/**
 * @file vector_attractive.h
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

class VectorAttractive
{
private:
    float _max_field_radius, _max_field_gradient;

    // Function 1 => y = ax + b
    // Function 2 => y = cx^2
    float _a, _b, _c;

public:
    VectorAttractive();
    ~VectorAttractive();

    void init(float max_field_radius, float max_field_gradient);
    void update(float x1, float y1, float x2, float y2, float& r, float& theta);
};

//------------------------------------------------------------------------------
//==============================================================================

VectorAttractive::VectorAttractive()
{
}

VectorAttractive::~VectorAttractive()
{
}

//------------------------------------------------------------------------------
//==============================================================================

void VectorAttractive::init(float max_field_radius, float max_field_gradient)
{
    _max_field_radius = max_field_radius;
    _max_field_gradient = max_field_gradient;

    _c = _max_field_gradient / (2 * _max_field_radius);
    _a = _max_field_gradient;
    _b = _c * _max_field_radius * _max_field_radius - _max_field_radius;
}

void VectorAttractive::update(float x1, float y1, float x2, float y2, float& r, float& theta)
{
    float distanceBetweenPoints = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    float angleBetweenPoints = atan2(y2 - y1, x2 - x1);

    if (distanceBetweenPoints <= _max_field_radius)
        r = 2 * _c * distanceBetweenPoints;
    else
        r = _a;

    theta = angleBetweenPoints;
}
