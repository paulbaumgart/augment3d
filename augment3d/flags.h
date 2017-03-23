#pragma once

#include "gflags/gflags.h"
#include "base_transformer.h"

static bool ValidateThreeTupleString(const char* flagname, const std::string& value) {
    return boost::regex_match(value.begin(), value.end(), BaseTransformer::three_tuple_match);
}

static bool ValidateIntegerGreaterThanOne(const char* flagname, gflags::int32 value) {
    return value > 1;
}

DEFINE_bool(reflect, false,
            "augment input point cloud data by reflecting it over a plane through the origin");
DEFINE_string(reflect_normal, "0,1,0", "vector normal to the reflection plane");
DEFINE_validator(reflect_normal, &ValidateThreeTupleString);

DEFINE_bool(rotate, false, "augment input point cloud data by rotating it along an axis");
DEFINE_string(rotate_axis, "0,1,0", "rotation axis");
DEFINE_validator(rotate_axis, &ValidateThreeTupleString);
DEFINE_double(rotate_from, -90.0, "rotation start angle, in degrees");
DEFINE_double(rotate_to, 90.0, "rotation end angle, in degrees");
DEFINE_int32(rotate_steps, 5,
             "number of rotation steps, linearly interpolated between rotate_from and rotate_to"
             "(must be 2 or more)");
DEFINE_validator(rotate_steps, &ValidateIntegerGreaterThanOne);

DEFINE_bool(scale, false, "augment input point cloud data by scaling its dimensions");
DEFINE_string(scale_from, "1,1,1", "x,y,z values from which to start scaling");
DEFINE_validator(scale_from, &ValidateThreeTupleString);
DEFINE_string(scale_to, "2,2,2", "x,y,z values at which to stop scaling");
DEFINE_validator(scale_to, &ValidateThreeTupleString);
DEFINE_int32(scale_steps, 2,
             "number of scaling steps, linearly interpolated between scale_from and scale_to"
             "(must be 2 or more)");
DEFINE_validator(scale_steps, &ValidateIntegerGreaterThanOne);
