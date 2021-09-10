// Five Bar Scara driven by the elbows
//
// Copyright (C) 2020 Pontus Borg <glpontus@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

struct fivebarelbow_stepper {
    struct stepper_kinematics sk;
    double inner_arm_length;
    double outer_arm_length;
    double inner_arm_offset;
};

static inline double sqr(double a) { return a*a; };

static inline double distance(double x0, double y0,
                              double x1, double y1)
{
    double dx = x1-x0;
    double dy = y1-y0;
    return sqrt(sqr(dx) + sqr(dy));
}



// Find the angle of the corner opposite to side c
static inline double triangle_angle(double a, double b, double c)
{
    // cosine rule: c^2 = a^2 + b^2 − 2ab cos(C)
    // C = arccos((a^2 + b^2 - c^2) / 2ab)
    double cosC = (sqr(a) + sqr(b) - sqr(c)) / (2 * a * b);
    return acos(cosC);
}


// Each arm is computed separately
static double
fivebarelbow_stepper_calc_position(
    struct stepper_kinematics *sk,
    struct move *m,
    double move_time)
{
    struct fivebarelbow_stepper *fs =
        container_of(sk, struct fivebarelbow_stepper, sk);
    struct coord c = move_get_coord(m, move_time);

    double d = distance(fs->inner_arm_offset, 0, c.x, c.y);
    double angle = triangle_angle(fs->inner_arm_length,
                                  fs->outer_arm_length,
                                  d);
    return angle;
}


struct stepper_kinematics * __visible
fivebarelbow_stepper_alloc(char arm,
                           double inner_arm_length,
                           double outer_arm_length,
                           double inner_arms_distance)
{

    struct fivebarelbow_stepper *fs = malloc(sizeof(*fs));
    memset(fs, 0, sizeof(*fs));

    fs->sk.calc_position_cb = fivebarelbow_stepper_calc_position;
    fs->inner_arm_length = inner_arm_length;
    fs->outer_arm_length = outer_arm_length;
    if (arm == 'l') {
        fs->inner_arm_offset = -inner_arms_distance / 2.0;
    } else if (arm == 'r') {
        fs->inner_arm_offset = inner_arms_distance / 2.0;
    }
    fs->sk.active_flags = AF_X | AF_Y;
    return &fs->sk;
}
