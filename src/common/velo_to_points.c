#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <lcm/lcm.h>

#include "lcmtypes/diff_drive_t.h"
#include "lcmtypes/laser_t.h"
#include "lcmtypes/pose_t.h"
#include "lcmtypes/raw_t.h"

#include "common/config.h"
#include "common/doubles.h"
#include "common/floats.h"
#include "common/getopt.h"
#include "common/interpolator.h"
#include "common/math_util.h"
#include "common/zqueue.h"

#include "velodyne/april_velodyne.h"

#include "velo_to_points.h"

const int VELO_QUEUE_SIZE = 100;

struct velo_to_laser
{
    interpolator_t *pose_interp;

    float S2B[16];
    april_velodyne_t *velo;

    zarray_t *curr_pts;
    zarray_t *new_pts;
};

v2p_t *v2p_create(float S2B[16])
{
    v2p_t *state = calloc(1, sizeof(v2p_t));

    state->pose_interp = interpolator_create(sizeof(pose_t), offsetof(pose_t, utime), 5.0, 100E3);
    interpolator_add_field(state->pose_interp,
                           INTERPOLATOR_DOUBLE_LINEAR, 3, offsetof(pose_t, pos));
    interpolator_add_field(state->pose_interp,
                           INTERPOLATOR_DOUBLE_QUAT, 4, offsetof(pose_t, orientation));


    state->velo = april_velodyne_create();

    memcpy(state->S2B, S2B, 16*sizeof(float));
    state->curr_pts = NULL;
    state->new_pts = zarray_create(sizeof(float[2]));

    return state;
}

void v2p_destroy(v2p_t *state)
{
    interpolator_destroy(state->pose_interp);
    april_velodyne_destroy(state->velo);
    free(state);
}

static void on_slice(const zarray_t *pts,
                     const zarray_t *ranges,
                     const zarray_t *intensities,
                     uint8_t mode,
                     void *user)
{
    v2p_t *state = user;

    float min_range = 1000.0;
    int min_idx = -1;
    for (int i = 0; i < zarray_size(ranges); i++) {
        float range;
        zarray_get(ranges, i, &range);

        // Filter out self-image.
        if (range < 0.2) {
            continue;
        }

        if (range >= min_range) {
            continue;
        }

        float xyz[3];
        zarray_get(pts, i, &xyz);

        // If it is floor ignore it.
        if (xyz[2] < 0.10 && xyz[2] > -0.05) {
            continue;
        }

        // If it is taller than the robot ignore it.
        if (xyz[2] > 1.5) {
            continue;
        }

        min_range = range;
        min_idx = i;
    }

    if (min_idx >= 0) {
        float xyz[3];
        zarray_get(pts, min_idx, &xyz);
        zarray_add(state->new_pts, &xyz); // Should we accept a method to filter out bad points?
    }
}

static void on_sweep(void *user)
{
    v2p_t *state = user;
    zarray_destroy(state->curr_pts);
    state->curr_pts = state->new_pts;
    state->new_pts = zarray_create(sizeof(float[2]));
}

zarray_t* v2p_get_sweep(v2p_t *state) {
    if (state->curr_pts) {
        zarray_t* ret = state->curr_pts;
        state->curr_pts = NULL;
        return ret;
    } else {
        return NULL;
    }
}


void v2p_add_pose(v2p_t *v2l, const pose_t *pose)
{
    interpolator_add(v2l->pose_interp, pose);
}

int v2p_get_pose(v2p_t *v2l, int64_t utime, pose_t *pose) {
    return interpolator_get(v2l->pose_interp, utime, pose);
}

void v2p_add_velo(v2p_t *state, const raw_t *msg) {
    // Determine when data was gathered
    pose_t pose;
    int ret = v2p_get_pose(state, msg->utime, &pose);
    if (ret == -2) {
        printf("Outside interpolation range\n");
        return;
    }

    double curr_xyt[3];
    doubles_quat_xyz_to_xyt(pose.orientation,
                            pose.pos,
                            curr_xyt);

    float M[16];
    float XYT[16];
    float fxyt[3] = { (float) curr_xyt[0], (float) curr_xyt[1], (float) curr_xyt[2] };
    floats_xyt_to_mat44(fxyt, XYT);
    floats_mat_AB(XYT, 4, 4,
                  state->S2B, 4, 4,
                  M, 4, 4);

    april_velodyne_on_packet(state->velo,
                             msg->buf,
                             msg->len,
                             M,
                             on_slice,
                             on_sweep,
                             state);
}
