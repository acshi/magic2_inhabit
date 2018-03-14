#ifndef VELO_TO_LASER_T_H
#define VELO_TO_LASER_T_H

#include "common/getopt.h"
#include "common/image_f32.h"

#include "lcmtypes/diff_drive_t.h"
#include "lcmtypes/laser_t.h"
#include "lcmtypes/pose_t.h"
#include "lcmtypes/raw_t.h"

typedef struct sweep sweep_t;
struct sweep
{
    pose_t *pose;
    laser_t *laser;
    image_f32_t *range_im;
};

typedef struct velo_to_laser v2p_t;
struct velo_to_laser;

sweep_t *sweep_create(const pose_t *pose,
                      const laser_t *laser,
                      image_f32_t *im);
void sweep_destroy(sweep_t *sweep);

v2p_t *v2p_create(float S2B[16]);
void v2p_destroy(v2p_t *state);

void v2p_add_pose(v2p_t *v2l, const pose_t *pose);
int v2p_get_pose(v2p_t *v2l, int64_t utime, pose_t *pose);
void v2p_add_velo(v2p_t *v2l, const raw_t *msg);
zarray_t* v2p_get_sweep(v2p_t *state);

// Return NULL if no new sweep, else gives ownership of sweep to caller
sweep_t *v2p_last_sweep(v2p_t *v2l);

#endif
