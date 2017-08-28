#include <unistd.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>

#include "common/time_util.h"
#include "common/matd.h"
#include "common/homography.h"
#include "lcm/lcm.h"
#include "lcmtypes/tag_detection_list_t.h"
#include "lcmtypes/tag_detection_t.h"
#include "lcmtypes/diff_drive_t.h"
#include "lcmtypes/image_t.h"

#define TARGET_TAG_ID 415

// 642, 642 for the webcam
#define FOCAL_X 525.477389
#define FOCAL_Y 525.383880
#define FOCAL_CENT_X 331.037877
#define FOCAL_CENT_Y 247.117271
#define TAG_SCALE 0.0777875 // in meters

// We want to be facing the april tag and be 2 meters away from it
#define DESIRED_X 0.0
#define DESIRED_Z -0.5

// in meters
#define DEAD_BAND 0.05

typedef struct _tag_follower_t {
    lcm_t *lcm;

    int image_width;
    int image_height;

    int64_t lastDectectionUtime;
    double lastLeftMotor;
    double lastRightMotor;
} tag_follower_t;

double lerp(double a, double b, double p) {
    if (p > 1.0) {
        p = 1.0;
    } else if (p < 0.0) {
        p = 0.0;
    }
    return a * (1.0 - p) + b * p;
}

void receive_detections(const lcm_recv_buf_t *rbuf, const char *channel,
                        const tag_detection_list_t *msg, void *user)
{
    tag_follower_t *tagf = user;

    if (tagf->image_width == 0 || tagf->image_height == 0) {
        return;
    }

    if (msg->utime <= tagf->lastDectectionUtime) {
        return;
    }
    double timePassedSec = (msg->utime - tagf->lastDectectionUtime) / 1e6;
    tagf->lastDectectionUtime = msg->utime;

    tag_detection_t targetDetection;
    for (int i = 0; i < msg->ndetections; i++) {
        if (msg->detections[i].id == TARGET_TAG_ID) {
            targetDetection = msg->detections[i];
        }
    }

    // default to a slow turn to try to find the target
    double leftMotor = 0.1;
    double rightMotor = -0.1;

    if (targetDetection.id == TARGET_TAG_ID) {
        leftMotor = 0.0f;
        rightMotor = 0.0;

        matd_t *hmat = matd_create_dataf(3, 3, (const float*)targetDetection.H);
        matd_t *pose = homography_to_pose(hmat, -FOCAL_X, FOCAL_Y, tagf->image_width / 2, tagf->image_height / 2);//FOCAL_CENT_X, FOCAL_CENT_Y);//

        // in centimeters
        double tag_x = matd_get(pose, 0, 3) * TAG_SCALE / 2 * 100;
        double tag_y = matd_get(pose, 1, 3) * TAG_SCALE / 2 * 100;
        double tag_z = matd_get(pose, 2, 3) * TAG_SCALE / 2 * 100;

        matd_destroy(hmat);
        matd_destroy(pose);

        double deadBandCm = DEAD_BAND * 100;
        double xOff = tag_x - (DESIRED_X * 100);
        if (xOff > deadBandCm) {
            leftMotor += 0.3;
            rightMotor -= 0.3;
        } else if (xOff < -deadBandCm) {
            leftMotor -= 0.3;
            rightMotor += 0.3;
        }

        double zOff = tag_z - (DESIRED_Z * 100);
        if (zOff > deadBandCm) {
            leftMotor -= 0.2;
            rightMotor -= 0.2f;
        } else if (zOff < -deadBandCm) {
            leftMotor += 0.2;
            rightMotor += 0.2f;
        }

        printf("C[0]: %.2f C[1]: %.2f\tX: %.4fcm Y: %.4fcm Z: %.4fcm\n", targetDetection.cxy[0], targetDetection.cxy[1], tag_x, tag_y, tag_z);
    }

    leftMotor = lerp(tagf->lastLeftMotor, leftMotor, timePassedSec);
    rightMotor = lerp(tagf->lastRightMotor, rightMotor, timePassedSec);
    tagf->lastLeftMotor = leftMotor;
    tagf->lastRightMotor = rightMotor;

    printf("\t->dt: %.2f l: %.2f r: %.2f\n", timePassedSec, leftMotor, rightMotor);

    diff_drive_t driveCommand = { .utime = utime_now(), .left = leftMotor, .left_enabled = true, .right = rightMotor, .right_enabled = true };
    diff_drive_t_publish(tagf->lcm, "DIFF_DRIVE", &driveCommand);
}

void receive_image(const lcm_recv_buf_t *rbuf, const char *channel,
                   const image_t *msg, void *user)
{
    tag_follower_t *tagf = user;
    tagf->image_width = msg->width;
    tagf->image_height = msg->height;
}

int main(int argc, char **argv)
{
    tag_follower_t *tagf = calloc(1, sizeof(tag_follower_t));

    tagf->lcm = lcm_create(NULL);
    if (!tagf->lcm) {
        fprintf(stderr, "LCM Failed to initialize. Aborting.\n");
        return 1;
    }

    tag_detection_list_t_subscribe(tagf->lcm, "TAG_DETECTIONS", receive_detections, tagf);
    image_t_subscribe(tagf->lcm, "IMAGE", receive_image, tagf);

    while (1) {
        lcm_handle(tagf->lcm);
    }

    lcm_destroy(tagf->lcm);
    free(tagf);

    return 0;
}
