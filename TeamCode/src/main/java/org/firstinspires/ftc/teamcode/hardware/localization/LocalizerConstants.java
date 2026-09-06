package org.firstinspires.ftc.teamcode.hardware.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

/**
 * All tunable constants for the localizer live here.
 * Nothing outside this file should have a raw numeric constant related to localization.
 */
public final class LocalizerConstants {

    private LocalizerConstants() {} // no instances

    // ============ Unit conversions ============
    public static final double METERS_TO_INCHES = 39.3701;

    // ============ Pinpoint / Odometry ============
    public static final class Pinpoint {
        // Offsets of each pod from the robot's center of rotation, in mm.
        // X pod offset = forward pod's Y-distance from center.
        // Y pod offset = strafe pod's X-distance from center.
        // MEASURE THESE ON THE ACTUAL ROBOT.
        public static final double X_POD_OFFSET_MM = -84.0;
        public static final double Y_POD_OFFSET_MM = -168.0;

        public static final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

        public static final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public static final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }

    // ============ Vision / Limelight ============
    public static final class Vision {
        public static final int APRILTAG_PIPELINE_INDEX = 0;

        // Distance (inches) at/below which a tag reading is fully trusted.
        public static final double FULL_CONFIDENCE_DISTANCE_IN = 24.0;
        // Distance (inches) beyond which a tag reading is barely trusted.
        public static final double MIN_CONFIDENCE_DISTANCE_IN = 120.0;
        // Confidence never drops all the way to zero, to avoid hard cutoffs.
        public static final double CONFIDENCE_FLOOR = 0.02;
        // Per extra simultaneous tag, small bonus added to overall confidence.
        public static final double MULTI_TAG_CONFIDENCE_BONUS = 0.15;
    }

    // ============ Fusion ============
    public static final class Fusion {
        // Max blend weight given to vision even at perfect confidence (0-1).
        // Prevents one great reading from fully overriding odometry in a single frame.
        public static final double MAX_VISION_TRUST = 0.7;
    }
}