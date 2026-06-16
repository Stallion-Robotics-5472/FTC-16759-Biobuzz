package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class FTCKalmanFilter {

    private final GoBildaPinpointDriver pinpoint;
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    private final PoseKalmanFilter kf;
    private Pose2D lastPinpointPose;

    // --- OPTIMIZATION CACHES ---
    // We store raw primitives (double) instead of Pose2D objects to avoid unit
    // conversion math inside the fast loop.
    private final Map<Integer, double[]> cachedTags = new HashMap<>();
    private final double camOffsetX;
    private final double camOffsetY;
    private final double camOffsetHeading;

    // --- TUNING PARAMETERS ---
    private final double qX = 0.005;
    private final double qY = 0.005;
    private final double qHeading = 0.001;

    private final double rxBase = 0.1;
    private final double ryBase = 0.1;
    private final double rHeadingBase = 0.05;

    public FTCKalmanFilter(HardwareMap hwMap, String pinpointName, double xOffset, double yOffset,
                           String webcamName, Pose2D startingPose, Map<Integer, Pose2D> knownTags, Pose2D cameraOffset) {

        this.camOffsetX = cameraOffset.getX(DistanceUnit.INCH);
        this.camOffsetY = cameraOffset.getY(DistanceUnit.INCH);
        this.camOffsetHeading = cameraOffset.getHeading(AngleUnit.RADIANS);

        for (Map.Entry<Integer, Pose2D> entry : knownTags.entrySet()) {
            Pose2D pose = entry.getValue();
            cachedTags.put(entry.getKey(), new double[]{
                    pose.getX(DistanceUnit.INCH),
                    pose.getY(DistanceUnit.INCH),
                    pose.getHeading(AngleUnit.RADIANS)
            });
        }

        this.pinpoint = hwMap.get(GoBildaPinpointDriver.class, pinpointName);
        this.pinpoint.resetPosAndIMU();
        this.pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.INCH);

        this.aprilTag = new AprilTagProcessor.Builder().build();

        this.visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, webcamName))
                .addProcessor(aprilTag)
                .build();

        this.kf = new PoseKalmanFilter(
                startingPose.getX(DistanceUnit.INCH),
                startingPose.getY(DistanceUnit.INCH),
                startingPose.getHeading(AngleUnit.RADIANS)
        );

        this.pinpoint.update();
        this.lastPinpointPose = pinpoint.getPosition();
    }

    public void update() {
        // --- 1. ODOMETRY PREDICTION ---
        pinpoint.update();
        Pose2D currentPinpointPose = pinpoint.getPosition();

        double deltaX = currentPinpointPose.getX(DistanceUnit.INCH) - lastPinpointPose.getX(DistanceUnit.INCH);
        double deltaY = currentPinpointPose.getY(DistanceUnit.INCH) - lastPinpointPose.getY(DistanceUnit.INCH);
        double deltaHeading = currentPinpointPose.getHeading(AngleUnit.RADIANS) - lastPinpointPose.getHeading(AngleUnit.RADIANS);

        kf.predict(deltaX, deltaY, deltaHeading, qX, qY, qHeading);
        lastPinpointPose = currentPinpointPose;

        // --- 2. VISION UPDATE ---
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (int i = 0; i < currentDetections.size(); i++) {
            AprilTagDetection detection = currentDetections.get(i);

            // FIX 1: Prevent NullPointerExceptions if the SDK ignores a tag
            if (detection.ftcPose == null) continue;

            double[] tagData = cachedTags.get(detection.id);

            if (tagData != null) {
                // FIX 2: Use camera-centric Cartesian vectors instead of spherical Range/Bearing.
                // In the FTC SDK: 'y' is straight out from the lens. 'x' is lateral (left/right).
                double cameraForward = detection.ftcPose.y;
                double cameraStrafe  = detection.ftcPose.x;
                double cameraYaw     = Math.toRadians(detection.ftcPose.yaw);

                // Calculate absolute robot heading on the field
                double visionHeading = AngleUnit.normalizeRadians(tagData[2] - cameraYaw - camOffsetHeading);

                // Cache trig for the rotations
                double cosVisHead = Math.cos(visionHeading);
                double sinVisHead = Math.sin(visionHeading);

                // FIX 3: Rotate the camera-to-tag vectors to match the field coordinate system
                // and subtract them from the tag's known field position
                double camFieldX = tagData[0] - (cameraForward * cosVisHead - cameraStrafe * sinVisHead);
                double camFieldY = tagData[1] - (cameraForward * sinVisHead + cameraStrafe * cosVisHead);

                // Shift from the camera lens back to the robot's physical center
                double visionX = camFieldX - (camOffsetX * cosVisHead - camOffsetY * sinVisHead);
                double visionY = camFieldY - (camOffsetX * sinVisHead + camOffsetY * cosVisHead);

                // Calculate dynamic noise based on distance (range is still fine to use just for scaling noise)
                double rangeSquared = (cameraForward * cameraForward) + (cameraStrafe * cameraStrafe);
                double scaleFactor = rangeSquared / 225.0;

                double dynamicRx = rxBase + (rxBase * scaleFactor);
                double dynamicRy = ryBase + (ryBase * scaleFactor);
                double dynamicRHeading = rHeadingBase + (rHeadingBase * scaleFactor);

                kf.update(visionX, visionY, visionHeading, dynamicRx, dynamicRy, dynamicRHeading);
            }
        }
    }

    public double getX() { return kf.x; }
    public double getY() { return kf.y; }
    public double getHeading() { return kf.heading; }

    public void close() {
        visionPortal.close();
    }

    // =========================================================================
    // INNER CLASS: The Kalman Filter Mathematics
    // =========================================================================
    private static class PoseKalmanFilter {
        public double x, y, heading;
        public double pX, pY, pHeading;

        public PoseKalmanFilter(double startX, double startY, double startHeading) {
            this.x = startX;
            this.y = startY;
            this.heading = startHeading;
            this.pX = 0.1;
            this.pY = 0.1;
            this.pHeading = 0.1;
        }

        public void predict(double deltaX, double deltaY, double deltaHeading, double qX, double qY, double qHeading) {
            x += deltaX;
            y += deltaY;
            heading = AngleUnit.normalizeRadians(heading + deltaHeading);

            pX += qX;
            pY += qY;
            pHeading += qHeading;
        }

        public void update(double measX, double measY, double measHeading, double rX, double rY, double rHeading) {
            double kX = pX / (pX + rX);
            double kY = pY / (pY + rY);
            double kHeading = pHeading / (pHeading + rHeading);

            x = x + kX * (measX - x);
            y = y + kY * (measY - y);

            double headingError = AngleUnit.normalizeRadians(measHeading - heading);
            heading = AngleUnit.normalizeRadians(heading + kHeading * headingError);

            pX = (1 - kX) * pX;
            pY = (1 - kY) * pY;
            pHeading = (1 - kHeading) * pHeading;
        }
    }
}