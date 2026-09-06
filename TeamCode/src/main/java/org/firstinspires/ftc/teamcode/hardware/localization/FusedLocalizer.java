package org.firstinspires.ftc.teamcode.hardware.localization;

public class FusedLocalizer {

    private final PinpointLocalizer odometry;
    private final org.firstinspires.ftc.teamcode.localizer.LimelightAprilTagLocalizer vision;

    private Pose2D fusedPose;
    private org.firstinspires.ftc.teamcode.localizer.VisionPoseEstimate lastVisionEstimate;

    public FusedLocalizer(PinpointLocalizer odometry, org.firstinspires.ftc.teamcode.localizer.LimelightAprilTagLocalizer vision, Pose2D startPose) {
        this.odometry = odometry;
        this.vision = vision;
        this.fusedPose = startPose;
    }

    public Pose2D update() {
        Pose2D odomPose = odometry.update();
        org.firstinspires.ftc.teamcode.localizer.VisionPoseEstimate visionEstimate = vision.getFieldPoseEstimate();
        lastVisionEstimate = visionEstimate;

        if (visionEstimate == null) {
            fusedPose = odomPose;
        } else {
            double trust = LocalizerConstants.Fusion.MAX_VISION_TRUST * visionEstimate.confidence;
            Pose2D visionPose = visionEstimate.pose;

            double x = odomPose.x + trust * (visionPose.x - odomPose.x);
            double y = odomPose.y + trust * (visionPose.y - odomPose.y);
            double heading = Pose2D.normalizeAngle(
                    odomPose.heading + trust * Pose2D.normalizeAngle(visionPose.heading - odomPose.heading)
            );
            fusedPose = new Pose2D(x, y, heading);
            odometry.setPose(fusedPose);
        }
        return fusedPose;
    }

    public Pose2D getPose() {
        return fusedPose;
    }

    public org.firstinspires.ftc.teamcode.localizer.VisionPoseEstimate getLastVisionEstimate() {
        return lastVisionEstimate;
    }
}