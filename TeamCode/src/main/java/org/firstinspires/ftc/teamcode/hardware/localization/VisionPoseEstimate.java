package org.firstinspires.ftc.teamcode.localizer;

import org.firstinspires.ftc.teamcode.hardware.localization.Pose2D;

public class VisionPoseEstimate {
    public final Pose2D pose;
    public final double confidence;   // 0.0 (untrustworthy) .. 1.0 (very trustworthy)
    public final double avgDistanceInches;
    public final int tagCount;

    public VisionPoseEstimate(Pose2D pose, double confidence, double avgDistanceInches, int tagCount) {
        this.pose = pose;
        this.confidence = confidence;
        this.avgDistanceInches = avgDistanceInches;
        this.tagCount = tagCount;
    }
}