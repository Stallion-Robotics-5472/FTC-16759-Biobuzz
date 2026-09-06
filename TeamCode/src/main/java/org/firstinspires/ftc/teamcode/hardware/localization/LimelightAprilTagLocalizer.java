package org.firstinspires.ftc.teamcode.localizer;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.hardware.localization.LocalizerConstants;
import org.firstinspires.ftc.teamcode.hardware.localization.Pose2D;

import java.util.List;

public class LimelightAprilTagLocalizer {

    private final Limelight3A limelight;
    private final org.firstinspires.ftc.teamcode.localizer.AprilTagFieldMap fieldMap;

    public LimelightAprilTagLocalizer(Limelight3A limelight, org.firstinspires.ftc.teamcode.localizer.AprilTagFieldMap fieldMap) {
        this.limelight = limelight;
        this.fieldMap = fieldMap;
        limelight.pipelineSwitch(LocalizerConstants.Vision.APRILTAG_PIPELINE_INDEX);
        limelight.start();
    }

    public org.firstinspires.ftc.teamcode.localizer.VisionPoseEstimate getFieldPoseEstimate() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        double sumWeight = 0;
        double sumX = 0, sumY = 0, sumSin = 0, sumCos = 0;
        double sumDistance = 0;
        int count = 0;

        for (LLResultTypes.FiducialResult f : fiducials) {
            int id = f.getFiducialId();
            if (!fieldMap.hasTag(id)) continue;

            Pose2D tagFieldPose = fieldMap.getTagPose(id);

            org.firstinspires.ftc.robotcore.external.navigation.Pose3D robotSpacePose = f.getTargetPoseRobotSpace();
            if (robotSpacePose == null) continue;

            // BUG FIX: Limelight reports position in meters -- convert to inches
            // to match the rest of the codebase before mixing with field coordinates.
            double camToTagX = robotSpacePose.getPosition().x * LocalizerConstants.METERS_TO_INCHES;
            double camToTagY = robotSpacePose.getPosition().y * LocalizerConstants.METERS_TO_INCHES;
            double camToTagYaw = Math.toRadians(robotSpacePose.getOrientation().getYaw());

            double distance = Math.hypot(camToTagX, camToTagY);

            double tagHeading = tagFieldPose.heading;
            double robotHeading = Pose2D.normalizeAngle(tagHeading - camToTagYaw - Math.PI);

            double cos = Math.cos(robotHeading);
            double sin = Math.sin(robotHeading);
            double dx = camToTagX * cos - camToTagY * sin;
            double dy = camToTagX * sin + camToTagY * cos;

            double robotX = tagFieldPose.x - dx;
            double robotY = tagFieldPose.y - dy;

            double weight = distanceToWeight(distance);

            sumX += robotX * weight;
            sumY += robotY * weight;
            sumSin += Math.sin(robotHeading) * weight;
            sumCos += Math.cos(robotHeading) * weight;
            sumWeight += weight;
            sumDistance += distance;
            count++;
        }

        if (count == 0 || sumWeight == 0) return null;

        double avgX = sumX / sumWeight;
        double avgY = sumY / sumWeight;
        double avgHeading = Math.atan2(sumSin / sumWeight, sumCos / sumWeight);
        double avgDistance = sumDistance / count;

        double distanceConfidence = distanceToWeight(avgDistance);
        double multiTagBonus = Math.min(1.0,
                LocalizerConstants.Vision.MULTI_TAG_CONFIDENCE_BONUS * (count - 1));
        double confidence = Math.min(1.0, distanceConfidence + multiTagBonus);

        return new org.firstinspires.ftc.teamcode.localizer.VisionPoseEstimate(new Pose2D(avgX, avgY, avgHeading), confidence, avgDistance, count);
    }

    private double distanceToWeight(double distanceInches) {
        double full = LocalizerConstants.Vision.FULL_CONFIDENCE_DISTANCE_IN;
        double min = LocalizerConstants.Vision.MIN_CONFIDENCE_DISTANCE_IN;
        double floor = LocalizerConstants.Vision.CONFIDENCE_FLOOR;

        if (distanceInches <= full) return 1.0;
        if (distanceInches >= min) return floor;
        double t = (distanceInches - full) / (min - full);
        return 1.0 - t;
    }
}