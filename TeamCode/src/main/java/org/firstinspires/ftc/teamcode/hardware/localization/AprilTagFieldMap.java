package org.firstinspires.ftc.teamcode.localizer;

import org.firstinspires.ftc.teamcode.hardware.localization.Pose2D;

import java.util.HashMap;
import java.util.Map;

/**
 * Stores the known field position + orientation of each AprilTag, keyed by tag ID.
 * heading = direction the tag's face is pointing, in field-relative radians.
 */
public class AprilTagFieldMap {
    private final Map<Integer, Pose2D> tagPoses = new HashMap<>();

    public void addTag(int id, double x, double y, double headingRad) {
        tagPoses.put(id, new Pose2D(x, y, headingRad));
    }

    public Pose2D getTagPose(int id) {
        return tagPoses.get(id);
    }

    public boolean hasTag(int id) {
        return tagPoses.containsKey(id);
    }

    /** Example: standard INTO THE DEEP-style field map — replace with your season's coords. */
    public static AprilTagFieldMap defaultFieldMap() {
        AprilTagFieldMap map = new AprilTagFieldMap();
        map.addTag(1, -60.0, 41.4, Math.toRadians(90));
        map.addTag(2,   0.0, 41.4, Math.toRadians(90));
        map.addTag(3,  60.0, 41.4, Math.toRadians(90));
        map.addTag(4, -60.0, -41.4, Math.toRadians(-90));
        map.addTag(5,   0.0, -41.4, Math.toRadians(-90));
        map.addTag(6,  60.0, -41.4, Math.toRadians(-90));
        return map;
    }
}