package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.HashMap;
import java.util.Map;

public class Constants {
    /* define the constants */

    //---------- ELEV CONSTANTS ----------
    public static final int tuckedExt = 0;
    public static final int highExt = 2600;
    public static final int midExt = highExt/2;
    public static final int lowExt = highExt/4;
    public static final double elevkP = 0.012;
    public static final double elevkI = 0.000;
    public static final double elevkD = 0.0006;
    public static final double elevkF = 0.15;
    public static final double maxVel = 500;
    public static final double maxAccel = 1000;

    //---------- INTAKE CONSTANTS ----------
    public static final double intakekP = 0.1;
    public static final double intakekI = 0.1;
    public static final double intakekD = 0.1;

    //---------- GAMEPAD CONSTANTS ----------
    public static final float triggerThresh = 0.7f;

    //---------- LOCALIZATION CONSTANTS ----------
    public static final String pinpointName = "pinpoint";
    public static final double xPodOffset = 0;
    public static final double yPodOffset = 0;
    public static final String limelightName = "limelight";
    public static final double limelightForwardOffset = 0;
    public static final double limelightRightOffset = 0;
    public static final double limelightYawOffset = 0;
    public static final Map<Integer, LocalizationSubsystem.TagPose> tagMap = new HashMap<Integer, LocalizationSubsystem.TagPose>(){{
        tagMap.put(11, new LocalizationSubsystem.TagPose(-72, -35.0, 90.0));
            // tag ID 11, in bottom left + facing east
        tagMap.put(12, new LocalizationSubsystem.TagPose(0, -72, 0));
            // tag ID 12, in bottom center + facing north
        }};
    public static final double minSizeTrust = 15;
    public static final double maxDistTrust = 80;
    public static final Pose2D startPose = new Pose2D(
            DistanceUnit.INCH,
            0,
            0,
            AngleUnit.RADIANS,
            Math.toRadians(0)
    );
}