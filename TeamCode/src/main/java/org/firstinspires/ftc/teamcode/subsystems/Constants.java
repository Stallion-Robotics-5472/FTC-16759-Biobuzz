package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Constants {
    /* define the constants */
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
    public static final float triggerThresh = 0.7f;
    public static final double xPodOffset = 0;
    public static final double yPodOffset = 0;
    public static final Pose2D startPose = new Pose2D(
            DistanceUnit.INCH,
            0,
            0,
            AngleUnit.RADIANS,
            Math.toRadians(0)
    );
    public static final Pose2D camOffset = new Pose2D(
            DistanceUnit.INCH,
            0,
            0,
            AngleUnit.RADIANS,
            Math.toRadians(0)
    );
}