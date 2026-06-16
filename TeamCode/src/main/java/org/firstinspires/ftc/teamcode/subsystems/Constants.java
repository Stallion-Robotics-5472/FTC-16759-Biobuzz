package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.HashMap;
import java.util.Map;

public class Constants {
    /* define the constants */
    public double speedMultiplier = 1;
    public final int maxExt = 3650;
    public final int halfExt = maxExt/2;
    public final int tuckedExt = 0;
    public final int tipAng = 25;
    public final int levelAng = 0;
    public final double elevKP = 2.2;
    public final float triggerThresh = 0.7f;
    public final double xPodOffset = 0;
    public final double yPodOffset = 0;
    public final Pose2D startPose = new Pose2D(
            DistanceUnit.INCH,
            0,
            0,
            AngleUnit.RADIANS,
            Math.toRadians(0)
    );
    public final Pose2D camOffset = new Pose2D(
            DistanceUnit.INCH,
            0,
            0,
            AngleUnit.RADIANS,
            Math.toRadians(0)
    );
}