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
    public static final int highExt = 2000;
    public static final double elevkP = 0.012;
    public static final double elevkF = 0.15;
    public static final double intakeOpen = 0.15;
    public static final double intakeClosed = 0;
    public static final double outtakeOpen = 0.15;
    public static final double outtakeClosed = 0;

    //---------- GAMEPAD CONSTANTS ----------
    public static final float triggerThresh = 0.7f;
}