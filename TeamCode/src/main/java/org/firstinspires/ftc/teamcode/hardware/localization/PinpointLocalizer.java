package org.firstinspires.ftc.teamcode.hardware.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.localization.LocalizerConstants;
import org.firstinspires.ftc.teamcode.hardware.localization.Pose2D;

public class PinpointLocalizer {

    private final GoBildaPinpointDriver pinpoint;

    public PinpointLocalizer(GoBildaPinpointDriver pinpoint, Pose2D startPose) {
        this.pinpoint = pinpoint;

        pinpoint.setOffsets(
                LocalizerConstants.Pinpoint.X_POD_OFFSET_MM,
                LocalizerConstants.Pinpoint.Y_POD_OFFSET_MM,
                DistanceUnit.MM
        );
        pinpoint.setEncoderResolution(LocalizerConstants.Pinpoint.POD_TYPE);
        pinpoint.setEncoderDirections(
                LocalizerConstants.Pinpoint.X_POD_DIRECTION,
                LocalizerConstants.Pinpoint.Y_POD_DIRECTION
        );

        pinpoint.resetPosAndIMU();
        setPose(startPose);
    }

    /** Call once per loop before reading pose. */
    public Pose2D update() {
        pinpoint.update();
        return readPose();
    }

    public Pose2D getPose() {
        return readPose();
    }

    private Pose2D readPose() {
        org.firstinspires.ftc.robotcore.external.navigation.Pose2D p = pinpoint.getPosition();
        return new Pose2D(
                p.getX(DistanceUnit.INCH),
                p.getY(DistanceUnit.INCH),
                p.getHeading(AngleUnit.RADIANS)
        );
    }

    /** Used by the fused localizer to snap-correct drift after a vision update. */
    public void setPose(Pose2D newPose) {
        org.firstinspires.ftc.robotcore.external.navigation.Pose2D pinpointPose = new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                DistanceUnit.INCH, newPose.x, newPose.y,
                AngleUnit.RADIANS, newPose.heading
        );
        pinpoint.setPosition(pinpointPose);
    }

    public GoBildaPinpointDriver.DeviceStatus getStatus() {
        return pinpoint.getDeviceStatus();
    }
}