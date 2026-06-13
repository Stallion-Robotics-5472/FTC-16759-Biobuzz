package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class LocalizationSubsystem {
    final Telemetry telemetry;
    Constants constants = new Constants();
//    List<AprilTagDetection> detectedTags = new ArrayList<>();
//    AprilTagProcessor processor;
//    VisionPortal cam;
    public Pose2D curPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    /*
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */
    Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    GoBildaPinpointDriver pinpoint;

    public LocalizationSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        pinpoint.setOffsets(0,0,DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.DEGREES,0));

//        processor = new AprilTagProcessor.Builder().
//                setCameraPose(cameraPosition, cameraOrientation)
//                .setDrawAxes(true)
//                .setDrawCubeProjection(true)
//                .setDrawTagOutline(true)
//                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//                .build();

//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

//        builder.addProcessor(processor);

//        cam = builder.build();

        this.telemetry = telemetry;
    } // initialization

    public void Periodic(){
        updatePos();
//        update();
//        AprilTagDetection id20 = getTagByID(20);
//        displayDetectionTelemetry(id20);
    }
//
//    public void update() {
//        detectedTags = processor.getDetections();
//
//    }
//
//    public List<AprilTagDetection> getDetectedTags() {
//        return detectedTags;
//    }
//
//    public void displayDetectionTelemetry(AprilTagDetection detectedID){
//        if (detectedID == null){return;}
//        if (detectedID.metadata != null) {
//            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedID.id, detectedID.metadata.name));
//            // Only use tags that don't have Obelisk in them
//            if (!detectedID.metadata.name.contains("Obelisk")) {
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
//                        detectedID.robotPose.getPosition().x,
//                        detectedID.robotPose.getPosition().y,
//                        detectedID.robotPose.getPosition().z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
//                        detectedID.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
//                        detectedID.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
//                        detectedID.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
//            }
//        } else {
//            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedID.id));
//            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedID.center.x, detectedID.center.y));
//        }
//    }
//
//    public AprilTagDetection getTagByID(int id) {
//        for (AprilTagDetection detection : detectedTags) {
//            if (detection.id == id) {
//                return detection;
//            }
//        }
//        return null;
//    }
//
//    public void stop(){
//        if (cam != null){
//            cam.close();
//        }
//    }

    public double getX(){
        return pinpoint.getPosX(DistanceUnit.INCH);
    }

    public double getY(){
        return pinpoint.getPosY(DistanceUnit.INCH);
    }

    public double getHeading(){
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public void updatePos(){
        curPose = new Pose2D(
                DistanceUnit.INCH,
                0 + getX(),
                0 + getY(),
                AngleUnit.DEGREES,
                0 + getHeading());
    }
}