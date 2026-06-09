package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem {
    final DcMotor frontLeftMotor;
    final DcMotor backLeftMotor;
    final DcMotor frontRightMotor;
    final DcMotor backRightMotor;
    final IMU imu;
    final Telemetry telemetry;
    Constants constants = new Constants();
    Gamepad driveCon;
    public DriveSubsystem(Gamepad driveCon, HardwareMap hardwareMap, Telemetry telemetry){
        frontLeftMotor = hardwareMap.dcMotor.get("fl");
        backLeftMotor = hardwareMap.dcMotor.get("bl");
        frontRightMotor = hardwareMap.dcMotor.get("fr");
        backRightMotor = hardwareMap.dcMotor.get("br");
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        driveCon.setTriggerThreshold(constants.triggerThresh);

        this.driveCon = driveCon;

        this.telemetry = telemetry;
    } // initialization

    public void FieldCentric(){
        double y = -driveCon.left_stick_y;
        double x = -driveCon.left_stick_x;
        double rx = driveCon.right_stick_x;

        if (driveCon.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

//        changeSpeed();

        frontLeftMotor.setPower(frontLeftPower*constants.speedMultiplier);
        backLeftMotor.setPower(backLeftPower*constants.speedMultiplier);
        frontRightMotor.setPower(frontRightPower*constants.speedMultiplier);
        backRightMotor.setPower(backRightPower*constants.speedMultiplier);
    }

    public void RobotCentric(){
        double y = -driveCon.left_stick_y; // Remember, Y stick value is reversed
        double x = -driveCon.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = driveCon.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        changeSpeed();

        frontLeftMotor.setPower(frontLeftPower*constants.speedMultiplier);
        backLeftMotor.setPower(backLeftPower*constants.speedMultiplier);
        frontRightMotor.setPower(frontRightPower*constants.speedMultiplier);
        backRightMotor.setPower(backRightPower*constants.speedMultiplier);
    }

    void changeSpeed(){
        if (driveCon.left_trigger_pressed){
            constants.speedMultiplier = 0.4;
        } else {
            constants.speedMultiplier = 1;
        }
    }

    public void teleData(){
        telemetry.addLine("--DRIVE TELE--");
        telemetry.addData("Multiplier", constants.speedMultiplier);
        telemetry.addLine();
    }
}