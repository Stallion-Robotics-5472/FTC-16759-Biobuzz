package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem extends Constants{
    /* define the motors/classes/variables */
    final DcMotor frontLeftMotor;
    final DcMotor backLeftMotor;
    final DcMotor frontRightMotor;
    final DcMotor backRightMotor;
    final IMU imu;
    final Telemetry telemetry;
    Gamepad driveCon;
    public double speedMultiplier = 1;
    public DriveSubsystem(Gamepad driveCon, HardwareMap hardwareMap, Telemetry telemetry){
        /* set settings for hardware */
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

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        driveCon.setTriggerThreshold(triggerThresh);

        this.driveCon = driveCon;

        this.telemetry = telemetry;
    } // initialization

    public void FieldCentric(double heading){
        double y = -driveCon.left_stick_y;
        double x = driveCon.left_stick_x;
        double rx = driveCon.right_stick_x;

        if (Math.abs(y) > 0.03 || Math.abs(x) > 0.03 || Math.abs(rx) > 0.03) {
            double botHeading = Math.toRadians(heading);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            changeSpeed();

            frontLeftMotor.setPower(frontLeftPower * speedMultiplier);
            backLeftMotor.setPower(backLeftPower * speedMultiplier);
            frontRightMotor.setPower(frontRightPower * speedMultiplier);
            backRightMotor.setPower(backRightPower * speedMultiplier);
        } else {
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
        }
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

        frontLeftMotor.setPower(frontLeftPower*speedMultiplier);
        backLeftMotor.setPower(backLeftPower*speedMultiplier);
        frontRightMotor.setPower(frontRightPower*speedMultiplier);
        backRightMotor.setPower(backRightPower*speedMultiplier);
    }

    void changeSpeed(){
        if (driveCon.left_trigger_pressed){
            speedMultiplier = 0.4;
        } else {
            speedMultiplier = 1;
        }
    }
}