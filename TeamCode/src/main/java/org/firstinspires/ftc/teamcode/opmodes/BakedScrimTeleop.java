package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "ScrimTeleOp", group = "Scrim")
public class BakedScrimTeleop extends LinearOpMode {
    DcMotor fl;
    DcMotor bl;
    DcMotor fr;
    DcMotor br;
    DcMotor intake;
    CRServo transferA;
    CRServo transferB;
    DcMotorEx shooter;
    IMU imu;
    double targetVelocity = 2000;
    double kP = 10;
    double kD = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transferA = hardwareMap.get(CRServo.class, "transferA");
        transferB = hardwareMap.get(CRServo.class, "transferB");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        imu = hardwareMap.get(IMU.class, "imu");

//        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
//        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
//        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transferA.setDirection(CRServo.Direction.REVERSE);
//        transferB.setDirection(CRServo.Direction.REVERSE);
//        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        imu.initialize(parameters);

//        shooter.setVelocityPIDFCoefficients(kP, 0, kD, 0);

        waitForStart();

        while (opModeIsActive()){
            telemetry.update();

            fieldCentricDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1){
                intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.right_bumper) {
                shooter.setVelocity(targetVelocity);
                if (shooter.getVelocity() > targetVelocity-100){
                    transferA.setPower(1);
                    transferB.setPower(1);
                }
            } else {
                shooter.setVelocity(0);
                transferA.setPower(0);
                transferB.setPower(0);
            }

            if (gamepad1.dpadUpWasPressed()) { kP += 0.1; replacePID(); }
            if (gamepad1.dpadDownWasPressed()) { kP -= 0.1; replacePID(); }
            if (gamepad1.dpadLeftWasPressed()) { kD += 0.05; replacePID(); }
            if (gamepad1.dpadRightWasPressed()) { kD -= 0.05; replacePID(); }

            telemetry.addData("velo", shooter.getVelocity());
        }
    }

    public void fieldCentricDrive(double y, double x, double rx){
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);
    }

    public void replacePID(){
        shooter.setVelocityPIDFCoefficients(kP, 0, kD, 0);
    }
}