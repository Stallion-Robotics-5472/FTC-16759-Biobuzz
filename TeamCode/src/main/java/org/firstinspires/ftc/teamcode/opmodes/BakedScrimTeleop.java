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

@TeleOp(name = "Scrim TeleOp", group = "Scrim")
public class BakedScrimTeleop extends LinearOpMode {
    DcMotor fl;
    DcMotor bl;
    DcMotor fr;
    DcMotor br;
    DcMotor intake;
    CRServo transferA;
    CRServo transferB;
    DcMotorEx shooter;
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

//        shooter.setVelocityPIDFCoefficients(kP, 0, kD, 0);

        waitForStart();

        while (opModeIsActive()){
            telemetry.update();

            fieldCentricDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x*1.1, gamepad1.right_stick_x);

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
                transferA.setPower(-1);
                transferB.setPower(-1);
            }

            if (gamepad1.dpadUpWasPressed()) {targetVelocity += 50;}
            if (gamepad1.dpadDownWasPressed()) {targetVelocity -= 50;}

            telemetry.addData("velo", shooter.getVelocity());
            telemetry.addData("trigger", gamepad1.left_trigger);
            telemetry.addData("Target", targetVelocity);
        }
    }

    public void fieldCentricDrive(double y, double x, double rx){
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);
    }

    public void replacePID(){
        shooter.setVelocityPIDFCoefficients(kP, 0, kD, 0);
    }
}