package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TempTeleop")
public class ScrimbotTemp extends LinearOpMode {
    DcMotor fr;
    DcMotor fl;
    DcMotor br;
    DcMotor bl;
    DcMotor intake;
    DcMotorEx shooter;
    Servo gate;
    boolean shot = false;
    double targetVelo = 2000;
    double gatePos = 1;
    double open = 0.05;
    double closed = 0.09;

    @Override
    public void runOpMode() throws InterruptedException {

        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        bl = hardwareMap.get(DcMotor.class, "bl");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        gate = hardwareMap.get(Servo.class, "gate");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        shooter.setVelocity(0);
        gate.setPosition(0);

        while (opModeIsActive()){
            shot = gamepad2.right_bumper;

            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x*1.1, gamepad1.right_stick_x);

            if (gamepad2.left_trigger > 0.1 || gamepad2.right_trigger > 0.1){
                intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            } else {
                intake.setPower(0);
            }

            if (shot){
                shooter.setVelocity(targetVelo);
            } else {
                shooter.setVelocity(0);
            }

//            if (shooter.getVelocity() > targetVelo-50){
//                gate.setPosition();
//            }

            if (gamepad2.dpadUpWasPressed()){ gatePos += 0.1; }
            if (gamepad2.dpadDownWasPressed()){ gatePos -= 0.1; }
            if (gamepad2.dpadRightWasPressed()){ gatePos -= 0.01; }
            if (gamepad2.dpadLeftWasPressed()){ gatePos += 0.01; }

            gate.setPosition(gatePos);

            telemetry.addData("shooter", shooter.getVelocity());
            telemetry.addData("servo", gate.getPosition());
            telemetry.update();
        }
    }

    private void drive(double y, double x, double rx){
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
}
