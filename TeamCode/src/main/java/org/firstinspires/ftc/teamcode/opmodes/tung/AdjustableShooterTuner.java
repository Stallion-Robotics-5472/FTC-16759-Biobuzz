package org.firstinspires.ftc.teamcode.opmodes.tung;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "AdjShooterTuning")
@Disabled
public class AdjustableShooterTuner extends LinearOpMode {
    DcMotorEx shooter;
    Servo servo;
    double pollenVelo = 2000;
    double nectarVelo = 2200;
    double pollenPos = 0.55;
    double nectarPos = 0.29;

    enum Mode {
        POLLEN,
        NECTAR,
        IDLE
    }
    Mode curmode;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        servo = hardwareMap.get(Servo.class, "servo");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        curmode = Mode.IDLE;

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.right_bumper){ curmode = Mode.NECTAR; }
            if (gamepad1.left_bumper){ curmode = Mode.POLLEN; }
            if (gamepad1.right_trigger + gamepad1.left_trigger > 0.1){ curmode = Mode.IDLE; }

            if (curmode == Mode.POLLEN) {
                if (gamepad1.dpadUpWasPressed()) {
                    pollenVelo += 50;
                }
                if (gamepad1.dpadDownWasPressed()) {
                    pollenVelo -= 50;
                }
                if (gamepad1.dpadRightWasPressed()) {
                    pollenPos += 0.01;
                }
                if (gamepad1.dpadLeftWasPressed()) {
                    pollenPos -= 0.01;
                }
                if (gamepad1.yWasPressed()) {
                    pollenPos += 0.1;
                }
                if (gamepad1.aWasPressed()) {
                    pollenPos -= 0.1;
                }

                shooter.setVelocity(pollenVelo);
                servo.setPosition(pollenPos);
            } else if (curmode == Mode.NECTAR){
                if (gamepad1.dpadUpWasPressed()) {
                    nectarVelo += 50;
                }
                if (gamepad1.dpadDownWasPressed()) {
                    nectarVelo -= 50;
                }
                if (gamepad1.dpadRightWasPressed()) {
                    nectarPos += 0.01;
                }
                if (gamepad1.dpadLeftWasPressed()) {
                    nectarPos -= 0.01;
                }
                if (gamepad1.yWasPressed()) {
                    nectarPos += 0.1;
                }
                if (gamepad1.aWasPressed()) {
                    nectarPos -= 0.1;
                }

                shooter.setVelocity(nectarVelo);
                servo.setPosition(nectarPos);
            } else if (curmode == Mode.IDLE){
                shooter.setVelocity(0);
            }

            telemetry.addData("pollenPos", pollenPos);
            telemetry.addData("pollenVelo", pollenVelo);
            telemetry.addData("nectarPos", nectarPos);
            telemetry.addData("nectarVelo", nectarVelo);
            telemetry.addData("mode", curmode);
            telemetry.addData("curVelo", shooter.getVelocity());
            telemetry.update();
        }
    }
}
