package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoTuner")
public class ServoTuner extends LinearOpMode {

    Servo servo;
    double pos;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo");

        servo.setPosition(0);

        waitForStart();

        while(opModeIsActive()){
            if (gamepad1.dpadUpWasPressed()){ pos+= 0.01; }
            if (gamepad1.dpadDownWasPressed()){ pos -= 0.01; }

            servo.setPosition(pos);

            telemetry.addData("pos", servo.getPosition());
            telemetry.update();
        }
    }
}
