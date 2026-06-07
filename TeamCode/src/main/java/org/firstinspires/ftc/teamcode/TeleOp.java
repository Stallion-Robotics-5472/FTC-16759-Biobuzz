package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Buzz")
//@Disabled
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Superstructure superstructure = new Superstructure(gamepad1, gamepad2, hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {

            superstructure.Periodic();

            telemetry.update();
        }
    }
}
