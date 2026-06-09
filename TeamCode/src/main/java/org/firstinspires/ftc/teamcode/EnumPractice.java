package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp", group = "Buzz")
@Disabled
public class EnumPractice extends LinearOpMode {

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
