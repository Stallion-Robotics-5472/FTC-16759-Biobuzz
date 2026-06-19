package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp", group = "Buzz")
@Disabled
public class BuzzTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Superstructure ss = new Superstructure(gamepad1, gamepad2, hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {

            ss.Periodic();

            telemetry.update();
        }
    }
}
