package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "TeleOp", group = "Buzz")
@Disabled
public class BuzzTeleOp extends LinearOpMode {
    List<LynxModule> allHubs;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Superstructure ss = new Superstructure(gamepad1, gamepad2, hardwareMap, telemetry);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            ss.Periodic();

            telemetry.update();
        }
    }
}
