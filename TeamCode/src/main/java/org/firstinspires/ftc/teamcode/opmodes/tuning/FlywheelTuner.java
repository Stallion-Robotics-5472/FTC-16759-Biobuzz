package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.util.VelocityController;

import java.util.List;

/**
 * Test/tuning OpMode for FlywheelController.
 *
 *   A         spin up to the target speed
 *   B         stop the flywheel
 *   D-pad up  target +100 RPM
 *   D-pad dn  target -100 RPM
 *
 * Launch a ball through while watching RPM / Mode / Ready to see the dip and recovery.
 */
@TeleOp(name = "Flywheel Tuner", group = "Test")
public class FlywheelTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Manual bulk caching = fewer hub reads = faster loop = tighter velocity control.
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // "flywheel" must match the motor name in your robot configuration.
        VelocityController flywheel = new VelocityController(hardwareMap, "flywheel");
        // If positive power spins the wheel the wrong way, uncomment:
        // flywheel.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);

        // Free-speed limit in FLYWHEEL RPM (real max is lower under load / on a weak battery).
        double maxTarget = 6000.0 / VelocityController.MOTOR_REVS_PER_FLYWHEEL_REV;
        double target = Math.min(3000, maxTarget);
        boolean spinning = false;
        boolean lastUp = false;
        boolean lastDown = false;

        telemetry.addLine("Ready. A = spin up, B = stop, D-pad = change target.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            if (gamepad1.dpad_up && !lastUp) target += 100;
            if (gamepad1.dpad_down && !lastDown) target -= 100;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            target = Range.clip(target, 0, maxTarget);

            if (gamepad1.a) spinning = true;
            if (gamepad1.b) spinning = false;

            flywheel.setTargetRpm(spinning ? target : 0);
            flywheel.update();

            telemetry.addData("Target RPM", "%.0f", flywheel.getTargetRpm());
            telemetry.addData("Actual RPM", "%.0f", flywheel.getRpm());
            telemetry.addData("Error", "%.0f", flywheel.getTargetRpm() - flywheel.getRpm());
            telemetry.addData("Mode", flywheel.isBoosting() ? "BOOST" : "HOLD");
            telemetry.addData("Ready", flywheel.isReady());
            telemetry.addData("Power", "%.2f", flywheel.getPower());
            telemetry.addData("Battery (V)", "%.2f", flywheel.getBatteryVolts());
            telemetry.addData("Integral (V)", "%.2f", flywheel.getIntegralVolts());
            telemetry.update();
        }

        flywheel.stop();
    }
}