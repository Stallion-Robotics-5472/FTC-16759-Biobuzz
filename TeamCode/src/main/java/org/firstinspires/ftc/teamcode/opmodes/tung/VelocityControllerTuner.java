package org.firstinspires.ftc.teamcode.opmodes.tung;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.util.VelocityController;

/**
 * Measures kS and kV for FlywheelController.
 *
 * SAFETY: the flywheel will reach several thousand RPM. Clear the launcher of balls, hands and
 * loose items, keep everyone clear of the wheel, and wear eye protection.
 *
 * Press START. It runs the flywheel at 5 power levels. At each level it WAITS until the speed
 * has stopped changing (a heavier flywheel takes longer), then averages the speed and the actual
 * voltage applied, and finally fits   volts = kS + kV * RPM.
 * Copy the kS and kV shown on the Driver Station into FlywheelController.
 *
 * Takes roughly 10-60 seconds depending on how heavy the flywheel is.
 */
@TeleOp(name = "VelocityControllerTuner", group = "Test")
//@Disabled
public class VelocityControllerTuner extends LinearOpMode {

    private static final double[] POWERS = {0.30, 0.45, 0.60, 0.75, 0.90};

    private static final double MIN_SETTLE_SEC = 1.0;      // always wait at least this long
    private static final double MAX_SETTLE_SEC = 12.0;     // give up waiting after this long
    private static final double CHECK_WINDOW_SEC = 0.5;    // compare speed averaged over windows this long
    private static final double STABLE_FRACTION = 0.005;   // "settled" = speed changed < 0.5% between windows
    private static final double STABLE_MIN_RPM = 10.0;     //            (but never demand better than this)
    private static final double SAMPLE_SEC = 1.0;          // final averaging window

    private DcMotorEx motor;
    private VoltageSensor battery;

    private double motorTpsToFlywheelRpm(double ticksPerSec) {
        return ticksPerSec / VelocityController.TICKS_PER_MOTOR_REV * 60.0
                / VelocityController.MOTOR_REVS_PER_FLYWHEEL_REV;
    }

    /** Average flywheel RPM over the next 'seconds' seconds. */
    private double averageRpm(double seconds) {
        ElapsedTime t = new ElapsedTime();
        double sum = 0;
        int n = 0;
        while (opModeIsActive() && t.seconds() < seconds) {
            sum += motorTpsToFlywheelRpm(motor.getVelocity());
            n++;
        }
        return n > 0 ? sum / n : 0;
    }

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "shooter");
        battery = hardwareMap.voltageSensor.iterator().next();
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Match whatever direction you use in FlywheelController:
        // motor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("CLEAR THE LAUNCHER: no balls, hands or loose items.");
        telemetry.addLine("Press START to run.");
        telemetry.update();
        waitForStart();

        int n = POWERS.length;
        double[] rpm = new double[n];
        double[] volts = new double[n];
        double[] settleTime = new double[n];
        boolean allSettled = true;
        int done = 0;

        for (int i = 0; i < n && opModeIsActive(); i++) {
            motor.setPower(POWERS[i]);
            telemetry.addData("Running at power", "%.2f", POWERS[i]);
            telemetry.addLine("Waiting for speed to settle...");
            telemetry.update();

            // Wait until the speed stops changing, instead of using a fixed time.
            ElapsedTime settle = new ElapsedTime();
            double prev = -1;
            boolean settled = false;
            while (opModeIsActive() && settle.seconds() < MAX_SETTLE_SEC) {
                double avg = averageRpm(CHECK_WINDOW_SEC);
                if (prev >= 0 && settle.seconds() >= MIN_SETTLE_SEC
                        && Math.abs(avg - prev) < Math.max(STABLE_MIN_RPM, STABLE_FRACTION * avg)) {
                    settled = true;
                    break;
                }
                prev = avg;
            }
            if (!settled) allSettled = false;
            settleTime[i] = settle.seconds();

            // Measure: average speed and the actual volts applied (power x battery voltage).
            double sumRpm = 0;
            double sumVolts = 0;
            int samples = 0;
            ElapsedTime sample = new ElapsedTime();
            while (opModeIsActive() && sample.seconds() < SAMPLE_SEC) {
                sumRpm += motorTpsToFlywheelRpm(motor.getVelocity());
                sumVolts += battery.getVoltage() * POWERS[i];
                samples++;
            }
            if (samples > 0) {
                rpm[done] = sumRpm / samples;
                volts[done] = sumVolts / samples;
                done++;
            }
        }
        motor.setPower(0);

        if (done < 2) {
            telemetry.addLine("Not enough data collected.");
            telemetry.update();
            while (opModeIsActive()) {
                idle();
            }
            return;
        }

        // Least-squares line fit: volts = kS + kV * rpm
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (int i = 0; i < done; i++) {
            sx += rpm[i];
            sy += volts[i];
            sxx += rpm[i] * rpm[i];
            sxy += rpm[i] * volts[i];
        }
        double kV = (done * sxy - sx * sy) / (done * sxx - sx * sx);
        double kS = (sy - kV * sx) / done;

        while (opModeIsActive()) {
            telemetry.addData("kS (volts)", "%.3f", kS);
            telemetry.addData("kV (volts per RPM)", "%.5f", kV);
            if (!allSettled) {
                telemetry.addLine("WARNING: speed never fully settled at some level; kV may read high. Re-run.");
            }
            for (int i = 0; i < done; i++) {
                telemetry.addData("Point " + (i + 1), "%.0f RPM @ %.2f V (settled in %.1fs)",
                        rpm[i], volts[i], settleTime[i]);
            }
            telemetry.addLine("Copy kS and kV into FlywheelController.");
            telemetry.update();
            sleep(100);
        }
    }
}