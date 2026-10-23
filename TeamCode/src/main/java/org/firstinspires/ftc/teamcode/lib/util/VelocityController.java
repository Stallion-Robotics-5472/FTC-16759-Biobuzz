package org.firstinspires.ftc.teamcode.lib.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Flywheel velocity controller: fast spin-up, quick recovery after a ball, battery compensated.
 *
 * How it works (all speeds are FLYWHEEL RPM):
 *
 *   BOOST  - if the flywheel is more than boostEnterRpm below target (spin-up, or a ball just
 *            dragged it down), apply 100% power until it is within boostExitRpm of target.
 *   HOLD   - otherwise: voltage = kS + kV*target   (feedforward, does most of the work)
 *                               + kP*error         (proportional correction)
 *                               + integral         (trims leftover error, anti-windup clamped)
 *
 *   Everything in HOLD is computed in VOLTS, then divided by the current battery voltage to get
 *   motor power. That is what makes the same gains work on a fresh 13 V battery and a tired 11 V one.
 *
 * Call update() once per loop. Output is never negative, so when the target drops the flywheel
 * just coasts down (no reverse torque on the belt).
 */
public class VelocityController {

    /* ================= HARDWARE CONSTANTS: EDIT THESE ================= */

    /** Encoder ticks per MOTOR revolution. goBILDA 5202 6000 RPM (1:1) Yellow Jacket = 28. */
    public static final double TICKS_PER_MOTOR_REV = 28.0;

    /** Belt ratio: motor revolutions per flywheel revolution. 1.0 = 1:1 belt. 2.0 = motor spins 2x flywheel. */
    public static final double MOTOR_REVS_PER_FLYWHEEL_REV = 1.0;

    /* ================= TUNABLE GAINS (public so you can tweak live) ================= */

    /** Volts needed just to overcome static friction. Run FlywheelCharacterize to measure. */
    public double kS = 0.30;

    /** Volts per flywheel RPM. First guess is 12 V / 6000 RPM plus a little for load. Run FlywheelCharacterize. */
    public double kV = 0.0021 * MOTOR_REVS_PER_FLYWHEEL_REV;

    /** Volts per RPM of error (HOLD mode). Raise for a stiffer hold, lower if it oscillates. */
    public double kP = 0.004;

    /** Volts per (RPM * second) of accumulated error. Slowly removes leftover error. */
    public double kI = 0.004;

    /** Integral can never contribute more than +/- this many volts. */
    public double maxIntegralVolts = 1.0;

    /** Integral only accumulates when |error| is inside this band (and not boosting). */
    public double integralZoneRpm = 200.0;

    /** Fall this many RPM below target -> switch to full-power BOOST. Lower = reacts to smaller ball dips. */
    public double boostEnterRpm = 150.0;

    /** Leave BOOST once within this many RPM of target. Raise if you see overshoot after boost. */
    public double boostExitRpm = 40.0;

    /** isReady() needs |error| <= this... */
    public double readyToleranceRpm = 60.0;

    /** ...for at least this long (seconds). */
    public double readyDebounceSec = 0.08;

    /** Velocity smoothing, 0..1. 1.0 = no smoothing; lower = smoother but laggier. */
    public double velocityFilter = 0.7;

    /** How often to re-read battery voltage (the read is slow, a few ms). */
    public double voltageRefreshSec = 0.25;

    /* ================= INTERNALS ================= */

    private final DcMotorEx motor;
    private final VoltageSensor battery;

    private double targetRpm = 0.0;
    private double rpm = 0.0;
    private double batteryVolts = 12.0;
    private double integralVolts = 0.0;
    private double lastPower = 0.0;
    private double readyTimer = 0.0;

    private boolean boosting = false;
    private boolean ready = false;
    private boolean rpmInit = false;
    private boolean voltageInit = false;

    private long lastNanos = 0;
    private long lastVoltageNanos = 0;

    public VelocityController(HardwareMap hardwareMap, String motorName) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        battery = hardwareMap.voltageSensor.iterator().next();

        // We run our own control loop, so bypass the SDK's built-in velocity PID.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // FLOAT so the flywheel coasts instead of braking when power hits zero.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Access the motor, e.g. to call setDirection(). Positive power must give positive RPM. */
    public DcMotorEx getMotor() {
        return motor;
    }

    /** Set target flywheel speed in RPM. 0 turns the flywheel off. */
    public void setTargetRpm(double target) {
        targetRpm = Math.max(0.0, target);
    }

    public void stop() {
        targetRpm = 0.0;
        motor.setPower(0.0);
    }

    /** Call exactly once per loop iteration. */
    public void update() {
        long now = System.nanoTime();
        double dt = (lastNanos == 0) ? 0.02 : (now - lastNanos) * 1e-9;
        lastNanos = now;
        dt = Range.clip(dt, 0.001, 0.1);

        // ---- Battery voltage: refreshed a few times per second and smoothed ----
        if (!voltageInit || (now - lastVoltageNanos) * 1e-9 > voltageRefreshSec) {
            double v = battery.getVoltage();
            if (v > 6.0) { // ignore garbage readings
                batteryVolts = voltageInit ? (0.6 * batteryVolts + 0.4 * v) : v;
                voltageInit = true;
            }
            lastVoltageNanos = now;
        }

        // ---- Flywheel speed in RPM (motor ticks/sec -> flywheel RPM), lightly filtered ----
        double rawRpm = motor.getVelocity() / TICKS_PER_MOTOR_REV * 60.0 / MOTOR_REVS_PER_FLYWHEEL_REV;
        rpm = rpmInit ? rpm + velocityFilter * (rawRpm - rpm) : rawRpm;
        rpmInit = true;

        double error = targetRpm - rpm;
        double volts;

        if (targetRpm <= 0.0) {
            boosting = false;
            integralVolts = 0.0;
            volts = 0.0;
        } else {
            // Hysteresis so it doesn't chatter between modes
            if (!boosting && error > boostEnterRpm) {
                boosting = true;
            } else if (boosting && error < boostExitRpm) {
                boosting = false;
            }

            if (boosting) {
                volts = batteryVolts; // 100% power
            } else {
                if (Math.abs(error) < integralZoneRpm) {
                    integralVolts += kI * error * dt;
                    integralVolts = Range.clip(integralVolts, -maxIntegralVolts, maxIntegralVolts);
                }
                volts = kS + kV * targetRpm + kP * error + integralVolts;
            }
        }

        // Battery compensation: volts -> fraction of the CURRENT battery voltage.
        lastPower = Range.clip(volts / batteryVolts, 0.0, 1.0);
        motor.setPower(lastPower);

        // ---- Ready flag: near target (and not boosting) for a short debounce time ----
        if (targetRpm > 0.0 && !boosting && Math.abs(error) <= readyToleranceRpm) {
            readyTimer += dt;
        } else {
            readyTimer = 0.0;
        }
        ready = readyTimer >= readyDebounceSec;
    }

    /** True when the flywheel has settled at target. Use it to gate feeding the next ball. */
    public boolean isReady() {
        return ready;
    }

    public boolean isBoosting() {
        return boosting;
    }

    public double getRpm() {
        return rpm;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getPower() {
        return lastPower;
    }

    public double getBatteryVolts() {
        return batteryVolts;
    }

    public double getIntegralVolts() {
        return integralVolts;
    }
}