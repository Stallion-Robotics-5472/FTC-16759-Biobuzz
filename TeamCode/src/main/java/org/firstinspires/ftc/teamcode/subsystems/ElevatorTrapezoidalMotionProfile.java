package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * ElevatorSubsystem
 *
 * Controls a dual-motor elevator built with two goBILDA Swyft Linear Slide Kits,
 * each driven by a goBILDA 5203-2402-0014 (435 RPM Yellow Jacket) motor.
 *
 * A software trapezoidal motion profile is generated on every call to
 * {@link #goToPos(double)}, producing three phases:
 *   1. Acceleration  — velocity ramps from 0 → maxVelocity
 *   2. Cruise        — constant velocity (may be skipped for short moves)
 *   3. Deceleration  — velocity ramps from maxVelocity → 0
 *
 * A PIDF controller tracks the profile's instantaneous setpoint each loop.
 * The feedforward term (kF) compensates for gravity / friction, while PID
 * corrects any position error relative to the profile.
 *
 * ┌──────────────────────────────────────────────────────┐
 * │ PHYSICAL CONSTANTS (goBILDA 435 RPM + Swyft Kit)    │
 * │  Encoder resolution : 383.6 ticks / revolution      │
 * │  Spool diameter     : 32 mm                         │
 * │  Spool circumference: π × 32 ≈ 100.53 mm / rev      │
 * │  Ticks per mm       : 383.6 / 100.53 ≈ 3.816        │
 * └──────────────────────────────────────────────────────┘
 *
 * USAGE (inside an OpMode):
 * <pre>
 *   ElevatorSubsystem elevator = new ElevatorSubsystem(
 *       hardwareMap.get(DcMotorEx.class, "leftSlide"),
 *       hardwareMap.get(DcMotorEx.class, "rightSlide"),
 *       0.01, 0.0, 0.0005, 0.15,   // kP, kI, kD, kF
 *       500.0, 1000.0               // maxVelocity mm/s, maxAccel mm/s²
 *   );
 *
 *   // In loop():
 *   elevator.goToPosition(400.0);   // Start a move to 400 mm
 *   while (elevator.isBusy()) {
 *       elevator.update();          // Drive motors toward target
 *   }
 * </pre>
 */
public class ElevatorTrapezoidalMotionProfile {

    // ─── Hardware ─────────────────────────────────────────────────────────────
    private final DcMotorEx leftSlide;
    private final DcMotorEx rightSlide;

    // ─── Physical Constants ───────────────────────────────────────────────────

    /**
     * goBILDA Yellow Jacket 5203-2402-0014 (435 RPM) encoder resolution.
     * 383.6 quadrature ticks per revolution at the output shaft.
     */
    public static final double TICKS_PER_REV = 383.6;

    /**
     * Diameter of the Swyft Linear Slide Kit's driving spool (mm).
     * goBILDA Swyft kit uses a 32 mm spool; each motor revolution
     * advances the carriage by π × 32 ≈ 100.53 mm.
     */
    public static final double SPOOL_DIAMETER_MM     = 32.0;
    public static final double SPOOL_CIRCUMFERENCE_MM = Math.PI * SPOOL_DIAMETER_MM;

    /** Encoder ticks equivalent to one millimeter of carriage travel. */
    public static final double TICKS_PER_MM = TICKS_PER_REV / SPOOL_CIRCUMFERENCE_MM;

    /**
     * Soft travel limits (mm). Adjust to match your physical slide length.
     * These guard against commanding the elevator beyond its safe range.
     */
    public static final double MIN_HEIGHT_MM = 0.0;
    public static final double MAX_HEIGHT_MM = 700.0;

    // ─── PIDF Coefficients ────────────────────────────────────────────────────
    private double kP;
    private double kI;
    private double kD;
    /**
     * Feedforward gain. Scaled so that kF = 1.0 applies full motor power at
     * maximum velocity. A gravity/friction offset is typical here (e.g. 0.10–0.20).
     */
    private double kF;

    /**
     * Maximum contribution of the integral term to output power.
     * Prevents integral windup from saturating the motor command.
     */
    private static final double INTEGRAL_MAX_OUTPUT = 0.25;

    // ─── Motion Profile Constraints ───────────────────────────────────────────
    /** Maximum allowed velocity in encoder ticks per second. */
    private final double maxVelocity;

    /** Maximum allowed acceleration/deceleration in ticks per second². */
    private final double maxAcceleration;

    // ─── Active Trapezoidal Profile ───────────────────────────────────────────
    private double profileStartTicks;
    private double profileTargetTicks;

    /** +1 for extension (upward), −1 for retraction (downward). */
    private double profileDirection;

    private double accelTime;         // seconds to ramp up
    private double cruiseTime;        // seconds at max velocity
    private double decelTime;         // seconds to ramp down (== accelTime)
    private double totalTime;         // total profile duration

    private double accelDistTicks;    // ticks covered during acceleration
    private double cruiseDistTicks;   // ticks covered during cruise

    private boolean profileActive = false;

    // ─── PID State ────────────────────────────────────────────────────────────
    private double integralSum = 0.0;
    private double lastError   = 0.0;

    // ─── Timers ───────────────────────────────────────────────────────────────
    /** Elapsed time since the current profile began. */
    private final ElapsedTime profileTimer = new ElapsedTime();

    /** Delta-time between consecutive update() calls (for derivative term). */
    private final ElapsedTime loopTimer    = new ElapsedTime();

    // =========================================================================
    // Constructors
    // =========================================================================

    /**
     * Primary constructor — accepts raw PIDF gains.
     *
     * @param leftSlide          Left DcMotorEx (faces inward, FORWARD direction)
     * @param rightSlide         Right DcMotorEx (will be set to REVERSE internally)
     * @param kP                 Proportional gain  (start ~0.01 per tick of error)
     * @param kI                 Integral gain      (start near 0; increase for steady-state error)
     * @param kD                 Derivative gain    (start ~0.0005; dampens oscillation)
     * @param kF                 Feedforward gain   (start ~0.1–0.2 to offset gravity)
     * @param maxVelocityMmPerS  Elevator velocity cap in mm/s   (suggest 400–600)
     * @param maxAccelMmPerS2    Elevator acceleration cap mm/s² (suggest 800–1500)
     */
    public ElevatorTrapezoidalMotionProfile(
            DcMotorEx leftSlide,
            DcMotorEx rightSlide,
            double kP, double kI, double kD, double kF,
            double maxVelocityMmPerS,
            double maxAccelMmPerS2) {

        this.leftSlide  = leftSlide;
        this.rightSlide = rightSlide;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

        // Store motion constraints in tick-space for internal calculations.
        this.maxVelocity     = maxVelocityMmPerS * TICKS_PER_MM;
        this.maxAcceleration = maxAccelMmPerS2   * TICKS_PER_MM;

        configureMotors();
    }

    /**
     * Convenience constructor — accepts a {@link PIDFCoefficients} object.
     * Useful when tuning coefficients via FTC Dashboard or a config file.
     *
     * @param leftSlide          Left DcMotorEx
     * @param rightSlide         Right DcMotorEx
     * @param pidf               PIDF coefficient bundle
     * @param maxVelocityMmPerS  Elevator velocity cap in mm/s
     * @param maxAccelMmPerS2    Elevator acceleration cap in mm/s²
     */
    public ElevatorTrapezoidalMotionProfile(
            DcMotorEx leftSlide,
            DcMotorEx rightSlide,
            PIDFCoefficients pidf,
            double maxVelocityMmPerS,
            double maxAccelMmPerS2) {

        this(leftSlide, rightSlide,
             pidf.p, pidf.i, pidf.d, pidf.f,
             maxVelocityMmPerS, maxAccelMmPerS2);
    }

    // =========================================================================
    // Core Public API
    // =========================================================================

    /**
     * Commands the elevator to move to {@code targetMM} millimeters above
     * the home (zeroed) position, using a trapezoidal velocity profile.
     *
     * <p>Both slides are driven to the same encoder setpoint, ensuring
     * they remain mechanically synchronized throughout the move.
     *
     * <p>Call {@link #update()} on every loop iteration to execute the profile.
     * The elevator will hold its target position after the profile completes.
     *
     * @param targetMM Desired elevator height in millimeters. Clamped to
     *                 [{@link #MIN_HEIGHT_MM}, {@link #MAX_HEIGHT_MM}].
     */
    public void goToPos(double targetMM) {
        // Enforce soft limits.
        targetMM = Math.max(MIN_HEIGHT_MM, Math.min(MAX_HEIGHT_MM, targetMM));

        double currentTicks = getAveragePosition();
        double targetTicks  = targetMM * TICKS_PER_MM;
        double totalDist    = Math.abs(targetTicks - currentTicks);

        // Store profile endpoints.
        profileStartTicks  = currentTicks;
        profileTargetTicks = targetTicks;
        profileDirection   = Math.signum(targetTicks - currentTicks);

        // Trivially close — nothing to do.
        if (totalDist < 1.0) {
            profileActive = false;
            return;
        }

        // ── Compute trapezoidal profile timing ────────────────────────────────
        //
        //  Velocity
        //   ^
        //   |       ┌─────────┐           ← maxVelocity
        //   |      /           \
        //   |     /             \
        //   |    /               \
        //   └───────────────────────→ Time
        //       │accel│ cruise  │decel│
        //
        // Time to ramp from 0 to maxVelocity.
        double tAccel = maxVelocity / maxAcceleration;
        // Distance consumed by that ramp.
        double dAccel = 0.5 * maxAcceleration * tAccel * tAccel;

        if (2.0 * dAccel >= totalDist) {
            // ── Triangle profile (short move) ─────────────────────────────────
            // Not enough room to reach maxVelocity.
            // Peak velocity = √(maxAcceleration × totalDist).
            tAccel = Math.sqrt(totalDist / maxAcceleration);
            dAccel = 0.5 * maxAcceleration * tAccel * tAccel;

            accelTime     = tAccel;
            cruiseTime    = 0.0;
            decelTime     = tAccel;
            accelDistTicks  = dAccel;
            cruiseDistTicks = 0.0;
        } else {
            // ── Trapezoidal profile (normal move) ─────────────────────────────
            double dCruise = totalDist - 2.0 * dAccel;
            double tCruise = dCruise / maxVelocity;

            accelTime       = tAccel;
            cruiseTime      = tCruise;
            decelTime       = tAccel;
            accelDistTicks  = dAccel;
            cruiseDistTicks = dCruise;
        }

        totalTime = accelTime + cruiseTime + decelTime;

        // Reset PID state and timers.
        integralSum = 0.0;
        lastError   = 0.0;
        profileTimer.reset();
        loopTimer.reset();
        profileActive = true;
    }

    /**
     * Advances the motion profile and drives both motors to track it.
     *
     * <p><b>Must be called once per OpMode loop iteration.</b>
     *
     * <p>After the profile finishes the elevator switches to a proportional
     * hold, applying just enough power to resist any disturbance.
     */
    public void update() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        if (!profileActive) {
            // Position hold: proportional-only correction against the last target.
            double holdError = profileTargetTicks - getAveragePosition();
            setPower(Math.max(-1.0, Math.min(1.0, kP * holdError + kF)));
            return;
        }

        double elapsed = profileTimer.seconds();

        // ── Evaluate the profile at the current timestamp ─────────────────────
        double desiredPosition;
        double desiredVelocity; // signed ticks/s

        if (elapsed >= totalTime) {
            // Profile complete — snap to exact target.
            desiredPosition = profileTargetTicks;
            desiredVelocity = 0.0;
            profileActive   = false;

        } else if (elapsed < accelTime) {
            // ── Phase 1: Acceleration ─────────────────────────────────────────
            desiredVelocity = profileDirection * maxAcceleration * elapsed;
            desiredPosition = profileStartTicks
                    + profileDirection * 0.5 * maxAcceleration * elapsed * elapsed;

        } else if (elapsed < accelTime + cruiseTime) {
            // ── Phase 2: Cruise ───────────────────────────────────────────────
            double t = elapsed - accelTime;
            desiredVelocity = profileDirection * maxVelocity;
            desiredPosition = profileStartTicks
                    + profileDirection * (accelDistTicks + maxVelocity * t);

        } else {
            // ── Phase 3: Deceleration ─────────────────────────────────────────
            double t = elapsed - accelTime - cruiseTime;
            desiredVelocity = profileDirection * (maxVelocity - maxAcceleration * t);
            desiredPosition = profileStartTicks
                    + profileDirection * (accelDistTicks + cruiseDistTicks
                    + maxVelocity * t
                    - 0.5 * maxAcceleration * t * t);
        }

        // ── PIDF Controller ───────────────────────────────────────────────────
        double currentPosition = getAveragePosition();
        double error           = desiredPosition - currentPosition;

        // Guard against unreliable dt on the very first loop.
        if (dt > 1e-3) {
            integralSum += error * dt;

            // Anti-windup: clamp integral contribution to INTEGRAL_MAX_OUTPUT.
            if (kI > 0.0) {
                double maxSum = INTEGRAL_MAX_OUTPUT / kI;
                integralSum = Math.max(-maxSum, Math.min(maxSum, integralSum));
            }
        }

        double derivative = (dt > 1e-3) ? (error - lastError) / dt : 0.0;
        lastError = error;

        // Feedforward: scales desired velocity to [−1, 1] motor power space.
        double feedforward = (maxVelocity > 0.0)
                ? kF * (desiredVelocity / maxVelocity)
                : 0.0;

        double output = kP * error
                      + kI * integralSum
                      + kD * derivative
                      + feedforward;

        setPower(Math.max(-1.0, Math.min(1.0, output)));
    }

    /**
     * @return {@code true} while a motion profile is actively executing.
     *         Returns {@code false} once the elevator reaches its target
     *         and switches to position-hold mode.
     */
    public boolean isBusy() {
        return profileActive;
    }

    /**
     * Immediately cuts power to both motors and cancels the active profile.
     * The elevator will coast/brake depending on {@link DcMotor.ZeroPowerBehavior}.
     */
    public void stop() {
        profileActive = false;
        integralSum   = 0.0;
        lastError     = 0.0;
        setPower(0.0);
    }

    /**
     * Resets both encoders to zero. <b>Only call this when the elevator is
     * fully retracted at its hard-stop / home position.</b>
     */
    public void resetEncoders() {
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // =========================================================================
    // Getters & Setters
    // =========================================================================

    /**
     * @return Current average elevator height in millimeters,
     *         derived from encoder readings.
     */
    public double getCurrentPositionMM() {
        return getAveragePosition() / TICKS_PER_MM;
    }

    /** @return The most recently commanded target height in millimeters. */
    public double getTargetPositionMM() {
        return profileTargetTicks / TICKS_PER_MM;
    }

    /** @return Position error between the profile target and current position (mm). */
    public double getPositionErrorMM() {
        return (profileTargetTicks - getAveragePosition()) / TICKS_PER_MM;
    }

    /**
     * Updates PIDF gains at runtime without rebuilding the subsystem.
     * Particularly useful when tuning with FTC Dashboard.
     * Clears the integral accumulator whenever gains change.
     *
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param kF Feedforward gain
     */
    public void setPIDFCoefficients(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        integralSum = 0.0; // Reset accumulator to avoid a windup spike.
    }

    /**
     * Convenience overload accepting a {@link PIDFCoefficients} object.
     */
    public void setPIDFCoefficients(PIDFCoefficients pidf) {
        setPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);
    }

    // =========================================================================
    // Private Helpers
    // =========================================================================

    /**
     * Configures both motors with consistent settings.
     * The right motor is reversed so that positive power extends both slides.
     */
    private void configureMotors() {
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Manual PID loop needs raw encoder feedback, not the SDK's built-in controller.
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // BRAKE holds position when power is 0 (resists back-driving from gravity).
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // On a symmetric elevator the two motors face opposite directions,
        // so one must be reversed for both to lift together.
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Averages the encoder positions of both slides.
     * Using the mean reduces the effect of any minor mechanical asymmetry
     * (e.g., slight belt tension differences) on the control signal.
     *
     * @return Average encoder ticks across both motors.
     */
    private double getAveragePosition() {
        return (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition()) / 2.0;
    }

    /**
     * Writes {@code power} to both motors simultaneously, keeping them locked
     * to the same command so the slides stay synchronized under load.
     *
     * @param power Motor power in [−1.0, 1.0].
     *              Positive → extension (upward). Negative → retraction (downward).
     */
    private void setPower(double power) {
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }
}
