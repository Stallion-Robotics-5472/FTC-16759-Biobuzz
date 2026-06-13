package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TrapezoidalMotionProfile {
    private final DcMotorEx motor;
    private final ElapsedTime timer;

    // Profile Constraints (in encoder ticks per second / per second squared)
    private final double maxVel;
    private final double maxAccel;

    // Tuning constants for control
    public double kP = 0.005; // Proportional gain (corrects position error)
    public double kV = 0.0001; // Velocity feedforward (predicts power needed for target speed)
    public double kG = 0.002; //  Power to overcome gravity

    // State variables
    private double startPos;
    private double targetPos;
    private double direction;
    private double maxReachedVel;

    // Profile time segments
    private double timeAccel;
    private double timeCruise;

    public TrapezoidalMotionProfile(DcMotorEx motor, double maxVel, double maxAccel, double kP, double kV, double kG) {
        this.motor = motor;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.kP = kP;
        this.kV = kV;
        this.kG = kG;
        this.timer = new ElapsedTime();

        // We use custom power control, so we don't want the built-in PID fighting us
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Initializes the profile to travel to a new position.
     * @param target Target position in encoder ticks
     */
    public void goToPosition(int target) {
        this.startPos = motor.getCurrentPosition();
        this.targetPos = target;

        double distance = Math.abs(targetPos - startPos);
        this.direction = Math.signum(targetPos - startPos);

        // Calculate distance required to reach maximum velocity
        double accelDist = 0.5 * maxVel * maxVel / maxAccel;

        if (distance < 2 * accelDist) {
            // Triangular profile: We don't have enough distance to reach max velocity
            maxReachedVel = Math.sqrt(distance * maxAccel);
            timeAccel = maxReachedVel / maxAccel;
            timeCruise = 0;
        } else {
            // Standard Trapezoidal profile
            maxReachedVel = maxVel;
            timeAccel = maxReachedVel / maxAccel;
            double cruiseDist = distance - (2 * accelDist);
            timeCruise = cruiseDist / maxVel;
        }

        // Restart the timer for the new profile
        timer.reset();
    }

    /**
     * MUST be called repeatedly in the main OpMode loop to update motor power.
     */
    public void update() {
        double t = timer.seconds();
        double expectedPos = 0;
        double expectedVel = 0;

        // --- 1. ACCELERATION PHASE ---
        if (t < timeAccel) {
            expectedPos = startPos + direction * (0.5 * maxAccel * t * t);
            expectedVel = direction * (maxAccel * t);
        }
        // --- 2. CRUISE PHASE ---
        else if (t < timeAccel + timeCruise) {
            double accelDist = 0.5 * maxAccel * timeAccel * timeAccel;
            expectedPos = startPos + direction * (accelDist + maxReachedVel * (t - timeAccel));
            expectedVel = direction * maxReachedVel;
        }
        // --- 3. DECELERATION PHASE ---
        else if (t < 2 * timeAccel + timeCruise) {
            double accelDist = 0.5 * maxAccel * timeAccel * timeAccel;
            double cruiseDist = maxReachedVel * timeCruise;
            double decelTime = t - timeAccel - timeCruise;

            expectedPos = startPos + direction * (accelDist + cruiseDist + maxReachedVel * decelTime - 0.5 * maxAccel * decelTime * decelTime);
            expectedVel = direction * (maxReachedVel - maxAccel * decelTime);
        }
        // --- 4. PROFILE FINISHED ---
        else {
            expectedPos = targetPos;
            expectedVel = 0;
        }

        // --- CONTROL ALGORITHM (P + kV) ---
        double currentPos = motor.getCurrentPosition();
        double error = expectedPos - currentPos;

        // Calculate required motor power
        // kP pushes it to the correct position, kV predicts the power needed for the current speed
        double power = (kP * error) + (kV * expectedVel) + kG;

        // Clip power to safe bounds (-1.0 to 1.0)
        power = Math.max(-1.0, Math.min(1.0, power));

        motor.setPower(power);
    }

    // Optional: Check if the mechanism is close enough to the target to move on
    public boolean isFinished(double tolerance) {
        return Math.abs(targetPos - motor.getCurrentPosition()) < tolerance
                && timer.seconds() > (2 * timeAccel + timeCruise);
    }
}