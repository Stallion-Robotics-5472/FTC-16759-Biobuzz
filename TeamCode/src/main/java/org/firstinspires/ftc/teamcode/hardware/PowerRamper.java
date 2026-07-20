package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Pure Java Slew Rate Limiter for FTC (No FTCLib required).
 * Prevents rapid changes in motor power/velocity to reduce mechanical jerk.
 */
public class PowerRamper {
    private final double maxRatePerSecond;
    private double result;
    private final ElapsedTime timer;
    double dt = 0;

    public PowerRamper(double maxRatePerSecond) {
        this.maxRatePerSecond = maxRatePerSecond;
        this.result = 0.0;
        this.timer = new ElapsedTime();
    }

    public double calculate(double targetValue) {
        dt = timer.seconds();
        timer.reset();

        if (dt > 0.2) {
            dt = 0.02;
        }

        double maxAllowedChange = maxRatePerSecond * dt;
        double error = targetValue - result;
        double step = Math.max(-maxAllowedChange, Math.min(maxAllowedChange, error));

        result += step;
        return result;
    }

    public void reset(double initialValue) {
        this.result = initialValue;
        this.timer.reset();
    }
}