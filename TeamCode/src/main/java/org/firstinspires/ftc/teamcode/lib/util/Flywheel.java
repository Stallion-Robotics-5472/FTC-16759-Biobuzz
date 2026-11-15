package org.firstinspires.ftc.teamcode.lib.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Flywheel {
    DcMotorEx shooter;
    PowerRamper ramper = new PowerRamper(500);
    double target = 0;
    double stopBoostRange;
    double error = 0;
    double kP;
    double kD;
    double kF;
    public Flywheel(HardwareMap hardwareMap, String name, double target, double kP, double kD, double kF, double stopBoostRange){
        shooter = hardwareMap.get(DcMotorEx.class, name);

        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, 0, kD, kF));

        this.target = target;
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.stopBoostRange = stopBoostRange;
    }

    public void setTPS(double target){
        this.target = target;
        error = target - shooter.getVelocity();
        if (shooter.getVelocity() < target-stopBoostRange){
            shooter.setPower(0.75);
        } else {
            shooter.setVelocity(target);
        }
    }

    public void updateParameters(double kPchange, double kDchange, double kFchange, double boostRangeChange){
        this.stopBoostRange = stopBoostRange + boostRangeChange;
        this.kP = kP + kPchange;
        this.kD = kD + kDchange;
        this.kF = kF + kFchange;

        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, 0, kD, kF));
    }

    public DcMotorEx getShooter(){ return shooter; }
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Velocity", this.shooter.getVelocity());
        telemetry.addData("Target", this.target);
        telemetry.addData("Error", this.target - this.shooter.getVelocity());
        telemetry.addData("kP", this.kP);
        telemetry.addData("kD", this.kD);
        telemetry.addData("kF", this.kF);
    }
}
