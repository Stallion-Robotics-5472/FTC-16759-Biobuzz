package org.firstinspires.ftc.teamcode.subsystems.scrimbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.lib.util.VelocityController;

public class ScrimShooterSubsys extends SubsystemBase {
    DcMotorEx shooter;
    Servo gate;

    public ScrimShooterSubsys(HardwareMap hardwareMap){
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        gate = hardwareMap.get(Servo.class, "gate");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(400, 0, 0.05, 0));
    }

    @Override
    public void periodic() {

    }

    public void setTPS(double TPS){ shooter.setVelocity(TPS); }
    public void setGate(double pos){ gate.setPosition(pos); }
    public DcMotorEx getShooter(){ return shooter; }
    public Servo getGate(){ return gate; }
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("velocity", shooter.getVelocity());
    }
    public void stop() { shooter.setVelocity(0); }
}
