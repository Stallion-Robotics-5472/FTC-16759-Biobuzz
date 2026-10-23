package org.firstinspires.ftc.teamcode.subsystems.scrimbot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.command.SubsystemBase;

public class ScrimShooterSubsys extends SubsystemBase {
    DcMotorEx shooter;
    DcMotor transfer;
    double curVelocity = 0;

    public ScrimShooterSubsys(HardwareMap hardwareMap){
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setVelocityPIDFCoefficients(40,0,0.000001,0);
    }

    @Override
    public void periodic() {
        curVelocity = shooter.getVelocity();
    }

    public void setShooter(int velocity) { shooter.setVelocity(velocity); }
    public void setTransfer(int power) { transfer.setPower(power); }
    public void getShooter() { shooter.getVelocity(); }
    public void stop() { setShooter(0); }
}
