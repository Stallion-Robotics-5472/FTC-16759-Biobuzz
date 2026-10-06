package org.firstinspires.ftc.teamcode.subsystems.scrimbot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.command.SubsystemBase;

public class ScrimShooterSubsys extends SubsystemBase {
    DcMotorEx shooter;
    CRServo transferA;
    CRServo transferB;
    double curVelocity = 0;

    public ScrimShooterSubsys(HardwareMap hardwareMap){
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        transferA = hardwareMap.get(CRServo.class, "transferA");
        transferB = hardwareMap.get(CRServo.class, "transferB");
    }

    @Override
    public void periodic() {
        curVelocity = shooter.getVelocity();
    }

    public void setShooter(int velocity) { shooter.setVelocity(velocity); }
    public void setTransfer(int power) { transferA.setPower(power); transferB.setPower(power); }
    public void stop() { setShooter(0); }
}
